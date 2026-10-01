// torque.c - pedal position to torque command.
//
// The steps run in this order, and the order matters:
//
//   pedal map -> power ceiling -> derates -> overspeed -> regen
//     -> direction -> slew limit -> state gate -> fault gate
//
// Everything before the gates can only make the number smaller. The two
// gates come last and set it to zero rather than clamping, so no step added
// later can give torque back after a safety check has taken it away.

#include "core/decide/torque.h"

static nm_x10_t clamp_mag(nm_x10_t v, nm_x10_t max_mag)
{
    if (v >  max_mag) return max_mag;
    if (v < -max_mag) return (nm_x10_t)(-max_mag);
    return v;
}

// Linear fade: PCT_MAX at `full`, 0 at `none`, clamped outside. Works in
// either direction, so temperature (rising into trouble) and cell voltage
// (falling into it) share it.
static pct_x10_t fade(int32_t v, int32_t full, int32_t none)
{
    int32_t den = none - full;
    int32_t num = none - v;
    if (den == 0) return PCT_MAX;
    if (den < 0) { den = -den; num = -num; }
    if (num <= 0)   return 0;
    if (num >= den) return PCT_MAX;
    return (pct_x10_t)((num * PCT_MAX) / den);
}

static pct_x10_t min_pct(pct_x10_t a, pct_x10_t b) { return a < b ? a : b; }

static nm_x10_t apply_pct(nm_x10_t v, pct_x10_t pct)
{
    return (nm_x10_t)(((int32_t)v * pct) / PCT_MAX);
}

// P = V * I. 0.1 A times 0.1 V is 0.01 W, hence the /100.
static watt_t amps_to_watts(amp_x10_t amps_x10, dv_t pack_dv)
{
    if (amps_x10 <= 0) return 0;
    return (watt_t)(((uint32_t)amps_x10 * pack_dv) / 100u);
}

// Torque that uses exactly `w` watts at this speed. P = T * omega, so
// T[0.1 Nm] = P * 600 / (2 pi rpm) = P * 95.493 / rpm. Working in whole kW
// keeps the multiply inside 32 bits and rounds down, which is the safe way.
// At low rpm this comes out huge, which is right: power isn't the limit at
// low speed, current is, and that's already part of `w`.
static nm_x10_t power_to_torque(watt_t w, rpm_t rpm)
{
    if (rpm == 0) return INT16_MAX;
    uint32_t t = ((w / 1000u) * 95493u) / rpm;
    return (t > (uint32_t)INT16_MAX) ? INT16_MAX : (nm_x10_t)t;
}

void torque_init(torque_t *t)
{
    const torque_t zero = { 0 };
    *t = zero;
    t->derate = PCT_MAX;
}

void torque_step(torque_t *t, const vcu_cfg_t *cfg, const vcu_in_t *in,
                 const pedals_t *pedals, const fault_mgr_t *faults,
                 vcu_state_t state, bool bms_ok, bool inv_ok, uint16_t dt_ms)
{
    // Every limit below needs live numbers from both devices. If we can't
    // trust one of them we can't work out the limits, so no torque rather
    // than skipping the limits. (A CRITICAL fault will be along shortly in
    // that case, this covers the ticks before it lands.)
    const bool have_data = bms_ok && inv_ok && in->bms.pack_dv > 0;
    const dv_t  pack = in->bms.pack_dv;
    const rpm_t rpm  = in->inv.rpm;

    // 1. Pedal map. Linear until the drivers have something to say about it.
    nm_x10_t want = (nm_x10_t)(((int32_t)pedals->apps.value * cfg->torque_max) / PCT_MAX);

    // 2. Power ceiling. EV2.2.1 (80 kW) and EV2.2.2 (500 A) are measured
    //    at the accumulator outlet, so they're electrical. We only control
    //    shaft torque, so the limit is scaled down by the drivetrain
    //    efficiency first. The BMS discharge limit goes through the same
    //    path: amps * volts = watts, watts / rpm = torque, no motor
    //    constant needed. Lowest of the three wins.
    t->power_limit_w = 0;
    if (have_data) {
        watt_t limit = cfg->power_max_w;
        const watt_t by_current = amps_to_watts(cfg->current_max, pack);
        const watt_t by_bms     = amps_to_watts(in->bms.discharge_limit, pack);
        if (by_current < limit) limit = by_current;
        if (by_bms     < limit) limit = by_bms;
        t->power_limit_w = limit;

        // limit <= power_max_w <= 80 kW (vcu_cfg_valid), so this fits in 32 bits.
        const watt_t shaft_w = (limit * cfg->drive_efficiency) / PCT_MAX;
        const nm_x10_t cap = power_to_torque(shaft_w, rpm);
        if (want > cap) want = cap;
    } else {
        want = 0;
    }

    // 3. Derates, worst of motor temp, inverter temp and min cell voltage.
    //    All fades rather than cut-offs. A pack sags under load, so a hard
    //    cut at a cell threshold makes the car surge: torque drops, voltage
    //    recovers, torque comes back, voltage sags again.
    t->derate = 0;
    if (have_data) {
        t->derate = fade(in->inv.temp_motor, cfg->derate_motor_start, cfg->derate_motor_stop);
        t->derate = min_pct(t->derate,
                    fade(in->inv.temp_inverter, cfg->derate_inv_start, cfg->derate_inv_stop));
        t->derate = min_pct(t->derate,
                    fade(in->bms.cell_min_mv, cfg->cell_derate_mv, cfg->cell_min_mv));
    }
    want = apply_pct(want, t->derate);

    // 4. Overspeed.
    if (rpm >= cfg->rpm_max) want = 0;

    // 5. Regen when the driver is fully off the pedal. Faded out at low rpm
    //    so the car doesn't lurch to a stop, and capped by what the pack
    //    says it can take (EV2.2.3 puts no rule limit on regen power, but
    //    the cells still have one). With the pedal at zero the map already
    //    gave 0 and regen only makes it more negative, which is what keeps
    //    T11.8.12 (released pedal -> wheel torque <= 0) true.
    if (pedals->apps.value == 0 && have_data && state == VCU_DRIVE &&
        rpm > 0 && in->bms.soc < cfg->regen_soc_max) {
        nm_x10_t regen = cfg->torque_regen_max;
        if (cfg->regen_fade_rpm > 0 && rpm < cfg->regen_fade_rpm)
            regen = (nm_x10_t)(((int32_t)regen * rpm) / cfg->regen_fade_rpm);

        const nm_x10_t cap = power_to_torque(amps_to_watts(in->bms.charge_limit, pack), rpm);
        if (regen > cap) regen = cap;

        want = (nm_x10_t)(-regen);
    }

    // 6. Direction. No reverse (EV2.2.4), so anything but forward is zero.
    if (in->dir_request != DIR_FORWARD) want = 0;

    t->unlimited = want;

    // 7. Slew limit, so a step on the pedal isn't a step through the
    //    driveline. Per ms rather than per tick, so it doesn't change if
    //    the task rate does.
    const int32_t step = (int32_t)cfg->torque_slew_per_ms * dt_ms;
    if (want > t->cmd + step)      want = (nm_x10_t)(t->cmd + step);
    else if (want < t->cmd - step) want = (nm_x10_t)(t->cmd - step);

    want = clamp_mag(want, cfg->torque_max);

    // 8. State gate: torque only in DRIVE.
    if (state != VCU_DRIVE) want = 0;

    // 9. Fault gate. LIMP and CRITICAL both mean zero. DERATE was handled
    //    as a fade in step 3.
    if (!pedals_torque_permitted(pedals))      want = 0;
    if (fault_worst(faults) >= FAULT_SEV_LIMP) want = 0;

    t->cmd = want;
}
