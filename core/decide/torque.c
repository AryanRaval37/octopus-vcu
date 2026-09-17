/* torque.c — how much, right now.
 *
 * ORDER MATTERS. That is the whole file. The pipeline is:
 *
 *   pedal -> map -> power ceiling -> thermal derate -> cell derate
 *         -> overspeed -> regen -> direction -> rate limit
 *         -> STATE GATE -> PLAUSIBILITY GATE -> out
 *
 * Everything before the gates may only ever make the number smaller. The two
 * gates come last and they are assignments, not clamps. Put a gate anywhere
 * else and some future derate step gets to hand torque back after a safety
 * check already took it away — and it will not look like a bug when someone
 * writes it, which is the dangerous part.
 *
 * Next: core/datalog.c.
 */
#include "core/decide/torque.h"

static nm_x10_t clamp_mag(nm_x10_t v, nm_x10_t max_mag)
{
    if (v >  max_mag) return  max_mag;
    if (v < -max_mag) return (nm_x10_t)(-max_mag);
    return v;
}

/* Linear fade between "still fine" and "nothing left", returning 1000 for
 * full torque down to 0 for none.
 *
 * It reads a little odd because it handles bands that run downwards as well
 * as upwards — temperature climbs into trouble, cell voltage sags into it.
 * Flipping the sign of the denominator turns one into the other, so both
 * derates share this and there is only one place to get it wrong. */
static pct_x10_t fade(int32_t v, int32_t full, int32_t none)
{
    int32_t den = none - full;
    if (den == 0) return PCT_MAX;
    int32_t num = none - v;
    if (den < 0) { den = -den; num = -num; }
    if (num <= 0)   return 0;
    if (num >= den) return PCT_MAX;
    return (pct_x10_t)((num * PCT_MAX) / den);
}

static nm_x10_t apply_pct(nm_x10_t v, pct_x10_t pct)
{
    return (nm_x10_t)(((int32_t)v * pct) / PCT_MAX);
}

// Watts available if you are allowed `amps_x10` out of a pack sitting at
// `pack_dv`. P = V x I, with the two scalings cancelling into /100.
static watt_t amps_to_watts(amp_x10_t amps_x10, dv_t pack_dv)
{
    if (amps_x10 <= 0) return 0;
    return (watt_t)(((uint32_t)amps_x10 * (uint32_t)pack_dv) / 100u);
}

/* The torque that would draw exactly `limit_w` at this speed.
 *
 * P = T x omega, and with T in 0.1 Nm and omega from rpm that comes out as
 * T_x10 = P * 600 / (rpm * 2*pi) = P * 95.493 / rpm. The watts are scaled
 * down by 1000 first so the multiply cannot overflow an int32 at any power
 * you could plausibly configure.
 *
 * Below a few hundred rpm this returns something enormous, which is correct:
 * at low speed you can make full torque on very little power. The real
 * ceiling down there is current, and that is already folded into limit_w. */
static nm_x10_t power_to_torque(watt_t limit_w, rpm_t rpm)
{
    if (rpm == 0) return INT16_MAX;
    uint32_t t = ((limit_w / 1000u) * 95493u) / rpm;
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
                 vcu_state_t state, bool bms_ok, bool inv_ok)
{
    // 1. pedal -> torque. Linear for now. Swap in a lookup table from NVM
    //    once you can actually feel the car. Keep it monotonic.
    nm_x10_t want = (nm_x10_t)(((int32_t)pedals->apps.value * cfg->torque_max) / PCT_MAX);

    /* 2. Power ceiling — EV2.2.1 caps TS power at 80 kW and EV2.2.2 caps TS
     *    current at 500 A, and nothing downstream enforces either for you.
     *
     *    This is also where the BMS discharge limit lands, and going through
     *    power is what makes that possible at all: amps become newton-metres
     *    only if you know the motor's torque constant, which is not in this
     *    repo. But amps times pack volts are watts, and watts divided by
     *    rpm are newton-metres. No motor datasheet required.
     *
     *    Take the lowest of the three ceilings. The rules are a hard limit;
     *    the BMS is telling you what the pack can actually do today. */
    if (bms_ok && in->bms.pack_dv > 0) {
        watt_t limit = cfg->power_max_w;

        const watt_t by_rule_current = amps_to_watts(cfg->current_max, in->bms.pack_dv);
        if (by_rule_current < limit) limit = by_rule_current;

        const watt_t by_bms = amps_to_watts(in->bms.discharge_limit, in->bms.pack_dv);
        if (by_bms < limit) limit = by_bms;

        t->power_limit_w = limit;
        if (inv_ok) {
            const nm_x10_t cap = power_to_torque(limit, in->inv.rpm);
            if (want > cap) want = cap;
        }
    } else {
        t->power_limit_w = 0;
    }

    // 3. thermal derate, worst of motor and inverter.
    pct_x10_t d_motor = fade(in->inv.temp_motor,
                             cfg->derate_motor_start, cfg->derate_motor_stop);
    pct_x10_t d_inv   = fade(in->inv.temp_inverter,
                             cfg->derate_inv_start, cfg->derate_inv_stop);
    t->derate = (d_motor < d_inv) ? d_motor : d_inv;

    /* 4. cell undervoltage, faded the same way. Worth being careful here: a
     *    pack sags under load, so a hard cut at a threshold means torque
     *    drops, the sag recovers, torque comes back, it sags again. The car
     *    surges. Fading lets the pack settle somewhere instead of
     *    oscillating, and the driver gets a car that goes slowly rather than
     *    a car that bucks. */
    pct_x10_t d_cell = bms_ok
                     ? fade(in->bms.cell_min_mv, cfg->cell_derate_mv, cfg->cell_min_mv)
                     : PCT_MAX;
    if (d_cell < t->derate) t->derate = d_cell;

    want = apply_pct(want, t->derate);

    // 5. overspeed. Not a fade — there is no useful amount of too fast.
    if (inv_ok && in->inv.rpm >= cfg->rpm_max) want = 0;

    /* 6. regen. Negative torque when the driver is off the pedal, faded out
     *    at low rpm so the car does not jerk to a halt, and refused outright
     *    when the pack has nowhere to put the energy.
     *
     *    This is also what satisfies T11.8.12 — "a fully released
     *    accelerator pedal must result in a wheel torque of <= 0 Nm". With
     *    the pedal at zero the map already gives zero, and regen only ever
     *    makes that more negative. Never let a future step raise it. */
    if (pedals->apps.value == 0 && inv_ok && state == VCU_DRIVE) {
        bool allowed = bms_ok &&
                       in->bms.soc < cfg->regen_soc_max &&
                       in->bms.charge_limit > 0;
        if (allowed && in->inv.rpm > 0) {
            nm_x10_t regen = cfg->torque_regen_max;
            if (in->inv.rpm < cfg->regen_fade_rpm && cfg->regen_fade_rpm > 0)
                regen = (nm_x10_t)(((int32_t)regen * in->inv.rpm) / cfg->regen_fade_rpm);
            want = (nm_x10_t)(-regen);
        }
    }

    // 7. direction. There is no reverse — EV2.2.4 forbids spinning the
    //    wheels backwards, so neutral is the only other option.
    if (in->dir_request != DIR_FORWARD) want = 0;

    t->unlimited = want;

    // 8. rate limit. An unfiltered pedal glitch is a shock through the
    //    driveline, and the driveline is the expensive part.
    if (want > t->cmd + cfg->torque_rate_per_tick)
        want = (nm_x10_t)(t->cmd + cfg->torque_rate_per_tick);
    else if (want < t->cmd - cfg->torque_rate_per_tick)
        want = (nm_x10_t)(t->cmd - cfg->torque_rate_per_tick);

    want = clamp_mag(want, cfg->torque_max);

    // 9. STATE GATE. Torque exists in DRIVE and nowhere else.
    if (state != VCU_DRIVE) want = 0;

    // 10. PLAUSIBILITY GATE. The last word. LIMP and CRITICAL both mean
    //     zero; DERATE was already applied proportionally up at step 3.
    if (!pedals_torque_permitted(pedals))       want = 0;
    if (fault_worst(faults) >= FAULT_SEV_LIMP)  want = 0;

    t->cmd = want;
}
