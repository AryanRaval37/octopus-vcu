/* torque.c — how much, right now.
 *
 * ORDER MATTERS. That is the whole file. The pipeline is:
 *
 *   pedal -> map -> (BMS current limit, see below) -> thermal derate
 *         -> cell derate -> overspeed -> regen -> direction -> rate limit
 *         -> STATE GATE -> PLAUSIBILITY GATE -> out
 *
 * Everything before the gates may only ever make the number smaller. The
 * two gates come last and they are assignments, not clamps. Put a gate
 * anywhere else and some future derate step gets to hand torque back after
 * a safety check already took it away — and it will not look like a bug
 * when someone writes it, which is the dangerous part.
 *
 * Next: src/app/fault.c.
 */
#include "vcu/torque.h"

static nm_x10_t clamp_mag(nm_x10_t v, nm_x10_t max_mag)
{
    if (v >  max_mag) return  max_mag;
    if (v < -max_mag) return (nm_x10_t)(-max_mag);
    return v;
}

/* Linear fade between "still fine" and "nothing left", returning 1000 for
 * full torque down to 0 for none.
 *
 * It reads a little odd because it handles bands that run *downwards* as
 * well as upwards — temperature climbs into trouble, cell voltage sags into
 * it. Flipping the sign of the denominator turns one into the other, so
 * both derates share this and there is only one place to get it wrong. */
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

void torque_init(torque_t *t)
{
    const torque_t zero = { 0 };
    *t = zero;
    t->derate = PCT_MAX;
}

void torque_step(torque_t *t, const vcu_cfg_t *cfg, const vcu_in_t *in,
                 const apps_t *apps, const fault_mgr_t *faults,
                 vcu_state_t state)
{
    const drive_mode_t mode = (in->mode < DRIVE_MODE_COUNT) ? in->mode : DRIVE_MODE_ECO;

    // 1. pedal -> torque. Linear for now. Swap in a lookup table from NVM
    //    once you can actually feel the car. Keep it monotonic.
    nm_x10_t want = (nm_x10_t)(((int32_t)apps->pedal * cfg->torque_max[mode]) / PCT_MAX);

    /* 2. BMS current limit — the one stage of this pipeline that is not
     *    here yet, and the slot is left in the order so it goes back in the
     *    right place.
     *
     *    in->bms.discharge_limit is amps. want is newton-metres. Going
     *    between them needs a motor torque constant and a DC-link voltage,
     *    and inventing a number for that would give you a calibration you
     *    cannot check against anything. It belongs here, between the map
     *    and the thermal derate, once the motor datasheet exists.
     *
     *    Note that charge_limit IS used, down at step 6 — but only as a
     *    yes/no, "can the pack take any regen at all". That needs no
     *    conversion. It is the magnitudes that are missing, not the flags. */

    // 3. thermal derate, worst of motor and inverter.
    pct_x10_t d_motor = fade(in->inv.temp_motor,
                             cfg->derate_motor_start, cfg->derate_motor_stop);
    pct_x10_t d_inv   = fade(in->inv.temp_inverter,
                             cfg->derate_inv_start, cfg->derate_inv_stop);
    t->derate = (d_motor < d_inv) ? d_motor : d_inv;

    /* 4. cell undervoltage, faded the same way. Worth being careful here:
     *    a pack sags under load, so a hard cut at a threshold means torque
     *    drops, the sag recovers, torque comes back, it sags again. The car
     *    surges. Fading it means the pack settles somewhere instead of
     *    oscillating, and the driver gets a car that goes slowly rather
     *    than a car that bucks. */
    pct_x10_t d_cell = in->bms.valid
                     ? fade(in->bms.cell_min_mv, cfg->cell_derate_mv, cfg->cell_min_mv)
                     : PCT_MAX;
    if (d_cell < t->derate) t->derate = d_cell;

    want = apply_pct(want, t->derate);

    // 5. overspeed. Not a fade — there is no useful amount of too fast.
    if (in->inv.valid && in->inv.rpm >= cfg->rpm_max) want = 0;

    /* 6. regen. Negative torque when the driver is off the pedal, faded out
     *    at low rpm so the car does not jerk to a halt, and refused
     *    outright when the pack has nowhere to put the energy. */
    if (apps->pedal == 0 && in->inv.valid && state == VCU_DRIVE) {
        bool allowed = in->bms.valid &&
                       in->bms.soc < cfg->regen_soc_max &&
                       in->bms.charge_limit > 0;
        if (allowed && in->inv.rpm > 0) {
            nm_x10_t regen = cfg->torque_regen_max;
            if (in->inv.rpm < cfg->regen_fade_rpm && cfg->regen_fade_rpm > 0)
                regen = (nm_x10_t)(((int32_t)regen * in->inv.rpm) / cfg->regen_fade_rpm);
            want = (nm_x10_t)(-regen);
        }
    }

    // 7. direction.
    if (in->dir_request == DIR_NEUTRAL) want = 0;
    else if (in->dir_request == DIR_REVERSE && want > 0) want = (nm_x10_t)(-want);

    t->unlimited = want;

    // 8. rate limit. An unfiltered pedal glitch is a shock through the
    //    driveline, and the driveline is the expensive part.
    if (want > t->cmd + cfg->torque_rate_per_tick)
        want = (nm_x10_t)(t->cmd + cfg->torque_rate_per_tick);
    else if (want < t->cmd - cfg->torque_rate_per_tick)
        want = (nm_x10_t)(t->cmd - cfg->torque_rate_per_tick);

    want = clamp_mag(want, cfg->torque_max[mode]);

    // 9. STATE GATE. Torque exists in DRIVE and nowhere else.
    if (state != VCU_DRIVE) want = 0;

    // 10. PLAUSIBILITY GATE. The last word. LIMP and CRITICAL both mean
    //     zero; DERATE was already applied proportionally up at step 3.
    if (!apps_torque_permitted(apps))            want = 0;
    if (fault_worst(faults) >= FAULT_SEV_LIMP)   want = 0;

    t->cmd = want;
}
