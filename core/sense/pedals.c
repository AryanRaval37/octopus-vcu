/* pedals.c — the pedals, and whether to believe them.
 *
 * Smallest file here, and the one that can hurt someone. It all comes from
 * one sentence: a sensor cannot tell you it is lying. So there are two of
 * them on each pedal, and the job is not to read the pedal — it is to decide
 * whether the two stories agree well enough to act on.
 *
 * The accelerator and the brake are the same problem twice, so the
 * comparison lives in one function and each pedal supplies its own
 * thresholds and its own idea of which way to fail.
 *
 * Rules in play: T11.8 (accelerator), T11.9 (system critical signals),
 * A6.4.4 (the accelerator-and-brake check).
 *
 * Next: core/decide/faults.c.
 */
#include "core/sense/pedals.h"

// Raw millivolts -> 0..100.0 %, handling a falling sensor (hi < lo).
// Saturates instead of wrapping: a pedal reading 103 % because of tolerance
// is a 100 % pedal, not a fault.
static pct_x10_t scale(mv_t raw, mv_t lo, mv_t hi)
{
    int32_t num, den;

    if (lo <= hi) {
        num = (int32_t)raw - (int32_t)lo;
        den = (int32_t)hi  - (int32_t)lo;
    } else {
        num = (int32_t)lo - (int32_t)raw;
        den = (int32_t)lo - (int32_t)hi;
    }
    if (den <= 0) return 0;
    if (num <= 0) return 0;
    if (num >= den) return PCT_MAX;
    return (pct_x10_t)((num * PCT_MAX) / den);
}

static pct_x10_t abs_diff(pct_x10_t a, pct_x10_t b)
{
    return (pct_x10_t)(a > b ? a - b : b - a);
}

/* One tick of a redundant pair.
 *
 * `fail_low` is the interesting argument. When the two channels disagree we
 * still have to hand back a number, and which one you pick says what you
 * think the dangerous direction is. For the accelerator, believe the lower
 * channel — a drifting sensor then makes the car slower than asked, never
 * faster. For the brake, believe the higher one, for exactly the same
 * reason pointed the other way. */
static void pair_step(pair_t *p, mv_t raw1, mv_t raw2,
                      mv_t lo1, mv_t hi1, mv_t lo2, mv_t hi2,
                      mv_t range_min, mv_t range_max,
                      pct_x10_t dev_max, uint16_t dev_ms,
                      bool fail_low, uint16_t dt_ms)
{
    /* Range check first. An open or shorted wire lands outside the sensor's
     * legal window, and T11.9.2 treats that as a different animal from "the
     * two disagree" — different cause, different safe state, and different
     * thing to go and fix. */
    p->ch1_ok = (raw1 >= range_min) && (raw1 <= range_max);
    p->ch2_ok = (raw2 >= range_min) && (raw2 <= range_max);

    p->ch1 = scale(raw1, lo1, hi1);
    p->ch2 = scale(raw2, lo2, hi2);

    /* Deviation past its time limit, and power goes away. The timer is the
     * rule, so add up real elapsed time rather than counting ticks: one
     * noisy sample must not trip it, and a real disagreement must not get
     * averaged away into something prettier. */
    if (pair_range_bad(p)) {
        p->deviation_ms = dev_ms;                 // broken wire: no grace period
    } else if (abs_diff(p->ch1, p->ch2) > dev_max) {
        uint32_t t = (uint32_t)p->deviation_ms + dt_ms;
        p->deviation_ms = (t > 0xFFFFu) ? 0xFFFFu : (uint16_t)t;
    } else {
        p->deviation_ms = 0;
    }

    if (p->deviation_ms >= dev_ms) p->implausible = true;

    p->value = fail_low ? ((p->ch1 < p->ch2) ? p->ch1 : p->ch2)
                        : ((p->ch1 > p->ch2) ? p->ch1 : p->ch2);
}

void pedals_init(pedals_t *p)
{
    const pedals_t zero = { 0 };
    *p = zero;
}

void pedals_step(pedals_t *p, const vcu_cfg_t *cfg, const vcu_in_t *in,
                 uint16_t dt_ms)
{
    // Brake first, because the accelerator-and-brake check below needs it.
    pair_step(&p->brake, in->brake1_mv, in->brake2_mv,
              cfg->brake1_lo_mv, cfg->brake1_hi_mv,
              cfg->brake2_lo_mv, cfg->brake2_hi_mv,
              cfg->brake_range_min_mv, cfg->brake_range_max_mv,
              cfg->brake_deviation_max, cfg->brake_deviation_ms,
              false, dt_ms);                       // brake: believe the higher

    pair_step(&p->apps, in->apps1_mv, in->apps2_mv,
              cfg->apps1_lo_mv, cfg->apps1_hi_mv,
              cfg->apps2_lo_mv, cfg->apps2_hi_mv,
              cfg->apps_range_min_mv, cfg->apps_range_max_mv,
              cfg->apps_deviation_max, cfg->apps_deviation_ms,
              true, dt_ms);                        // accelerator: believe the lower

    p->brake_applied = p->brake.value >= cfg->brake_applied_pct;

    /* Clearing an implausibility latch takes more than agreement — the
     * driver has to physically release the pedal. Otherwise an intermittent
     * sensor is something you can drive straight through by feathering the
     * throttle, which is the exact thing the rule exists to stop. */
    if (p->apps.implausible && !pair_range_bad(&p->apps) &&
        abs_diff(p->apps.ch1, p->apps.ch2) <= cfg->apps_deviation_max &&
        p->apps.value < cfg->apps_reset_below) {
        p->apps.implausible  = false;
        p->apps.deviation_ms = 0;
    }

    if (p->brake.implausible && !pair_range_bad(&p->brake) &&
        abs_diff(p->brake.ch1, p->brake.ch2) <= cfg->brake_deviation_max &&
        p->brake.value < cfg->apps_reset_below) {
        p->brake.implausible  = false;
        p->brake.deviation_ms = 0;
    }

    /* Brakes on and a real throttle demand at the same time means something
     * is stuck, and torque stays off until the accelerator comes back under
     * the reset threshold. Lifting off the brake does not count — if it did,
     * a jammed throttle cable would be cleared by the one action a startled
     * driver is least likely to take.
     *
     * A6.4.4 requires this check to be working. It is NOT the BSPD, which is
     * a separate non-programmable circuit on the shutdown loop (T11.6) —
     * see the note in core/config.h about keeping the two thresholds
     * consistent once that board is trimmed. */
    if (!p->bppc_latched) {
        if (p->brake_applied && p->apps.value > cfg->bppc_apps_trip)
            p->bppc_latched = true;
    } else {
        if (p->apps.value < cfg->bppc_apps_reset) p->bppc_latched = false;
    }
}
