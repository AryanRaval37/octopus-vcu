// pedals.c - plausibility for the accelerator and brake pairs, and the
// brake + throttle check.
//
// Rules: T11.8 (APPS), T11.9 (SCS failures), A6.4.4 (the BPPC has to work).

#include "core/sense/pedals.h"
#include "core/util.h"

// Raw reading -> 0..100.0 % travel. Works for a falling sensor (hi < lo).
// Clamps rather than wraps, so a pedal that reads 103 % because of
// tolerance is just 100 %. Whether the raw value is sane at all is the
// range check's job, not this.
static pct_x10_t scale(uint16_t raw, uint16_t lo, uint16_t hi)
{
    int32_t num, den;

    if (lo <= hi) {
        num = (int32_t)raw - lo;
        den = (int32_t)hi - lo;
    } else {
        num = (int32_t)lo - raw;
        den = (int32_t)lo - hi;
    }
    if (den <= 0 || num <= 0) return 0;
    if (num >= den) return PCT_MAX;
    return (pct_x10_t)((num * PCT_MAX) / den);
}

static pct_x10_t abs_diff(pct_x10_t a, pct_x10_t b)
{
    return (pct_x10_t)(a > b ? a - b : b - a);
}

static bool in_window(uint16_t raw, const sensor_cal_t *c)
{
    return raw >= c->min && raw <= c->max;
}

// One tick of a redundant pair.
//
// fail_low picks which channel to believe while they disagree. For the
// accelerator that's the lower one (the car does less than asked, never
// more). For the brake it's the higher one.
static void pair_step(sensor_pair_t *p, uint16_t raw1, uint16_t raw2,
                      const sensor_cal_t *c1, const sensor_cal_t *c2,
                      pct_x10_t dev_max, uint16_t dev_ms, pct_x10_t reset_below,
                      bool fail_low, uint16_t dt_ms)
{
    p->ch1_ok = in_window(raw1, c1);
    p->ch2_ok = in_window(raw2, c2);
    p->ch1 = scale(raw1, c1->lo, c1->hi);
    p->ch2 = scale(raw2, c2->lo, c2->hi);

    p->value = fail_low ? min_u16(p->ch1, p->ch2) : max_u16(p->ch1, p->ch2);

    // T11.8.9 counts both a disagreement and a T11.9 wiring fault as an
    // implausibility, and T11.8.8 allows 100 ms before power has to go. So
    // both run on the same timer. (This used to jump straight to "latched"
    // on the first out-of-range sample, so one bit of ADC noise meant no
    // torque until the driver lifted.) A wire that is actually broken still
    // gets caught quickly by the *_RANGE fault, which opens the AIRs.
    const bool agree = abs_diff(p->ch1, p->ch2) <= dev_max;
    const bool bad   = pair_range_bad(p) || !agree;

    p->deviation_ms = bad ? sat_add(p->deviation_ms, dt_ms) : 0;
    if (p->deviation_ms >= dev_ms) p->implausible = true;

    // Agreeing again isn't enough to clear it, the pedal also has to come
    // back up. Otherwise an intermittent sensor is something you can drive
    // through by feathering the pedal.
    if (p->implausible && !bad && p->value < reset_below) {
        p->implausible  = false;
        p->deviation_ms = 0;
    }
}

void pedals_init(pedals_t *p)
{
    const pedals_t zero = { 0 };
    *p = zero;
}

void pedals_step(pedals_t *p, const vcu_cfg_t *cfg, const vcu_in_t *in,
                 uint16_t dt_ms)
{
    pair_step(&p->brake, in->brake1_mv, in->brake2_mv, &cfg->brake1, &cfg->brake2,
              cfg->brake_deviation_max, cfg->brake_deviation_ms, cfg->brake_reset_below,
              false, dt_ms);

    pair_step(&p->apps, in->apps1_mv, in->apps2_duty, &cfg->apps1, &cfg->apps2,
              cfg->apps_deviation_max, cfg->apps_deviation_ms, cfg->apps_reset_below,
              true, dt_ms);

    // With a channel electrically broken there's no telling which one is
    // right, so the pedal reads zero until it's fixed.
    if (pair_range_bad(&p->apps)) p->apps.value = 0;

    // "Applied" uses the higher channel, which is the cautious reading for
    // the BPPC and the brake light. Entering R2D wants the opposite: both
    // channels have to agree the brake is pressed, or a sensor stuck high
    // would let the car go live without the driver's foot on the brake.
    const pct_x10_t on = cfg->brake_applied_pct;
    p->brake_applied   = p->brake.value >= on;
    p->brake_confirmed = p->brake.ch1 >= on && p->brake.ch2 >= on
                      && !pair_range_bad(&p->brake) && !p->brake.implausible;

    // BPPC. Brakes on with more than bppc_apps_trip of throttle latches the
    // torque off, and only lifting the throttle below bppc_apps_reset clears
    // it. Letting go of the brake doesn't count: with a stuck throttle that's
    // the last thing a startled driver will do.
    if (!p->bppc_latched) {
        if (p->brake_applied && p->apps.value > cfg->bppc_apps_trip)
            p->bppc_latched = true;
    } else if (p->apps.value < cfg->bppc_apps_reset) {
        p->bppc_latched = false;
    }
}
