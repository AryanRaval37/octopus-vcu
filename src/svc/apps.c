// ! REVIEWED
//   Lot of work pending here also.
//   Mostly to do with checking the rules, implementing different datatype for pwn sensor input.
//   Deciding the control flow, where should the sensor be read? who should call the function, dt_ms for apps_step vs where the actual read sensor is called.
//   I agree that the sensor shouldn't be read anywhere here because this needs to be tested without the hardware also.

/* apps.c — Everything to do with reading Accelerator and Brake pedals.

 * There are two sensors for accelerator and brake each and the code here reads them, checks for plausibility, and produces a single reading.
 * Relevant Rules: T.4 and EV.4 
 *
 * Next: src/app/state.c.
 */

// Todo : 
// Units for both pedeal sensors
// APPS : one is voltage reading analog and other is pwm, see how to deal with them
// similar for brake i think.

#include "vcu/apps.h"

// Note: test this behavior mentioned here and see if it actually is a thing with the real pedal
// otherwise remove the flipping thing
// Raw millivolts -> 0..100.0 %, handling a flipped sensor (hi < lo).
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

static bool in_range(mv_t raw, const vcu_cfg_t *cfg)
{
    return raw >= cfg->apps_range_min_mv && raw <= cfg->apps_range_max_mv;
}

static pct_x10_t abs_diff(pct_x10_t a, pct_x10_t b)
{
    return (pct_x10_t)(a > b ? a - b : b - a);
}

// I don't see why fields of a are not mannually set to zero but ok.
void apps_init(apps_t *a)
{
    const apps_t zero = { 0 };
    *a = zero;
}

void apps_step(apps_t *a, const vcu_cfg_t *cfg, const vcu_in_t *in, uint16_t dt_ms)
{
    // NOTE:
    //  Here brake also has two sensors to be read. Checking plausibility and stuff is similar to the pedal sensors.
    //  i guess, hal folder should have the apps file which reads the sensors and dumps it to the common vcu_in_t struct.
    //  Does it make sense to have a separate dt_ms for apps_step and a different function to read the sensors?
    //  who calls the function in hal to actually read the sensors? vcu_step?

    // Brake first. BPPC needs it further down, and it is the easy one: one
    // sensor, nothing to arbitrate.
    a->brake = scale(in->brake_mv, cfg->brake_lo_mv, cfg->brake_hi_mv);
    a->brake_applied = a->brake >= cfg->brake_applied_pct;

    // Range check before anything else. A cut wire or a short lands outside
    // the sensor's legal window, and that is a different failure from "the
    // two disagree". Reading a log later, you want to know which one it was.
    a->ch1_ok = in_range(in->apps1_mv, cfg);
    a->ch2_ok = in_range(in->apps2_mv, cfg);

    a->ch1 = scale(in->apps1_mv, cfg->apps1_lo_mv, cfg->apps1_hi_mv);
    a->ch2 = scale(in->apps2_mv, cfg->apps2_lo_mv, cfg->apps2_hi_mv);

    // RECHECK : what to do in the 100ms during which the sensors disagree? 
    //           Currently it just takes the lower value.
    /* T.4.2.x: more than 10 % disagreement, held past 100 ms, and power goes
     * away. The timer is the rule, so add up real elapsed time rather than
     * counting ticks. One noisy sample must not trip it, and a real
     * disagreement must not get averaged away into something prettier. */
    if (!a->ch1_ok || !a->ch2_ok) {
        a->deviation_ms = cfg->apps_deviation_ms;   // broken wire: no grace period
    } else if (abs_diff(a->ch1, a->ch2) > cfg->apps_deviation_max) {
        uint32_t t = (uint32_t)a->deviation_ms + dt_ms;
        a->deviation_ms = (t > 0xFFFFu) ? 0xFFFFu : (uint16_t)t;
    } else {
        a->deviation_ms = 0;
    }

    if (a->deviation_ms >= cfg->apps_deviation_ms) a->implausible = true;

    // Take the lower channel. If a sensor drifts high the car accelerates
    // less than asked, never more.
    a->pedal = (a->ch1 < a->ch2) ? a->ch1 : a->ch2;

    // RECHECK: Resetting when things get back to normal again, check if this is right behavior.
    /* Clearing the latch. Agreement is not enough on its own — the driver
     * has to actually release the pedal. Otherwise an intermittent sensor
     * is something you can drive straight through by feathering the
     * throttle, which is the exact thing the rule exists to stop. */
    if (a->implausible && a->ch1_ok && a->ch2_ok &&
        abs_diff(a->ch1, a->ch2) <= cfg->apps_deviation_max &&
        a->pedal < cfg->apps_reset_below) {
        a->implausible  = false;
        a->deviation_ms = 0;
    }

    /* EV.4, brake plausibility: brakes on plus more than 25 % throttle means
     * something is stuck, and torque stays off until the pedal comes back
     * under 5 %. Lifting off the brake does not count.
     *
     * This is not the BSPD. That one is an analogue circuit on the shutdown
     * loop, and writing it here would satisfy neither the rule nor physics. */
    if (!a->bppc_latched) {
        if (a->brake_applied && a->pedal > cfg->bppc_apps_trip) a->bppc_latched = true;
    } else {
        if (a->pedal < cfg->bppc_apps_reset) a->bppc_latched = false;
    }
}
