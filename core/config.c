// config.c — the numbers the car actually runs on.
//
// Separated from the code that uses them so that "what is the car calibrated
// to?" is a file you can open, not a struct you have to go hunting for.

#include "core/config.h"

static const vcu_cfg_t DEFAULT_CFG = {
    // ch1 rises 500 -> 4500 mV, ch2 falls 4500 -> 500 mV (T11.8.6).
    .apps1_lo_mv = 500,  .apps1_hi_mv = 4500,
    .apps2_lo_mv = 4500, .apps2_hi_mv = 500,
    .apps_range_min_mv = 300, .apps_range_max_mv = 4700,
    .apps_deviation_max = 100,   // 10.0 percentage points -- T11.8.9
    .apps_deviation_ms  = 100,   // 100 ms                 -- T11.8.8
    .apps_reset_below   = 50,    //  5.0 %

    // Both brake channels rise. They are separate sensors on the same pedal,
    // so unlike the accelerator there is no short-circuit argument for
    // opposing slopes -- but they still get range-checked and compared.
    .brake1_lo_mv = 500, .brake1_hi_mv = 4500,
    .brake2_lo_mv = 500, .brake2_hi_mv = 4500,
    .brake_range_min_mv = 300, .brake_range_max_mv = 4700,
    .brake_deviation_max = 150,  // 15.0 pp -- looser than APPS on purpose:
    .brake_deviation_ms  = 250,  // two sensors on one hydraulic pedal have
                                 // more mechanical spread than one hall pair
    .brake_applied_pct = 150,    // 15.0 %

    .bppc_apps_trip  = 250,      // 25.0 % -- historical value, see config.h
    .bppc_apps_reset = 50,       //  5.0 %

    .precharge_target_pct  = 950, // 95.0 % of pack -- EV5.7.1
    .precharge_timeout_ms  = 5000,
    .precharge_min_rise_ms = 500,
    .precharge_min_rise_dv = 50,  // 5.0 V of movement proves the resistor

    .rtd_buzzer_ms = 1500,        // EV4.12.1 allows 1000..3000 ms

    .torque_max           = 2000, // 200.0 Nm
    .torque_regen_max     = 400,
    .torque_rate_per_tick = 40,
    .rpm_max        = 6000,
    .regen_fade_rpm = 500,
    .regen_soc_max  = 950,

    .power_max_w = 80000,         // EV2.2.1
    .current_max = 5000,          // 500.0 A -- EV2.2.2

    .derate_motor_start = 90, .derate_motor_stop = 110,
    .derate_inv_start   = 70, .derate_inv_stop   = 85,
    .cell_derate_mv = 3300,   // start easing off here
    .cell_min_mv    = 3000,   // nothing left here

    // Well inside the 500 ms that T11.9.4 allows. The inverter is tighter
    // than the BMS because it is the one being told to make torque.
    .bms_timeout_ms   = 200,
    .inv_timeout_ms   = 100,
    .counter_stall_ms = 300,
};

/* Catch a calibration that cannot possibly be right. These are the mistakes
 * that produce a car which drives -- badly, or briefly -- rather than one
 * that refuses to start, which makes them the expensive kind. */
static bool cfg_sane(const vcu_cfg_t *c)
{
    return c->apps_deviation_max <= PCT_MAX
        && c->apps_reset_below   <  PCT_MAX
        && c->bppc_apps_reset    <  c->bppc_apps_trip
        && c->precharge_target_pct <= PCT_MAX
        && c->rtd_buzzer_ms >= 1000 && c->rtd_buzzer_ms <= 3000   // EV4.12.1
        && c->derate_motor_start < c->derate_motor_stop
        && c->derate_inv_start   < c->derate_inv_stop
        && c->cell_min_mv        < c->cell_derate_mv
        && c->torque_max > 0
        && c->power_max_w > 0
        && c->bms_timeout_ms <= 500 && c->inv_timeout_ms <= 500;  // T11.9.4
}

const vcu_cfg_t *vcu_cfg_default(void)
{
    // A bad calibration is worse than no calibration: it will drive. Refuse
    // to hand one out rather than let the car run on it.
    return cfg_sane(&DEFAULT_CFG) ? &DEFAULT_CFG : 0;
}
