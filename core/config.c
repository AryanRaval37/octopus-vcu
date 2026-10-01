// config.c - the numbers the car runs on.
//
// Anything marked "placeholder" is a guess until we have the real part or
// measurement. docs/need-from-team.md has the list of who to ask.

#include "core/config.h"

static const vcu_cfg_t DEFAULT_CFG = {
    // APPS1 analog, rising 0.5 -> 4.5 V. APPS2 PWM, falling 90 % -> 10 %
    // duty. Both placeholders until the pedal box is built and measured.
    .apps1 = { .lo = 500, .hi = 4500, .min = 300, .max = 4700 },   // mV
    .apps2 = { .lo = 900, .hi = 100,  .min = 50,  .max = 950  },   // 0.1 % duty
    .apps_deviation_max = 100,   // 10.0 pp, T11.8.9
    .apps_deviation_ms  = 100,   // T11.8.8
    .apps_reset_below   = 50,

    .brake1 = { .lo = 500, .hi = 4500, .min = 300, .max = 4700 },
    .brake2 = { .lo = 500, .hi = 4500, .min = 300, .max = 4700 },
    // Looser than the APPS: two sensors on a hydraulic system have more
    // spread between them than two on one pedal shaft.
    .brake_deviation_max = 150,
    .brake_deviation_ms  = 250,
    .brake_reset_below   = 50,
    .brake_applied_pct   = 150,

    .bppc_apps_trip  = 250,
    .bppc_apps_reset = 50,

    .precharge_target_pct  = 950,
    .precharge_timeout_ms  = 5000,
    .precharge_min_rise_ms = 500,
    .precharge_min_rise_dv = 50,     // 5.0 V

    .rtd_buzzer_ms = 1500,

    .torque_max         = 2000,      // 200.0 Nm, placeholder
    .torque_regen_max   = 400,
    .torque_slew_per_ms = 40,        // 0 -> full torque in 50 ms
    .rpm_max            = 6000,
    .regen_fade_rpm     = 500,
    .regen_soc_max      = 950,

    .power_max_w      = 80000,
    .current_max      = 5000,        // 500.0 A
    .drive_efficiency = 900,         // placeholder, want the motor+inverter map

    .derate_motor_start = 90, .derate_motor_stop = 110,
    .derate_inv_start   = 70, .derate_inv_stop   = 85,
    .cell_derate_mv = 3300,
    .cell_min_mv    = 3000,

    // The inverter is tighter because it's the one making torque.
    .bms_timeout_ms   = 200,
    .inv_timeout_ms   = 100,
    .counter_stall_ms = 300,

    .boot_grace_ms = 2000,
};

const vcu_cfg_t *vcu_cfg_default(void)
{
    return &DEFAULT_CFG;
}

// Endpoints have to sit strictly inside the valid window, otherwise a fully
// pressed (or fully released) pedal reads as a broken wire.
static bool cal_ok(const sensor_cal_t *s)
{
    const uint16_t bottom = s->lo < s->hi ? s->lo : s->hi;
    const uint16_t top    = s->lo < s->hi ? s->hi : s->lo;
    return s->lo != s->hi && s->min < bottom && top < s->max;
}

bool vcu_cfg_valid(const vcu_cfg_t *c)
{
    if (!c) return false;

    return cal_ok(&c->apps1) && cal_ok(&c->apps2)
        && cal_ok(&c->brake1) && cal_ok(&c->brake2)

        && c->apps_deviation_max <= PCT_MAX
        && c->apps_deviation_ms  <= 100                 // T11.8.8
        && c->apps_reset_below   <  PCT_MAX
        && c->brake_deviation_max <= PCT_MAX
        && c->brake_reset_below   <  PCT_MAX
        && c->brake_applied_pct > 0 && c->brake_applied_pct < PCT_MAX
        && c->bppc_apps_reset < c->bppc_apps_trip && c->bppc_apps_trip <= PCT_MAX

        && c->precharge_target_pct >= 950 && c->precharge_target_pct <= PCT_MAX   // EV5.7.1
        && c->precharge_min_rise_ms < c->precharge_timeout_ms
        && c->rtd_buzzer_ms >= 1000 && c->rtd_buzzer_ms <= 3000                  // EV4.12.1

        && c->torque_max > 0 && c->torque_regen_max >= 0 && c->torque_slew_per_ms > 0
        && c->rpm_max > 0
        && c->power_max_w > 0 && c->power_max_w <= 80000                         // EV2.2.1
        && c->current_max > 0 && c->current_max <= 5000                          // EV2.2.2
        && c->drive_efficiency > 0 && c->drive_efficiency <= PCT_MAX

        && c->derate_motor_start < c->derate_motor_stop
        && c->derate_inv_start   < c->derate_inv_stop
        && c->cell_min_mv        < c->cell_derate_mv

        && c->bms_timeout_ms > 0 && c->bms_timeout_ms <= 500                     // T11.9.4
        && c->inv_timeout_ms > 0 && c->inv_timeout_ms <= 500
        && c->counter_stall_ms > 0 && c->counter_stall_ms <= 500
        && c->boot_grace_ms <= 10000;
}
