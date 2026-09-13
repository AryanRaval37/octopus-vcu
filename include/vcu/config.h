/* config.h — every tunable number in the car, in one struct.
 *
 * Not #defines, for two reasons. On the target these live in NVM, because
 * you will want to change the torque map in a paddock without a toolchain.
 * And tests want to build deliberately silly configurations to see what
 * breaks. A magic number buried in a .c file is one nobody can tune and
 * nobody can test.
 *
 * The defaults are at the top of src/vcu.c.
 */
#ifndef VCU_CONFIG_H
#define VCU_CONFIG_H

#include "vcu/types.h"

typedef struct {
    /* APPS. Two sensors on different slopes, per T.4. Channel 2 is inverted
     * on purpose: if the harness shorts the two lines together they cannot
     * both read the same plausible value, so the short shows up. */
    mv_t apps1_lo_mv, apps1_hi_mv;     // 0 % and 100 % pedal
    mv_t apps2_lo_mv, apps2_hi_mv;     // hi < lo means a falling slope
    mv_t apps_range_min_mv;            // outside this window = broken wire
    mv_t apps_range_max_mv;

    pct_x10_t apps_deviation_max;      // T.4: 10.0 %
    uint16_t  apps_deviation_ms;       // T.4: 100 ms
    pct_x10_t apps_reset_below;        // release below this to clear

    // brake
    mv_t      brake_lo_mv, brake_hi_mv;
    pct_x10_t brake_applied_pct;       // "the brakes are on" threshold

    // BPPC (EV.4)
    pct_x10_t bppc_apps_trip;          // 25.0 %
    pct_x10_t bppc_apps_reset;         //  5.0 %

    // precharge
    pct_x10_t precharge_target_pct;    // link must reach this % of pack
    uint16_t  precharge_timeout_ms;
    uint16_t  precharge_min_rise_ms;   // look for movement after this long
    dv_t      precharge_min_rise_dv;   // ...and demand at least this much

    uint16_t rtd_buzzer_ms;            // EV.10: at least 1000 ms

    // torque
    nm_x10_t torque_max[DRIVE_MODE_COUNT];
    nm_x10_t torque_regen_max;         // magnitude; applied negative
    nm_x10_t torque_rate_per_tick;
    rpm_t    rpm_max;
    rpm_t    regen_fade_rpm;           // regen fades out below this
    pct_x10_t regen_soc_max;           // and stops entirely above this SOC

    // Derating. Each of these is a start/stop pair and torque fades
    // linearly between them: start is where backing off begins, stop is
    // where there is nothing left. The cell pair runs downwards.
    degc_t derate_motor_start, derate_motor_stop;
    degc_t derate_inv_start,   derate_inv_stop;
    mv_t   cell_derate_mv;             // start fading here...
    mv_t   cell_min_mv;                // ...reach zero torque here

    uint16_t bms_timeout_ms;
    uint16_t inv_timeout_ms;
} vcu_cfg_t;

// A calibration that works. Copy it and edit the copy, so a test always has
// a known-good baseline to diff against.
const vcu_cfg_t *vcu_cfg_default(void);

#endif /* VCU_CONFIG_H */
