// config.h - every tunable number, in one struct.
//
// A struct rather than #defines so it can eventually live in NVM and be
// changed in the paddock without reflashing, and so tests can build broken
// calibrations on purpose. Values are in config.c.

#ifndef VCU_CONFIG_H
#define VCU_CONFIG_H

#include "core/types.h"

// Calibration for one sensor channel, in that channel's own raw unit
// (millivolts for an analog sensor, 0.1 % duty for a PWM one).
typedef struct {
    uint16_t lo, hi;      // reading at 0 % and 100 % travel. hi < lo is a falling sensor
    uint16_t min, max;    // anything outside this is a wiring fault (T11.9.2)
} sensor_cal_t;

typedef struct {
    // Accelerator. T11.8.6 wants two analog sensors to have different,
    // non-intersecting transfer functions so a short between the signal
    // lines shows up as an implausibility. Ours is one analog and one PWM,
    // which can't short into agreeing, but channel 2 still runs the
    // opposite way to channel 1.
    sensor_cal_t apps1, apps2;
    pct_x10_t apps_deviation_max;      // T11.8.9: 10.0 percentage points
    uint16_t  apps_deviation_ms;       // T11.8.8: 100 ms
    pct_x10_t apps_reset_below;        // pedal must come back under this to clear

    // Brake. Two channels, treated as SCSs too: T6.1.13 allows regen on
    // brake travel, and anything that influences wheel torque is an SCS
    // under T11.9.1.
    sensor_cal_t brake1, brake2;
    pct_x10_t brake_deviation_max;
    uint16_t  brake_deviation_ms;
    pct_x10_t brake_reset_below;
    pct_x10_t brake_applied_pct;       // counts as "brakes on" above this

    // Brake + throttle at the same time. A6.4.4 requires this check to be
    // working but FB2027 doesn't give numbers for it; 25 % / 5 % are the
    // values from older FSAE/FSG rules. Not the BSPD, see bottom of file.
    pct_x10_t bppc_apps_trip;
    pct_x10_t bppc_apps_reset;

    // Precharge. EV5.7.1: at least 95 % of pack voltage before the second AIR.
    pct_x10_t precharge_target_pct;
    uint16_t  precharge_timeout_ms;
    uint16_t  precharge_min_rise_ms;   // by this point the link must have moved...
    dv_t      precharge_min_rise_dv;   // ...by at least this much

    uint16_t rtd_buzzer_ms;            // EV4.12.1: 1 to 3 s

    nm_x10_t  torque_max;
    nm_x10_t  torque_regen_max;        // magnitude, applied as negative torque
    nm_x10_t  torque_slew_per_ms;      // max change in command per ms
    rpm_t     rpm_max;
    rpm_t     regen_fade_rpm;          // regen fades to zero below this
    pct_x10_t regen_soc_max;           // no regen above this SOC

    // EV2.2.1 / EV2.2.2: 80 kW and 500 A at the accumulator outlet. That's
    // electrical power, and we can only limit torque (mechanical power), so
    // the limit is scaled by the drivetrain efficiency. Lower efficiency
    // here means a bigger safety margin.
    watt_t    power_max_w;
    amp_x10_t current_max;
    pct_x10_t drive_efficiency;

    // Derating: linear fade from full torque at *_start to none at *_stop.
    degc_t derate_motor_start, derate_motor_stop;
    degc_t derate_inv_start,   derate_inv_stop;
    mv_t   cell_derate_mv;             // start fading here...
    mv_t   cell_min_mv;                // ...zero torque here

    // Message health, T11.9.2.d. T11.9.4 caps the timeouts at 500 ms.
    uint16_t bms_timeout_ms;
    uint16_t inv_timeout_ms;
    uint16_t counter_stall_ms;

    // How long after power-on a device may stay silent before that counts
    // as a timeout. The BMS and inverter boot slower than we do.
    uint16_t boot_grace_ms;
} vcu_cfg_t;

// The calibration the car runs on. Tests copy it and change the copy.
const vcu_cfg_t *vcu_cfg_default(void);

// False for a calibration that can't be right (reversed bands, endpoints
// outside the valid window, timeouts over the rule limit, ...). The VCU
// refuses to leave INIT with one. See vcu_init().
bool vcu_cfg_valid(const vcu_cfg_t *c);

// ---------------------------------------------------------------------------
// About the BSPD
//
// T11.6 requires a standalone, non-programmable circuit that opens the SDC
// when hard braking happens while >= 5 kW goes to the motors. It is a
// separate board with its own brake pressure and current sensors. It is not
// this code and can't be (T11.6.1, T11.6.4), partly because one of the
// things it protects against is this MCU hanging.
//
// What we can do is make sure the software check trips before the BSPD
// does, since a BSPD trip needs an LVMS power cycle and a BPPC trip just
// needs the driver to lift. Once the BSPD is trimmed, write its numbers here:
//
//   TODO: brake pressure at trip: ____ bar  (T11.6.5: <= 30 bar)
//         DC power at trip:       ____ kW   (T11.6.1: >= 5 kW)
// ---------------------------------------------------------------------------

#endif // VCU_CONFIG_H
