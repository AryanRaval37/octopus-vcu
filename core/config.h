/* config.h — every tunable number in the car, in one struct.
 *
 * Not #defines. On the target these live in NVM, because you will want to
 * change the torque map in a paddock without a toolchain, and tests want to
 * build deliberately silly configurations to see what breaks. A magic number
 * buried in a .c file is one nobody can tune and nobody can test.
 *
 * The values themselves are in core/config.c.
 */
#ifndef VCU_CONFIG_H
#define VCU_CONFIG_H

#include "core/types.h"

typedef struct {
    /* Accelerator. Two sensors, and T11.8.6 requires analog ones to have
     * different, non-intersecting transfer functions — so channel 2 is
     * inverted. That is not belt-and-braces: it means a harness short that
     * ties the two signal lines together cannot produce a plausible pair,
     * which is precisely what T11.8.6 asks you to guarantee. */
    mv_t apps1_lo_mv, apps1_hi_mv;     // 0 % and 100 % pedal
    mv_t apps2_lo_mv, apps2_hi_mv;     // hi < lo means a falling slope
    mv_t apps_range_min_mv;            // outside this window = broken wire
    mv_t apps_range_max_mv;

    pct_x10_t apps_deviation_max;      // T11.8.9: 10.0 percentage points
    uint16_t  apps_deviation_ms;       // T11.8.8: 100 ms
    pct_x10_t apps_reset_below;        // release below this to clear

    /* Brake. Also two channels, and they are System Critical Signals for
     * the same reason the accelerator is: T6.1.13 lets the first 90 % of
     * brake travel command regen, and T11.9.1 makes anything that
     * influences wheel torque an SCS. */
    mv_t brake1_lo_mv, brake1_hi_mv;
    mv_t brake2_lo_mv, brake2_hi_mv;
    mv_t brake_range_min_mv, brake_range_max_mv;
    pct_x10_t brake_deviation_max;
    uint16_t  brake_deviation_ms;
    pct_x10_t brake_applied_pct;       // "the brakes are on" threshold

    /* Accelerator-and-brake-together check.
     *
     * A6.4.4 lists a working "APPS/brake pedal plausibility check" as a
     * requirement for operating the car, but FB2027 no longer specifies its
     * numbers anywhere. These two are carried over from older FSAE/FSG
     * rulebooks, which is what scrutineers will recognise. They are yours to
     * justify — write down why if you change them.
     *
     * This is NOT the BSPD. See the note at the bottom of this file. */
    pct_x10_t bppc_apps_trip;          // torque off above this, if braking
    pct_x10_t bppc_apps_reset;         // and stays off until below this

    // Pre-charge. EV5.7.1 requires >= 95 % of pack before the second AIR.
    pct_x10_t precharge_target_pct;
    uint16_t  precharge_timeout_ms;
    uint16_t  precharge_min_rise_ms;   // look for movement after this long
    dv_t      precharge_min_rise_dv;   // ...and demand at least this much

    // EV4.12.1: the R2D sound runs for at least 1 s and at most 3 s.
    uint16_t rtd_buzzer_ms;

    // Torque.
    nm_x10_t torque_max;
    nm_x10_t torque_regen_max;         // magnitude; applied negative
    nm_x10_t torque_rate_per_tick;
    rpm_t    rpm_max;
    rpm_t    regen_fade_rpm;           // regen fades out below this
    pct_x10_t regen_soc_max;           // and stops entirely above this SOC

    /* Power and current ceilings. EV2.2.1 caps TS power at 80 kW and
     * EV2.2.2 caps TS current at 500 A, and enforcing both is the VCU's
     * job — nothing downstream will do it for you. */
    watt_t   power_max_w;
    amp_x10_t current_max;

    // Derating. Each is a start/stop pair with a linear fade between them:
    // start is where backing off begins, stop is where nothing is left. The
    // cell pair runs downwards.
    degc_t derate_motor_start, derate_motor_stop;
    degc_t derate_inv_start,   derate_inv_stop;
    mv_t   cell_derate_mv;             // start fading here...
    mv_t   cell_min_mv;                // ...reach zero torque here

    /* Message health, per T11.9.2.d. A signal must survive three separate
     * failures: corruption, loss, and delay. T11.9.4 caps the delay you are
     * allowed to tolerate at 500 ms no matter what you put here. */
    uint16_t bms_timeout_ms;
    uint16_t inv_timeout_ms;
    uint16_t counter_stall_ms;         // frames arriving, counter frozen
} vcu_cfg_t;

/* ---------------------------------------------------------------------
 * A note about the BSPD, because this is where people get it wrong.
 *
 * T11.6 requires a Brake System Plausibility Device: a standalone,
 * NON-PROGRAMMABLE circuit that opens the shutdown circuit when hard
 * braking coincides with >= 5 kW going to the motors. It is a board with a
 * brake-pressure sensor, a current sensor, and a trip threshold set by a
 * varistor. It is not this file and it must not be.
 *
 * Writing a BSPD in software satisfies neither T11.6.1 ("standalone
 * non-programmable") nor T11.6.4 ("no additional functionality implemented
 * on all required PCBs"), and it does not make the car safe, because the
 * failure it guards against includes this MCU having hung.
 *
 * What this file SHOULD do is agree with it. When the BSPD board's varistor
 * is trimmed, write the resulting trip point here so the software backs off
 * before the hardware has to fire:
 *
 *   TODO: record the BSPD trip threshold once the board is trimmed.
 *         - brake pressure at trip:  ____ bar   (T11.6.5 requires <= 30 bar)
 *         - DC power at trip:        ____ kW    (T11.6.1 requires >= 5 kW)
 *   Then set bppc_apps_trip so the software cut happens FIRST. A BSPD that
 *   fires is a car that stops on track and needs an LVMS power cycle
 *   (T11.6.1); a software cut is a car that just stops pulling. You want to
 *   lose the argument in software every time.
 * --------------------------------------------------------------------- */

// A calibration that works. Copy it and edit the copy, so a test always has
// a known-good baseline to diff against.
const vcu_cfg_t *vcu_cfg_default(void);

#endif /* VCU_CONFIG_H */
