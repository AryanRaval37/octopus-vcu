/* vcu.c — one tick of the whole car.
 *
 * Everything else in src/ is a specialist. This file is the one that knows
 * what order the specialists talk in, and that order is the design:
 *
 *   how long since last time?  ->  did the driver acknowledge anything?
 *   ->  read the pedals  ->  tell the fault manager what the world looks
 *   like  ->  decide what state we are in  ->  work out the torque
 *   ->  write it all down
 *
 * Nothing in here reads a clock or touches a pin. Everything arrives in
 * `in` and leaves in `out`, which is what lets a test replay a whole drive
 * on a laptop with no hardware attached.
 */
#include "vcu/vcu.h"

// The numbers the car actually runs on. Copy this struct, don't edit it.
static const vcu_cfg_t DEFAULT_CFG = {
    // ch1 rises 500 -> 4500 mV, ch2 falls 4500 -> 500 mV. Different slopes
    // are a rule requirement and a good idea anyway: a harness short that
    // ties both lines together then cannot look plausible.
    .apps1_lo_mv = 500,  .apps1_hi_mv = 4500,
    .apps2_lo_mv = 4500, .apps2_hi_mv = 500,
    .apps_range_min_mv = 300, .apps_range_max_mv = 4700,
    .apps_deviation_max = 100,   // 10.0 % -- T.4
    .apps_deviation_ms  = 100,   // 100 ms -- T.4
    .apps_reset_below   = 50,    //  5.0 %

    .brake_lo_mv = 500, .brake_hi_mv = 4500,
    .brake_applied_pct = 150,    // 15.0 %

    .bppc_apps_trip  = 250,      // 25.0 % -- EV.4
    .bppc_apps_reset = 50,       //  5.0 %

    .precharge_target_pct  = 950, // 95.0 % of pack
    .precharge_timeout_ms  = 5000,
    .precharge_min_rise_ms = 500,
    .precharge_min_rise_dv = 50,  // 5.0 V of movement proves the resistor

    .rtd_buzzer_ms = 1500,        // rule says at least 1000

    .torque_max = { [DRIVE_MODE_ECO] = 800, [DRIVE_MODE_SPORT] = 2000 },
    .torque_regen_max    = 400,
    .torque_rate_per_tick = 40,
    .rpm_max         = 6000,
    .regen_fade_rpm  = 500,
    .regen_soc_max   = 950,

    .derate_motor_start = 90, .derate_motor_stop = 110,
    .derate_inv_start   = 70, .derate_inv_stop   = 85,
    .cell_derate_mv = 3300,   // start easing off here
    .cell_min_mv    = 3000,   // nothing left here

    .bms_timeout_ms = 200,
    .inv_timeout_ms = 100,
};

const vcu_cfg_t *vcu_cfg_default(void) { return &DEFAULT_CFG; }

void vcu_init(vcu_t *v, const vcu_cfg_t *cfg)
{
    v->cfg = cfg ? cfg : &DEFAULT_CFG;
    apps_init(&v->apps);
    fault_init(&v->faults);
    state_init(&v->state);
    torque_init(&v->torque);
    v->last_ms = 0;
    v->started = false;
    v->prev_ts_request = false;
}

/* Translate "what the world looks like" into "which conditions are true".
 *
 * Note that every condition is reported every tick, including the false
 * ones. That looks wasteful and it is how healing works: a fault the
 * manager stops hearing about is treated as absent, so silence has to mean
 * something definite. Saying "no, still fine" out loud is cheaper than
 * inventing a rule for what missing news means. */
static void report_faults(vcu_t *v, const vcu_in_t *in, uint16_t dt)
{
    fault_mgr_t *f = &v->faults;

    fault_report(f, FAULT_APPS_IMPLAUSIBLE, v->apps.implausible, dt);
    fault_report(f, FAULT_APPS_RANGE, !v->apps.ch1_ok || !v->apps.ch2_ok, dt);
    fault_report(f, FAULT_BPPC, v->apps.bppc_latched, dt);
    fault_report(f, FAULT_BRAKE_RANGE, false, dt);

    fault_report(f, FAULT_BMS_TIMEOUT, !in->bms.valid, dt);
    fault_report(f, FAULT_BMS_FAULT, in->bms.valid && in->bms.fault, dt);
    fault_report(f, FAULT_INVERTER_TIMEOUT, !in->inv.valid, dt);
    fault_report(f, FAULT_INVERTER_FAULT, in->inv.valid && in->inv.fault, dt);

    // Undervoltage is reported when the fade *starts*, not when it bottoms
    // out, so the severity in the table (DERATE, "fade the torque down")
    // describes what is actually happening at the moment it is logged.
    fault_report(f, FAULT_CELL_UNDERVOLT,
                 in->bms.valid && in->bms.cell_min_mv < v->cfg->cell_derate_mv, dt);
    fault_report(f, FAULT_OVERTEMP_MOTOR,
                 in->inv.valid && in->inv.temp_motor >= v->cfg->derate_motor_stop, dt);
    fault_report(f, FAULT_OVERTEMP_INVERTER,
                 in->inv.valid && in->inv.temp_inverter >= v->cfg->derate_inv_stop, dt);
    fault_report(f, FAULT_OVERSPEED,
                 in->inv.valid && in->inv.rpm >= v->cfg->rpm_max, dt);
    fault_report(f, FAULT_CAN_BUSOFF, in->can_busoff, dt);

    // The precharge faults are raised by the state machine instead, because
    // only it knows what the link was doing a moment ago. Report them clear
    // once we are out of precharge, so the latch holds while we are in it.
    if (v->state.state != VCU_PRECHARGE) {
        fault_report(f, FAULT_PRECHARGE_TIMEOUT, false, dt);
        fault_report(f, FAULT_PRECHARGE_NO_RISE, false, dt);
    }
}

void vcu_step(vcu_t *v, const vcu_in_t *in, vcu_out_t *out)
{
    /* How long has it been? Derived from the caller's clock rather than
     * assumed, because a scheduler that runs long must not get to quietly
     * shorten a 100 ms safety window.
     *
     * The clamp is for debuggers. Halt on a breakpoint for ten seconds and
     * dt comes back enormous, which would age every timer in the car at
     * once. First call has dt = 0. */
    uint16_t dt = 0;
    if (v->started) {
        uint32_t d = in->now_ms - v->last_ms;    // wraps correctly on uint32
        dt = (d > 1000u) ? 1000u : (uint16_t)d;
    }
    v->last_ms = in->now_ms;
    v->started = true;

    const vcu_out_t zero = { 0 };
    *out = zero;

    fault_begin(&v->faults);

    // Letting go of the TS request is how the driver says "I saw that".
    // It clears latched faults whose cause has actually gone away, so
    // recovering from a precharge failure means deliberately switching off
    // and on again — never just waiting long enough.
    if (v->prev_ts_request && !in->ts_request) fault_acknowledge(&v->faults);
    v->prev_ts_request = in->ts_request;

    // The order below is the design. Pedals before faults, because the
    // pedal module is what decides whether a pedal fault exists. Faults
    // before state, because a critical fault sends the state machine to
    // FAULT. State before torque, because torque is zero outside DRIVE.
    apps_step(&v->apps, v->cfg, in, dt);
    report_faults(v, in, dt);
    state_step(&v->state, v->cfg, in, &v->apps, &v->faults, dt);
    torque_step(&v->torque, v->cfg, in, &v->apps, &v->faults, v->state.state);

    state_outputs(&v->state, out);
    out->torque_cmd = v->torque.cmd;
    out->faults     = v->faults.active;
    out->pedal      = v->apps.pedal;

    // Belt and braces. The enable line is gated by the same conditions as
    // the torque number, so there are two independent paths to "no drive"
    // and a mistake in one of them is not enough to move the car.
    if (!apps_torque_permitted(&v->apps) ||
        fault_worst(&v->faults) >= FAULT_SEV_CRITICAL)
        out->inverter_enable = false;
}
