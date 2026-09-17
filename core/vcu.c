/* vcu.c — one tick of the whole car.
 *
 * Everything else in core/ is a specialist. This file knows what order the
 * specialists talk in, and that order is the design:
 *
 *   how long has it been?
 *     -> did the driver acknowledge anything?
 *     -> SENSE:  is the bus trustworthy, what are the pedals saying
 *     -> report what is wrong
 *     -> DECIDE: what state are we in, how much torque
 *     -> write it down
 *
 * Nothing in here reads a clock or touches a pin. Everything arrives in `in`
 * and leaves in `out`, which is what lets a test replay a whole drive on a
 * laptop with no hardware attached.
 */
#include "core/vcu.h"

void vcu_init(vcu_t *v, const vcu_cfg_t *cfg)
{
    v->cfg = cfg ? cfg : vcu_cfg_default();
    pedals_init(&v->pedals);
    sig_init(&v->bms_health);
    sig_init(&v->inv_health);
    fault_init(&v->faults);
    state_init(&v->state);
    torque_init(&v->torque);
    datalog_init(&v->log);
    v->last_ms = 0;
    v->started = false;
    v->prev_ts_request = false;
    v->log_period_ms = 100;          // 10 Hz heartbeat; events are extra
}

/* Translate "what the world looks like" into "which conditions are true".
 *
 * Every condition is reported every tick, including the false ones. That
 * looks wasteful and it is how healing works: a fault the manager stops
 * hearing about is treated as absent, so silence has to mean something
 * definite. Saying "no, still fine" out loud is cheaper than inventing a
 * rule for what missing news means. */
static void report_faults(vcu_t *v, const vcu_in_t *in, uint16_t dt)
{
    fault_mgr_t *f = &v->faults;
    const bool bms_ok = sig_trustworthy(&v->bms_health);
    const bool inv_ok = sig_trustworthy(&v->inv_health);

    // Pedals. Note the split: "the two channels disagree" and "a channel is
    // electrically broken" are different rows with different severities.
    // See the comment on those rows in decide/faults.c.
    fault_report(f, FAULT_APPS_IMPLAUSIBLE,  v->pedals.apps.implausible, dt);
    fault_report(f, FAULT_APPS_RANGE,        pair_range_bad(&v->pedals.apps), dt);
    fault_report(f, FAULT_BRAKE_IMPLAUSIBLE, v->pedals.brake.implausible, dt);
    fault_report(f, FAULT_BRAKE_RANGE,       pair_range_bad(&v->pedals.brake), dt);
    fault_report(f, FAULT_BPPC,              v->pedals.bppc_latched, dt);

    // The three ways a talker can let you down, per T11.9.2.d.
    fault_report(f, FAULT_BMS_TIMEOUT,      v->bms_health.timeout, dt);
    fault_report(f, FAULT_BMS_STALE,        v->bms_health.stale,   dt);
    fault_report(f, FAULT_BMS_CORRUPT,      v->bms_health.corrupt, dt);
    fault_report(f, FAULT_INVERTER_TIMEOUT, v->inv_health.timeout, dt);
    fault_report(f, FAULT_INVERTER_STALE,   v->inv_health.stale,   dt);
    fault_report(f, FAULT_INVERTER_CORRUPT, v->inv_health.corrupt, dt);

    // Only believe a device's own fault flag while you believe the device.
    fault_report(f, FAULT_BMS_FAULT,      bms_ok && in->bms.fault, dt);
    fault_report(f, FAULT_INVERTER_FAULT, inv_ok && in->inv.fault, dt);

    // Undervoltage is reported when the fade starts, not when it bottoms
    // out, so the DERATE severity in the table describes what is actually
    // happening at the moment it is logged.
    fault_report(f, FAULT_CELL_UNDERVOLT,
                 bms_ok && in->bms.cell_min_mv < v->cfg->cell_derate_mv, dt);
    fault_report(f, FAULT_OVERTEMP_MOTOR,
                 inv_ok && in->inv.temp_motor >= v->cfg->derate_motor_stop, dt);
    fault_report(f, FAULT_OVERTEMP_INVERTER,
                 inv_ok && in->inv.temp_inverter >= v->cfg->derate_inv_stop, dt);
    fault_report(f, FAULT_OVERSPEED,
                 inv_ok && in->inv.rpm >= v->cfg->rpm_max, dt);

    // EV4.11.8: R2D must be left immediately when the shutdown circuit
    // opens. Routed through the fault table so it uses the same trapdoor
    // into FAULT as everything else, rather than getting its own path that
    // some future state could forget to check.
    fault_report(f, FAULT_SDC_OPEN, !in->shutdown_ok, dt);

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

    // Letting go of the TS request is how the driver says "I saw that". It
    // clears latched faults whose cause has actually gone away, so
    // recovering from a precharge failure means deliberately switching off
    // and on again — never just waiting long enough.
    if (v->prev_ts_request && !in->ts_request) fault_acknowledge(&v->faults);
    v->prev_ts_request = in->ts_request;

    /* --- sense ---------------------------------------------------------
     * Work out what can be believed before working out what to do about it.
     * Message health comes first because everything downstream, including
     * whether a BMS fault flag means anything, depends on whether that BMS
     * is still talking. */
    sig_step(&v->bms_health, in->bms.rx, in->bms.counter, in->bms.crc_ok,
             v->cfg->bms_timeout_ms, v->cfg->counter_stall_ms, dt);
    sig_step(&v->inv_health, in->inv.rx, in->inv.counter, in->inv.crc_ok,
             v->cfg->inv_timeout_ms, v->cfg->counter_stall_ms, dt);

    const bool bms_ok = sig_trustworthy(&v->bms_health);
    const bool inv_ok = sig_trustworthy(&v->inv_health);

    pedals_step(&v->pedals, v->cfg, in, dt);

    /* --- decide --------------------------------------------------------
     * Faults before state, because a critical fault sends the state machine
     * to FAULT. State before torque, because torque is zero outside DRIVE. */
    report_faults(v, in, dt);
    state_step(&v->state, v->cfg, in, &v->pedals, &v->faults, bms_ok, inv_ok, dt);
    torque_step(&v->torque, v->cfg, in, &v->pedals, &v->faults,
                v->state.state, bms_ok, inv_ok);

    /* --- act ----------------------------------------------------------- */
    state_outputs(&v->state, out);
    out->torque_cmd = v->torque.cmd;
    out->faults     = v->faults.active;
    out->pedal      = v->pedals.apps.value;
    out->brake      = v->pedals.brake.value;

    // Belt and braces. The enable line is gated by the same conditions as
    // the torque number, so there are two independent paths to "no drive"
    // and a mistake in one of them is not enough to move the car.
    if (!pedals_torque_permitted(&v->pedals) ||
        fault_worst(&v->faults) >= FAULT_SEV_LIMP)
        out->inverter_enable = false;

    /* --- record --------------------------------------------------------
     * Last, so the log shows what was actually commanded rather than what
     * was briefly considered. */
    datalog_step(&v->log, in, out, &v->torque, bms_ok, inv_ok,
                 v->log_period_ms, dt);
}
