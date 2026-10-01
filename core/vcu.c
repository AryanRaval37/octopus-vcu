// vcu.c - one control tick.
//
// Order within a tick: work out dt, handle the driver's acknowledge, check
// message health and pedals (sense), report faults, run the state machine
// and torque (decide), set outputs, log. Faults go before the state machine
// so a critical one takes effect the same tick; state goes before torque
// because torque is zero outside DRIVE.

#include "core/vcu.h"
#include "core/util.h"

void vcu_init(vcu_t *v, const vcu_cfg_t *cfg)
{
    v->cfg    = cfg ? cfg : vcu_cfg_default();
    v->cfg_ok = vcu_cfg_valid(v->cfg);

    pedals_init(&v->pedals);
    sig_init(&v->bms_health);
    sig_init(&v->inv_health);
    fault_init(&v->faults);
    state_init(&v->state);
    torque_init(&v->torque);
    datalog_init(&v->log);

    v->log_period_ms   = 100;   // 10 Hz, plus a row on every state/fault change
    v->last_ms         = 0;
    v->boot_ms         = 0;
    v->started         = false;
    v->prev_ts_request = false;
}

// Every fault is reported every tick, including the ones that are fine.
// Healing only happens on "not present" reports, so not reporting a fault
// would leave it stuck however it was.
static void report_faults(vcu_t *v, const vcu_in_t *in, uint16_t dt)
{
    fault_mgr_t     *f   = &v->faults;
    const vcu_cfg_t *cfg = v->cfg;
    const bool bms_ok = sig_trustworthy(&v->bms_health);
    const bool inv_ok = sig_trustworthy(&v->inv_health);

    fault_report(f, FAULT_BAD_CONFIG, !v->cfg_ok, dt);

    fault_report(f, FAULT_APPS_IMPLAUSIBLE,  v->pedals.apps.implausible,       dt);
    fault_report(f, FAULT_APPS_RANGE,        pair_range_bad(&v->pedals.apps),  dt);
    fault_report(f, FAULT_BRAKE_IMPLAUSIBLE, v->pedals.brake.implausible,      dt);
    fault_report(f, FAULT_BRAKE_RANGE,       pair_range_bad(&v->pedals.brake), dt);
    fault_report(f, FAULT_BPPC,              v->pedals.bppc_latched,           dt);

    // A device we've never heard from is still booting, not timed out,
    // until the grace period runs out. Without this the car powers up
    // straight into FAULT, because the VCU wakes up faster than the BMS.
    const bool booting     = v->boot_ms < cfg->boot_grace_ms;
    const bool bms_timeout = v->bms_health.timeout && (v->bms_health.seen || !booting);
    const bool inv_timeout = v->inv_health.timeout && (v->inv_health.seen || !booting);

    fault_report(f, FAULT_BMS_TIMEOUT,      bms_timeout,           dt);
    fault_report(f, FAULT_BMS_STALE,        v->bms_health.stale,   dt);
    fault_report(f, FAULT_BMS_CORRUPT,      v->bms_health.corrupt, dt);
    fault_report(f, FAULT_INVERTER_TIMEOUT, inv_timeout,           dt);
    fault_report(f, FAULT_INVERTER_STALE,   v->inv_health.stale,   dt);
    fault_report(f, FAULT_INVERTER_CORRUPT, v->inv_health.corrupt, dt);

    // A device's own fault flag only means something while we trust the
    // device.
    fault_report(f, FAULT_BMS_FAULT,      bms_ok && in->bms.fault, dt);
    fault_report(f, FAULT_INVERTER_FAULT, inv_ok && in->inv.fault, dt);

    // The derate faults go up as soon as the fade starts taking torque away,
    // so the log shows why the car got slower. The fade itself is in
    // torque.c.
    fault_report(f, FAULT_CELL_UNDERVOLT,
                 bms_ok && in->bms.cell_min_mv < cfg->cell_derate_mv, dt);
    fault_report(f, FAULT_OVERTEMP_MOTOR,
                 inv_ok && in->inv.temp_motor > cfg->derate_motor_start, dt);
    fault_report(f, FAULT_OVERTEMP_INVERTER,
                 inv_ok && in->inv.temp_inverter > cfg->derate_inv_start, dt);
    fault_report(f, FAULT_OVERSPEED,
                 inv_ok && in->inv.rpm >= cfg->rpm_max, dt);

    // EV4.11.8. Going through the fault table means the state machine's
    // one CRITICAL check covers it, instead of a special case that some
    // state might forget.
    fault_report(f, FAULT_SDC_OPEN,   !in->shutdown_ok, dt);
    fault_report(f, FAULT_CAN_BUSOFF, in->can_busoff,   dt);

    // The precharge faults are raised by the state machine, since only it
    // knows what the link was doing. Clear them from here once we're out of
    // precharge (they latch, so this just lets acknowledge work).
    if (v->state.state != VCU_PRECHARGE) {
        fault_report(f, FAULT_PRECHARGE_TIMEOUT, false, dt);
        fault_report(f, FAULT_PRECHARGE_NO_RISE, false, dt);
    }
}

void vcu_step(vcu_t *v, const vcu_in_t *in, vcu_out_t *out)
{
    // dt comes from the caller's clock, so a late tick counts as the time
    // it really took and can't shorten a safety window. Clamped so it fits
    // in 16 bits; after a long stall (or a breakpoint) every timer simply
    // reads as expired.
    uint16_t dt = 0;
    if (v->started) {
        const uint32_t d = in->now_ms - v->last_ms;   // unsigned, wraps correctly
        dt = (d > 1000u) ? 1000u : (uint16_t)d;
    }
    v->last_ms = in->now_ms;
    v->started = true;

    if (v->boot_ms < v->cfg->boot_grace_ms) v->boot_ms = sat_add(v->boot_ms, dt);

    const vcu_out_t zero = { 0 };
    *out = zero;

    fault_begin(&v->faults);

    // Letting go of the TS request is the driver's "acknowledge". It clears
    // latched faults whose cause has gone, so recovering from e.g. a failed
    // precharge is always a deliberate off-and-on, never just waiting.
    if (v->prev_ts_request && !in->ts_request) fault_acknowledge(&v->faults);
    v->prev_ts_request = in->ts_request;

    // Sense. Message health first, since whether a BMS fault flag (or any
    // BMS number) means anything depends on whether the BMS is still
    // talking.
    sig_step(&v->bms_health, in->bms.rx, in->bms.counter, in->bms.crc_ok,
             v->cfg->bms_timeout_ms, v->cfg->counter_stall_ms, dt);
    sig_step(&v->inv_health, in->inv.rx, in->inv.counter, in->inv.crc_ok,
             v->cfg->inv_timeout_ms, v->cfg->counter_stall_ms, dt);

    const bool bms_ok = sig_trustworthy(&v->bms_health);
    const bool inv_ok = sig_trustworthy(&v->inv_health);

    pedals_step(&v->pedals, v->cfg, in, dt);

    // Decide.
    report_faults(v, in, dt);
    state_step(&v->state, v->cfg, in, &v->pedals, &v->faults, bms_ok, inv_ok, dt);
    torque_step(&v->torque, v->cfg, in, &v->pedals, &v->faults,
                v->state.state, bms_ok, inv_ok, dt);

    // Act.
    state_outputs(&v->state, v->cfg, out);
    out->torque_cmd = v->torque.cmd;
    out->faults     = v->faults.active;
    out->pedal      = v->pedals.apps.value;
    out->brake      = v->pedals.brake.value;

    // T6.3.1: brake light on if and only if the hydraulic brakes are on or
    // we're regenerating.
    out->brake_light = v->pedals.brake_applied || out->torque_cmd < 0;

    // Open our switch in the shutdown loop for our own critical faults
    // (T11.9.5 wants SDC and AIRs open). An open loop is not by itself a
    // reason to hold it open: if the sense point is after our switch we'd
    // read our own open loop as SDC_OPEN and never get out of FAULT.
    const fault_mask_t own = ~FAULT_BIT(FAULT_SDC_OPEN);
    out->shutdown_assert = fault_worst_in(&v->faults, own) == FAULT_SEV_CRITICAL;

    // The enable line gets the same gates as the torque number, so there
    // are two separate paths to "no drive" and one bug isn't enough to move
    // the car.
    if (!pedals_torque_permitted(&v->pedals) ||
        fault_worst(&v->faults) >= FAULT_SEV_LIMP)
        out->inverter_enable = false;

    // Log last, so it records what was actually commanded.
    datalog_step(&v->log, in, out, &v->torque, bms_ok, inv_ok,
                 v->log_period_ms, dt);
}
