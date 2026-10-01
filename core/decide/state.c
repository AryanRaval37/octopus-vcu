// state.c - vehicle state machine.
//
//   INIT -> LV_READY -> PRECHARGE -> TS_ACTIVE -> RTD_WAIT -> DRIVE
//
// Any CRITICAL fault sends us to FAULT from wherever we are. Getting out of
// FAULT needs the fault gone *and* the driver to let go of the TS request,
// so the car never comes back up on its own (EV4.11.4).

#include "core/decide/state.h"

// Time with everything open after a normal shutdown before we'll accept a
// new TS request. Lets the relays drop out and the discharge circuit start.
#define SHUTDOWN_HOLD_MS 500u

static const char *const NAMES[VCU_STATE_COUNT] = {
    [VCU_INIT]      = "INIT",
    [VCU_LV_READY]  = "LV_READY",
    [VCU_PRECHARGE] = "PRECHARGE",
    [VCU_TS_ACTIVE] = "TS_ACTIVE",
    [VCU_RTD_WAIT]  = "RTD_WAIT",
    [VCU_DRIVE]     = "DRIVE",
    [VCU_FAULT]     = "FAULT",
    [VCU_SHUTDOWN]  = "SHUTDOWN",
};

const char *vcu_state_name(vcu_state_t s)
{
    return (s < VCU_STATE_COUNT && NAMES[s]) ? NAMES[s] : "?";
}

void state_init(state_mgr_t *s)
{
    const state_mgr_t zero = { 0 };
    *s = zero;
    s->state   = VCU_INIT;
    s->prev    = VCU_INIT;
    s->entered = true;
}

static void go(state_mgr_t *s, vcu_state_t next)
{
    if (next == s->state) return;
    s->prev        = s->state;
    s->state       = next;
    s->entered     = true;
    s->in_state_ms = 0;
}

// DC link as a percentage of pack voltage. Needs both devices trusted, and
// the pack_dv floor is so a BMS reporting 0 V doesn't divide by zero.
static pct_x10_t dc_link_pct(const vcu_in_t *in, bool bms_ok, bool inv_ok)
{
    if (!inv_ok || !bms_ok || in->bms.pack_dv < 100) return 0;
    return (pct_x10_t)(((uint32_t)in->inv.dc_link_dv * PCT_MAX) / in->bms.pack_dv);
}

void state_step(state_mgr_t *s, const vcu_cfg_t *cfg, const vcu_in_t *in,
                const pedals_t *pedals, fault_mgr_t *faults,
                bool bms_ok, bool inv_ok, uint16_t dt_ms)
{
    const bool entering = s->entered;
    s->entered = false;
    s->in_state_ms += dt_ms;

    // Edge, not level, so a button that's held (or stuck) from earlier
    // can't count as the driver's "dedicated additional action".
    const bool rtd_pressed = in->rtd_button && !s->prev_rtd_button;
    s->prev_rtd_button = in->rtd_button;

    // One check here instead of one in every state. An open SDC comes in
    // through the fault table as well (FAULT_SDC_OPEN), which is how
    // EV4.11.8 gets handled.
    if (fault_worst(faults) == FAULT_SEV_CRITICAL &&
        s->state != VCU_FAULT && s->state != VCU_SHUTDOWN) {
        go(s, VCU_FAULT);
        return;
    }

    switch (s->state) {

    case VCU_INIT:
        // Don't go anywhere until both CAN partners have been heard from.
        if (bms_ok && inv_ok) go(s, VCU_LV_READY);
        break;

    case VCU_LV_READY:
        if (in->ts_request && in->shutdown_ok && fault_worst(faults) < FAULT_SEV_LIMP)
            go(s, VCU_PRECHARGE);
        break;

    case VCU_PRECHARGE: {
        // AIR- and the precharge relay are closed, the DC link charges
        // through the resistor, and AIR+ waits for 95 % (EV5.7.1).
        if (entering) {
            s->pc_start_dv     = inv_ok ? in->inv.dc_link_dv : 0;
            s->pc_rise_checked = false;
        }

        // An open precharge resistor means the link never moves. Checking
        // early gives a useful fault instead of a generic timeout 5 s later.
        if (!s->pc_rise_checked && s->in_state_ms >= cfg->precharge_min_rise_ms) {
            s->pc_rise_checked = true;
            const dv_t now = inv_ok ? in->inv.dc_link_dv : 0;
            if (now < s->pc_start_dv + cfg->precharge_min_rise_dv)
                fault_report(faults, FAULT_PRECHARGE_NO_RISE, true, dt_ms);
        }

        if (dc_link_pct(in, bms_ok, inv_ok) >= cfg->precharge_target_pct)
            go(s, VCU_TS_ACTIVE);
        else if (s->in_state_ms >= cfg->precharge_timeout_ms)
            fault_report(faults, FAULT_PRECHARGE_TIMEOUT, true, dt_ms);

        if (!in->ts_request) go(s, VCU_LV_READY);
        break;
    }

    case VCU_TS_ACTIVE:
        // Only lasts a tick. It's here so the log has a clean "AIRs closed"
        // row separate from waiting on the driver.
        go(s, in->ts_request ? VCU_RTD_WAIT : VCU_SHUTDOWN);
        break;

    case VCU_RTD_WAIT:
        if (!in->ts_request) { go(s, VCU_SHUTDOWN); break; }

        // EV4.11.7: the transition may only happen while the brakes are on,
        // together with a dedicated action (the button press). EV4.11.6
        // says R2D starts the moment the motors respond to the APPS, and in
        // DRIVE they respond straight away, so everything has to be true on
        // this one tick. The throttle has to be at rest as well, so the car
        // can't jump the instant it goes live, and nothing may be holding
        // torque off (a latched LIMP fault clearing later would otherwise be
        // the moment R2D really starts, brake or no brake).
        if (rtd_pressed && pedals->brake_confirmed &&
            pedals->apps.value < cfg->apps_reset_below &&
            fault_worst(faults) < FAULT_SEV_LIMP)
            go(s, VCU_DRIVE);
        break;

    case VCU_DRIVE:
        if (!in->ts_request) go(s, VCU_SHUTDOWN);
        break;

    case VCU_FAULT:
        // fault_acknowledge() won't clear a latched fault whose cause is
        // still there, so holding/toggling TS can't force a way out.
        if (fault_worst(faults) < FAULT_SEV_CRITICAL && !in->ts_request)
            go(s, VCU_LV_READY);
        break;

    case VCU_SHUTDOWN:
        if (s->in_state_ms >= SHUTDOWN_HOLD_MS) go(s, VCU_LV_READY);
        break;

    default:
        go(s, VCU_FAULT);
        break;
    }
}

void state_outputs(const state_mgr_t *s, const vcu_cfg_t *cfg, vcu_out_t *out)
{
    out->state = s->state;

    // The only place relay states are decided. AIR+ never closes before
    // precharge has finished.
    switch (s->state) {
    case VCU_PRECHARGE:
        out->air_neg = true;  out->air_pos = false; out->precharge_relay = true;  break;
    case VCU_TS_ACTIVE:
    case VCU_RTD_WAIT:
    case VCU_DRIVE:
        out->air_neg = true;  out->air_pos = true;  out->precharge_relay = false; break;
    default:
        out->air_neg = false; out->air_pos = false; out->precharge_relay = false; break;
    }

    // EV4.12.1: 1-3 s of sound while entering R2D, so it plays over the
    // first rtd_buzzer_ms of DRIVE. If we leave DRIVE early (SDC opened, TS
    // off) it stops with it, which is fine: the car isn't in R2D any more.
    out->rtd_buzzer      = s->state == VCU_DRIVE && s->in_state_ms < cfg->rtd_buzzer_ms;
    out->inverter_enable = s->state == VCU_DRIVE;
}
