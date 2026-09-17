/* state.c — what the car is doing, and what it will let you do next.
 *
 * The states are the car's life story and they mostly run one way:
 *
 *   INIT -> LV_READY -> PRECHARGE -> TS_ACTIVE -> RTD_WAIT -> DRIVE
 *
 * with a trapdoor to FAULT underneath all of them. Two are worth slowing
 * down for. PRECHARGE, because getting it wrong welds a contactor shut.
 * RTD_WAIT, because that is the gate between "the car is on" and "the car
 * can move", and an electric car is quiet enough that the gate has to
 * include making a noise.
 *
 * Next: core/decide/torque.c.
 */
#include "core/decide/state.h"

static const char *NAMES[VCU_STATE_COUNT] = {
    "INIT", "LV_READY", "PRECHARGE", "TS_ACTIVE", "RTD_WAIT",
    "DRIVE", "FAULT", "SHUTDOWN"
};

const char *vcu_state_name(vcu_state_t s)
{
    return (s < VCU_STATE_COUNT) ? NAMES[s] : "?";
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

// How full the DC link is, as a percentage of the pack. The pack_dv guard is
// there because a dead BMS reports zero volts, and dividing by that ends the
// tick rather abruptly.
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

    /* The trapdoor, checked before any per-state logic. A critical fault
     * means FAULT from wherever you happen to be standing.
     *
     * Writing it once here instead of as a branch inside all eight states is
     * the difference between a state machine you can hold in your head and
     * one you can only grep. Note that an open shutdown circuit reaches this
     * through the fault table rather than as a special case — EV4.11.8 wants
     * R2D left immediately, and the fastest way to be sure of that is to
     * give it no separate path to get wrong. */
    if (fault_worst(faults) == FAULT_SEV_CRITICAL &&
        s->state != VCU_FAULT && s->state != VCU_SHUTDOWN) {
        go(s, VCU_FAULT);
        return;
    }

    switch (s->state) {

    case VCU_INIT:
        // Self-tests would go here. Both CAN partners have to be alive
        // before we are willing to discuss closing a contactor.
        if (bms_ok && inv_ok) go(s, VCU_LV_READY);
        break;

    case VCU_LV_READY:
        if (in->ts_request && in->shutdown_ok && fault_worst(faults) < FAULT_SEV_LIMP)
            go(s, VCU_PRECHARGE);
        break;

    case VCU_PRECHARGE: {
        /* You cannot just close a contactor onto 400 V. The inverter's
         * DC-link capacitors look like a dead short and the contactor welds
         * itself shut. So AIR- closes, a relay feeds the link through a
         * resistor, and we sit here watching the voltage climb until it
         * reaches the 95 % that EV5.7.1 requires. */
        if (entering) {
            s->pc_start_dv     = inv_ok ? in->inv.dc_link_dv : 0;
            s->pc_rise_checked = false;
        }

        // The failure this catches is an open precharge resistor. The link
        // voltage simply never moves. Without the check you sit here for the
        // full five seconds and the timeout tells you nothing about why.
        if (!s->pc_rise_checked && s->in_state_ms >= cfg->precharge_min_rise_ms) {
            s->pc_rise_checked = true;
            const dv_t now = inv_ok ? in->inv.dc_link_dv : 0;
            if (now < s->pc_start_dv + cfg->precharge_min_rise_dv)
                fault_report(faults, FAULT_PRECHARGE_NO_RISE, true, dt_ms);
        }

        if (dc_link_pct(in, bms_ok, inv_ok) >= cfg->precharge_target_pct) {
            go(s, VCU_TS_ACTIVE);
        } else if (s->in_state_ms >= cfg->precharge_timeout_ms) {
            fault_report(faults, FAULT_PRECHARGE_TIMEOUT, true, dt_ms);
        }

        if (!in->ts_request) go(s, VCU_LV_READY);
        break;
    }

    case VCU_TS_ACTIVE:
        // A single tick of "HV is up". It exists so the log shows the moment
        // the AIRs closed, separately from waiting on the driver.
        if (!in->ts_request) go(s, VCU_SHUTDOWN);
        else                 go(s, VCU_RTD_WAIT);
        break;

    case VCU_RTD_WAIT:
        if (!in->ts_request) { go(s, VCU_SHUTDOWN); break; }

        /* EV4.11.7: the move into R2D must happen "during the actuation of
         * the mechanical brakes and a simultaneous dedicated additional
         * action" — brake held, button pressed. EV4.12.1 then wants a sound
         * for at least 1 s and at most 3 s.
         *
         * Both conditions are re-checked every tick, so letting go of the
         * brake mid-buzz restarts the timer. It reads like box-ticking and
         * it isn't: the car is silent, and a silent car that can lurch
         * without warning in a pit lane full of people is the thing this
         * prevents. */
        if (pedals->brake_applied && in->rtd_button) {
            s->buzzer_ms += dt_ms;
            if (s->buzzer_ms >= cfg->rtd_buzzer_ms) { s->buzzer_ms = 0; go(s, VCU_DRIVE); }
        } else {
            s->buzzer_ms = 0;
        }
        break;

    case VCU_DRIVE:
        if (!in->ts_request) go(s, VCU_SHUTDOWN);
        break;

    case VCU_FAULT:
        // Two conditions to leave, and both matter. Nothing critical still
        // active, and the driver has let go of the TS request.
        // fault_acknowledge() refuses to clear a fault whose cause is still
        // there, so this cannot be button-mashed into submission.
        if (fault_worst(faults) < FAULT_SEV_CRITICAL && !in->ts_request)
            go(s, VCU_LV_READY);
        break;

    case VCU_SHUTDOWN:
        if (s->in_state_ms > 500) go(s, VCU_LV_READY);
        break;

    default:
        go(s, VCU_FAULT);
        break;
    }
}

void state_outputs(const state_mgr_t *s, vcu_out_t *out)
{
    out->state = s->state;

    // AIR+ closes only after precharge is done. Closing it early is the
    // welded-contactor, dead-resistor failure the whole sequence exists to
    // avoid, so the ordering lives here in one switch and nowhere else.
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

    out->rtd_buzzer      = (s->state == VCU_RTD_WAIT) && (s->buzzer_ms > 0);
    out->shutdown_assert = (s->state == VCU_FAULT);
    out->inverter_enable = (s->state == VCU_DRIVE);
}
