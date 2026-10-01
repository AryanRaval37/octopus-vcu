// faults.c - the fault table and the logic that runs it.
// "What does the car do if X?" should be answerable from this table alone.

#include "core/decide/faults.h"
#include "core/util.h"

// deb  = must be present this long before it sets (ms)
// heal = must be absent this long before it clears (ms), unless latching
//
// A 0 debounce usually means the timing already happened upstream: pedals.c
// runs the T11.8.8 100 ms window and signals.c runs the message timeouts.
// Debouncing those again here would quietly stretch the rule limits.
static const fault_def_t DEFS[FAULT_COUNT] = {
    //                           severity             deb  heal  latch  name

    // Two channels disagreeing is LIMP: T11.8.8 says motor power off is
    // enough and the TS can stay up, and the driver recovers by lifting.
    // A channel out of its valid window is an SCS failure, and T11.9.5 makes
    // the safe state "SDC and AIRs open", so CRITICAL.
    // This split is our reading of two rules that both use the word
    // "implausibility". Check it with a scrutineer before the event.
    [FAULT_APPS_IMPLAUSIBLE]  = { FAULT_SEV_LIMP,       0,    0, false, "APPS_IMPLAUSIBLE" },
    [FAULT_APPS_RANGE]        = { FAULT_SEV_CRITICAL,  20,  500, false, "APPS_RANGE" },
    [FAULT_BRAKE_IMPLAUSIBLE] = { FAULT_SEV_LIMP,       0,    0, false, "BRAKE_IMPLAUSIBLE" },
    [FAULT_BRAKE_RANGE]       = { FAULT_SEV_CRITICAL,  20,  500, false, "BRAKE_RANGE" },

    // pedals.c owns this latch and its release condition, so not latching
    // here as well. Two latches for one condition and the driver can't
    // tell which one is holding the car.
    [FAULT_BPPC]              = { FAULT_SEV_LIMP,       0,    0, false, "BPPC" },

    // Latching: a precharge that failed once should need a person to decide
    // to try again, not just a timer running out.
    [FAULT_PRECHARGE_TIMEOUT] = { FAULT_SEV_CRITICAL,   0,    0, true,  "PRECHARGE_TIMEOUT" },
    [FAULT_PRECHARGE_NO_RISE] = { FAULT_SEV_CRITICAL,   0,    0, true,  "PRECHARGE_NO_RISE" },

    // T11.9.2.d, three separate rows because they're three different
    // things to go and fix (wiring/power, sender firmware, bus noise).
    // CORRUPT is only a warning because a bad frame doesn't refresh the
    // message age, so sustained corruption becomes a TIMEOUT by itself.
    // The device's own fault flag latches: it told us something about
    // itself, and going quiet again doesn't take that back.
    [FAULT_BMS_TIMEOUT]       = { FAULT_SEV_CRITICAL,   0, 1000, false, "BMS_TIMEOUT" },
    [FAULT_BMS_STALE]         = { FAULT_SEV_CRITICAL,   0, 1000, false, "BMS_STALE" },
    [FAULT_BMS_CORRUPT]       = { FAULT_SEV_WARN,       0, 1000, false, "BMS_CORRUPT" },
    [FAULT_BMS_FAULT]         = { FAULT_SEV_CRITICAL,  50, 1000, true,  "BMS_FAULT" },
    [FAULT_INVERTER_TIMEOUT]  = { FAULT_SEV_CRITICAL,   0, 1000, false, "INVERTER_TIMEOUT" },
    [FAULT_INVERTER_STALE]    = { FAULT_SEV_CRITICAL,   0, 1000, false, "INVERTER_STALE" },
    [FAULT_INVERTER_CORRUPT]  = { FAULT_SEV_WARN,       0, 1000, false, "INVERTER_CORRUPT" },
    [FAULT_INVERTER_FAULT]    = { FAULT_SEV_CRITICAL,  50, 1000, true,  "INVERTER_FAULT" },

    // Temperatures and cell voltage move slowly, so anything faster than
    // these debounces is noise. The torque derate itself runs continuously
    // in torque.c; these rows are raised once the fade is under way and are
    // mostly there so it shows up in the log.
    [FAULT_CELL_UNDERVOLT]    = { FAULT_SEV_DERATE,   200, 2000, false, "CELL_UNDERVOLT" },
    [FAULT_OVERTEMP_MOTOR]    = { FAULT_SEV_DERATE,   500, 5000, false, "OVERTEMP_MOTOR" },
    [FAULT_OVERTEMP_INVERTER] = { FAULT_SEV_DERATE,   500, 5000, false, "OVERTEMP_INVERTER" },

    [FAULT_OVERSPEED]         = { FAULT_SEV_LIMP,      50,  500, false, "OVERSPEED" },

    // EV4.11.8 says "immediately", so no debounce.
    [FAULT_SDC_OPEN]          = { FAULT_SEV_CRITICAL,   0,  500, false, "SDC_OPEN" },
    [FAULT_CAN_BUSOFF]        = { FAULT_SEV_CRITICAL, 100, 1000, false, "CAN_BUSOFF" },

    // Never clears, because the config doesn't fix itself.
    [FAULT_BAD_CONFIG]        = { FAULT_SEV_CRITICAL,   0,    0, false, "BAD_CONFIG" },
};

void fault_init(fault_mgr_t *f)
{
    const fault_mgr_t zero = { 0 };
    *f = zero;

    // A fault added to the enum but not to the table would have severity
    // INFO and no name. Refuse to run rather than find that out on track.
    for (int i = 0; i < FAULT_COUNT; i++)
        if (!DEFS[i].name) f->active = ~(fault_mask_t)0;
}

void fault_begin(fault_mgr_t *f)
{
    f->just_set     = 0;
    f->just_cleared = 0;
}

// Two timers per fault. present_ms must fill before it sets, absent_ms
// before it clears, and each resets the other, so a condition sitting right
// on its threshold doesn't flicker the fault on and off.
void fault_report(fault_mgr_t *f, fault_id_t id, bool present, uint16_t dt_ms)
{
    const fault_def_t *d = &DEFS[id];
    const fault_mask_t bit = FAULT_BIT(id);

    if (present) {
        f->absent_ms[id]  = 0;
        f->present_ms[id] = sat_add(f->present_ms[id], dt_ms);
        if (!(f->active & bit) && f->present_ms[id] >= d->debounce_ms) {
            f->active   |= bit;
            f->just_set |= bit;
        }
    } else {
        f->present_ms[id] = 0;
        f->absent_ms[id]  = sat_add(f->absent_ms[id], dt_ms);
        if ((f->active & bit) && !d->latching && f->absent_ms[id] >= d->heal_ms) {
            f->active       &= ~bit;
            f->just_cleared |= bit;
        }
    }
}

void fault_acknowledge(fault_mgr_t *f)
{
    for (int i = 0; i < FAULT_COUNT; i++) {
        const fault_mask_t bit = FAULT_BIT(i);

        // present_ms == 0 means the cause is gone as of the last report.
        if ((f->active & bit) && DEFS[i].latching && f->present_ms[i] == 0) {
            f->active       &= ~bit;
            f->just_cleared |= bit;
        }
    }
}

const fault_def_t *fault_def(fault_id_t id) { return &DEFS[id]; }
const char *fault_name(fault_id_t id)       { return DEFS[id].name; }

fault_sev_t fault_worst_in(const fault_mgr_t *f, fault_mask_t mask)
{
    fault_sev_t worst = FAULT_SEV_INFO;
    for (int i = 0; i < FAULT_COUNT; i++)
        if ((f->active & mask & FAULT_BIT(i)) && DEFS[i].sev > worst) worst = DEFS[i].sev;
    return worst;
}

fault_sev_t fault_worst(const fault_mgr_t *f)
{
    return fault_worst_in(f, ~(fault_mask_t)0);
}
