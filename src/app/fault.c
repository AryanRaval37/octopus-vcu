/* fault.c — the entire fault policy of the car, on one screen.
 *
 * Scroll down. That table is the answer to every "what happens if..."
 * question anyone will ask about this vehicle, and it fits in a screenshot.
 * That is the reason the fault manager exists at all: the alternative is
 * the same policy spread across fifteen files as scattered `if`s, where
 * nobody can tell you what the car does without reading all of them.
 *
 * Next: tests/scenarios/ — start with 02_apps_disagreement.scn.
 */
#include "vcu/fault.h"

/* Picking debounce numbers is a trade. Long enough that noise and one
 * dropped CAN frame do not trip it, short enough that the response still
 * happens while it matters. The APPS rows are 0/0 because apps.c already
 * runs the 100 ms timer the rule demands, and debouncing it twice would
 * quietly turn 100 ms into 120. */
static const fault_def_t DEFS[FAULT_COUNT] = {
/*   id                        severity            deb  heal  latch  name */

    /* Not latching — even though it absolutely is a latched condition.
     *
     * APPS_IMPLAUSIBLE used to be latched here AND in apps.c, so the driver
     * could never recover by releasing the pedal — which is the exact
     * recovery T.4 describes. Two owners of one latch means neither is in
     * charge. apps.c owns it, because apps.c is where the rule's clearing
     * condition lives. A scenario test found it: 02_apps_disagreement.scn. */
    {FAULT_APPS_IMPLAUSIBLE,   FAULT_SEV_CRITICAL,   0,    0,  false, "APPS_IMPLAUSIBLE"},

    {FAULT_APPS_RANGE,         FAULT_SEV_CRITICAL,  20,  500,  false, "APPS_RANGE"},
    {FAULT_BPPC,               FAULT_SEV_LIMP,       0,    0,  false, "BPPC"},
    {FAULT_BRAKE_RANGE,        FAULT_SEV_WARN,      50,  500,  false, "BRAKE_RANGE"},

    // Precharge latches. A resistor that failed once will fail again, and
    // the recovery is a human deciding to try, not a timer expiring.
    {FAULT_PRECHARGE_TIMEOUT,  FAULT_SEV_CRITICAL,   0,    0,  true,  "PRECHARGE_TIMEOUT"},
    {FAULT_PRECHARGE_NO_RISE,  FAULT_SEV_CRITICAL,   0,    0,  true,  "PRECHARGE_NO_RISE"},

    // Timeouts do not latch: the bus coming back is real evidence. A device
    // reporting its own fault does latch, because it told us something
    // about itself that going quiet again does not undo.
    {FAULT_BMS_TIMEOUT,        FAULT_SEV_CRITICAL, 100, 1000,  false, "BMS_TIMEOUT"},
    {FAULT_BMS_FAULT,          FAULT_SEV_CRITICAL,  50, 1000,  true,  "BMS_FAULT"},
    {FAULT_INVERTER_TIMEOUT,   FAULT_SEV_CRITICAL, 100, 1000,  false, "INVERTER_TIMEOUT"},
    {FAULT_INVERTER_FAULT,     FAULT_SEV_CRITICAL,  50, 1000,  true,  "INVERTER_FAULT"},

    // The slow ones. Long debounce because temperature and pack voltage do
    // not change in 50 ms, so anything that fast is measurement noise.
    {FAULT_CELL_UNDERVOLT,     FAULT_SEV_DERATE,   200, 2000,  false, "CELL_UNDERVOLT"},
    {FAULT_OVERTEMP_MOTOR,     FAULT_SEV_DERATE,   500, 5000,  false, "OVERTEMP_MOTOR"},
    {FAULT_OVERTEMP_INVERTER,  FAULT_SEV_DERATE,   500, 5000,  false, "OVERTEMP_INVERTER"},

    {FAULT_OVERSPEED,          FAULT_SEV_LIMP,      50,  500,  false, "OVERSPEED"},
    {FAULT_CAN_BUSOFF,         FAULT_SEV_CRITICAL, 100, 1000,  false, "CAN_BUSOFF"},
};

// A fault's ID is its index here, so the table and the enum have to agree.
// They stopped agreeing once. This runs at init, costs nothing, and has
// paid for itself several times over.
static bool table_ok(void)
{
    for (int i = 0; i < FAULT_COUNT; i++)
        if (DEFS[i].id != (fault_id_t)i) return false;
    return true;
}

void fault_init(fault_mgr_t *f)
{
    const fault_mgr_t zero = { 0 };
    *f = zero;

    // A shuffled table applies the wrong severity to every fault in the
    // car, silently. Better to set all of them and refuse to move.
    if (!table_ok()) f->active = ~(fault_mask_t)0;
}

void fault_begin(fault_mgr_t *f)
{
    f->just_set     = 0;
    f->just_cleared = 0;
}

static uint16_t sat_add(uint16_t v, uint16_t d)
{
    uint32_t t = (uint32_t)v + d;
    return (t > 0xFFFFu) ? 0xFFFFu : (uint16_t)t;
}

/* Two timers per fault, not one. `present_ms` has to fill up before it
 * sets, `absent_ms` before it clears, and each resets the other. That gap
 * is deliberate hysteresis: a condition hovering right on its threshold
 * flickers, and a fault that flickers is worse than useless — it fills the
 * log and teaches everyone to ignore it. */
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

        // present_ms == 0 means the cause is gone as of this tick. Without
        // that check, acknowledging a still-broken car starts it.
        if ((f->active & bit) && DEFS[i].latching && f->present_ms[i] == 0) {
            f->active       &= ~bit;
            f->just_cleared |= bit;
        }
    }
}

const fault_def_t *fault_def(fault_id_t id) { return &DEFS[id]; }
const char *fault_name(fault_id_t id)       { return DEFS[id].name; }

fault_sev_t fault_worst(const fault_mgr_t *f)
{
    fault_sev_t worst = FAULT_SEV_INFO;
    for (int i = 0; i < FAULT_COUNT; i++)
        if ((f->active & FAULT_BIT(i)) && DEFS[i].sev > worst) worst = DEFS[i].sev;
    return worst;
}
