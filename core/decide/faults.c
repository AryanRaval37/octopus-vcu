/* faults.c — the entire fault policy of the car, on one screen.
 *
 * Scroll down. That table is the answer to every "what happens if..."
 * question anyone will ask about this vehicle, and it fits in a screenshot.
 * That is the reason the fault manager exists at all: the alternative is the
 * same policy spread across fifteen files as scattered `if`s, where nobody
 * can tell you what the car does without reading all of them.
 *
 * Next: core/decide/state.c.
 */
#include "core/decide/faults.h"

/* Picking debounce numbers is a trade. Long enough that noise and one
 * dropped frame do not trip it, short enough that the response still happens
 * while it matters. The pedal rows are 0/0 because pedals.c already runs the
 * timers the rules demand, and debouncing them twice would quietly turn
 * 100 ms into 120. */
static const fault_def_t DEFS[FAULT_COUNT] = {
/*   id                        severity            deb  heal  latch  name */

    /* The most consequential two rows in the file, and the reason they
     * differ is a distinction the rules make and it is easy to miss.
     *
     * T11.8.8, on the channels disagreeing: "The power to the motor(s) must
     * be immediately shut down completely. It is NOT necessary to completely
     * deactivate the tractive system." That is LIMP — torque to zero, HV
     * stays up, and the driver can recover by releasing the pedal, which is
     * the recovery T11.8.9 describes.
     *
     * T11.9.5, on a channel being open or shorted: the safe state for a
     * failed System Critical Signal is "opened shutdown circuit and opened
     * AIRs". That is CRITICAL, and no amount of pedal-releasing fixes it,
     * because the wire is still broken.
     *
     * Same sensor, two failures, two different cars afterwards. Note this
     * reading is an interpretation of two rules that both use the word
     * "implausibility" — worth confirming with a scrutineer. Being stricter
     * than required is safe; being looser is not. */
    {FAULT_APPS_IMPLAUSIBLE,   FAULT_SEV_LIMP,       0,    0,  false, "APPS_IMPLAUSIBLE"},
    {FAULT_APPS_RANGE,         FAULT_SEV_CRITICAL,  20,  500,  false, "APPS_RANGE"},

    // The brake pair gets the same treatment for the same reasons: T6.1.13
    // lets brake travel command regen, which makes these signals SCSs too.
    {FAULT_BRAKE_IMPLAUSIBLE,  FAULT_SEV_LIMP,       0,    0,  false, "BRAKE_IMPLAUSIBLE"},
    {FAULT_BRAKE_RANGE,        FAULT_SEV_CRITICAL,  20,  500,  false, "BRAKE_RANGE"},

    // Not latching here: pedals.c owns this latch and its own release
    // condition. Two owners of one latch means neither is in charge.
    {FAULT_BPPC,               FAULT_SEV_LIMP,       0,    0,  false, "BPPC"},

    // Pre-charge latches. A resistor that failed once will fail again, and
    // the recovery is a human deciding to try, not a timer expiring.
    {FAULT_PRECHARGE_TIMEOUT,  FAULT_SEV_CRITICAL,   0,    0,  true,  "PRECHARGE_TIMEOUT"},
    {FAULT_PRECHARGE_NO_RISE,  FAULT_SEV_CRITICAL,   0,    0,  true,  "PRECHARGE_NO_RISE"},

    /* The three ways a CAN talker can let you down, per T11.9.2.d, kept as
     * three rows because they are three different repairs. Silence is a
     * wiring or power problem; a frozen counter is a gateway or a firmware
     * bug; a bad checksum is noise on the bus.
     *
     * Debounce is 0 on all of them because sense/signals.c already runs the
     * timing -- the message timeout IS the debounce, and counting it twice
     * would quietly turn a 200 ms timeout into 300 ms.
     *
     * CORRUPT is only a warning, which looks wrong until you notice that a
     * corrupt frame does not refresh the age. Sustained corruption trips the
     * timeout by itself; this row exists so the log can tell you WHY it
     * timed out. Occasional corruption on a noisy bus is worth seeing and
     * not worth stopping for.
     *
     * None of them latch -- a bus that comes back is real evidence. A device
     * reporting its OWN fault does latch, because it told you something
     * about itself that going quiet again does not un-tell you. */
    {FAULT_BMS_TIMEOUT,        FAULT_SEV_CRITICAL,   0, 1000,  false, "BMS_TIMEOUT"},
    {FAULT_BMS_STALE,          FAULT_SEV_CRITICAL,   0, 1000,  false, "BMS_STALE"},
    {FAULT_BMS_CORRUPT,        FAULT_SEV_WARN,       0, 1000,  false, "BMS_CORRUPT"},
    {FAULT_BMS_FAULT,          FAULT_SEV_CRITICAL,  50, 1000,  true,  "BMS_FAULT"},

    {FAULT_INVERTER_TIMEOUT,   FAULT_SEV_CRITICAL,   0, 1000,  false, "INVERTER_TIMEOUT"},
    {FAULT_INVERTER_STALE,     FAULT_SEV_CRITICAL,   0, 1000,  false, "INVERTER_STALE"},
    {FAULT_INVERTER_CORRUPT,   FAULT_SEV_WARN,       0, 1000,  false, "INVERTER_CORRUPT"},
    {FAULT_INVERTER_FAULT,     FAULT_SEV_CRITICAL,  50, 1000,  true,  "INVERTER_FAULT"},

    // The slow ones. Long debounce because temperature and pack voltage do
    // not change in 50 ms, so anything that fast is measurement noise.
    {FAULT_CELL_UNDERVOLT,     FAULT_SEV_DERATE,   200, 2000,  false, "CELL_UNDERVOLT"},
    {FAULT_OVERTEMP_MOTOR,     FAULT_SEV_DERATE,   500, 5000,  false, "OVERTEMP_MOTOR"},
    {FAULT_OVERTEMP_INVERTER,  FAULT_SEV_DERATE,   500, 5000,  false, "OVERTEMP_INVERTER"},

    {FAULT_OVERSPEED,          FAULT_SEV_LIMP,      50,  500,  false, "OVERSPEED"},

    // EV4.11.8: "The R2D mode must be left immediately when the SDC is
    // opened." Immediately means no debounce.
    {FAULT_SDC_OPEN,           FAULT_SEV_CRITICAL,   0,  500,  false, "SDC_OPEN"},

    {FAULT_CAN_BUSOFF,         FAULT_SEV_CRITICAL, 100, 1000,  false, "CAN_BUSOFF"},
};

// A fault's ID is its index here, so the table and the enum have to agree.
// They stopped agreeing once. This runs at init, costs nothing, and has paid
// for itself several times over.
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

    // A shuffled table applies the wrong severity to every fault in the car,
    // silently. Better to set all of them and refuse to move.
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

/* Two timers per fault, not one. `present_ms` has to fill up before it sets,
 * `absent_ms` before it clears, and each resets the other. That gap is
 * deliberate hysteresis: a condition hovering right on its threshold
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
