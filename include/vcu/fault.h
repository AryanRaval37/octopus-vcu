/* fault.h — one table, one policy.
 *
 * The rule that keeps this from rotting: modules REPORT, they do not
 * DECIDE. Call fault_report(f, id, present) every tick and the fault
 * manager owns debounce, healing, latching and severity.
 *
 * The payoff is that "what does the car do if the BMS drops out?" has an
 * answer you can point at — one row in one table — instead of an answer you
 * have to reconstruct by grepping for BMS and reading every hit.
 */
#ifndef VCU_FAULT_H
#define VCU_FAULT_H

#include "vcu/types.h"

typedef struct {
    fault_id_t  id;
    fault_sev_t sev;
    uint16_t    debounce_ms;  // has to be present this long before it sets
    uint16_t    heal_ms;      // and absent this long before it clears
    bool        latching;     // if true, heal_ms never clears it
    const char *name;
} fault_def_t;

typedef struct {
    fault_mask_t active;
    uint16_t     present_ms[FAULT_COUNT];
    uint16_t     absent_ms[FAULT_COUNT];

    // Set for exactly one tick when a fault changes, so the logger can
    // catch edges without polling everything every time.
    fault_mask_t just_set;
    fault_mask_t just_cleared;
} fault_mgr_t;

void fault_init(fault_mgr_t *f);

// Start a tick. Clears the edge flags.
void fault_begin(fault_mgr_t *f);

// Is this condition true right now? Call it for every fault every tick — a
// fault you stop reporting counts as absent, which is what you want when a
// subsystem goes away entirely.
void fault_report(fault_mgr_t *f, fault_id_t id, bool present, uint16_t dt_ms);

// The driver says "I saw that". Clears latched faults whose cause has gone.
// Non-latching faults heal by themselves and are untouched.
void fault_acknowledge(fault_mgr_t *f);

const fault_def_t *fault_def(fault_id_t id);
const char        *fault_name(fault_id_t id);

// Worst severity currently active, or INFO when nothing is set.
fault_sev_t fault_worst(const fault_mgr_t *f);

static inline bool fault_is_set(const fault_mgr_t *f, fault_id_t id)
{
    return (f->active & FAULT_BIT(id)) != 0;
}

#endif /* VCU_FAULT_H */
