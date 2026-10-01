// faults.h - fault manager.
//
// Modules report whether a condition is true, every tick. This file decides
// what that means: debounce, healing, latching and severity all come from
// one table in faults.c.

#ifndef VCU_FAULT_H
#define VCU_FAULT_H

#include "core/types.h"

typedef struct {
    fault_sev_t sev;
    uint16_t    debounce_ms;  // present this long before it sets
    uint16_t    heal_ms;      // absent this long before it clears
    bool        latching;     // if set, only fault_acknowledge() clears it
    const char *name;
} fault_def_t;

typedef struct {
    fault_mask_t active;
    uint16_t     present_ms[FAULT_COUNT]; // ? ms stored in uint16?
    uint16_t     absent_ms[FAULT_COUNT];

    // Bits that changed this tick, for the logger.
    fault_mask_t just_set;
    fault_mask_t just_cleared;
} fault_mgr_t;

void fault_init(fault_mgr_t *f);

// Call at the start of every tick, clears the edge masks.
void fault_begin(fault_mgr_t *f);

// Report every fault every tick, false included. Healing only happens on
// ticks where the fault is reported absent.
void fault_report(fault_mgr_t *f, fault_id_t id, bool present, uint16_t dt_ms);

// Driver acknowledged. Clears latched faults whose cause is gone.
void fault_acknowledge(fault_mgr_t *f);

const fault_def_t *fault_def(fault_id_t id);
const char        *fault_name(fault_id_t id);

// Worst active severity (INFO if nothing is active).
fault_sev_t fault_worst(const fault_mgr_t *f);

// Same, only looking at the faults in `mask`.
fault_sev_t fault_worst_in(const fault_mgr_t *f, fault_mask_t mask);

static inline bool fault_is_set(const fault_mgr_t *f, fault_id_t id)
{
    return (f->active & FAULT_BIT(id)) != 0;
}

#endif // VCU_FAULT_H
