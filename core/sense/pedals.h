// pedals.h - accelerator and brake: two sensors each, and whether to
// believe them. All four are System Critical Signals (T11.8.7, T11.9.1).

#ifndef VCU_PEDALS_H
#define VCU_PEDALS_H

#include "core/config.h"
#include "core/types.h"

// One redundant pair. The accelerator and the brake are the same problem,
// so they share the code and differ only in thresholds and which way they
// fail.
typedef struct {
    pct_x10_t value;          // the answer the rest of the VCU uses
    pct_x10_t ch1, ch2;       // each channel on its own, for the log
    bool      ch1_ok, ch2_ok; // inside its valid window
    uint16_t  deviation_ms;   // how long the pair has been implausible
    bool      implausible;    // latched until agreement + pedal released
} sensor_pair_t;

static inline bool pair_range_bad(const sensor_pair_t *p)
{
    return !p->ch1_ok || !p->ch2_ok;
}

typedef struct {
    sensor_pair_t apps;
    sensor_pair_t brake;

    bool brake_applied;       // either brake channel says the brakes are on
    bool brake_confirmed;     // both do, and the pair is healthy

    // Brake and throttle together. Not a sensor fault, both pedals can be
    // reading correctly and still be asking for something that makes no
    // sense.
    bool bppc_latched;
} pedals_t;

void pedals_init(pedals_t *p);
void pedals_step(pedals_t *p, const vcu_cfg_t *cfg, const vcu_in_t *in,
                 uint16_t dt_ms);

// Whether the pedals allow any torque at all. Only covers the "readings
// don't make sense" cases; a broken wire is a CRITICAL fault handled by the
// fault table.
static inline bool pedals_torque_permitted(const pedals_t *p)
{
    return !p->apps.implausible && !p->brake.implausible && !p->bppc_latched;
}

#endif // VCU_PEDALS_H
