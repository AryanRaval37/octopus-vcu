/* pedals.h — what the driver is asking for, and whether to believe them.
 *
 * Four sensors: two on the accelerator, two on the brake. Every one of them
 * is a System Critical Signal under T11.9.1, because all four can influence
 * wheel torque.
 *
 * Code is in core/sense/pedals.c, and it is the first thing to read here.
 */
#ifndef VCU_PEDALS_H
#define VCU_PEDALS_H

#include "core/config.h"
#include "core/types.h"

/* One redundant pair. Both pedals are the same shape of problem — two
 * sensors, do they agree, which one do I believe — so they share this and
 * there is one place to get it wrong instead of two. */
typedef struct {
    pct_x10_t value;          // the arbitrated answer
    pct_x10_t ch1, ch2;       // per-channel, for telemetry
    bool      ch1_ok, ch2_ok; // in range?
    uint16_t  deviation_ms;   // how long they have been arguing
    bool      implausible;    // latched: they argued for too long
} pair_t;

static inline bool pair_range_bad(const pair_t *p)
{
    return !p->ch1_ok || !p->ch2_ok;
}

typedef struct {
    pair_t apps;
    pair_t brake;
    bool   brake_applied;

    /* Torque must be zero while this is set. It is separate from the two
     * pairs because it is not a sensor failure — both pedals can be working
     * perfectly and still be telling you something impossible. */
    bool bppc_latched;
} pedals_t;

void pedals_init(pedals_t *p);

// Call at a fixed rate, 1 kHz on the real thing. dt_ms has to be the time
// that actually passed: the deviation timer is the rule, and T11.8.8 is
// written in milliseconds.
void pedals_step(pedals_t *p, const vcu_cfg_t *cfg, const vcu_in_t *in,
                 uint16_t dt_ms);

/* The pedal subsystem's veto. Torque code asks; it does not get to argue.
 *
 * Note this covers only the "these readings are inconsistent" faults. A
 * channel being out of range is a signal failure, and T11.9.5 makes that a
 * shutdown-circuit matter rather than a torque one — so it is reported to
 * the fault manager instead and handled there. */
static inline bool pedals_torque_permitted(const pedals_t *p)
{
    return !p->apps.implausible && !p->brake.implausible && !p->bppc_latched;
}

#endif /* VCU_PEDALS_H */
