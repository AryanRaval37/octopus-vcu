// ! REVIEWED
// ! Todo written in apps.c

/* apps.h — pedals in, "allowed to move" out.
 *
 * Two accelerator sensors, two brake sensors 
 * Code is in src/svc/apps.c.
 */
#ifndef VCU_APPS_H
#define VCU_APPS_H

#include "vcu/config.h"
#include "vcu/types.h"

typedef struct {
    pct_x10_t pedal;          // recheck: lower of the two
    pct_x10_t brake;
    bool      brake_applied;

    // Either of these means no torque. Both latch, and each has its own
    // rule for clearing — see apps.c.
    bool implausible;         // channels disagree, or one is out of range
    bool bppc_latched;        // brake and throttle at the same time

    // for testing and logging only
    uint16_t  deviation_ms;   // how long they have been arguing
    pct_x10_t ch1, ch2;
    bool      ch1_ok, ch2_ok;
} apps_t;

void apps_init(apps_t *a);

// Call at a fixed rate, 1 kHz on the real thing. dt_ms has to be the time that actually passed
void apps_step(apps_t *a, const vcu_cfg_t *cfg, const vcu_in_t *in, uint16_t dt_ms);

// The pedal subsystem's veto. Torque code asks; it does not get to argue.
static inline bool apps_torque_permitted(const apps_t *a)
{
    return !a->implausible && !a->bppc_latched;
}

#endif /* VCU_APPS_H */
