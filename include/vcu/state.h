/* state.h — the vehicle state machine, and the precharge sequencer.
 *
 * Precharge lives in here rather than in a module of its own because it is
 * a state, not a service. Every way out of it is a state transition, and
 * splitting it off would leave two pieces of code that have to agree about
 * one timer. They would stop agreeing.
 */
#ifndef VCU_STATE_H
#define VCU_STATE_H

#include "vcu/apps.h"
#include "vcu/config.h"
#include "vcu/fault.h"
#include "vcu/types.h"

typedef struct {
    vcu_state_t state;
    vcu_state_t prev;         // what we left, for one tick
    bool        entered;      // true on the first tick in a state
    uint32_t    in_state_ms;

    // precharge bookkeeping
    dv_t        pc_start_dv;
    bool        pc_rise_checked;

    uint32_t    buzzer_ms;
} state_mgr_t;

void state_init(state_mgr_t *s);

void state_step(state_mgr_t *s, const vcu_cfg_t *cfg, const vcu_in_t *in,
                const apps_t *apps, fault_mgr_t *faults, uint16_t dt_ms);

// Contactors and buzzer fall out of the state plus a timer, so derive them
// instead of storing them. One less thing that can disagree with itself.
void state_outputs(const state_mgr_t *s, vcu_out_t *out);

#endif /* VCU_STATE_H */
