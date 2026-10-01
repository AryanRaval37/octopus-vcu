// state.h - vehicle state machine, including the precharge sequence.
//
// Precharge is a state here rather than its own module because every way
// out of it is a state transition, and two modules sharing one timer is
// asking for them to disagree.

#ifndef VCU_STATE_H
#define VCU_STATE_H

#include "core/config.h"
#include "core/decide/faults.h"
#include "core/sense/pedals.h"
#include "core/types.h"

typedef struct {
    vcu_state_t state;
    vcu_state_t prev;         // state we came from
    bool        entered;      // true on the first tick in a state
    uint32_t    in_state_ms;

    dv_t        pc_start_dv;
    bool        pc_rise_checked;

    bool        prev_rtd_button;
} state_mgr_t;

void state_init(state_mgr_t *s);

// bms_ok / inv_ok are sig_trustworthy() for each device.
void state_step(state_mgr_t *s, const vcu_cfg_t *cfg, const vcu_in_t *in,
                const pedals_t *pedals, fault_mgr_t *faults,
                bool bms_ok, bool inv_ok, uint16_t dt_ms);

// Relays, buzzer and inverter enable for the current state.
void state_outputs(const state_mgr_t *s, const vcu_cfg_t *cfg, vcu_out_t *out);

#endif // VCU_STATE_H
