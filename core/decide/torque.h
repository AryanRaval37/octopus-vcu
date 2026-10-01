// torque.h - pedal in, torque command out. The order of the steps inside
// torque_step() is a safety property; see the top of torque.c.

#ifndef VCU_TORQUE_H
#define VCU_TORQUE_H

#include "core/config.h"
#include "core/decide/faults.h"
#include "core/sense/pedals.h"
#include "core/types.h"

typedef struct {
    nm_x10_t  cmd;           // last command, needed for the slew limit
    nm_x10_t  unlimited;     // before slew limit and gates, for the log
    pct_x10_t derate;        // 1000 = full torque available, 0 = none
    watt_t    power_limit_w; // electrical limit in force this tick, for the log
} torque_t;

void torque_init(torque_t *t);

void torque_step(torque_t *t, const vcu_cfg_t *cfg, const vcu_in_t *in,
                 const pedals_t *pedals, const fault_mgr_t *faults,
                 vcu_state_t state, bool bms_ok, bool inv_ok, uint16_t dt_ms);

#endif // VCU_TORQUE_H
