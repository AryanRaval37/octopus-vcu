/* torque.h — pedal position in, torque command out.
 *
 * The order of operations inside torque_step() is a safety property, not a
 * matter of taste. Read the top of src/app/torque.c before changing it.
 */
#ifndef VCU_TORQUE_H
#define VCU_TORQUE_H

#include "vcu/apps.h"
#include "vcu/config.h"
#include "vcu/fault.h"
#include "vcu/types.h"

typedef struct {
    nm_x10_t  cmd;        // last command, kept for the rate limiter
    nm_x10_t  unlimited;  // what it would have been, for the log
    pct_x10_t derate;     // 1000 = full torque, 0 = nothing left
} torque_t;

void torque_init(torque_t *t);

void torque_step(torque_t *t, const vcu_cfg_t *cfg, const vcu_in_t *in,
                 const apps_t *apps, const fault_mgr_t *faults,
                 vcu_state_t state);

#endif /* VCU_TORQUE_H */
