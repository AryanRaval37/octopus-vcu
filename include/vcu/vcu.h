/* vcu.h — the entire controller, behind one function.
 *
 *     vcu_step(v, &in, &out);
 *
 * No clocks, no pins, no CAN, no RTOS. Inputs in, outputs out. That one
 * property is why the same object code can run under a scenario test on a
 * laptop and in a car, and why a test can kill a sensor at exactly
 * t=3500 ms, forty times, in a second.
 *
 * The hardware lives in port/<target>/main.c, which is the only file that
 * differs between POSIX and the S32K344.
 */
#ifndef VCU_VCU_H
#define VCU_VCU_H

#include "vcu/apps.h"
#include "vcu/config.h"
#include "vcu/fault.h"
#include "vcu/state.h"
#include "vcu/torque.h"
#include "vcu/types.h"

typedef struct {
    const vcu_cfg_t *cfg;
    apps_t      apps;
    fault_mgr_t faults;
    state_mgr_t state;
    torque_t    torque;
    uint32_t    last_ms;
    bool        started;
    bool        prev_ts_request;   // falling edge = the driver acknowledging
} vcu_t;

void vcu_init(vcu_t *v, const vcu_cfg_t *cfg);
void vcu_step(vcu_t *v, const vcu_in_t *in, vcu_out_t *out);

#endif /* VCU_VCU_H */
