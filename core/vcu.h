/* vcu.h — the entire controller, behind one function.
 *
 *     vcu_step(v, &in, &out);
 *
 * No clocks, no pins, no CAN, no RTOS. Inputs in, outputs out. That one
 * property is why the same object code can run under a scenario test on a
 * laptop and in a car, and why a test can kill a sensor at exactly
 * t=3500 ms, forty times, in a second.
 *
 * Hardware lives in board/<target>/, which is the only thing that differs
 * between the laptop and the S32K344.
 */
#ifndef VCU_VCU_H
#define VCU_VCU_H

#include "core/config.h"
#include "core/datalog.h"
#include "core/decide/faults.h"
#include "core/decide/state.h"
#include "core/decide/torque.h"
#include "core/sense/pedals.h"
#include "core/sense/signals.h"
#include "core/types.h"

typedef struct {
    const vcu_cfg_t *cfg;

    // sense
    pedals_t     pedals;
    sig_health_t bms_health;
    sig_health_t inv_health;

    // decide
    fault_mgr_t faults;
    state_mgr_t state;
    torque_t    torque;

    // record
    datalog_t log;
    uint16_t  log_period_ms;

    uint32_t last_ms;
    bool     started;
    bool     prev_ts_request;   // falling edge = the driver acknowledging
} vcu_t;

void vcu_init(vcu_t *v, const vcu_cfg_t *cfg);
void vcu_step(vcu_t *v, const vcu_in_t *in, vcu_out_t *out);

#endif /* VCU_VCU_H */
