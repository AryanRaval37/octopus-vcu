// vcu.h - the whole controller behind one call:
//
//     vcu_step(&vcu, &in, &out);
//
// No clock, no pins, no CAN, no RTOS in here. The board fills `in`, calls
// this, and applies `out`. That's what lets the same code run in the
// scenario tests on a laptop and on the S32K344.
//
// All state lives in vcu_t, which the caller allocates (statically, on the
// target). Only one task may touch it.

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
    bool             cfg_ok;

    pedals_t     pedals;
    sig_health_t bms_health;
    sig_health_t inv_health;

    fault_mgr_t faults;
    state_mgr_t state;
    torque_t    torque;

    datalog_t log;
    uint16_t  log_period_ms;

    uint32_t last_ms;
    uint16_t boot_ms;           // counts up to cfg->boot_grace_ms, then stops
    bool     started;
    bool     prev_ts_request;
} vcu_t;

// cfg may be NULL for the default calibration. An invalid cfg isn't
// rejected here; the VCU raises FAULT_BAD_CONFIG and stays out of the way.
void vcu_init(vcu_t *v, const vcu_cfg_t *cfg);
void vcu_step(vcu_t *v, const vcu_in_t *in, vcu_out_t *out);

#endif // VCU_VCU_H
