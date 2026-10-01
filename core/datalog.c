// datalog.c - ring buffer plus the decision of when to write a row.
//
// Logging every tick at 1 kHz drowns you, logging once a second misses the
// thing you wanted. So: a slow heartbeat, plus a row on the exact tick that
// the state or a fault changes.

#include "core/datalog.h"

#include <inttypes.h>
#include <stdio.h>

void datalog_init(datalog_t *d)
{
    const datalog_t zero = { 0 };
    *d = zero;
}

static void push(datalog_t *d, const log_rec_t *r)
{
    // Full means the drain isn't keeping up. Drop the oldest row, not the
    // newest: whatever is happening right now is the reason someone will
    // read this log.
    if (d->count == DATALOG_DEPTH) {
        d->tail = (uint16_t)((d->tail + 1u) & (DATALOG_DEPTH - 1u));
        d->count--;
        d->dropped++;
    }
    d->rec[d->head] = *r;
    d->head = (uint16_t)((d->head + 1u) & (DATALOG_DEPTH - 1u));
    d->count++;
}

void datalog_step(datalog_t *d, const vcu_in_t *in, const vcu_out_t *out,
                  const torque_t *tq, bool bms_ok, bool inv_ok,
                  uint16_t period_ms, uint16_t dt_ms)
{
    d->since_ms = (uint16_t)(d->since_ms + dt_ms);

    log_reason_t reason;
    if (!d->started)                        reason = LOG_STATE;
    else if (out->faults & ~d->prev_faults) reason = LOG_FAULT_SET;
    else if (d->prev_faults & ~out->faults) reason = LOG_FAULT_CLEARED;
    else if (out->state != d->prev_state)   reason = LOG_STATE;
    else if (d->since_ms >= period_ms)      reason = LOG_PERIODIC;
    else return;

    uint8_t flags = 0;
    if (out->air_pos)         flags |= LOG_F_AIR_POS;
    if (out->air_neg)         flags |= LOG_F_AIR_NEG;
    if (out->precharge_relay) flags |= LOG_F_PRECHARGE;
    if (out->inverter_enable) flags |= LOG_F_ENABLE;
    if (in->shutdown_ok)      flags |= LOG_F_SDC_OK;
    if (bms_ok)               flags |= LOG_F_BMS_OK;
    if (inv_ok)               flags |= LOG_F_INV_OK;

    const log_rec_t r = {
        .t_ms             = in->now_ms,
        .faults           = out->faults,
        .torque_cmd       = out->torque_cmd,
        .torque_unlimited = tq->unlimited,
        .rpm              = in->inv.rpm,
        .dc_link_dv       = in->inv.dc_link_dv,
        .pedal            = out->pedal,
        .brake            = out->brake,
        .derate           = tq->derate,
        .cell_min_mv      = in->bms.cell_min_mv,
        .state            = (uint8_t)out->state,
        .reason           = (uint8_t)reason,
        .flags            = flags,
    };
    push(d, &r);

    d->since_ms    = 0;
    d->prev_faults = out->faults;
    d->prev_state  = (uint8_t)out->state;
    d->started     = true;
}

bool datalog_pop(datalog_t *d, log_rec_t *out)
{
    if (d->count == 0) return false;
    *out = d->rec[d->tail];
    d->tail = (uint16_t)((d->tail + 1u) & (DATALOG_DEPTH - 1u));
    d->count--;
    return true;
}

const char *datalog_reason_name(log_reason_t r)
{
    switch (r) {
    case LOG_PERIODIC:      return "periodic";
    case LOG_STATE:         return "state";
    case LOG_FAULT_SET:     return "fault_set";
    case LOG_FAULT_CLEARED: return "fault_clr";
    default:                return "?";
    }
}

const char *datalog_csv_header(void)
{
    return "t_ms,state,reason,pedal,brake,torque,torque_raw,derate,"
           "rpm,dclink_dv,cell_mv,faults,air_pos,air_neg,precharge,"
           "enable,sdc_ok,bms_ok,inv_ok";
}

static int bit(uint8_t flags, uint8_t f) { return (flags & f) ? 1 : 0; }

// PRIu32 and friends because uint32_t is `unsigned long` on arm-none-eabi
// and `unsigned int` on a desktop, and plain %u is wrong on one of them.
int datalog_format_row(const log_rec_t *r, char *buf, unsigned cap)
{
    return snprintf(buf, cap,
        "%" PRIu32 ",%s,%s,%u,%u,%d,%d,%u,%u,%u,%u,0x%06" PRIX32 ",%d,%d,%d,%d,%d,%d,%d",
        r->t_ms,
        vcu_state_name((vcu_state_t)r->state),
        datalog_reason_name((log_reason_t)r->reason),
        r->pedal, r->brake, r->torque_cmd, r->torque_unlimited, r->derate,
        r->rpm, r->dc_link_dv, r->cell_min_mv,
        r->faults,
        bit(r->flags, LOG_F_AIR_POS),  bit(r->flags, LOG_F_AIR_NEG),
        bit(r->flags, LOG_F_PRECHARGE), bit(r->flags, LOG_F_ENABLE),
        bit(r->flags, LOG_F_SDC_OK),   bit(r->flags, LOG_F_BMS_OK),
        bit(r->flags, LOG_F_INV_OK));
}
