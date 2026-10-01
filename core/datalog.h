// datalog.h - in-memory flight recorder.
//
// The core only fills a ring buffer; the board drains it to wherever that
// target can write (UART, SD card, CAN). That keeps I/O out of the control
// loop and lets scenario tests assert on what got logged.
//
// It never blocks and never grows. If the drain falls behind, the oldest
// rows are overwritten and `dropped` counts how many.

#ifndef VCU_DATALOG_H
#define VCU_DATALOG_H

#include "core/decide/torque.h"
#include "core/types.h"

// Power of two so wrapping is a mask. At 10 Hz this is about 25 s of
// history if nothing drains it.
#define DATALOG_DEPTH 256u

typedef enum {
    LOG_PERIODIC = 0,
    LOG_STATE,          // state machine changed state
    LOG_FAULT_SET,
    LOG_FAULT_CLEARED
} log_reason_t;

// One row. Flat and fixed-size so it can be dumped as raw bytes to an SD
// card and decoded later on a laptop.
typedef struct {
    uint32_t     t_ms;
    fault_mask_t faults;
    nm_x10_t     torque_cmd;
    nm_x10_t     torque_unlimited;
    rpm_t        rpm;
    dv_t         dc_link_dv;
    pct_x10_t    pedal;
    pct_x10_t    brake;
    pct_x10_t    derate;
    mv_t         cell_min_mv;
    uint8_t      state;
    uint8_t      reason;      // log_reason_t
    uint8_t      flags;       // LOG_F_*
    uint8_t      _pad;
} log_rec_t;

// If this changes, anything decoding old binary logs has to change with it.
_Static_assert(sizeof(log_rec_t) == 28, "log_rec_t layout changed");

#define LOG_F_AIR_POS   0x01u
#define LOG_F_AIR_NEG   0x02u
#define LOG_F_PRECHARGE 0x04u
#define LOG_F_ENABLE    0x08u
#define LOG_F_SDC_OK    0x10u
#define LOG_F_BMS_OK    0x20u
#define LOG_F_INV_OK    0x40u

typedef struct {
    log_rec_t rec[DATALOG_DEPTH];
    uint16_t  head;          // next write
    uint16_t  tail;          // next read
    uint16_t  count;
    uint32_t  dropped;       // rows lost because the drain fell behind
    uint16_t  since_ms;      // since the last periodic row
    fault_mask_t prev_faults;
    uint8_t   prev_state;
    bool      started;
} datalog_t;

void datalog_init(datalog_t *d);

// Once per tick, after everything has been decided. Writes a row every
// period_ms, and also on any tick where the state or a fault changed.
void datalog_step(datalog_t *d, const vcu_in_t *in, const vcu_out_t *out,
                  const torque_t *tq, bool bms_ok, bool inv_ok,
                  uint16_t period_ms, uint16_t dt_ms);

// Oldest row first. False when empty.
bool datalog_pop(datalog_t *d, log_rec_t *out);

static inline uint16_t datalog_pending(const datalog_t *d) { return d->count; }

const char *datalog_reason_name(log_reason_t r);

// CSV header and row, kept next to each other so they can't drift apart.
// Here rather than in a board so every target produces the same format.
const char *datalog_csv_header(void);
int datalog_format_row(const log_rec_t *r, char *buf, unsigned cap);

#endif // VCU_DATALOG_H
