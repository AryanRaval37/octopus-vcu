/* datalog.h — the flight recorder.
 *
 * Not optional. You cannot debug a moving vehicle: by the time it is back in
 * the paddock the evidence is gone, and the driver's account of what
 * happened is a story about what they felt, not what the contactors did.
 *
 * Two rules shape this file.
 *
 * It does no I/O. It fills a ring buffer and the board drains it — to a UART,
 * an SD card, a CAN stream, whatever that target has. That keeps the core
 * pure and it means a scenario test can assert on what WOULD have been
 * logged, which is how you find out your logging is useless before the event
 * rather than after it.
 *
 * It never blocks and it never grows. A logger that can stall the control
 * loop is a logger that can stop the car; if the buffer fills, the oldest
 * samples go and a counter goes up, so you always know you lost some.
 */
#ifndef VCU_DATALOG_H
#define VCU_DATALOG_H

#include "core/decide/torque.h"
#include "core/types.h"

// Power of two so the wrap is a mask. 256 samples at 100 Hz is 2.5 s of
// history, which is more than enough to survive a slow drain.
#define DATALOG_DEPTH 256u

/* Why a sample happened. Reading a log later, the question is almost always
 * "what changed just before this?" — so the reason is recorded rather than
 * inferred from timestamps. */
typedef enum {
    LOG_PERIODIC = 0,   // the routine heartbeat
    LOG_STATE,          // the state machine moved
    LOG_FAULT_SET,      // a fault appeared
    LOG_FAULT_CLEARED   // a fault healed or was acknowledged
} log_reason_t;

/* One row. Deliberately small and flat — 32 bytes, no pointers, no padding
 * games — so it can be written to NVM or squirted down a UART as raw bytes
 * without a serialiser. */
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
    uint8_t      flags;       // see LOG_F_* below
    uint8_t      _pad;
} log_rec_t;

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
    uint32_t  dropped;       // samples lost to a slow drain -- watch this
    uint16_t  since_ms;      // time since the last periodic sample
    fault_mask_t prev_faults;
    uint8_t   prev_state;
    bool      started;
} datalog_t;

void datalog_init(datalog_t *d);

/* Call once per tick, after the controller has decided. `period_ms` is the
 * routine sampling interval — events are recorded regardless of it, because
 * the moment a fault sets is worth more than any number of quiet rows. */
void datalog_step(datalog_t *d, const vcu_in_t *in, const vcu_out_t *out,
                  const torque_t *tq, bool bms_ok, bool inv_ok,
                  uint16_t period_ms, uint16_t dt_ms);

// Drain one row, oldest first. False when empty. The board calls this from
// whatever task can afford to wait on a filesystem.
bool datalog_pop(datalog_t *d, log_rec_t *out);

static inline uint16_t datalog_pending(const datalog_t *d) { return d->count; }

const char *datalog_reason_name(log_reason_t r);

// Column headers matching datalog_format_row(), so a CSV and its parser
// cannot drift apart.
const char *datalog_csv_header(void);

/* Render one row as a CSV line into `buf`. Lives here rather than in the
 * board so that every target logs the identical format, and so a real drive
 * can be replayed through the simulator without a translation step. */
int datalog_format_row(const log_rec_t *r, char *buf, unsigned cap);

#endif /* VCU_DATALOG_H */
