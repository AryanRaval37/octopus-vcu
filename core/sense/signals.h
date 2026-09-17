/* signals.h — is anything on this bus still worth believing?
 *
 * One of these per CAN talker. It answers a single question — may I steer a
 * car by what this device last said? — and T11.9.2.d says three separate
 * things have to be true for the answer to be yes.
 */
#ifndef VCU_SIGNALS_H
#define VCU_SIGNALS_H

#include "core/types.h"

typedef struct {
    uint16_t age_ms;         // since the last frame we accepted
    uint16_t stall_ms;       // frames arriving, counter not moving
    uint8_t  last_counter;
    bool     seen;           // have we ever heard from it?

    // The three failure modes of T11.9.2.d, kept apart on purpose. "The BMS
    // is quiet", "the BMS is repeating itself" and "the BMS is talking
    // nonsense" are three different repairs.
    bool timeout;
    bool stale;
    bool corrupt;
} sig_health_t;

void sig_init(sig_health_t *h);

/* Feed it what the board saw this tick: whether a frame turned up, its
 * rolling counter, and whether the checksum passed. Call every tick,
 * including the ticks where nothing arrived — silence is the input. */
void sig_step(sig_health_t *h, bool rx, uint8_t counter, bool crc_ok,
              uint16_t timeout_ms, uint16_t stall_ms, uint16_t dt_ms);

// The only question the rest of the VCU is allowed to ask.
static inline bool sig_trustworthy(const sig_health_t *h)
{
    return h->seen && !h->timeout && !h->stale && !h->corrupt;
}

#endif /* VCU_SIGNALS_H */
