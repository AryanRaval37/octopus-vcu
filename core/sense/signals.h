// signals.h - message health for one CAN device (T11.9.2.d).
//
// One of these per device we listen to. It answers one question: can we
// still act on what this device last told us?
#ifndef VCU_SIGNALS_H
#define VCU_SIGNALS_H

#include "core/types.h"

typedef struct {
    uint16_t age_ms;         // since the last good frame
    uint16_t stall_ms;       // since the rolling counter last changed
    uint8_t  last_counter;
    bool     seen;           // have we ever heard from it?

    // The three failure modes of T11.9.2.d, kept apart on purpose. "The BMS
    // is quiet", "the BMS is repeating itself" and "the BMS is talking
    // nonsense" are three different repairs.
    bool timeout;
    bool stale;
    // Last frame failed its checksum. Diagnostic only, not part of
    // sig_trustworthy(): bad frames don't refresh age_ms, so a device that
    // keeps sending garbage already ends up as a timeout. Having it in the
    // trust check as well meant one corrupt frame made the BMS "untrusted"
    // for a whole frame period while the fault table only called it a WARN.
    bool corrupt;
} sig_health_t; // ? sig_health_t? why not signal_t

// cheesy idea here:
// all these init functions everywhere need you have already made the memory and here you just set it to zero
// No mallocs anywhere in the codebase everything is on stack
// which ever function needs this struct they make it there on the stack predefined and this function just populates it.
void sig_init(sig_health_t *h);

// Feed it what the board saw this tick: whether a frame arrived, its
// rolling counter, and whether the checksum passed. Call it every tick,
// including ticks with no frame, since the timeouts count those.
void sig_step(sig_health_t *h, bool rx, uint8_t counter, bool crc_ok,
              uint16_t timeout_ms, uint16_t stall_ms, uint16_t dt_ms);

// What the rest of the VCU should use rather than poking at the flags.
static inline bool sig_trustworthy(const sig_health_t *h)
{
    return h->seen && !h->timeout && !h->stale;
}

#endif // VCU_SIGNALS_H
