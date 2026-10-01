// signals.c - message health.
//
// T11.9.2.d wants digitally transmitted signals protected against data
// corruption (checksum) and loss and delay of messages (timeouts). The
// rolling counter adds the case a timeout can't see: frames still arriving
// but with the same old data in them. An old reading that still looks
// sensible is worse than a missing one.
#include "core/sense/signals.h"
#include "core/util.h"   // sat_add

void sig_init(sig_health_t *h)
{
    const sig_health_t zero = { 0 };
    *h = zero;
    h->timeout = true;      // nobody has said anything yet, so: not trusted
}

void sig_step(sig_health_t *h, bool rx, uint8_t counter, bool crc_ok,
              uint16_t timeout_ms, uint16_t stall_ms, uint16_t dt_ms)
{
    // Both clocks run every tick and get reset by the event they measure.
    // An earlier version only advanced stall_ms on ticks where a frame
    // arrived, which is fine when frames come every tick (the sim used to
    // do that) but at a 100 ms frame period turned a 300 ms stall limit
    // into 30 seconds.
    h->age_ms   = sat_add(h->age_ms, dt_ms);
    h->stall_ms = sat_add(h->stall_ms, dt_ms);

    // A frame with a bad checksum is thrown away: it doesn't refresh the
    // age, so a device sending garbage times out exactly like a silent one.
    // The flag is only kept so the log can say why.
    if (rx) h->corrupt = !crc_ok;

    if (rx && crc_ok) {
        // The counter catches a sender (or gateway) replaying the same frame
        // forever. A timeout can't see that, the frames keep coming.
        if (!h->seen || counter != h->last_counter) h->stall_ms = 0;
        h->last_counter = counter;
        h->age_ms = 0;
        h->seen   = true;
    }

    h->timeout = !h->seen || h->age_ms >= timeout_ms;

    // Only call it stale while frames are still arriving. If they've
    // stopped, that's a timeout, and reporting both just muddies the log.
    h->stale = h->seen && !h->timeout && h->stall_ms >= stall_ms;
}
