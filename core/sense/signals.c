/* signals.c — message health.
 *
 * T11.9.2.d asks for three defences on any digitally transmitted signal:
 * data corruption (checked by a checksum), and loss and delay of messages
 * (checked by timeouts). This file is those three, and nothing else.
 *
 * The reason it is worth a file of its own: a stale reading that still looks
 * sensible is more dangerous than a missing one. A pack voltage of 400 V is
 * a fine number right up until you learn it arrived four seconds ago.
 *
 * Next: core/sense/pedals.c.
 */
#include "core/sense/signals.h"

static uint16_t sat_add(uint16_t v, uint16_t d)
{
    uint32_t t = (uint32_t)v + d;
    return (t > 0xFFFFu) ? 0xFFFFu : (uint16_t)t;
}

void sig_init(sig_health_t *h)
{
    const sig_health_t zero = { 0 };
    *h = zero;
    h->timeout = true;      // nobody has said anything yet, so: not trusted
}

void sig_step(sig_health_t *h, bool rx, uint8_t counter, bool crc_ok,
              uint16_t timeout_ms, uint16_t stall_ms, uint16_t dt_ms)
{
    const bool good = rx && crc_ok;

    /* A corrupt frame is worse than no frame, so it does not refresh
     * anything -- the age keeps climbing underneath it. That is deliberate,
     * and it is why corruption needs no trip threshold of its own: a device
     * babbling rubbish stops being trusted on exactly the same schedule as
     * a device that went quiet. */
    if (rx) h->corrupt = !crc_ok;

    if (good) {
        /* The counter catches the failure a timeout cannot see: a gateway,
         * buffer or sender that keeps replaying one good frame forever. The
         * bus looks healthy and the data is frozen. */
        if (h->seen && counter == h->last_counter) {
            h->stall_ms = sat_add(h->stall_ms, dt_ms);
        } else {
            h->stall_ms = 0;
        }
        h->last_counter = counter;
        h->age_ms = 0;
        h->seen   = true;
    } else {
        h->age_ms = sat_add(h->age_ms, dt_ms);
    }

    h->timeout = !h->seen || (h->age_ms >= timeout_ms);
    h->stale   = h->stall_ms >= stall_ms;
}
