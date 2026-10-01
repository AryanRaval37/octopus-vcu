// util.h - small integer helpers shared across core/.

#ifndef VCU_UTIL_H
#define VCU_UTIL_H

#include <stdint.h>

// saturation add:
// make sure there is no rollover
// if it sum can't fit (> 65535) then just caps it at 65535 instead of rolling over.
static inline uint16_t sat_add(uint16_t v, uint16_t d)
{
    uint32_t t = (uint32_t)v + d;
    return (t > 0xFFFFu) ? 0xFFFFu : (uint16_t)t;
}

static inline uint16_t min_u16(uint16_t a, uint16_t b) { return a < b ? a : b; }
static inline uint16_t max_u16(uint16_t a, uint16_t b) { return a > b ? a : b; }

#endif // VCU_UTIL_H
