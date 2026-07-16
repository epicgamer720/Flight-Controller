#pragma once
#include <stdint.h>
#include <stddef.h>

/* CRC16-CCITT (poly 0x1021, init 0xFFFF), shared FC/GS, per CLAUDE.md §7 */
static inline uint16_t crc16_ccitt(const uint8_t *d, size_t n) {
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        c ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++)
            c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}
