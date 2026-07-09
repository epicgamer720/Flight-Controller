/* ============================================================
 * gps.c — external NMEA GPS on USART6 (PC6=TX, PC7=RX, J8)
 *
 * RX path: USART6 IRQ (register-level, stm32f7xx_it.c ->
 * gps_uart_irq) pushes bytes into a 512-byte SPSC ring; the
 * superloop drains it in gps_poll() through a line assembler
 * and parses checksummed GGA/RMC sentences.
 *
 * Autobaud: 9600 -> 38400 -> 57600 -> 115200 (wraps), advancing
 * every 2 s until the first checksum-valid sentence locks it.
 * No blocking anywhere; all pacing via HAL_GetTick().
 * ============================================================ */
#include "app.h"
#include <string.h>

#define GPS_RING_SIZE     512u            /* power of two */
#define GPS_RING_MASK     (GPS_RING_SIZE - 1u)
#define GPS_LINE_MAX      100u            /* NMEA max is 82; margin for sloppy modules */
#define GPS_NMEA_MAXF     20              /* GGA has 15 fields */
#define GPS_AUTOBAUD_MS   2000u
#define GPS_FIX_STALE_MS  5000u

/* ---- ISR <-> superloop SPSC ring (indices are single 16-bit writes,
 * atomic on Cortex-M7; head written only by ISR, tail only by poll) ---- */
static volatile uint8_t  s_ring[GPS_RING_SIZE];
static volatile uint16_t s_head;          /* producer: gps_uart_irq */
static volatile uint16_t s_tail;          /* consumer: gps_poll */

/* ---- line assembler + parser state (superloop only) ---- */
static char      s_line[GPS_LINE_MAX];
static uint16_t  s_line_len;
static bool      s_in_sentence;

static gps_fix_t s_fix;                   /* updated only in gps_poll */
static bool      s_locked;                /* first valid checksum seen */
static uint8_t   s_baud_idx;
static uint32_t  s_baud_t0;
static const uint32_t s_bauds[4] = { 9600u, 38400u, 57600u, 115200u };

/* ================= USART6 IRQ (keep minimal, never block) ================= */
void gps_uart_irq(void)
{
    /* Drain every pending byte (F722 USART has no RX FIFO; loop guards
     * against a byte landing while we are in the handler). RDR read
     * clears RXNE. */
    while (USART6->ISR & USART_ISR_RXNE) {
        uint8_t  b = (uint8_t)(USART6->RDR & 0xFFu);
        uint16_t h = s_head;
        uint16_t n = (uint16_t)((h + 1u) & GPS_RING_MASK);
        if (n != s_tail) {                /* ring full: drop newest byte */
            s_ring[h] = b;
            s_head    = n;
        }
    }
    /* Error flags are NOT cleared by reading RDR on F7 — an uncleared ORE
     * permanently gates further RXNE events and hangs RX. Always clear. */
    if (USART6->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE))
        USART6->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF | USART_ICR_PECF;
}

/* ================= tiny local parse helpers ================= */
static int hexv(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static uint32_t dec_u(const char *s)      /* unsigned decimal; stops at non-digit */
{
    uint32_t v = 0;
    while (*s >= '0' && *s <= '9') v = v * 10u + (uint32_t)(*s++ - '0');
    return v;
}

/* "$xxYYY,...*hh" — XOR of bytes between '$' and '*' must equal hh,
 * and the two hex digits must be the last chars of the line. */
static bool nmea_checksum_ok(const char *l, uint16_t n)
{
    if (n < 6 || l[0] != '$') return false;
    uint16_t star = 0;
    for (uint16_t i = 1; i < n; i++) {
        if (l[i] == '*') { star = i; break; }
    }
    if (star == 0 || (uint16_t)(star + 3u) != n) return false;
    int hi = hexv(l[star + 1]), lo = hexv(l[star + 2]);
    if (hi < 0 || lo < 0) return false;
    uint8_t cs = 0;
    for (uint16_t i = 1; i < star; i++) cs ^= (uint8_t)l[i];
    return cs == (uint8_t)((hi << 4) | lo);
}

/* Split in place on ',', terminate at '*'. Returns field count. */
static int split_fields(char *s, char *f[], int maxf)
{
    int n = 0;
    f[n++] = s;
    for (char *p = s; *p; p++) {
        if (*p == '*') { *p = '\0'; break; }
        if (*p == ',') {
            *p = '\0';
            if (n >= maxf) break;
            f[n++] = p + 1;
        }
    }
    return n;
}

/* ddmm.mmmm / dddmm.mmmm + N/S/E/W  ->  deg * 1e7, signed.
 * Integer math: minutes scaled to 1e-6 then /6 => deg*1e7 (error < 1e-7 deg). */
static bool parse_coord(const char *f, const char *hemi, bool is_lon, int32_t *out_e7)
{
    if (f[0] == '\0' || hemi[0] == '\0') return false;
    const char *p = f;
    int64_t ip = 0;
    int     nd = 0;
    while (*p >= '0' && *p <= '9') { ip = ip * 10 + (*p++ - '0'); if (++nd > 5) return false; }
    int64_t frac_e6 = 0;
    if (*p == '.') {
        p++;
        int fd = 0;
        while (*p >= '0' && *p <= '9') {
            if (fd < 6) { frac_e6 = frac_e6 * 10 + (*p - '0'); fd++; }
            p++;                          /* extra precision beyond 1e-6 min ignored */
        }
        while (fd < 6) { frac_e6 *= 10; fd++; }
    }
    if (*p != '\0') return false;
    int64_t deg   = ip / 100;
    int64_t min_i = ip % 100;
    if (min_i >= 60) return false;
    if (deg > (is_lon ? 180 : 90)) return false;
    int64_t min_e6 = min_i * 1000000 + frac_e6;
    int64_t v = deg * 10000000LL + (min_e6 * 10 + 30) / 60;   /* rounded */
    char h = hemi[0];
    if (h == 'S' || h == 'W')      v = -v;
    else if (h != 'N' && h != 'E') return false;
    *out_e7 = (int32_t)v;                 /* max 1.8e9 fits int32 */
    return true;
}

/* Decimal meters (optionally signed, fractional) -> centimeters. */
static bool parse_m_to_cm(const char *f, int32_t *out_cm)
{
    const char *p = f;
    bool neg = false, any = false;
    if (*p == '-') { neg = true; p++; }
    else if (*p == '+') p++;
    int64_t ip = 0;
    while (*p >= '0' && *p <= '9') { ip = ip * 10 + (*p++ - '0'); any = true; if (ip > 100000000LL) return false; }
    int32_t fr = 0, fd = 0;
    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') {
            if (fd < 2) { fr = fr * 10 + (*p - '0'); fd++; }
            p++;
            any = true;
        }
    }
    if (!any || *p != '\0') return false;
    while (fd < 2) { fr *= 10; fd++; }
    int64_t cm = ip * 100 + fr;
    *out_cm = (int32_t)(neg ? -cm : cm);
    return true;
}

/* ================= sentence handlers ================= */
/* GGA: 0=xxGGA 1=time 2=lat 3=N/S 4=lon 5=E/W 6=quality 7=sats 8=HDOP
 *      9=alt(MSL,m) 10='M' ... */
static void handle_gga(char *f[], int nf)
{
    if (nf < 8) return;
    uint32_t q    = dec_u(f[6]);
    uint32_t sats = dec_u(f[7]);
    s_fix.sats = (sats > 255u) ? 255u : (uint8_t)sats;
    if (q == 0u) {                        /* valid sentence, no fix */
        s_fix.fix = false;
        return;
    }
    int32_t lat, lon, alt;
    if (!parse_coord(f[2], f[3], false, &lat) ||
        !parse_coord(f[4], f[5], true,  &lon))
        return;
    s_fix.lat_e7 = lat;
    s_fix.lon_e7 = lon;
    if (nf > 9 && parse_m_to_cm(f[9], &alt))
        s_fix.alt_msl_cm = alt;
    s_fix.fix         = true;
    s_fix.last_fix_ms = HAL_GetTick();
}

/* RMC (fix fallback): 0=xxRMC 1=time 2=status(A/V) 3=lat 4=N/S 5=lon 6=E/W */
static void handle_rmc(char *f[], int nf)
{
    if (nf < 7 || f[2][0] != 'A') return;
    int32_t lat, lon;
    if (!parse_coord(f[3], f[4], false, &lat) ||
        !parse_coord(f[5], f[6], true,  &lon))
        return;
    s_fix.lat_e7 = lat;                   /* alt/sats keep last GGA values */
    s_fix.lon_e7 = lon;
    s_fix.fix         = true;
    s_fix.last_fix_ms = HAL_GetTick();
}

static void process_line(void)
{
    if (!nmea_checksum_ok(s_line, s_line_len)) return;
    s_locked = true;                      /* baud confirmed: stop autobaud cycling */

    char *f[GPS_NMEA_MAXF];
    int nf = split_fields(s_line + 1, f, GPS_NMEA_MAXF);   /* skip '$' */
    size_t tl = strlen(f[0]);             /* talker+type, e.g. "GPGGA"/"GNGGA" */
    if (tl < 3) return;
    const char *type = f[0] + (tl - 3);
    if      (memcmp(type, "GGA", 3) == 0) handle_gga(f, nf);
    else if (memcmp(type, "RMC", 3) == 0) handle_rmc(f, nf);
}

/* ================= UART control ================= */
static void gps_rx_enable(void)
{
    /* Clear any latched errors + stale byte before turning RX IRQ on. */
    USART6->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF | USART_ICR_PECF;
    (void)USART6->RDR;
    USART6->CR1 |= USART_CR1_RXNEIE;      /* NVIC already enabled by MspInit */
}

static int gps_set_baud(uint32_t baud)
{
    /* MspDeInit disables USART6 NVIC, so no ISR runs while state resets;
     * MspInit re-enables it. RXNEIE stays off until gps_rx_enable(). */
    HAL_UART_DeInit(&huart6);
    s_head = 0;
    s_tail = 0;
    s_line_len     = 0;
    s_in_sentence  = false;
    huart6.Init.BaudRate = baud;
    if (HAL_UART_Init(&huart6) != HAL_OK) return -1;
    gps_rx_enable();
    return 0;
}

/* ================= public API ================= */
int gps_init(void)
{
    if (huart6.Instance != USART6) return -1;   /* main.c must init first */
    s_head = 0;
    s_tail = 0;
    s_line_len    = 0;
    s_in_sentence = false;
    memset(&s_fix, 0, sizeof s_fix);
    s_locked   = false;
    s_baud_idx = 0;                        /* main.c already inited at 9600 */
    s_baud_t0  = HAL_GetTick();
    gps_rx_enable();
    return 0;
}

void gps_poll(void)
{
    uint32_t now = HAL_GetTick();

    /* Drain ring -> line assembler (bounded by ring size, non-blocking). */
    while (s_tail != s_head) {
        uint8_t b = s_ring[s_tail];
        s_tail = (uint16_t)((s_tail + 1u) & GPS_RING_MASK);
        if (b == '$') {                    /* sentence start (also resyncs) */
            s_line[0]     = '$';
            s_line_len    = 1;
            s_in_sentence = true;
        } else if (!s_in_sentence) {
            /* skip noise between sentences */
        } else if (b == '\r' || b == '\n') {
            s_line[s_line_len] = '\0';
            s_in_sentence = false;
            process_line();
        } else if (s_line_len < GPS_LINE_MAX - 1u && b >= 0x20u && b < 0x7Fu) {
            s_line[s_line_len++] = (char)b;
        } else {
            s_in_sentence = false;         /* overlong/binary garbage: abandon */
        }
    }

    /* Fix staleness: 5 s without a valid fix sentence => stale. */
    if (s_fix.fix && (now - s_fix.last_fix_ms) > GPS_FIX_STALE_MS)
        s_fix.fix = false;

    /* Autobaud: cycle every 2 s until a checksummed sentence locks it. */
    if (!s_locked && (now - s_baud_t0) >= GPS_AUTOBAUD_MS) {
        s_baud_idx = (uint8_t)((s_baud_idx + 1u) & 3u);
        (void)gps_set_baud(s_bauds[s_baud_idx]);   /* on failure, retry next cycle */
        s_baud_t0 = now;
    }
}

void gps_get(gps_fix_t *f)
{
    *f = s_fix;    /* s_fix mutated only in gps_poll (same superloop thread) */
}
