#pragma once
#include <stdint.h>
#include <stddef.h>
#include "crc16.h"

/* ============================================================
 * SINGLE source of truth for link params + packet structs.
 * Compiled identically by FC (STM32F722) and GS (RP2040).
 * Both little-endian ARM: packed structs transfer byte-for-byte.
 * ============================================================ */

/* ---- Link parameters (MUST match FC and GS) ---- */
#define LORA_FREQ_HZ      915000000UL
#define LORA_BW_KHZ       250.0f
#define LORA_SF           8
#define LORA_CR           5          /* 4/5 */
#define LORA_SYNC_WORD    0x12       /* RadioLib private */
#define LORA_PREAMBLE     8
#define LORA_TX_DBM       22
/* Wio-SX1262 module is TCXO-based (DIO3 supply). FC firmware falls back
 * to XTAL config automatically if TCXO init fails. */
#define LORA_TCXO_V       1.8f
#define LORA_TCXO_DELAYMS 5

/* ---- Protocol ---- */
#define LINK_MAGIC    0x52   /* 'R' */
#define PROTO_VERSION 3      /* v3: +seq, last_evt_state/count, main_alt_m echo (54 B) */
#define NUM_PYRO      1      /* confirmed from schematic: gate PB13 (Q2 AO3400A), sense PC1 */
#define MCU_TEMP_NODATA (-32768)  /* mcu_temp_c10 sentinel: die-temp read failed */

typedef enum {
    PKT_TELEMETRY = 0x01,
    PKT_EVENT     = 0x02,   /* state change / pyro fired / fault */
    PKT_COMMAND   = 0x10,   /* GS -> FC */
    PKT_ACK       = 0x11,
} packet_type_t;

typedef enum {
    ST_INIT = 0, ST_GROUND_IDLE, ST_ARMED, ST_BOOST, ST_COAST,
    ST_APOGEE, ST_DROGUE, ST_MAIN, ST_DESCENT, ST_LANDED, ST_FAULT
} flight_state_t;

/* status flags bitfield */
#define FLAG_GPS_FIX   (1u<<0)
#define FLAG_ACCEL_SAT (1u<<1)
#define FLAG_ARMED     (1u<<2)
#define FLAG_SD_OK     (1u<<3)
#define FLAG_CHG_OK    (1u<<4)   /* charger init healthy (BQ25883) */
#define FLAG_GPS_OK    (1u<<5)   /* GPS UART init healthy (module may still lack fix) */
#define FLAG_LOW_BATT  (1u<<6)   /* vbat below ARM_MIN_VBAT_MV */
#define FLAG_DEPLOY_FAIL (1u<<7) /* continuity persisted after fire+retry (charge did not go) */

typedef struct __attribute__((packed)) {
    uint8_t  magic;          /* LINK_MAGIC */
    uint8_t  version;        /* PROTO_VERSION */
    uint8_t  type;           /* packet_type_t */
    uint8_t  state;          /* flight_state_t */
    uint32_t t_ms;           /* ms since boot */
    int32_t  lat_e7;         /* deg * 1e7 */
    int32_t  lon_e7;         /* deg * 1e7 */
    int32_t  alt_baro_cm;    /* AGL, cm */
    int32_t  alt_gps_cm;     /* MSL, cm */
    int16_t  vel_up_cms;     /* vertical, cm/s */
    int16_t  accel_g_x100[3];/* g*100; may rail at +/-1600 (16 g) */
    int16_t  gyro_dps_x10[3];/* deg/s *10 */
    uint16_t batt_mv;
    uint8_t  pyro_cont;      /* continuity bitfield, 1 = continuity */
    uint8_t  pyro_fired;     /* fired bitfield */
    uint8_t  sats;
    uint8_t  flags;          /* FLAG_* */
    int16_t  mcu_temp_c10;   /* STM32 internal die temp, deg C * 10; MCU_TEMP_NODATA if unread */
    uint16_t seq;            /* v3: TX frame counter (wraps) -> host packet-delivery ratio */
    uint8_t  last_evt_state; /* v3: flight_state_t of the most recent PKT_EVENT (echo) */
    uint8_t  last_evt_count; /* v3: total PKT_EVENTs emitted (wraps) -> missed-event detect */
    uint16_t main_alt_m;     /* v3: configured main-deploy AGL (m), read-back before arm */
    uint16_t crc16;          /* CRC16-CCITT over all preceding bytes */
} telem_packet_t;

typedef enum {
    CMD_NOP = 0, CMD_ARM, CMD_DISARM, CMD_TEST_FIRE,
    CMD_SET_MAIN_ALT, CMD_ZERO_BARO, CMD_REBOOT, CMD_ENTER_BOOTLOADER,
    CMD_SET_TX_POWER   /* arg = signed dBm (-9..22); pad-only via RX window */
} command_id_t;

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  type;     /* PKT_COMMAND */
    uint8_t  cmd;      /* command_id_t */
    uint32_t arg;      /* e.g. main altitude cm, or TEST_FIRE passcode+channel */
    uint32_t nonce;    /* anti-replay; FC rejects repeats */
    uint16_t crc16;
} command_packet_t;

/* RX validation: check magic, version, then crc16_ccitt(buf, len-2) == crc.
 * CMD_TEST_FIRE: only in ST_GROUND_IDLE + test-enabled + passcode/channel in
 * arg + fresh nonce. */
