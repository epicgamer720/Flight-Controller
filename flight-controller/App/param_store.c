/* ============================================================
 * param_store.c — flash parameter persistence (sector 7).
 *
 * Append-record store in the last 128 KB flash sector @ 0x08060000
 * (F722 sectors: 0-3 16K, 4 64K, 5-7 128K). The linker script caps
 * FLASH LENGTH at 384K, so code can never overlap this sector —
 * an oversized image fails loudly at link time instead.
 *
 * Slot layout (16 bytes, little-endian, blank flash = all 0xFF):
 *   off  0  uint16 magic        0x5250 ("PR" on the wire)
 *   off  2  uint8  ver          1
 *   off  3  uint8  len          payload bytes = 8 (main_alt_cm + seq)
 *   off  4  uint32 main_alt_cm  the ONLY persisted parameter
 *   off  8  uint32 seq          monotonic; highest CRC-valid seq wins
 *   off 12  uint8  spare[2]     0x00 pad so crc16 lands at offset 14
 *   off 14  uint16 crc16        CRC16-CCITT over bytes 0..13
 *
 * Save = program the next blank slot (a few µs per word; the CPU
 * stalls briefly while executing from flash — acceptable on ground,
 * and saves are hard-gated to ground states). Erase happens only
 * when all 8192 slots are used: it blocks up to seconds, so the
 * IWDG is kicked before AND after (worst-case timeout ~2.8 s).
 * No cache maintenance needed: DCache is off (ICache only) and the
 * sector is data, never executed.
 *
 * WHAT MUST NEVER BE PERSISTED HERE (by design — do not "improve"):
 *  - test_enabled: must boot OFF, always (CLAUDE.md §2). A vehicle
 *    that powers up test-fire-enabled is a safety violation.
 *  - gyro bias: recalibrated stationary on the pad each session
 *    (thermal drift over the LoRa module makes stale bias wrong).
 *  - baro zero / sea-level ref: re-zeroed each pad session; a stale
 *    AGL reference corrupts every deploy altitude decision.
 * ============================================================ */
#include "app.h"
#include <string.h>

#define PARAM_BASE_ADDR    0x08060000UL          /* flash sector 7 */
#define PARAM_SECTOR       FLASH_SECTOR_7
#define PARAM_SECTOR_BYTES (128u * 1024u)
#define PARAM_SLOT_BYTES   16u
#define PARAM_NSLOTS       (PARAM_SECTOR_BYTES / PARAM_SLOT_BYTES)  /* 8192 */
#define PARAM_MAGIC        0x5250u
#define PARAM_VER          1u
#define PARAM_PAYLOAD_LEN  8u                    /* main_alt_cm + seq */

typedef struct __attribute__((packed)) {
    uint16_t magic;          /* PARAM_MAGIC */
    uint8_t  ver;            /* PARAM_VER */
    uint8_t  len;            /* PARAM_PAYLOAD_LEN */
    uint32_t main_alt_cm;
    uint32_t seq;
    uint8_t  spare[2];       /* 0x00 */
    uint16_t crc16;          /* CRC16-CCITT over bytes 0..13 */
} param_rec_t;
_Static_assert(sizeof(param_rec_t) == PARAM_SLOT_BYTES,
               "param record must be exactly one 16-byte slot");

static const volatile uint8_t *slot_addr(uint32_t idx)
{
    return (const volatile uint8_t *)(PARAM_BASE_ADDR +
                                      (idx * PARAM_SLOT_BYTES));
}

static bool slot_blank(uint32_t idx)
{
    const volatile uint8_t *p = slot_addr(idx);
    for (uint32_t i = 0; i < PARAM_SLOT_BYTES; i++) {
        if (p[i] != 0xFFu)
            return false;
    }
    return true;
}

/* CRC-validated read of one slot; returns false on any mismatch. */
static bool slot_valid(uint32_t idx, param_rec_t *out)
{
    param_rec_t r;
    memcpy(&r, (const void *)slot_addr(idx), sizeof r);
    if (r.magic != PARAM_MAGIC || r.ver != PARAM_VER ||
        r.len != PARAM_PAYLOAD_LEN)
        return false;
    if (crc16_ccitt((const uint8_t *)&r, sizeof r - 2u) != r.crc16)
        return false;
    if (out != NULL)
        *out = r;
    return true;
}

/* Scan the whole sector once: newest valid record (highest seq) and the
 * index of the last non-blank slot (append point = that + 1). A corrupt
 * or half-programmed slot just occupies space; we append after it. */
static void scan(bool *found, param_rec_t *best, int32_t *last_nonblank)
{
    *found = false;
    *last_nonblank = -1;
    for (uint32_t i = 0; i < PARAM_NSLOTS; i++) {
        if (slot_blank(i))
            continue;
        *last_nonblank = (int32_t)i;
        param_rec_t r;
        if (slot_valid(i, &r)) {
            if (!*found || (r.seq > best->seq)) {
                *best  = r;
                *found = true;
            }
        }
    }
}

/* Returns 0 and fills *main_alt_cm from the newest valid record;
 * -1 if the store holds no valid record (fresh/erased flash). */
int param_load(uint32_t *main_alt_cm)
{
    bool        found;
    param_rec_t best;
    int32_t     last_nonblank;

    scan(&found, &best, &last_nonblank);
    if (!found)
        return -1;
    if (main_alt_cm != NULL)
        *main_alt_cm = best.main_alt_cm;
    return 0;
}

static int prog_slot(uint32_t idx, const param_rec_t *rec)
{
    uint32_t addr = PARAM_BASE_ADDR + (idx * PARAM_SLOT_BYTES);
    uint32_t w[PARAM_SLOT_BYTES / 4u];
    memcpy(w, rec, sizeof w);

    if (HAL_FLASH_Unlock() != HAL_OK)
        return -2;
    int rc = 0;
    for (uint32_t i = 0; i < (PARAM_SLOT_BYTES / 4u); i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + (i * 4u),
                              (uint64_t)w[i]) != HAL_OK) {
            rc = -3;                    /* program error */
            break;
        }
    }
    (void)HAL_FLASH_Lock();

    if (rc == 0 &&
        memcmp((const void *)addr, rec, PARAM_SLOT_BYTES) != 0)
        rc = -4;                        /* readback verify failed */
    return rc;
}

static int erase_sector(void)
{
    FLASH_EraseInitTypeDef er;
    uint32_t bad = 0;

    er.TypeErase    = FLASH_TYPEERASE_SECTORS;
    er.Sector       = PARAM_SECTOR;
    er.NbSectors    = 1;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;   /* 2.7-3.6 V: x32 parallelism */

    if (HAL_FLASH_Unlock() != HAL_OK)
        return -2;
    wdg_refresh();                      /* erase blocks up to seconds; IWDG
                                           worst case ~2.8 s — kick both sides */
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&er, &bad);
    wdg_refresh();
    (void)HAL_FLASH_Lock();
    return (st == HAL_OK) ? 0 : -5;     /* erase error */
}

/* Persist main_alt_cm. Ground states ONLY (ST_INIT / ST_GROUND_IDLE /
 * ST_LANDED): programming stalls the CPU and a full-store erase can
 * block for seconds — never acceptable in ARMED or any flight state.
 * Returns 0 = saved; -1 = refused (in flight); -2..-5 = flash error. */
int param_save(uint32_t main_alt_cm)
{
    if (g_fsm.state != ST_INIT && g_fsm.state != ST_GROUND_IDLE &&
        g_fsm.state != ST_LANDED)
        return -1;

    bool        found;
    param_rec_t best;
    int32_t     last_nonblank;
    scan(&found, &best, &last_nonblank);

    param_rec_t rec;
    memset(&rec, 0, sizeof rec);
    rec.magic       = PARAM_MAGIC;
    rec.ver         = PARAM_VER;
    rec.len         = PARAM_PAYLOAD_LEN;
    rec.main_alt_cm = main_alt_cm;
    rec.seq         = found ? (best.seq + 1u) : 1u;
    rec.crc16       = crc16_ccitt((const uint8_t *)&rec, sizeof rec - 2u);

    uint32_t idx = (uint32_t)(last_nonblank + 1);
    if (idx < PARAM_NSLOTS) {
        int rc = prog_slot(idx, &rec);
        if (rc == 0)
            return 0;
        /* Append failed: corrupt/disturbed tail — recycle the sector. */
    }
    /* Sector full (8192 saves) or unappendable: erase, write slot 0. */
    int rc = erase_sector();
    if (rc != 0)
        return rc;
    return prog_slot(0, &rec);
}
