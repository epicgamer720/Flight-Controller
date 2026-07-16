/* ============================================================
 * datalog.c: SD binary logging (CLAUDE.md §5.5)
 * 68-byte fixed records -> LOGnnn.BIN via FatFs; human-readable
 * events -> LOGnnn.TXT sidecar. A byte ring decouples the 200 Hz
 * control loop (datalog_push) from SD write latency (datalog_poll,
 * superloop). Any SD/FS error flips sd_ok=false and every entry
 * point becomes a no-op, since logging must never block or crash the
 * control loop.
 *
 * Alignment invariant: ring capacity and write chunk are exact
 * multiples of sizeof(log_record_t), so head/tail are always
 * record-aligned, drop-oldest discards exactly one whole record,
 * and the file only ever receives whole records.
 * ============================================================ */
#include "app.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include <string.h>
#include <stdio.h>

#define REC_SZ          sizeof(log_record_t)                 /* 68 */
#define RING_CAP        ((LOG_RING_BYTES / REC_SZ) * REC_SZ) /* 65484: whole records */
#define LOG_CHUNK_BYTES (60U * REC_SZ)                       /* 4080 ~= 8 SD sectors */

static FATFS    s_fs;
static FIL      s_bin, s_txt;
static char     s_path[4];      /* "0:/" filled by FATFS_LinkDriver */
static bool     s_linked   = false;
static bool     s_sd_ok    = false;
static bool     s_bin_open = false;
static bool     s_txt_open = false;
static bool     s_dirty    = false;   /* unsynced data in LOGnnn.BIN */
static uint32_t s_drops    = 0;       /* records lost to ring overflow */
static uint32_t s_hw_used  = 0;       /* ring occupancy high-water, bytes */
static uint32_t s_last_write_ms, s_last_sync_ms;

/* SPSC byte ring: producer datalog_push (control loop), consumer
 * datalog_poll/close (superloop). Free-running indices; used = head-tail. */
static uint8_t           s_ring[LOG_RING_BYTES];
static volatile uint32_t s_head, s_tail;

static void dl_fail(void)
{
  s_sd_ok = false;              /* everything no-ops from here on */
}

/* Write one contiguous, record-aligned run from the ring to LOGnnn.BIN.
 * Returns bytes written, 0 if ring empty, -1 on error (sd_ok cleared). */
static int dl_write_chunk(void)
{
  uint32_t used = s_head - s_tail;
  if (used == 0)
    return 0;

  uint32_t off = s_tail % RING_CAP;
  uint32_t n   = used;                  /* used, off, caps: all multiples of REC_SZ */
  if (n > LOG_CHUNK_BYTES) n = LOG_CHUNK_BYTES;
  if (n > RING_CAP - off)  n = RING_CAP - off;

  UINT bw = 0;
  if (f_write(&s_bin, &s_ring[off], (UINT)n, &bw) != FR_OK || bw != (UINT)n) {
    dl_fail();
    return -1;
  }
  s_tail += n;
  s_dirty = true;
  return (int)n;
}

int datalog_init(void)
{
  s_sd_ok = false;
  s_bin_open = s_txt_open = false;
  s_dirty = false;
  s_head = s_tail = 0;
  s_drops = 0;
  s_hw_used = 0;

  /* Card detect: internal pull-up, switch to GND when inserted -> HIGH = no card. */
  if (HAL_GPIO_ReadPin(SD_CD_PORT, SD_CD_PIN) == GPIO_PIN_SET)
    return -1;                          /* no card: quiet, sd_ok stays false */

  if (s_linked) {                       /* re-init: force a fresh disk_initialize */
    (void)f_mount(NULL, s_path, 0);
    (void)FATFS_UnLinkDriver(s_path);
    s_linked = false;
  }
  if (FATFS_LinkDriver(&SD_Driver, s_path) != 0)
    return -2;
  s_linked = true;

  if (f_mount(&s_fs, s_path, 1) != FR_OK)   /* forced mount -> disk_initialize */
    return -3;

  /* Next free LOGnnn slot, nnn = 000..999. */
  char bin_name[24], txt_name[24];
  int idx = -1;
  for (int n = 0; n < 1000; n++) {
    snprintf(bin_name, sizeof bin_name, "0:/LOG%03d.BIN", n);
    FRESULT fr = f_stat(bin_name, NULL);
    if (fr == FR_NO_FILE) { idx = n; break; }
    if (fr != FR_OK)
      return -4;                        /* disk/FS error during scan */
  }
  if (idx < 0)
    return -5;                          /* all 1000 slots used */
  snprintf(txt_name, sizeof txt_name, "0:/LOG%03d.TXT", idx);

  if (f_open(&s_bin, bin_name, FA_WRITE | FA_CREATE_NEW) != FR_OK)
    return -6;
  s_bin_open = true;

  if (f_open(&s_txt, txt_name, FA_WRITE | FA_CREATE_NEW) != FR_OK) {
    (void)f_close(&s_bin);
    s_bin_open = false;
    return -7;
  }
  s_txt_open = true;

  uint32_t now = HAL_GetTick();
  s_last_write_ms = now;
  s_last_sync_ms  = now;
  s_sd_ok = true;

  char msg[32];
  snprintf(msg, sizeof msg, "log open LOG%03d", idx);
  datalog_event(msg);
  return 0;
}

void datalog_push(const log_record_t *r)
{
  if (!s_sd_ok || !s_bin_open || r == NULL)
    return;

  /* Caller passes crc=0; seal record: CRC16-CCITT over first 66 bytes. */
  log_record_t rec = *r;
  rec.crc16 = crc16_ccitt((const uint8_t *)&rec, REC_SZ - 2U);

  uint32_t used = s_head - s_tail;
  if ((RING_CAP - used) < REC_SZ) {
    s_tail += REC_SZ;                   /* drop-oldest: exactly one whole record */
    s_drops++;
  }

  /* head is record-aligned and RING_CAP is a record multiple, so a record
   * never wraps; single contiguous copy. */
  memcpy(&s_ring[s_head % RING_CAP], &rec, REC_SZ);
  s_head += REC_SZ;

  uint32_t occ = s_head - s_tail;       /* occupancy high-water for log stat */
  if (occ > s_hw_used)
    s_hw_used = occ;
}

void datalog_poll(uint32_t now_ms)
{
  if (!s_sd_ok || !s_bin_open)
    return;

  bool due   = (now_ms - s_last_write_ms) >= LOG_FLUSH_MS;
  bool wrote = false;
  int  chunks = 0;
  for (;;) {
    uint32_t used = s_head - s_tail;
    if (!(used >= LOG_CHUNK_BYTES || (due && used > 0)))
      break;
    if (dl_write_chunk() <= 0)
      return;                           /* error (sd_ok cleared) or empty */
    wrote = true;
    if (++chunks >= 2)                  /* bound one pass's SD time (IWDG);
                                           the ring absorbs the backlog */
      break;
  }
  if (wrote)
    s_last_write_ms = now_ms;

  if ((now_ms - s_last_sync_ms) >= LOG_FLUSH_MS) {
    s_last_sync_ms = now_ms;
    if (s_dirty) {
      if (f_sync(&s_bin) != FR_OK) {
        dl_fail();
        return;
      }
      s_dirty = false;
    }
  }

  /* Rebase the free-running indices long before uint32 wrap: 2^32 is not
   * a multiple of RING_CAP, so wrapping would break record alignment
   * (~87 h of continuous logging). k is a RING_CAP multiple, so every
   * offset (idx % RING_CAP) is preserved. Producer and consumer both run
   * in the superloop thread, so it's safe to adjust both here. */
  if (s_tail >= 0x40000000u) {
    uint32_t k = (s_tail / RING_CAP) * RING_CAP;
    s_head -= k;
    s_tail -= k;
  }
}

/* FAULT-entry flush (CLAUDE.md §5.4 keeps logging in FAULT: files stay
 * OPEN; close happens on LANDED only). Bounded drain: at most 4 chunks so
 * a slow card cannot stall the control loop, then f_sync BOTH files so
 * everything already written survives a power loss. */
void datalog_flush(void)
{
  if (!s_sd_ok)
    return;

  if (s_bin_open) {
    for (int chunks = 0; chunks < 4; chunks++) {
      if (dl_write_chunk() <= 0)
        break;                          /* ring empty, or error (sd_ok cleared) */
    }
    if (!s_sd_ok)
      return;
    if (f_sync(&s_bin) != FR_OK) {
      dl_fail();
      return;
    }
    s_dirty = false;
  }
  if (s_txt_open) {
    if (f_sync(&s_txt) != FR_OK)
      dl_fail();
  }
}

void datalog_event(const char *msg)
{
  if (!s_sd_ok || !s_txt_open || msg == NULL)
    return;

  char line[128];
  int n = snprintf(line, sizeof line, "[%10lu] %s\n",
                   (unsigned long)HAL_GetTick(), msg);
  if (n < 0)
    return;
  if (n >= (int)sizeof line)
    n = (int)sizeof line - 1;           /* truncated */

  UINT bw = 0;
  if (f_write(&s_txt, line, (UINT)n, &bw) != FR_OK || bw != (UINT)n) {
    dl_fail();
    return;
  }
  if (f_sync(&s_txt) != FR_OK)
    dl_fail();
}

void datalog_close(void)
{
  if (!s_bin_open && !s_txt_open)
    return;

  if (s_sd_ok) {
    while ((s_head - s_tail) > 0) {     /* drain ring fully */
      if (dl_write_chunk() <= 0)
        break;                          /* error already flagged */
    }
    if (s_sd_ok && s_drops > 0) {
      char msg[48];
      snprintf(msg, sizeof msg, "ring overflow: %lu records dropped",
               (unsigned long)s_drops);
      datalog_event(msg);
    }
    if (s_sd_ok)
      datalog_event("log closed");
  }

  if (s_bin_open) {
    if (f_close(&s_bin) != FR_OK)
      dl_fail();
    s_bin_open = false;
  }
  if (s_txt_open) {
    if (f_close(&s_txt) != FR_OK)
      dl_fail();
    s_txt_open = false;
  }
  /* sd_ok intentionally stays true after a clean close (FLAG_SD_OK remains
   * truthful post-landing); push/poll no-op via the open flags until re-init. */
}

bool datalog_card_present(void)
{
  /* CD switch closes to GND when a card is seated: LOW = present. */
  return HAL_GPIO_ReadPin(SD_CD_PORT, SD_CD_PIN) == GPIO_PIN_RESET;
}

bool datalog_ok(void)
{
  return s_sd_ok;
}

void datalog_stats(uint32_t *drops, uint32_t *hw_bytes, uint32_t *cap_bytes)
{
  if (drops != NULL)
    *drops = s_drops;
  if (hw_bytes != NULL)
    *hw_bytes = s_hw_used;
  if (cap_bytes != NULL)
    *cap_bytes = RING_CAP;
}
