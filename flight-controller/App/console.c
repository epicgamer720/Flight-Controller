/* ============================================================
 * console.c: line-oriented USB-CDC command console.
 *
 * RX path: usbd_cdc_if RX callback (OTG_FS ISR) -> console_rx()
 *          -> 1 KB SPSC ring -> console_poll() assembles lines.
 * TX path: console_write()/console_printf() -> 2 KB ring ->
 *          console_poll() drains via cdc_transmit(); silently
 *          dropped while the host is unplugged/unconfigured.
 * Everything non-blocking except the deliberate HAL_Delay(50)
 * before entering the DFU bootloader.
 *
 * SAFETY (CLAUDE.md §2): "fire" is honored ONLY when
 *   state == ST_GROUND_IDLE && !armed && test_enabled &&
 *   passcode == TEST_FIRE_PASSCODE, mirroring CMD_TEST_FIRE.
 * ============================================================ */
#include "app.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

/* ---- rings (power-of-two sizes) ---- */
#define RX_RING   1024U
#define TX_RING   2048U
#define LINE_MAX  96U

static volatile uint8_t  rx_ring[RX_RING];
static volatile uint16_t rx_head;      /* producer: OTG_FS ISR */
static volatile uint16_t rx_tail;      /* consumer: console_poll */

static volatile uint8_t  tx_ring[TX_RING];
static volatile uint16_t tx_head;      /* producer: console_write (superloop) */
static volatile uint16_t tx_tail;      /* consumer: console_poll (superloop) */

static char     line_buf[LINE_MAX];
static uint16_t line_len;

/* ============================================================
 * Fixed-point float formatting (independent of printf float support).
 * Rotating static buffers so several can appear in one printf call.
 * ============================================================ */
static const char *ff(float v)
{
  static char bufs[6][16];
  static unsigned bi;
  char *b = bufs[bi++ % 6U];
  int32_t s = (int32_t)(v * 100.0f + ((v >= 0.0f) ? 0.5f : -0.5f));
  int32_t ip = s / 100, fp = s % 100;
  if (fp < 0) fp = -fp;
  snprintf(b, 16, "%s%ld.%02ld", ((s < 0) && (ip == 0)) ? "-" : "",
           (long)ip, (long)fp);
  return b;
}

/* ============================================================
 * TX ring -> CDC
 * ============================================================ */
static void tx_drain(void)
{
  uint8_t chunk[256];

  if (!usb_cdc_connected())
  {
    tx_tail = tx_head;               /* host gone: drop everything */
    return;
  }
  while (tx_tail != tx_head)
  {
    uint16_t n = 0, t = tx_tail;
    while ((t != tx_head) && (n < sizeof(chunk)))
    {
      chunk[n++] = tx_ring[t];
      t = (uint16_t)((t + 1U) & (TX_RING - 1U));
    }
    if (cdc_transmit(chunk, n) != 0)
    {
      break;                         /* busy: retry next poll, keep data */
    }
    tx_tail = t;
  }
}

int console_write(const void *buf, uint16_t len)
{
  const uint8_t *p = (const uint8_t *)buf;

  if (!usb_cdc_connected())
  {
    return 0;                        /* silent drop per contract */
  }
  for (uint16_t i = 0; i < len; i++)
  {
    uint16_t nh = (uint16_t)((tx_head + 1U) & (TX_RING - 1U));
    if (nh == tx_tail)
    {
      tx_drain();                    /* one non-blocking recovery attempt */
      nh = (uint16_t)((tx_head + 1U) & (TX_RING - 1U));
      if (nh == tx_tail)
      {
        return -1;                   /* still full: drop remainder */
      }
    }
    tx_ring[tx_head] = p[i];
    tx_head = nh;
  }
  return 0;
}

void console_printf(const char *fmt, ...)
{
  char buf[256];
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0)
  {
    return;
  }
  if (n > (int)sizeof(buf) - 1)
  {
    n = (int)sizeof(buf) - 1;
  }
  (void)console_write(buf, (uint16_t)n);
}

/* Best-effort flush before reboot/bootloader (bounded, HAL_GetTick). */
static void tx_flush(uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();
  while ((tx_tail != tx_head) && ((HAL_GetTick() - t0) < timeout_ms))
  {
    if (!usb_cdc_connected())
    {
      break;
    }
    tx_drain();
  }
}

/* ============================================================
 * RX from USB ISR: SPSC producer, never blocks
 * ============================================================ */
void console_rx(const uint8_t *buf, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++)
  {
    uint16_t nh = (uint16_t)((rx_head + 1U) & (RX_RING - 1U));
    if (nh == rx_tail)
    {
      return;                        /* ring full: drop, never block ISR */
    }
    rx_ring[rx_head] = buf[i];
    rx_head = nh;
  }
}

/* ============================================================
 * Commands
 * ============================================================ */
static void cmd_help(int argc, char **argv)
{
  (void)argc; (void)argv;
  /* Split across several console_printf calls: one big string overflows the
   * 256-byte vsnprintf buffer and the later commands never print. */
  console_printf(
    "commands:\r\n"
    "  help                    this text\r\n"
    "  status                  state/armed/alt/vel/batt/flags/health\r\n"
    "  preflight               GO/NO-GO checklist before arming\r\n"
    "  sensors                 raw IMU + baro\r\n"
    "  gps                     last GPS fix\r\n"
    "  servo <1-4> <500-2500>  set servo pulse (us)\r\n");
  console_printf(
    "  arm | disarm            arming gate / disarm\r\n"
    "  testen <on|off>         pad test mode (GROUND_IDLE only)\r\n"
    "  fire <ch> <hex-code>    pad test fire (testen + passcode)\r\n"
    "  zero                    zero baro AGL\r\n"
    "  mainalt <m>             set main deploy altitude\r\n"
    "  apogee <ms>             set apogee/backup deploy timer (per-motor)\r\n"
    "  log <stat|start|stop>   SD logging\r\n"
    "  cal                     start pad gyro bias cal\r\n");
  console_printf(
    "  radio                   radio health\r\n"
    "  radiodbg                raw SX1262 hardware probe\r\n"
    "  tx [power|mon|cw|frame] transmit dashboard / control\r\n"
    "  i2cscan                 scan I2C bus for devices\r\n"
    "  charge                  charger status\r\n"
    "  bootloader              reboot into USB DFU\r\n"
    "  reboot                  reset MCU\r\n"
    "  wdtest                  hang on purpose (IWDG must reset us)\r\n"
    "  sleep [s]               STOP low-power; 0/none = until RESET\r\n"
    "  led <r> <g> <b>|auto    WS2812 bench test (needs 5 V rail)\r\n");
}

static void cmd_preflight(int argc, char **argv)
{
  int fail = 0, warn = 0;
  (void)argc; (void)argv;
  /* Aggregate existing health getters into a GO/NO-GO checklist. Pure
   * read-out; no sensor is pulsed or re-read. Hard checks gate arming;
   * WARN items (radio/sd/gps) still allow an SD-logged flight. */
  #define CK(cond, name) do { \
      console_printf("  [%s] %s\r\n", (cond) ? "PASS" : "FAIL", name); \
      if (!(cond)) fail++; } while (0)
  #define WK(cond, name) do { \
      console_printf("  [%s] %s\r\n", (cond) ? "PASS" : "WARN", name); \
      if (!(cond)) warn++; } while (0)

  console_printf("preflight (%s):\r\n", fsm_state_name(g_fsm.state));
  CK(g_fsm.imu_ok,                          "imu healthy");
  CK(g_fsm.baro_ok,                         "baro healthy");
  CK(imu_gyro_cal_done(),                   "gyro cal done");
  CK(pyro_continuity(0),                    "pyro continuity");
  CK(g_fsm.chg.vbat_mv >= ARM_MIN_VBAT_MV,  "battery ok");
  CK(!pyro_deploy_failed(),                 "no prior deploy-fail");
  CK(!g_fsm.test_enabled,                   "test mode OFF");
  WK(g_fsm.radio_ok,                        "radio");
  WK(datalog_ok(),                          "sd log");
  WK(g_fsm.gps.fix,                         "gps fix");
  console_printf("  batt=%u mV  cont=0x%02X  sats=%u\r\n",
                 g_fsm.chg.vbat_mv, pyro_cont_bits(), g_fsm.gps.sats);
  console_printf("==> %s%s\r\n", fail ? "NO-GO" : "GO",
                 warn ? " (warnings)" : "");
  #undef CK
  #undef WK
}

static void cmd_status(int argc, char **argv)
{
  (void)argc; (void)argv;
  console_printf("state:   %s  t=%lums\r\n",
                 fsm_state_name(g_fsm.state), (unsigned long)HAL_GetTick());
  console_printf("armed:   %d  testen: %d  main_alt: %s m\r\n",
                 g_fsm.armed, g_fsm.test_enabled, ff(g_fsm.main_alt_m));
  console_printf("alt:     %s m AGL  vel: %s m/s\r\n",
                 ff(g_fsm.agl_m), ff(g_fsm.vel_ms));
  console_printf("batt:    %u mV\r\n", g_fsm.chg.vbat_mv);
  {
    float tc;
    if (mcu_temp_read(&tc) == 0)
      console_printf("mcu:     %s C (die)\r\n", ff(tc));
  }
  console_printf("flags:   gps_fix=%d accel_sat=%d\r\n",
                 g_fsm.gps.fix, g_fsm.imu.accel_sat);
  console_printf("health:  imu=%d baro=%d sd=%d radio=%d chg=%d gps=%d\r\n",
                 g_fsm.imu_ok, g_fsm.baro_ok, g_fsm.sd_ok, g_fsm.radio_ok,
                 g_fsm.chg_ok, g_fsm.gps_ok);
  console_printf("pyro:    cont=0x%02X fired=0x%02X sense=%u mV%s\r\n",
                 pyro_cont_bits(), pyro_fired_bits(), pyro_sense_mv(0),
                 pyro_deploy_failed() ? "  DEPLOY-FAIL" : "");
}

static void cmd_sensors(int argc, char **argv)
{
  imu_sample_t s;
  float press = 0.0f, temp = 0.0f;
  (void)argc; (void)argv;

  if (imu_read(&s) == 0)
  {
    console_printf("imu:  ax=%s ay=%s az=%s g  sat=%d\r\n",
                   ff(s.ax), ff(s.ay), ff(s.az), s.accel_sat);
    console_printf("      gx=%s gy=%s gz=%s dps\r\n",
                   ff(s.gx), ff(s.gy), ff(s.gz));
  }
  else
  {
    console_printf("imu:  read error\r\n");
  }
  if (baro_read(&press, &temp) == 0)
  {
    console_printf("baro: %s Pa  %s C  alt=%s m\r\n",
                   ff(press), ff(temp), ff(baro_altitude_m(press)));
  }
  else
  {
    console_printf("baro: read error\r\n");
  }
}

static void cmd_gps(int argc, char **argv)
{
  gps_fix_t f;
  (void)argc; (void)argv;
  gps_get(&f);
  console_printf("gps: fix=%d sats=%u\r\n", f.fix, f.sats);
  console_printf("     lat=%ld.%07ld lon=%ld.%07ld alt=%ld.%02ld m MSL\r\n",
                 (long)(f.lat_e7 / 10000000), labs((long)(f.lat_e7 % 10000000)),
                 (long)(f.lon_e7 / 10000000), labs((long)(f.lon_e7 % 10000000)),
                 (long)(f.alt_msl_cm / 100),  labs((long)(f.alt_msl_cm % 100)));
  console_printf("     last_fix=%lu ms ago\r\n",
                 (unsigned long)(f.fix ? (HAL_GetTick() - f.last_fix_ms) : 0U));
  {
    /* Persisted last-landing fix (recovery aid): survives power cycles. */
    params_t pp;
    if ((param_load(&pp) == 0) && (pp.last_lat_e7 != 0 || pp.last_lon_e7 != 0))
      console_printf("     last landing: lat=%ld.%07ld lon=%ld.%07ld\r\n",
                     (long)(pp.last_lat_e7 / 10000000), labs((long)(pp.last_lat_e7 % 10000000)),
                     (long)(pp.last_lon_e7 / 10000000), labs((long)(pp.last_lon_e7 % 10000000)));
  }
}

static void cmd_servo(int argc, char **argv)
{
  unsigned long idx, us;
  if (argc < 3)
  {
    console_printf("usage: servo <1-4> <500-2500>\r\n");
    return;
  }
  idx = strtoul(argv[1], NULL, 10);
  us  = strtoul(argv[2], NULL, 10);
  if ((idx < 1U) || (idx > (unsigned long)NUM_SERVO) || (us < 500U) || (us > 2500U))
  {
    console_printf("err: servo <1-4> <500-2500>\r\n");
    return;
  }
  servo_set_us((uint8_t)(idx - 1U), (uint16_t)us);
  console_printf("servo %lu -> %u us\r\n", idx, servo_get_us((uint8_t)(idx - 1U)));
}

static void cmd_arm(int argc, char **argv)
{
  (void)argc; (void)argv;
  int r = fsm_request_arm(false);
  if (r == 0)
  {
    console_printf("ARMED\r\n");
  }
  else
  {
    console_printf("arm refused (%d): %s\r\n", r,
      r == -1 ? "not in GROUND_IDLE" :
      r == -2 ? "imu/baro unhealthy" :
      r == -3 ? "gyro cal not done (run 'cal', keep still)" :
      r == -4 ? "not still (velocity)" :
      r == -5 ? "accel implausible / saturated" :
      r == -6 ? "no pyro continuity" :
      r == -7 ? "battery too low" :
      r == -8 ? "not vertical (orientation)" : "unknown");
  }
}

static void cmd_disarm(int argc, char **argv)
{
  (void)argc; (void)argv;
  fsm_disarm();
  console_printf("disarmed\r\n");
}

static void cmd_testen(int argc, char **argv)
{
  if (argc < 2)
  {
    console_printf("testen: %s\r\n", g_fsm.test_enabled ? "on" : "off");
    return;
  }
  if (strcmp(argv[1], "on") == 0)
  {
    /* CLAUDE.md §2: test mode is pad-only */
    if (g_fsm.state != ST_GROUND_IDLE)
    {
      console_printf("err: testen only in GROUND_IDLE\r\n");
      return;
    }
    g_fsm.test_enabled = true;
    console_printf("test mode ON, pyro test fire enabled on pad\r\n");
  }
  else if (strcmp(argv[1], "off") == 0)
  {
    g_fsm.test_enabled = false;
    console_printf("test mode off\r\n");
  }
  else
  {
    console_printf("usage: testen <on|off>\r\n");
  }
}

static void cmd_fire(int argc, char **argv)
{
  unsigned long ch, code;
  if (argc < 3)
  {
    console_printf("usage: fire <ch> <hex-passcode>\r\n");
    return;
  }
  ch   = strtoul(argv[1], NULL, 10);
  code = strtoul(argv[2], NULL, 16);

  /* Mirror CLAUDE.md §2 CMD_TEST_FIRE gating: ALL of these, no exceptions: */
  if (g_fsm.state != ST_GROUND_IDLE)
  {
    console_printf("DENIED: not in GROUND_IDLE\r\n");
    return;
  }
  if (g_fsm.armed)
  {
    console_printf("DENIED: disarm first (test fire is disarmed-but-test-enabled only)\r\n");
    return;
  }
  if (!g_fsm.test_enabled)
  {
    console_printf("DENIED: test mode off (use: testen on)\r\n");
    return;
  }
  if ((code & 0xFFFFU) != (unsigned long)TEST_FIRE_PASSCODE)
  {
    console_printf("DENIED: bad passcode\r\n");
    return;
  }
  if ((ch < 1U) || (ch > (unsigned long)NUM_PYRO))
  {
    console_printf("DENIED: channel 1..%d\r\n", NUM_PYRO);
    return;
  }
  console_printf("TEST FIRE ch%lu (cont=0x%02X)...\r\n", ch, pyro_cont_bits());
  if (pyro_fire((uint8_t)(ch - 1U)) == 0)
  {
    console_printf("fire pulse started (%d ms)\r\n", FIRE_PULSE_MS);
    telem_event((uint8_t)g_fsm.state);  /* post-decision: PKT_EVENT carries fired bits */
  }
  else
  {
    console_printf("pyro_fire refused\r\n");
  }
}

static void cmd_zero(int argc, char **argv)
{
  (void)argc; (void)argv;
  baro_zero();
  console_printf("baro zeroed: AGL = 0\r\n");
}

static void cmd_mainalt(int argc, char **argv)
{
  float m;
  if (argc < 2)
  {
    console_printf("main_alt: %s m\r\n", ff(g_fsm.main_alt_m));
    return;
  }
  m = strtof(argv[1], NULL);
  if ((m < 10.0f) || (m > 10000.0f))
  {
    console_printf("err: 10..10000 m\r\n");
    return;
  }
  g_fsm.main_alt_m = m;
  console_printf("main_alt = %s m\r\n", ff(g_fsm.main_alt_m));
  {
    /* Persist (ground states only; param_save gates internally). */
    params_t pp;
    fsm_params_snapshot(&pp);
    int rc = param_save(&pp);
    if (rc == 0)
    {
      console_printf("saved to flash\r\n");
    }
    else
    {
      console_printf("flash save FAILED (%d)\r\n", rc);
    }
  }
}

static void cmd_apogee(int argc, char **argv)
{
  unsigned long ms;
  if (argc < 2)
  {
    console_printf("apogee_timeout: %lu ms (per-motor backup deploy timer)\r\n",
                   (unsigned long)g_fsm.apogee_timeout_ms);
    return;
  }
  ms = strtoul(argv[1], NULL, 10);
  /* floor = MAX_BURN_MS: the backup deploy uses this timer and fires regardless
   * of state, so a value during boost would deploy under thrust. */
  if ((ms < (unsigned long)MAX_BURN_MS) || (ms > 120000UL))
  {
    console_printf("err: %d..120000 ms (must be past burnout)\r\n", MAX_BURN_MS);
    return;
  }
  g_fsm.apogee_timeout_ms = (uint32_t)ms;
  console_printf("apogee_timeout = %lu ms\r\n", ms);
  {
    params_t pp;
    fsm_params_snapshot(&pp);
    int rc = param_save(&pp);
    console_printf(rc == 0 ? "saved to flash\r\n" : "flash save FAILED (%d)\r\n", rc);
  }
}

static void cmd_log(int argc, char **argv)
{
  if (argc < 2)
  {
    console_printf("usage: log <stat|start|stop>\r\n");
    return;
  }
  if (strcmp(argv[1], "stat") == 0)
  {
    uint32_t drops, hw, cap;
    console_printf("log: %s\r\n", datalog_ok() ? "running" : "not running");
    datalog_stats(&drops, &hw, &cap);
    console_printf("  drops: %lu  ring high-water: %lu/%lu bytes\r\n",
                   (unsigned long)drops, (unsigned long)hw, (unsigned long)cap);
  }
  else if (strcmp(argv[1], "start") == 0)
  {
    if (datalog_ok())
    {
      console_printf("log: already running\r\n");
    }
    else
    {
      int rc = datalog_init();
      if (rc == 0)
        console_printf("log: started\r\n");
      else
        console_printf("log: start FAILED rc=%d "
                       "(-1 no card, -2 link, -3 mount, -4 scan, -5 slots, -6/-7 open)\r\n",
                       rc);
    }
  }
  else if (strcmp(argv[1], "stop") == 0)
  {
    datalog_close();
    console_printf("log: closed\r\n");
  }
  else
  {
    console_printf("usage: log <stat|start|stop>\r\n");
  }
}

static void cmd_cal(int argc, char **argv)
{
  (void)argc; (void)argv;
  imu_gyro_cal_start();
  console_printf("gyro cal started, keep vehicle still (~2 s)\r\n");
}

static void cmd_radio(int argc, char **argv)
{
  int et, ex;
  (void)argc; (void)argv;
  radio_debug(&et, &ex);
  console_printf("radio: %s  tcxo: %s\r\n",
                 radio_ok() ? "ok" : "FAIL",
                 radio_using_tcxo() ? "yes" : "no (xtal fallback)");
  console_printf("  init err: tcxo=%d xtal=%d (0=ok; -1 rst/busy, -3 no chip, -7/-21 dev errors)\r\n",
                 et, ex);
  {
    uint32_t txto, reinit;
    telem_debug(&txto, &reinit);
    console_printf("  reinit attempts: %lu  tx timeouts: %lu\r\n",
                   (unsigned long)reinit, (unsigned long)txto);
  }
  if (radio_ok() && !(LORA_BUSY_PORT->IDR & LORA_BUSY_PIN))
  { /* raw GetIrqStatus (non-clearing): pending bits the chip has latched.
     * TxDone pending here + DIO1 pin low = the DIO1 line is open.
     * Skipped while BUSY is high, since commanding a busy/half-dead chip
     * raw from console context is asking for trouble. */
    uint8_t tx[4] = { 0x12, 0x00, 0x00, 0x00 }, rx[4] = { 0 };
    if (spi_bus_txrx(LORA_CS_PORT, LORA_CS_PIN, tx, rx, 4) == 0)
      console_printf("  chip irq status: 0x%02X%02X (bit0 TxDone, bit1 RxDone, bit9 Timeout)\r\n",
                     rx[2], rx[3]);
  }
  console_printf("  BUSY pin: %d  DIO1 pin: %d\r\n",
                 (LORA_BUSY_PORT->IDR & LORA_BUSY_PIN) ? 1 : 0,
                 (LORA_IRQ_PORT->IDR & LORA_IRQ_PIN) ? 1 : 0);
}

/* Raw hardware probe of the Wio-SX1262: reset-pin/BUSY choreography plus
 * raw SPI bytes, to localize bad module pads (power, BUSY, NSS, MISO). */
static void cmd_radiodbg(int argc, char **argv)
{
  (void)argc; (void)argv;
  #define BUSY_IN() ((LORA_BUSY_PORT->IDR & LORA_BUSY_PIN) ? 1 : 0)

  console_printf("busy idle: %d\r\n", BUSY_IN());

  HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  console_printf("busy during reset (expect 1 on live chip): %d\r\n", BUSY_IN());
  HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);

  uint32_t t0 = HAL_GetTick();
  uint8_t b1 = BUSY_IN();
  while (BUSY_IN() && (HAL_GetTick() - t0) < 100U) { }
  console_printf("busy 0ms after release: %d -> low after %lu ms (timeout=100)\r\n",
                 b1, (unsigned long)(HAL_GetTick() - t0));

  uint8_t tx1[2] = { 0xC0, 0x00 }, rx1[2] = { 0xAA, 0xAA };
  int r = spi_bus_txrx(LORA_CS_PORT, LORA_CS_PIN, tx1, rx1, 2);
  console_printf("GetStatus  spi=%d rx: %02X %02X  (00/FF = MISO dead)\r\n",
                 r, rx1[0], rx1[1]);

  /* ReadRegister 0x0740/41: LoRa sync word, reset default 14 24 */
  uint8_t tx2[7] = { 0x1D, 0x07, 0x40, 0x00, 0x00, 0x00, 0x00 };
  uint8_t rx2[7] = { 0 };
  r = spi_bus_txrx(LORA_CS_PORT, LORA_CS_PIN, tx2, rx2, 7);
  console_printf("ReadReg 0740 spi=%d rx: %02X %02X %02X %02X %02X %02X %02X (last two expect 14 24)\r\n",
                 r, rx2[0], rx2[1], rx2[2], rx2[3], rx2[4], rx2[5], rx2[6]);

  console_printf("re-running full radio_init...\r\n");
  int ri = radio_init();
  int et, ex;
  radio_debug(&et, &ex);
  console_printf("radio_init=%d (tcxo=%d xtal=%d) ok=%d tcxo_used=%d\r\n",
                 ri, et, ex, radio_ok(), radio_using_tcxo());
  #undef BUSY_IN
}

static void cmd_i2cscan(int argc, char **argv)
{
  int found = 0;
  (void)argc; (void)argv;
  console_printf("i2c scan:");
  for (uint8_t a = 0x08; a <= 0x77; a++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 2, 5) == HAL_OK)
    {
      console_printf(" 0x%02X", a);
      found++;
    }
  }
  console_printf("%s\r\n", found ? "" : " (none)");
}

static void cmd_charge(int argc, char **argv)
{
  (void)argc; (void)argv;
  console_printf("charger: vbat=%u mV stat=0x%02X charging=%d fault=%d\r\n",
                 g_fsm.chg.vbat_mv, g_fsm.chg.chrg_stat,
                 g_fsm.chg.charging, g_fsm.chg.fault);
}

static void cmd_bootloader(int argc, char **argv)
{
  (void)argc; (void)argv;
  console_printf("byebye, entering DFU bootloader\r\n");
  tx_flush(200);
  HAL_Delay(50);              /* let the last IN transfer reach the host */
  dfu_enter_bootloader();     /* no return */
}

static void cmd_reboot(int argc, char **argv)
{
  (void)argc; (void)argv;
  console_printf("rebooting\r\n");
  tx_flush(200);
  HAL_Delay(20);
  NVIC_SystemReset();
}

static int led_parse_rgb(char **argv, unsigned long *r, unsigned long *g, unsigned long *b)
{
  *r = strtoul(argv[0], NULL, 10);
  *g = strtoul(argv[1], NULL, 10);
  *b = strtoul(argv[2], NULL, 10);
  return (*r <= 255U) && (*g <= 255U) && (*b <= 255U);
}

static void cmd_led(int argc, char **argv)
{
  /* Bench-only (needs the 5 V rail). Never lets a test/effect mask the
   * real ARMED indicator: led_poll() force-clears any override/bench
   * mode the instant armed is true, and these subcommands refuse
   * up-front too so the console reply is honest. */
  if (argc >= 2 && strcmp(argv[1], "auto") == 0)
  {
    led_override(0);
    led_bench_stop();
    console_printf("led: state patterns resumed\r\n");
    return;
  }

  if (argc == 1)
  {
    led_bench_mode_t m = led_bench_get();
    console_printf("led: brightness=%u%% mode=%s\r\n",
                    led_get_brightness(), led_bench_name(m));
    console_printf("usage: led <r> <g> <b> | led auto | led bright <0-100> |\r\n"
                    "       led cycle [period_ms] | led state <name|0-10> |\r\n"
                    "       led effect solid|blink|breathe|strobe <r> <g> <b> [period_ms] |\r\n"
                    "       led effect rainbow [period_ms] | led effect off\r\n");
    return;
  }

  if (strcmp(argv[1], "bright") == 0)
  {
    if (argc < 3) { console_printf("usage: led bright <0-100>\r\n"); return; }
    long pct = strtol(argv[2], NULL, 10);
    if ((pct < 0) || (pct > 100)) { console_printf("err: brightness 0-100\r\n"); return; }
    led_set_brightness((uint8_t)pct);
    console_printf("led: brightness=%ld%%\r\n", pct);
    return;
  }

  if (strcmp(argv[1], "cycle") == 0 || strcmp(argv[1], "effect") == 0 ||
      strcmp(argv[1], "state") == 0)
  {
    if (g_fsm.armed)
    {
      console_printf("DENIED: disarm first (bench effects are disarmed-only)\r\n");
      return;
    }
    if ((g_fsm.state != ST_INIT) && (g_fsm.state != ST_GROUND_IDLE))
    {
      console_printf("DENIED: bench effects are pad-only (INIT/GROUND_IDLE)\r\n");
      return;
    }
  }

  if (strcmp(argv[1], "state") == 0)
  {
    if (argc < 3)
    {
      console_printf("usage: led state <name|0-10> (INIT GROUND_IDLE ARMED BOOST COAST "
                      "APOGEE DROGUE MAIN DESCENT LANDED FAULT)\r\n");
      return;
    }
    int idx = -1;
    char *end;
    long n = strtol(argv[2], &end, 10);
    if ((*end == '\0') && (n >= 0) && (n <= 10))
    {
      idx = (int)n;
    }
    else
    {
      for (int i = 0; i <= 10; i++)
      {
        const char *nm = fsm_state_name((flight_state_t)i);
        size_t j = 0;
        for (; nm[j] != '\0' && argv[2][j] != '\0'; j++)
        {
          char a = nm[j], c = argv[2][j];
          if ((a >= 'a') && (a <= 'z')) a = (char)(a - 'a' + 'A');
          if ((c >= 'a') && (c <= 'z')) c = (char)(c - 'a' + 'A');
          if (a != c) break;
        }
        if ((nm[j] == '\0') && (argv[2][j] == '\0')) { idx = i; break; }
      }
    }
    if (idx < 0)
    {
      console_printf("err: unknown state %s\r\n", argv[2]);
      return;
    }
    led_bench_set_state((flight_state_t)idx);
    console_printf("led: holding on %s\r\n", fsm_state_name((flight_state_t)idx));
    return;
  }

  if (strcmp(argv[1], "cycle") == 0)
  {
    unsigned long ms = (argc >= 3) ? strtoul(argv[2], NULL, 10) : 1500UL;
    if ((ms < 200UL) || (ms > 10000UL)) { console_printf("err: period 200-10000 ms\r\n"); return; }
    led_bench_set(LED_BENCH_CYCLE, 0, 0, 0, (uint16_t)ms);
    console_printf("led: cycling all states, %lu ms each ('led auto' to stop)\r\n", ms);
    return;
  }

  if (strcmp(argv[1], "effect") == 0)
  {
    if (argc < 3)
    {
      console_printf("usage: led effect solid|blink|breathe|strobe <r> <g> <b> [period_ms] |"
                      " rainbow [period_ms] | off\r\n");
      return;
    }
    if (strcmp(argv[2], "off") == 0)
    {
      led_bench_stop();
      console_printf("led: effect off (state patterns resumed)\r\n");
      return;
    }
    if (strcmp(argv[2], "rainbow") == 0)
    {
      unsigned long ms = (argc >= 4) ? strtoul(argv[3], NULL, 10) : 4000UL;
      if ((ms < 200UL) || (ms > 60000UL)) { console_printf("err: period 200-60000 ms\r\n"); return; }
      led_bench_set(LED_BENCH_RAINBOW, 0, 0, 0, (uint16_t)ms);
      console_printf("led: effect rainbow, %lu ms/cycle\r\n", ms);
      return;
    }
    led_bench_mode_t mode;
    if (strcmp(argv[2], "solid") == 0)        mode = LED_BENCH_SOLID;
    else if (strcmp(argv[2], "blink") == 0)   mode = LED_BENCH_BLINK;
    else if (strcmp(argv[2], "breathe") == 0) mode = LED_BENCH_BREATHE;
    else if (strcmp(argv[2], "strobe") == 0)  mode = LED_BENCH_STROBE;
    else { console_printf("err: unknown effect %s\r\n", argv[2]); return; }
    if (argc < 6) { console_printf("usage: led effect %s <r> <g> <b> [period_ms]\r\n", argv[2]); return; }
    unsigned long r, g, b;
    if (!led_parse_rgb(&argv[3], &r, &g, &b)) { console_printf("err: values 0-255\r\n"); return; }
    unsigned long ms = (argc >= 7) ? strtoul(argv[6], NULL, 10) : 1000UL;
    if ((ms < 50UL) || (ms > 60000UL)) { console_printf("err: period 50-60000 ms\r\n"); return; }
    led_bench_set(mode, (uint8_t)r, (uint8_t)g, (uint8_t)b, (uint16_t)ms);
    console_printf("led: effect %s %lu %lu %lu, %lu ms\r\n", argv[2], r, g, b, ms);
    return;
  }

  /* Bare `led <r> <g> <b>`: raw color for 5 s, then state patterns resume. */
  if (argc < 4)
  {
    console_printf("usage: led <r> <g> <b> (0-255) | led auto | led help\r\n");
    return;
  }
  unsigned long r, g, b;
  if (!led_parse_rgb(&argv[1], &r, &g, &b))
  {
    console_printf("err: values 0-255\r\n");
    return;
  }
  led_override(5000U);
  led_set((uint8_t)r, (uint8_t)g, (uint8_t)b);
  console_printf("led: %lu %lu %lu for 5 s (needs 5 V rail; 'led auto' to resume)\r\n",
                 r, g, b);
}

static void cmd_tx(int argc, char **argv)
{
  if (argc >= 2 && strcmp(argv[1], "power") == 0)
  {
    if (argc < 3)
    {
      console_printf("tx power: %d dBm\r\n", radio_get_tx_power());
      return;
    }
    long dbm = strtol(argv[2], NULL, 10);
    if ((dbm < -9) || (dbm > 22))
    {
      console_printf("err: -9..22 dBm\r\n");
      return;
    }
    if (radio_set_tx_power((int)dbm) == 0)
      console_printf("tx power = %d dBm\r\n", radio_get_tx_power());
    else
      console_printf("err: radio not ready\r\n");
    return;
  }
  if (argc >= 2 && strcmp(argv[1], "mon") == 0)
  {
    bool on = (argc >= 3) && (strcmp(argv[2], "on") == 0);
    telem_monitor(on);
    console_printf("tx monitor %s\r\n", on ? "on" : "off");
    return;
  }
  if (argc >= 2 && strcmp(argv[1], "cw") == 0)
  {
    if (argc >= 3 && strcmp(argv[2], "off") == 0)
    {
      telem_cw(0);
      console_printf("cw off\r\n");
      return;
    }
    /* Real RF emission + heats the module over the IMU (gyro drift, §5.3):
     * pad + disarmed only, and always auto-off. */
    if (((g_fsm.state != ST_GROUND_IDLE) && (g_fsm.state != ST_INIT)) || g_fsm.armed)
    {
      console_printf("cw refused: GROUND_IDLE + disarmed only\r\n");
      return;
    }
    unsigned long secs = (argc >= 3) ? strtoul(argv[2], NULL, 10) : 10UL;
    if ((secs == 0UL) || (secs > 60UL))
    {
      console_printf("usage: tx cw <1-60 s> | tx cw off\r\n");
      return;
    }
    if (telem_cw((uint32_t)secs) == 0)
      console_printf("cw ON %lu s @ %d dBm, watch supply current ('tx cw off' to stop)\r\n",
                     secs, radio_get_tx_power());
    else
      console_printf("err: radio not ready\r\n");
    return;
  }
  if (argc >= 2 && strcmp(argv[1], "frame") == 0)
  {
    console_printf(telem_send_now() == 0 ? "frame sent\r\n"
                                         : "busy (cw up, tx pending, or radio down)\r\n");
    return;
  }
  if (argc >= 2)
  {
    console_printf("usage: tx [power <dbm>|mon <on|off>|cw <s>|cw off|frame]\r\n");
    return;
  }

  /* bare 'tx': transmit dashboard */
  {
    uint32_t txc, rxc, txto, reinit;
    int rssi, snr;
    telem_stats(&txc, &rxc);
    telem_debug(&txto, &reinit);
    console_printf("radio:   %s  tcxo:%s  power:%d dBm\r\n",
                   radio_ok() ? "ok" : "FAIL",
                   radio_using_tcxo() ? "yes" : "no", radio_get_tx_power());
    console_printf("packets: tx=%lu  rx=%lu\r\n",
                   (unsigned long)txc, (unsigned long)rxc);
    console_printf("errors:  tx_timeouts=%lu  reinit=%lu\r\n",
                   (unsigned long)txto, (unsigned long)reinit);
    if (radio_last_rx_status(&rssi, &snr) == 0)
      console_printf("last rx: rssi=%d dBm  snr=%d dB\r\n", rssi, snr);
    else
      console_printf("last rx: (none)\r\n");
    console_printf("monitor: %s   cw: %s\r\n",
                   telem_monitor_active() ? "on" : "off",
                   telem_cw_active() ? "ON" : "off");
  }
}

static void cmd_sleep(int argc, char **argv)
{
  /* STOP-mode low power. Ground + disarmed only: STOP retains the pyro gate
   * driven LOW (boot-safe state); we force-disarm too. Wakes on the RTC
   * timer or RESET (SW1) and comes back via a clean reboot. */
  if (((g_fsm.state != ST_INIT) && (g_fsm.state != ST_GROUND_IDLE)) ||
      g_fsm.armed)
  {
    console_printf("sleep refused: GROUND_IDLE + disarmed only\r\n");
    return;
  }
  unsigned long secs = (argc >= 2) ? strtoul(argv[1], NULL, 10) : 0UL;
  if (secs > 36000UL)
  {
    console_printf("usage: sleep [0-36000 s]   (0 / none = until RESET)\r\n");
    return;
  }
  fsm_disarm();                 /* belt-and-braces: pyros safe before going dark */
  datalog_close();              /* flush + close the SD log so it isn't truncated */
  if (secs)
    console_printf("sleeping (STOP) ~%lu s; USB drops, wakes on timer or RESET (SW1)\r\n",
                   secs);
  else
    console_printf("sleeping (STOP); USB drops, press RESET (SW1) to wake\r\n");
  tx_flush(200);                /* push the farewell out before USB dies */
  HAL_Delay(20);

  /* Only returns if RTC/LSE bring-up failed; otherwise resets on wake. */
  if (lowpower_sleep((uint32_t)secs) != 0)
    console_printf("sleep: RTC/LSE setup failed, staying awake\r\n");
}

static void cmd_wdtest(int argc, char **argv)
{
  (void)argc; (void)argv;
  /* Deliberate hang: ground + disarmed only (same gate as bootloader). */
  if (((g_fsm.state != ST_INIT) && (g_fsm.state != ST_GROUND_IDLE)) ||
      g_fsm.armed)
  {
    console_printf("wdtest refused: ground states + disarmed only\r\n");
    return;
  }
  console_printf("wdtest: halting (IRQs off), expect IWDG reset in ~4 s\r\n");
  tx_flush(200);
  HAL_Delay(20);
  __disable_irq();
  for (;;) { }                /* IWDG must bring us back */
}

typedef struct { const char *name; void (*fn)(int, char **); } cmd_t;

static const cmd_t cmd_table[] =
{
  { "help",       cmd_help       },
  { "status",     cmd_status     },
  { "preflight",  cmd_preflight  },
  { "sensors",    cmd_sensors    },
  { "gps",        cmd_gps        },
  { "servo",      cmd_servo      },
  { "arm",        cmd_arm        },
  { "disarm",     cmd_disarm     },
  { "testen",     cmd_testen     },
  { "fire",       cmd_fire       },
  { "zero",       cmd_zero       },
  { "mainalt",    cmd_mainalt    },
  { "apogee",     cmd_apogee     },
  { "log",        cmd_log        },
  { "cal",        cmd_cal        },
  { "radio",      cmd_radio      },
  { "radiodbg",   cmd_radiodbg   },
  { "tx",         cmd_tx         },
  { "i2cscan",    cmd_i2cscan    },
  { "charge",     cmd_charge     },
  { "bootloader", cmd_bootloader },
  { "reboot",     cmd_reboot     },
  { "wdtest",     cmd_wdtest     },
  { "sleep",      cmd_sleep      },
  { "led",        cmd_led        },
};

#define MAX_ARGS 8   /* longest command today: `led effect <name> <r> <g> <b> <period_ms>` = 7 */

static void dispatch_line(char *line)
{
  char *argv[MAX_ARGS];
  int argc = 0;
  char *tok = strtok(line, " \t");

  while ((tok != NULL) && (argc < MAX_ARGS))
  {
    argv[argc++] = tok;
    tok = strtok(NULL, " \t");
  }
  if (argc == 0)
  {
    return;
  }
  for (size_t i = 0; i < sizeof(cmd_table) / sizeof(cmd_table[0]); i++)
  {
    if (strcmp(argv[0], cmd_table[i].name) == 0)
    {
      cmd_table[i].fn(argc, argv);
      return;
    }
  }
  console_printf("unknown cmd '%s', try 'help'\r\n", argv[0]);
}

/* ============================================================
 * Public API
 * ============================================================ */
void console_init(void)
{
  rx_head = rx_tail = 0;
  tx_head = tx_tail = 0;
  line_len = 0;
}

void console_poll(void)
{
  /* 1. assemble lines from the RX ring (with echo + backspace) */
  while (rx_tail != rx_head)
  {
    uint8_t c = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) & (RX_RING - 1U));

    if ((c == (uint8_t)'\r') || (c == (uint8_t)'\n'))
    {
      if (line_len > 0U)
      {
        (void)console_write("\r\n", 2);
        line_buf[line_len] = '\0';
        line_len = 0;
        dispatch_line(line_buf);
      }
    }
    else if ((c == 0x08U) || (c == 0x7FU))      /* backspace / DEL */
    {
      if (line_len > 0U)
      {
        line_len--;
        (void)console_write("\b \b", 3);
      }
    }
    else if ((c >= 0x20U) && (c < 0x7FU))       /* printable */
    {
      if (line_len < (LINE_MAX - 1U))
      {
        line_buf[line_len++] = (char)c;
        (void)console_write(&c, 1);             /* echo */
      }
      else
      {
        line_len = 0;                           /* overlong: reset line */
        console_printf("\r\nerr: line too long\r\n");
      }
    }
    /* other control chars ignored */
  }

  /* 2. push pending TX out (or drop it if the host is gone) */
  tx_drain();
}
