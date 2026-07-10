/* ============================================================
 * console.c — line-oriented USB-CDC command console.
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
 *   passcode == TEST_FIRE_PASSCODE  — mirrors CMD_TEST_FIRE.
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
 * RX from USB ISR — SPSC producer, never blocks
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
  console_printf(
    "commands:\r\n"
    "  help                    this text\r\n"
    "  status                  state/armed/alt/vel/batt/flags/health\r\n"
    "  sensors                 raw IMU + baro\r\n"
    "  gps                     last GPS fix\r\n"
    "  servo <1-4> <500-2500>  set servo pulse (us)\r\n"
    "  arm | disarm            arming gate / disarm\r\n"
    "  testen <on|off>         pad test mode (GROUND_IDLE only)\r\n"
    "  fire <ch> <hex-code>    pad test fire (testen + passcode)\r\n"
    "  zero                    zero baro AGL\r\n"
    "  mainalt <m>             set main deploy altitude\r\n"
    "  log <stat|start|stop>   SD logging\r\n"
    "  cal                     start pad gyro bias cal\r\n"
    "  radio                   radio health\r\n"
    "  charge                  charger status\r\n"
    "  bootloader              reboot into USB DFU\r\n"
    "  reboot                  reset MCU\r\n"
    "  wdtest                  hang on purpose (IWDG must reset us)\r\n");
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
  console_printf("flags:   gps_fix=%d accel_sat=%d\r\n",
                 g_fsm.gps.fix, g_fsm.imu.accel_sat);
  console_printf("health:  imu=%d baro=%d sd=%d radio=%d\r\n",
                 g_fsm.imu_ok, g_fsm.baro_ok, g_fsm.sd_ok, g_fsm.radio_ok);
  console_printf("pyro:    cont=0x%02X fired=0x%02X sense=%u mV\r\n",
                 pyro_cont_bits(), pyro_fired_bits(), pyro_sense_mv(0));
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
    console_printf("arm refused (%d): need healthy sensors + still on pad\r\n", r);
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
    console_printf("test mode ON — pyro test fire enabled on pad\r\n");
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

  /* Mirror CLAUDE.md §2 CMD_TEST_FIRE gating — ALL of these, no exceptions: */
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
    console_printf("log: %s\r\n", datalog_ok() ? "running" : "not running");
  }
  else if (strcmp(argv[1], "start") == 0)
  {
    if (datalog_ok())
    {
      console_printf("log: already running\r\n");
    }
    else
    {
      console_printf("log: %s\r\n", (datalog_init() == 0) ? "started" : "start FAILED");
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
  console_printf("gyro cal started — keep vehicle still (~2 s)\r\n");
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
  console_printf("byebye — entering DFU bootloader\r\n");
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

static void cmd_wdtest(int argc, char **argv)
{
  (void)argc; (void)argv;
  console_printf("wdtest: halting (IRQs off) — expect IWDG reset in ~4 s\r\n");
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
  { "sensors",    cmd_sensors    },
  { "gps",        cmd_gps        },
  { "servo",      cmd_servo      },
  { "arm",        cmd_arm        },
  { "disarm",     cmd_disarm     },
  { "testen",     cmd_testen     },
  { "fire",       cmd_fire       },
  { "zero",       cmd_zero       },
  { "mainalt",    cmd_mainalt    },
  { "log",        cmd_log        },
  { "cal",        cmd_cal        },
  { "radio",      cmd_radio      },
  { "radiodbg",   cmd_radiodbg   },
  { "i2cscan",    cmd_i2cscan    },
  { "charge",     cmd_charge     },
  { "bootloader", cmd_bootloader },
  { "reboot",     cmd_reboot     },
  { "wdtest",     cmd_wdtest     },
};

static void dispatch_line(char *line)
{
  char *argv[6];
  int argc = 0;
  char *tok = strtok(line, " \t");

  while ((tok != NULL) && (argc < 6))
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
  console_printf("unknown cmd '%s' — try 'help'\r\n", argv[0]);
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
