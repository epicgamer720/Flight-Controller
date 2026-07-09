#pragma once
/* ============================================================
 * app.h — module APIs. Every App/*.c implements against this.
 * Conventions: all functions non-blocking unless noted; return 0 = OK,
 * negative = error. Superloop calls *_poll() functions; no RTOS.
 * ============================================================ */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "board.h"
#include "app_config.h"
#include "protocol.h"

/* ---------------- spi_bus.c — shared SPI1 arbitration ---------------- */
/* Mode 0 for both devices (SX1262 mandates it; ISM330DHCX supports it).
 * Never assert both CS. Superloop = naturally serialized; these guard
 * against nesting bugs and handle CS + BUSY etiquette. */
int  spi_bus_txrx(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                  const uint8_t *tx, uint8_t *rx, uint16_t len); /* full transaction under one CS */
int  spi_bus_txrx2(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                   const uint8_t *hdr, uint16_t hdr_len,
                   const uint8_t *tx, uint8_t *rx, uint16_t len); /* hdr then data, one CS */

/* ---------------- ism330dhcx.c — IMU ---------------- */
typedef struct { float ax, ay, az;       /* g      */
                 float gx, gy, gz;       /* dps    */
                 bool  accel_sat; } imu_sample_t;
int  imu_init(void);                      /* WHO_AM_I, ±16 g, ±2000 dps, ODR >= CTRL_HZ */
int  imu_read(imu_sample_t *s);           /* latest sample (bias-corrected gyro) */
void imu_gyro_cal_start(void);            /* pad-only stationary bias calibration */
bool imu_gyro_cal_done(void);
bool imu_ok(void);

/* ---------------- bmp580.c — barometer ---------------- */
int   baro_init(void);                    /* probe 0x46/0x47, continuous mode @ BARO_HZ */
int   baro_read(float *press_pa, float *temp_c); /* latest conversion */
float baro_altitude_m(float press_pa);    /* vs sea-level ref */
void  baro_zero(void);                    /* set current pressure = AGL 0 */
bool  baro_ok(void);

/* ---------------- bq25883.c — charger monitor ---------------- */
typedef struct { uint16_t vbat_mv; uint8_t chrg_stat; bool charging; bool fault; } charger_status_t;
int  charger_init(void);                  /* verify part, enable ADC; monitor-only otherwise */
int  charger_poll(charger_status_t *st);  /* call at CHARGER_HZ */

/* ---------------- sx126x.c — LoRa radio (Wio-SX1262) ---------------- */
/* Per CLAUDE.md §3.1: BUSY-wait before every command; NRESET pulse on init;
 * DIO2-as-RF-switch ON; TCXO 1.8 V/5 ms via DIO3, auto-fallback to XTAL if
 * init fails; PB2 stays high-Z; params from protocol.h. */
int  radio_init(void);
bool radio_ok(void);
bool radio_using_tcxo(void);
void radio_debug(int *tcxo_err, int *xtal_err);  /* config step codes from last init */
int  radio_send(const uint8_t *buf, uint8_t len);        /* non-blocking start; TxDone via IRQ */
bool radio_tx_done(void);                                /* since last send */
int  radio_rx_start(uint32_t timeout_ms);                /* single RX window */
int  radio_rx_get(uint8_t *buf, uint8_t maxlen);         /* >0 = packet length received */
void radio_irq(void);                                    /* EXTI15_10 -> DIO1 handler */

/* ---------------- gps.c — external module on USART6 ---------------- */
typedef struct { int32_t lat_e7, lon_e7, alt_msl_cm;
                 uint8_t sats; bool fix; uint32_t last_fix_ms; } gps_fix_t;
int  gps_init(void);                      /* autobaud 9600/38400/57600/115200, NMEA GGA/RMC */
void gps_poll(void);                      /* drain UART ring, parse */
void gps_get(gps_fix_t *f);
void gps_uart_irq(void);                  /* USART6 IRQ -> RX ring */

/* ---------------- pyro.c — SAFETY CRITICAL (CLAUDE.md §2) ---------------- */
void pyro_boot_safe(void);                /* FIRST line of main(): gate LOW, push-pull */
int  pyro_init(void);                     /* ADC for continuity */
bool pyro_continuity(uint8_t ch);         /* cached from CONT_HZ poll; NEVER pulses gate */
void pyro_poll(void);                     /* continuity ADC poll + fire-pulse timing */
int  pyro_fire(uint8_t ch);               /* state machine ONLY; ignores if !armed */
uint8_t pyro_cont_bits(void);
uint8_t pyro_fired_bits(void);
uint16_t pyro_sense_mv(uint8_t ch);       /* raw divider reading, battery-side mV */

/* ---------------- servo.c ---------------- */
int  servo_init(void);                    /* TIM2 CH1..4, 50 Hz, centered 1500 us */
void servo_set_us(uint8_t idx, uint16_t us);   /* idx 0..3, clamp 500..2500 */
uint16_t servo_get_us(uint8_t idx);

/* ---------------- status_led.c — WS2812 on PC13 (best-effort) -------- */
void led_init(void);
void led_set(uint8_t r, uint8_t g, uint8_t b); /* immediate bit-bang, ~30 us IRQ-off */
void led_poll(flight_state_t st, bool armed, bool sd_ok, bool gps_fix); /* LED_HZ pattern */

/* ---------------- kalman.c — 1-D alt/vel filter ---------------- */
typedef struct { float alt_m, vel_ms; float P[2][2]; bool init; } kalman_t;
void kal_reset(kalman_t *k);
void kal_predict(kalman_t *k, float accel_up_ms2, float dt, bool accel_valid);
void kal_update_baro(kalman_t *k, float alt_m);

/* ---------------- state_machine.c ---------------- */
typedef struct {
    flight_state_t state;
    bool     armed, test_enabled;
    float    agl_m, vel_ms;          /* filtered */
    float    main_alt_m;             /* runtime-settable */
    uint32_t t_launch_ms;
    imu_sample_t imu;
    gps_fix_t    gps;
    charger_status_t chg;
    float    press_pa, temp_c;
    bool     imu_ok, baro_ok, sd_ok, radio_ok;
} fsm_t;
extern fsm_t g_fsm;
void fsm_init(void);
void fsm_step(uint32_t now_ms);           /* CTRL_HZ: read sensors, filter, transition, fire */
int  fsm_request_arm(bool via_radio);     /* full arming gate; returns 0 if armed */
void fsm_disarm(void);
const char *fsm_state_name(flight_state_t s);

/* ---------------- telemetry.c ---------------- */
void telem_init(void);
void telem_poll(uint32_t now_ms);         /* cadence TX, pad RX windows, command handling */
void telem_build(telem_packet_t *p);      /* fill from g_fsm + CRC */
void telem_event(uint8_t event_state);    /* PKT_EVENT on transitions/pyro */

/* ---------------- datalog.c — SD binary logging ---------------- */
/* 68-byte fixed records -> LOGnnn.BIN via FatFs; ring buffer decouples the
 * control loop from SD latency; flush LOG_FLUSH_MS; close on LANDED/FAULT. */
typedef struct __attribute__((packed)) {
    uint8_t  magic;              /* LOG_RECORD_MAGIC */
    uint8_t  state;
    uint16_t flags;              /* low 8 = protocol FLAG_*, high 8 = reserved */
    uint32_t t_ms;
    float    ax_g, ay_g, az_g;
    float    gx_dps, gy_dps, gz_dps;
    float    press_pa, temp_c;
    float    agl_m, vel_ms;
    int32_t  lat_e7, lon_e7;
    int32_t  alt_gps_cm;
    uint8_t  sats, pyro_cont, pyro_fired, pad;
    uint16_t batt_mv;
    uint16_t crc16;              /* over preceding 66 bytes */
} log_record_t;
_Static_assert(sizeof(log_record_t) == 68, "log record must be 68 bytes");
int  datalog_init(void);                  /* mount, open next LOGnnn.BIN */
void datalog_push(const log_record_t *r); /* from control loop, never blocks */
void datalog_poll(uint32_t now_ms);       /* drain ring -> f_write, periodic f_sync */
void datalog_close(void);
bool datalog_ok(void);
void datalog_event(const char *msg);      /* human-readable event -> LOGnnn.TXT sidecar */

/* ---------------- console.c — USB-CDC command console ---------------- */
void console_init(void);
void console_poll(void);
void console_rx(const uint8_t *buf, uint32_t len);  /* from usbd_cdc_if */
int  console_write(const void *buf, uint16_t len);  /* to host, drops if unplugged */
void console_printf(const char *fmt, ...);

/* ---------------- usb_device glue (App/usb/) ---------------- */
void usb_device_init(void);               /* CDC device on OTG_FS */
bool usb_cdc_connected(void);

/* ---------------- dfu.c ---------------- */
void dfu_enter_bootloader(void);          /* deinit + jump 0x1FF00000, no return */

/* ---------------- app_main.c — superloop ---------------- */
void app_init(void);                      /* module init order + health flags */
void app_loop(void);                      /* one superloop pass */
