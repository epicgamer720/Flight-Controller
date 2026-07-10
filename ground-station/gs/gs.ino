/* ============================================================
 * gs.ino — Ground Station skeleton (RP2040 + SX1262 via RadioLib)
 *
 * Role (CLAUDE.md §6): IRQ-driven continuous RX of FC telemetry,
 * validate magic/version/CRC16, emit one line of JSON per packet on
 * USB-CDC Serial (field names match tools/telem_decode.py
 * parse_telem(), plus "rssi"/"snr"). Operator commands typed on the
 * same serial port are built into command_packet_t and transmitted
 * once; pad-only enforcement stays FC-side — the GS just sends.
 *
 * COMPILE-ONLY SKELETON: gs_pins.h pins are -1 sentinels until the
 * GS KiCad schematic is parsed (CLAUDE.md §0). The sketch compiles
 * with a #warning and refuses to run until they are filled in.
 *
 * Toolchain: Arduino-Pico (Earle Philhower core) + RadioLib.
 * Link params + packet structs come from shared/protocol.h — build
 * with -I<repo>/shared (see ground-station/README.md for the exact
 * arduino-cli command and its space-in-path quoting).
 * ============================================================ */

#include <SPI.h>
#include <RadioLib.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "gs_pins.h"
#include "protocol.h"   /* shared/ — also pulls in crc16.h */

/* Wire-format guards: both MCUs are little-endian ARM; the packed
 * structs must be byte-for-byte what telem_decode.py expects. */
static_assert(sizeof(telem_packet_t) == 46, "telem_packet_t must be 46 bytes");
static_assert(sizeof(command_packet_t) == 14, "command_packet_t must be 14 bytes");

/* ---- explicit prototypes (keeps the .ino preprocessor honest with
 *      user-defined struct types in signatures) ---- */
void on_dio1(void);
void radio_bringup(void);
void service_radio_rx(void);
void service_console(void);
void handle_line(char *line);
void send_cmd(uint8_t cmd, uint32_t arg);
void emit_packet(const uint8_t *buf, size_t len, float rssi, float snr);
void emit_telem(const telem_packet_t *p, float rssi, float snr);
void emit_cmd_echo(const command_packet_t *c, float rssi, float snr);
void state_name_str(uint8_t s, char *out, size_t n);

/* -1 sentinels equal RADIOLIB_NC once converted; harmless because
 * setup() refuses to call radio.begin() until the pins are real. */
SX1262 radio = new Module(GS_PIN_LORA_NSS, GS_PIN_LORA_DIO1,
                          GS_PIN_LORA_NRESET, GS_PIN_LORA_BUSY);

static volatile bool s_rx_flag = false;   /* set by DIO1 ISR */

static char     s_line[96];
static size_t   s_line_len = 0;

static uint32_t s_nonce = 0;
static bool     s_nonce_seeded = false;

static const char *const STATE_NAMES[] = {
    "INIT", "GROUND_IDLE", "ARMED", "BOOST", "COAST",
    "APOGEE", "DROGUE", "MAIN", "DESCENT", "LANDED", "FAULT",
};
static const char *const CMD_NAMES[] = {
    "NOP", "ARM", "DISARM", "TEST_FIRE",
    "SET_MAIN_ALT", "ZERO_BARO", "REBOOT", "ENTER_BOOTLOADER",
};

/* ============================================================ */

void on_dio1(void)
{
    s_rx_flag = true;
}

void setup()
{
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000)
        delay(10);

    /* Runtime refusal: never touch the radio with sentinel pins.
     * (Constant condition today; becomes dead code once gs_pins.h is
     * filled in — the radio path below still compiles either way.) */
    if (!GS_PINS_VALID) {
        for (;;) {
            Serial.println("{\"error\":\"gs_pins.h pin map not set - extract "
                           "RP2040 GPIOs from the GS KiCad schematic "
                           "(CLAUDE.md s0) and rebuild\"}");
            delay(2000);
        }
    }

    SPI.setSCK((pin_size_t)GS_PIN_SPI_SCK);
    SPI.setTX((pin_size_t)GS_PIN_SPI_MOSI);
    SPI.setRX((pin_size_t)GS_PIN_SPI_MISO);
    SPI.begin();

    radio_bringup();
}

void radio_bringup(void)
{
    for (;;) {
        int16_t st;
#if GS_RADIO_TCXO
        /* TCXO on DIO3 (§3.1): begin() with the shared TCXO voltage,
         * then set the explicit startup delay (RadioLib takes µs). */
        st = radio.begin((float)LORA_FREQ_HZ / 1.0e6f, LORA_BW_KHZ, LORA_SF,
                         LORA_CR, LORA_SYNC_WORD, LORA_TX_DBM, LORA_PREAMBLE,
                         LORA_TCXO_V);
        if (st == RADIOLIB_ERR_NONE)
            st = radio.setTCXO(LORA_TCXO_V, (uint32_t)LORA_TCXO_DELAYMS * 1000UL);
#else
        /* Plain 32 MHz XTAL: tcxoVoltage = 0 keeps DIO3 TCXO control
         * OFF — enabling it on an XTAL part kills radio init (§3.1). */
        st = radio.begin((float)LORA_FREQ_HZ / 1.0e6f, LORA_BW_KHZ, LORA_SF,
                         LORA_CR, LORA_SYNC_WORD, LORA_TX_DBM, LORA_PREAMBLE,
                         0.0f);
#endif
        /* DIO2 drives the PE4259 RF switch on this board (§3.1/§6.3). */
        if (st == RADIOLIB_ERR_NONE)
            st = radio.setDio2AsRfSwitch(true);
        /* LoRa HW CRC ON (§3) — on top of the payload CRC16. */
        if (st == RADIOLIB_ERR_NONE)
            st = radio.setCRC(2);
        if (st == RADIOLIB_ERR_NONE) {
            radio.setDio1Action(on_dio1);      /* IRQ-driven RX, no polling */
            st = radio.startReceive();         /* continuous RX */
        }
        if (st == RADIOLIB_ERR_NONE) {
            Serial.println("{\"gs\":\"boot\",\"radio\":\"ok\"}");
            return;
        }
        char js[96];
        snprintf(js, sizeof js,
                 "{\"error\":\"radio_init\",\"status\":%d}", (int)st);
        Serial.println(js);
        delay(5000);
    }
}

void loop()
{
    service_radio_rx();
    service_console();
}

/* ============================ RX =========================== */

void service_radio_rx(void)
{
    if (!s_rx_flag)
        return;
    s_rx_flag = false;

    uint8_t buf[64];
    size_t len = radio.getPacketLength();
    size_t rd = (len > sizeof buf) ? sizeof buf : len;
    int16_t st = radio.readData(buf, rd);
    float rssi = radio.getRSSI();
    float snr = radio.getSNR();
    radio.startReceive();                      /* resume continuous RX */

    if (st != RADIOLIB_ERR_NONE) {             /* incl. LoRa HW CRC fail */
        char js[96];
        snprintf(js, sizeof js,
                 "{\"error\":\"rx\",\"status\":%d,\"len\":%u}",
                 (int)st, (unsigned)len);
        Serial.println(js);
        return;
    }
    emit_packet(buf, len, rssi, snr);
}

/* Validate + dispatch one received frame.
 * 46 B = telem_packet_t (PKT_TELEMETRY / PKT_EVENT / PKT_ACK — the FC
 * sends ACKs as full telemetry-shaped frames with type = PKT_ACK).
 * 14 B = command_packet_t (only seen if we hear our own uplink echoed). */
void emit_packet(const uint8_t *buf, size_t len, float rssi, float snr)
{
    const char *why = NULL;

    if (len == sizeof(telem_packet_t)) {
        telem_packet_t p;
        memcpy(&p, buf, sizeof p);             /* memcpy: no unaligned access */
        if (p.magic != LINK_MAGIC)
            why = "bad_magic";
        else if (p.version != PROTO_VERSION)
            why = "bad_version";
        else if (crc16_ccitt(buf, sizeof p - 2) != p.crc16)
            why = "crc16";
        else {
            emit_telem(&p, rssi, snr);
            return;
        }
    } else if (len == sizeof(command_packet_t)) {
        command_packet_t c;
        memcpy(&c, buf, sizeof c);
        if (c.magic != LINK_MAGIC)
            why = "bad_magic";
        else if (c.version != PROTO_VERSION)
            why = "bad_version";
        else if (crc16_ccitt(buf, sizeof c - 2) != c.crc16)
            why = "crc16";
        else {
            emit_cmd_echo(&c, rssi, snr);
            return;
        }
    } else {
        why = "bad_len";
    }

    char js[128];
    snprintf(js, sizeof js,
             "{\"error\":\"%s\",\"len\":%u,\"rssi\":%.1f,\"snr\":%.2f}",
             why, (unsigned)len, (double)rssi, (double)snr);
    Serial.println(js);
}

void state_name_str(uint8_t s, char *out, size_t n)
{
    if (s < (sizeof STATE_NAMES / sizeof STATE_NAMES[0]))
        snprintf(out, n, "%s", STATE_NAMES[s]);
    else
        snprintf(out, n, "UNKNOWN(%u)", (unsigned)s);
}

/* One JSON line per packet. Key names + order mirror
 * tools/telem_decode.py parse_telem(), with "rssi"/"snr" appended. */
void emit_telem(const telem_packet_t *p, float rssi, float snr)
{
    char sn[24];
    state_name_str(p->state, sn, sizeof sn);

    char js[1024];
    snprintf(js, sizeof js,
        "{\"magic\":%u,\"version\":%u,\"type\":%u,\"state\":%u,\"t_ms\":%lu,"
        "\"lat_e7\":%ld,\"lon_e7\":%ld,"
        "\"alt_baro_cm\":%ld,\"alt_gps_cm\":%ld,\"vel_up_cms\":%d,"
        "\"accel_g_x100\":[%d,%d,%d],\"gyro_dps_x10\":[%d,%d,%d],"
        "\"batt_mv\":%u,\"pyro_cont\":%u,\"pyro_fired\":%u,\"sats\":%u,"
        "\"flags\":%u,\"crc16\":%u,"
        "\"state_name\":\"%s\","
        "\"lat_deg\":%.7f,\"lon_deg\":%.7f,"
        "\"alt_baro_m\":%.2f,\"alt_gps_m\":%.2f,\"vel_up_ms\":%.2f,"
        "\"accel_g\":[%.2f,%.2f,%.2f],\"gyro_dps\":[%.1f,%.1f,%.1f],"
        "\"batt_v\":%.3f,"
        "\"gps_fix\":%s,\"accel_sat\":%s,\"armed\":%s,\"sd_ok\":%s,"
        "\"chg_ok\":%s,\"gps_ok\":%s,"
        "\"rssi\":%.1f,\"snr\":%.2f}",
        (unsigned)p->magic, (unsigned)p->version, (unsigned)p->type,
        (unsigned)p->state, (unsigned long)p->t_ms,
        (long)p->lat_e7, (long)p->lon_e7,
        (long)p->alt_baro_cm, (long)p->alt_gps_cm, (int)p->vel_up_cms,
        (int)p->accel_g_x100[0], (int)p->accel_g_x100[1], (int)p->accel_g_x100[2],
        (int)p->gyro_dps_x10[0], (int)p->gyro_dps_x10[1], (int)p->gyro_dps_x10[2],
        (unsigned)p->batt_mv, (unsigned)p->pyro_cont, (unsigned)p->pyro_fired,
        (unsigned)p->sats, (unsigned)p->flags, (unsigned)p->crc16,
        sn,
        (double)p->lat_e7 / 1e7, (double)p->lon_e7 / 1e7,
        (double)p->alt_baro_cm / 100.0, (double)p->alt_gps_cm / 100.0,
        (double)p->vel_up_cms / 100.0,
        (double)p->accel_g_x100[0] / 100.0, (double)p->accel_g_x100[1] / 100.0,
        (double)p->accel_g_x100[2] / 100.0,
        (double)p->gyro_dps_x10[0] / 10.0, (double)p->gyro_dps_x10[1] / 10.0,
        (double)p->gyro_dps_x10[2] / 10.0,
        (double)p->batt_mv / 1000.0,
        (p->flags & FLAG_GPS_FIX)   ? "true" : "false",
        (p->flags & FLAG_ACCEL_SAT) ? "true" : "false",
        (p->flags & FLAG_ARMED)     ? "true" : "false",
        (p->flags & FLAG_SD_OK)     ? "true" : "false",
        (p->flags & FLAG_CHG_OK)    ? "true" : "false",
        (p->flags & FLAG_GPS_OK)    ? "true" : "false",
        (double)rssi, (double)snr);
    Serial.println(js);
}

/* Key names mirror telem_decode.py parse_command(). */
void emit_cmd_echo(const command_packet_t *c, float rssi, float snr)
{
    char js[256];
    snprintf(js, sizeof js,
        "{\"magic\":%u,\"version\":%u,\"type\":%u,\"cmd\":%u,"
        "\"cmd_name\":\"%s\",\"arg\":%lu,\"nonce\":%lu,\"crc16\":%u,"
        "\"rssi\":%.1f,\"snr\":%.2f}",
        (unsigned)c->magic, (unsigned)c->version, (unsigned)c->type,
        (unsigned)c->cmd,
        (c->cmd < (sizeof CMD_NAMES / sizeof CMD_NAMES[0]))
            ? CMD_NAMES[c->cmd] : "UNKNOWN",
        (unsigned long)c->arg, (unsigned long)c->nonce, (unsigned)c->crc16,
        (double)rssi, (double)snr);
    Serial.println(js);
}

/* ========================= UPLINK ========================== */

void service_console(void)
{
    while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        if (ch == '\r')
            continue;
        if (ch == '\n') {
            s_line[s_line_len] = '\0';
            if (s_line_len > 0)
                handle_line(s_line);
            s_line_len = 0;
        } else if (s_line_len < sizeof s_line - 1) {
            s_line[s_line_len++] = ch;
        }
    }
}

void handle_line(char *line)
{
    char *tok = strtok(line, " \t");
    if (!tok)
        return;
    for (char *c = tok; *c; c++)
        *c = (char)tolower((unsigned char)*c);

    if (!strcmp(tok, "arm"))    { send_cmd(CMD_ARM, 0);       return; }
    if (!strcmp(tok, "disarm")) { send_cmd(CMD_DISARM, 0);    return; }
    if (!strcmp(tok, "zero"))   { send_cmd(CMD_ZERO_BARO, 0); return; }
    if (!strcmp(tok, "reboot")) { send_cmd(CMD_REBOOT, 0);    return; }
    if (!strcmp(tok, "nop"))    { send_cmd(CMD_NOP, 0);       return; }

    if (!strcmp(tok, "mainalt")) {
        char *a = strtok(NULL, " \t");
        char *end = NULL;
        double m = a ? strtod(a, &end) : 0.0;
        /* mirror the FC's 30..2000 m validation so a NAK isn't a surprise */
        if (!a || end == a || m < 30.0 || m > 2000.0) {
            Serial.println("{\"error\":\"usage: mainalt <30..2000 m>\"}");
            return;
        }
        send_cmd(CMD_SET_MAIN_ALT, (uint32_t)(m * 100.0 + 0.5)); /* arg = cm */
        return;
    }

    if (!strcmp(tok, "fire")) {
        char *a = strtok(NULL, " \t");
        char *b = strtok(NULL, " \t");
        char *e1 = NULL, *e2 = NULL;
        unsigned long chn  = a ? strtoul(a, &e1, 10) : 0;
        unsigned long code = b ? strtoul(b, &e2, 16) : 0;
        if (!a || !b || e1 == a || e2 == b ||
            chn >= NUM_PYRO || code > 0xFFFFUL) {
            Serial.println("{\"error\":\"usage: fire <ch 0..NUM_PYRO-1> "
                           "<hex passcode>\"}");
            return;
        }
        /* FC CMD_TEST_FIRE gating (telemetry.c): arg = passcode << 16 |
         * channel; honored ONLY in ST_GROUND_IDLE + disarmed + testen +
         * matching passcode + fresh nonce. The GS just sends. */
        send_cmd(CMD_TEST_FIRE, ((uint32_t)code << 16) | (uint32_t)chn);
        return;
    }

    if (!strcmp(tok, "help")) {
        Serial.println("{\"help\":\"arm | disarm | mainalt <m> | zero | "
                       "reboot | fire <ch> <hexcode> | nop\"}");
        return;
    }

    Serial.println("{\"error\":\"unknown command (try: help)\"}");
}

void send_cmd(uint8_t cmd, uint32_t arg)
{
    /* Nonce start is randomized (seeded at first use, so operator timing
     * adds entropy on top of millis/micros) then increments monotonically.
     * A fixed 1,2,3... start would collide with the FC's 8-deep anti-replay
     * window after every GS restart, silently eating the first commands. */
    if (!s_nonce_seeded) {
        randomSeed(millis() ^ (micros() << 7));
        s_nonce = ((uint32_t)random(0x10000) << 16) | (uint32_t)random(0x10000);
        s_nonce_seeded = true;
    }

    command_packet_t c;
    c.magic   = LINK_MAGIC;
    c.version = PROTO_VERSION;
    c.type    = PKT_COMMAND;
    c.cmd     = cmd;
    c.arg     = arg;
    c.nonce   = ++s_nonce;
    c.crc16   = crc16_ccitt((const uint8_t *)&c, sizeof c - 2);

    /* One blocking TX, then straight back to continuous RX: the FC only
     * opens a short pad RX window between its frames (§4), and its ACK
     * arrives as a PKT_ACK telemetry-shaped frame. */
    int16_t st = radio.transmit((uint8_t *)&c, sizeof c);
    s_rx_flag = false;              /* TxDone also pulses DIO1 — discard */
    radio.startReceive();

    char js[160];
    snprintf(js, sizeof js,
             "{\"uplink\":\"%s\",\"cmd\":%u,\"arg\":%lu,\"nonce\":%lu,"
             "\"tx_status\":%d}",
             (cmd < (sizeof CMD_NAMES / sizeof CMD_NAMES[0]))
                 ? CMD_NAMES[cmd] : "UNKNOWN",
             (unsigned)cmd, (unsigned long)arg, (unsigned long)c.nonce,
             (int)st);
    Serial.println(js);
}
