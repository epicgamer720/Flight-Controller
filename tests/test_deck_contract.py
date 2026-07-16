"""Contract tests between the deck sources and the firmware they scrape.

Three directions:
  1. The sample replies below are copied VERBATIM from the current
     App/console.c printf formats (values substituted, two-space indents
     and \r\n included).  Every regex tools/deck/sources.py uses to scrape
     the FC console must match them.
  2. console.c / app_main.c are scanned for the anchor substrings deck
     depends on (markers, format fragments, the boot banner), following the
     same pattern as tests/test_console_contract.py.
  3. gs.ino is scanned for the JSON field names LineIngest expects, and
     sources.gs_json_line's key order is locked to parse_telem's.
"""

import json
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, os.pardir, "tools"))

import telem_decode as td
from deck import schema, sources

CONSOLE_C = os.path.join(_HERE, os.pardir,
                         "flight-controller", "App", "console.c")
APP_MAIN_C = os.path.join(_HERE, os.pardir,
                          "flight-controller", "App", "app_main.c")
GS_INO = os.path.join(_HERE, os.pardir, "ground-station", "gs", "gs.ino")

with open(CONSOLE_C, encoding="utf-8") as f:
    CONSOLE_SRC = f.read()
with open(APP_MAIN_C, encoding="utf-8") as f:
    APP_MAIN_SRC = f.read()
with open(GS_INO, encoding="utf-8") as f:
    GS_SRC = f.read()

# ---------------------------------------------------------------------------
# Sample replies: verbatim console.c printf shapes with values filled in.
# ---------------------------------------------------------------------------
STATUS_REPLY = (
    "state:   GROUND_IDLE  t=123456ms\r\n"
    "armed:   0  testen: 1  main_alt: 150.00 m\r\n"
    "alt:     1.23 m AGL  vel: -0.04 m/s\r\n"
    "batt:    7912 mV\r\n"
    "mcu:     23.90 C (die)\r\n"
    "flags:   gps_fix=1 accel_sat=0\r\n"
    "health:  imu=1 baro=1 sd=1 radio=1 chg=1 gps=1\r\n"
    "pyro:    cont=0x01 fired=0x00 sense=734 mV\r\n"
)

TX_REPLY = (
    "radio:   ok  tcxo:yes  power:22 dBm\r\n"
    "packets: tx=123  rx=4\r\n"
    "errors:  tx_timeouts=0  reinit=0\r\n"
    "last rx: rssi=-72 dBm  snr=9 dB\r\n"
    "monitor: off   cw: off\r\n"
)

SENSORS_REPLY = (
    "imu:  ax=0.01 ay=-0.02 az=1.00 g  sat=0\r\n"
    "      gx=0.12 gy=-0.34 gz=0.05 dps\r\n"
    "baro: 99123.45 Pa  24.53 C  alt=12.34 m\r\n"
)

GPS_REPLY = (
    "gps: fix=1 sats=8\r\n"
    "     lat=39.1234567 lon=-77.1234567 alt=123.45 m MSL\r\n"
    "     last_fix=250 ms ago\r\n"
)

RADIO_REPLY = (
    "radio: ok  tcxo: yes\r\n"
    "  init err: tcxo=0 xtal=0 (0=ok; -1 rst/busy, -3 no chip, -7/-21 dev errors)\r\n"
    "  reinit attempts: 2  tx timeouts: 1\r\n"
    "  chip irq status: 0x0001 (bit0 TxDone, bit1 RxDone, bit9 Timeout)\r\n"
    "  BUSY pin: 0  DIO1 pin: 1\r\n"
)

CHARGE_REPLY = "charger: vbat=7912 mV stat=0x05 charging=1 fault=0\r\n"

LOG_REPLY = (
    "log: running\r\n"
    "  drops: 12  ring high-water: 2048/65536 bytes\r\n"
)

BOOT_BANNER_REPLY = (
    "\r\n=== FC boot ===\r\n"
    "reset: IWDG\r\n"
    "imu:ok baro:ok chg:ok radio:ok(tcxo) gps:ok sd:ok pyro:ok wdg:ok\r\n"
)


class TestFcConsoleRegexes(unittest.TestCase):
    """Every regex FcConsoleSource scrapes with must match the verbatim
    console.c reply shapes above."""

    def test_re_state_with_uptime(self):
        m = sources.RE_STATE.search(STATUS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("GROUND_IDLE", "123456"))

    def test_re_armed_testen_mainalt(self):
        m = sources.RE_ARMED.search(STATUS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("0", "1", "150.00"))

    def test_re_altvel(self):
        m = sources.RE_ALTVEL.search(STATUS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("1.23", "-0.04"))

    def test_re_batt(self):
        m = sources.RE_BATT.search(STATUS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), "7912")

    def test_re_flags(self):
        m = sources.RE_FLAGS.search(STATUS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("1", "0"))

    def test_re_health_all_six(self):
        m = sources.RE_HEALTH.search(STATUS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("1", "1", "1", "1", "1", "1"))

    def test_re_health_old_four_field_line(self):
        m = sources.RE_HEALTH.search("health:  imu=1 baro=1 sd=0 radio=1\r\n")
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("1", "1", "0", "1", None, None))

    def test_re_pyro(self):
        m = sources.RE_PYRO.search(STATUS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("01", "00", "734"))

    def test_re_mcu(self):
        m = sources.RE_MCU.search(STATUS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), "23.90")

    def test_re_tx_power(self):
        m = sources.RE_TX_POWER.search(TX_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), "22")

    def test_re_tx_pkts(self):
        m = sources.RE_TX_PKTS.search(TX_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("123", "4"))

    def test_re_tx_rxq(self):
        m = sources.RE_TX_RXQ.search(TX_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("-72", "9"))
        # "last rx: (none)" must NOT match
        self.assertIsNone(sources.RE_TX_RXQ.search("last rx: (none)\r\n"))

    def test_re_imu(self):
        m = sources.RE_IMU.search(SENSORS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("0.01", "-0.02", "1.00"))

    def test_re_imu_sat(self):
        m = sources.RE_IMU_SAT.search(SENSORS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), "0")

    def test_re_gyr(self):
        m = sources.RE_GYR.search(SENSORS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("0.12", "-0.34", "0.05"))

    def test_re_baro(self):
        m = sources.RE_BARO.search(SENSORS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("99123.45", "24.53", "12.34"))

    def test_re_gpsfix(self):
        m = sources.RE_GPSFIX.search(GPS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("1", "8"))

    def test_re_gpspos(self):
        m = sources.RE_GPSPOS.search(GPS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("39.1234567", "-77.1234567", "123.45"))

    def test_re_gpsage(self):
        m = sources.RE_GPSAGE.search(GPS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), "250")

    def test_re_radio(self):
        m = sources.RE_RADIO.search(RADIO_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("ok", "yes"))

    def test_re_radio_cnt(self):
        m = sources.RE_RADIO_CNT.search(RADIO_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("2", "1"))

    def test_re_radio_irq(self):
        m = sources.RE_RADIO_IRQ.search(RADIO_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), "0001")

    def test_re_charge(self):
        m = sources.RE_CHARGE.search(CHARGE_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("7912", "05", "1", "0"))

    def test_re_log(self):
        m = sources.RE_LOG.search(LOG_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), "running")
        m = sources.RE_LOG.search("log: not running\r\n")
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), "not running")

    def test_re_logstat(self):
        m = sources.RE_LOGSTAT.search(LOG_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("12", "2048", "65536"))

    def test_boot_banner_and_reset(self):
        self.assertIn(sources.BOOT_BANNER, BOOT_BANNER_REPLY)
        m = sources.RE_RESET.search(BOOT_BANNER_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1).strip(), "IWDG")

    def test_txn_markers_present_in_replies(self):
        # The _service ladder's end-of-reply markers must appear in the
        # reply each transaction waits on.
        for marker, reply in ((b"sense=", STATUS_REPLY),
                              (b"baro:", SENSORS_REPLY),
                              (b"last_fix", GPS_REPLY),
                              (b"DIO1", RADIO_REPLY),
                              (b"cw:", TX_REPLY),
                              (b"fault=", CHARGE_REPLY),
                              (b"bytes", LOG_REPLY)):
            self.assertIn(marker.decode(), reply)


class TestConsoleSourceAnchors(unittest.TestCase):
    """Anchor substrings in App/console.c (+ boot banner in app_main.c)
    that deck's scrapers rely on.  Editing firmware output must keep
    these or update deck."""

    ANCHORS = (
        "state:   %s  t=%lums",
        "armed:   %d  testen: %d  main_alt: %s m",
        "alt:     %s m AGL  vel: %s m/s",
        "batt:    %u mV",
        "mcu:     %s C (die)",
        "flags:   gps_fix=%d accel_sat=%d",
        "health:  imu=%d baro=%d sd=%d radio=%d chg=%d gps=%d",
        "cont=0x%02X fired=0x%02X sense=%u mV",
        "ax=%s ay=%s az=%s g  sat=%d",
        "baro: %s Pa  %s C  alt=%s m",
        "fix=%d sats=%u",
        "last_fix=%lu ms ago",
        "radio: %s  tcxo: %s",
        "radio:   %s  tcxo:%s  power:%d dBm",
        "packets: tx=%lu  rx=%lu",
        "last rx: rssi=%d dBm  snr=%d dB",
        "reinit attempts: %lu  tx timeouts: %lu",
        "chip irq status: 0x%02X%02X",
        "BUSY pin: %d  DIO1 pin: %d",
        "charger: vbat=%u mV stat=0x%02X charging=%d fault=%d",
        "  drops: %lu  ring high-water: %lu/%lu bytes",
    )

    def test_console_anchor_substrings_present(self):
        for anchor in self.ANCHORS:
            self.assertIn(anchor, CONSOLE_SRC,
                          "console.c lost deck anchor %r" % anchor)

    def test_boot_banner_in_app_main(self):
        self.assertIn("=== FC boot ===", APP_MAIN_SRC)
        self.assertIn('reset:%s', APP_MAIN_SRC)

    def test_poll_commands_in_console_cmd_table(self):
        for cmd in ("status", "sensors", "gps", "radio", "tx", "charge",
                    "log"):
            self.assertIn('"%s"' % cmd, CONSOLE_SRC,
                          "console.c no longer has a %r command" % cmd)

    def test_status_marker_prints_after_parsed_lines(self):
        # _txn("status", b"sense=") assumes the pyro line prints last.
        self.assertLess(CONSOLE_SRC.index("chg=%d gps=%d"),
                        CONSOLE_SRC.index("sense="))

    def test_log_stat_marker_prints_after_running_line(self):
        # _txn("log stat", b"bytes") assumes the stats line prints after
        # the running/not-running line.
        self.assertLess(CONSOLE_SRC.index('datalog_ok() ? "running"'),
                        CONSOLE_SRC.index("ring high-water"))


class TestGsInoContract(unittest.TestCase):
    """gs.ino must keep emitting the JSON shapes LineIngest classifies."""

    FIELD_ANCHORS = (
        '\\"type\\":%u',
        '\\"t_ms\\":%lu',
        '\\"state_name\\":\\"%s\\"',
        '\\"accel_g_x100\\":[%d,%d,%d]',
        '\\"gyro_dps_x10\\":[%d,%d,%d]',
        '\\"batt_v\\":%.3f',
        '\\"mcu_temp_c10\\":%d',
        '\\"mcu_temp_c\\":%s',
        '\\"rssi\\":%.1f,\\"snr\\":%.2f}',
        '{\\"uplink\\":\\"%s\\"',
        '\\"tx_status\\":%d',
        '{\\"gs\\":\\"boot\\"',
        '\\"error\\":\\"%s\\"',
        '\\"error\\":\\"radio_init\\"',
        '\\"error\\":\\"rx\\"',
        '\\"cmd_name\\":\\"%s\\"',
        '\\"nonce\\":%lu',
    )

    def test_gs_json_field_anchors_present(self):
        for anchor in self.FIELD_ANCHORS:
            self.assertIn(anchor, GS_SRC,
                          "gs.ino lost deck JSON anchor %r" % anchor)

    def test_gs_sends_ack_as_telemetry_shaped_frame(self):
        # ACK semantics doc: the FC's PKT_ACK arrives as a 48-byte
        # telemetry-shaped frame, dispatched through emit_telem.
        self.assertIn("PKT_ACK", GS_SRC)

    def test_gs_stdin_has_power_command(self):
        # over-air TX power: deck's GS adapter sends "power <dbm>" ->
        # CMD_SET_TX_POWER uplink; gs.ino must parse it.
        self.assertIn("CMD_SET_TX_POWER", GS_SRC)
        self.assertIn("usage: power", GS_SRC)

    def test_gs_json_line_key_order_matches_parse_telem(self):
        # gs.ino emit_telem mirrors parse_telem() key order with
        # rssi/snr appended; sources.gs_json_line must be identical.
        d = td.parse_telem(td.build_telem(state=4, t_ms=1000, batt_mv=8000))
        line = sources.gs_json_line(d, -70.0, 9.25)
        obj = json.loads(line)
        self.assertEqual(list(obj.keys()),
                         list(schema.TELEM_KEYS) + ["rssi", "snr"])

    def test_gs_json_line_values_round_trip(self):
        d = td.parse_telem(td.build_telem(
            state=3, t_ms=123456, lat_e7=391234567, lon_e7=-771234567,
            alt_baro_cm=15243, alt_gps_cm=27243, vel_up_cms=7500,
            accel_g_x100=(12, -8, 1600), gyro_dps_x10=(900, -35, 25),
            batt_mv=8123, pyro_cont=1, sats=9,
            flags=schema.FLAG_GPS_FIX | schema.FLAG_ACCEL_SAT))
        obj = json.loads(sources.gs_json_line(d, -81.5, 6.75))
        self.assertEqual(obj["type"], schema.PKT_TELEMETRY)
        self.assertEqual(obj["t_ms"], 123456)
        self.assertEqual(obj["state_name"], "BOOST")
        self.assertEqual(obj["accel_g_x100"], [12, -8, 1600])
        self.assertAlmostEqual(obj["lat_deg"], 39.1234567, places=7)
        self.assertAlmostEqual(obj["alt_baro_m"], 152.43, places=2)
        self.assertAlmostEqual(obj["vel_up_ms"], 75.0, places=2)
        self.assertAlmostEqual(obj["batt_v"], 8.123, places=3)
        self.assertIs(obj["gps_fix"], True)
        self.assertIs(obj["accel_sat"], True)
        self.assertIs(obj["armed"], False)
        self.assertEqual(obj["rssi"], -81.5)
        self.assertEqual(obj["snr"], 6.75)
        # the raw fields rebuild into a CRC-valid frame
        rebuilt = td.build_telem(
            state=obj["state"], t_ms=obj["t_ms"], lat_e7=obj["lat_e7"],
            lon_e7=obj["lon_e7"], alt_baro_cm=obj["alt_baro_cm"],
            alt_gps_cm=obj["alt_gps_cm"], vel_up_cms=obj["vel_up_cms"],
            accel_g_x100=obj["accel_g_x100"],
            gyro_dps_x10=obj["gyro_dps_x10"], batt_mv=obj["batt_mv"],
            pyro_cont=obj["pyro_cont"], pyro_fired=obj["pyro_fired"],
            sats=obj["sats"], flags=obj["flags"], ptype=obj["type"])
        self.assertEqual(td.parse_telem(rebuilt)["crc16"], obj["crc16"])

    def test_gs_stdin_grammar_still_zero_indexed_fire(self):
        # GS `fire` takes ch 0..NUM_PYRO-1 (deck's GS adapter sends ch-1).
        self.assertIn("fire <ch 0..NUM_PYRO-1>", GS_SRC)


if __name__ == "__main__":
    unittest.main()
