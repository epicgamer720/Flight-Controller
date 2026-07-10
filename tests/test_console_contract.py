"""Contract test between the firmware console (App/console.c) and the
scrapers in tools/live_dashboard.py.

Two directions:
  1. The sample replies below are copied VERBATIM from the current console.c
     printf formats (values substituted).  Every RE_* the dashboard imports
     must match them, and every _txn() end-of-reply marker must appear in
     the reply it waits on.
  2. console.c's source is scanned for the anchor substrings the dashboard
     depends on ("sense=", "last_fix", "DIO1", "baro:") and for ordering
     guarantees (each marker's line prints AFTER the lines the dashboard
     parses, so the reply is complete when the marker arrives).

If a console printf edit would break the dashboard, this test fails loudly.
"""

import inspect
import os
import re
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, os.pardir, "tools"))

import live_dashboard as ld

CONSOLE_C = os.path.join(_HERE, os.pardir,
                         "flight-controller", "App", "console.c")
with open(CONSOLE_C, encoding="utf-8") as f:
    SRC = f.read()

# {command: end-of-reply marker} actually used by the dashboard's poll loop,
# extracted from live_dashboard.Poller.run's _txn(port, "<cmd>", b"<marker>")
# calls so the test tracks the dashboard source, not a copy of it.
TXN_RE = re.compile(r'_txn\(port,\s*"([a-z]+)",\s*b"([^"]+)"')
POLL = dict(TXN_RE.findall(inspect.getsource(ld.Poller.run)))

# ---------------------------------------------------------------------------
# Sample replies — verbatim console.c printf formats with values filled in.
# ---------------------------------------------------------------------------
STATUS_REPLY = (
    "state:   GROUND_IDLE  t=123456ms\r\n"
    "armed:   0  testen: 1  main_alt: 150.00 m\r\n"
    "alt:     1.23 m AGL  vel: -0.04 m/s\r\n"
    "batt:    7912 mV\r\n"
    "flags:   gps_fix=1 accel_sat=0\r\n"
    "health:  imu=1 baro=1 sd=1 radio=0 chg=1 gps=1\r\n"
    "pyro:    cont=0x01 fired=0x00 sense=734 mV\r\n"
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

RADIO_REPLY_FAIL = (
    "radio: FAIL  tcxo: no (xtal fallback)\r\n"
    "  init err: tcxo=-3 xtal=-3 (0=ok; -1 rst/busy, -3 no chip, -7/-21 dev errors)\r\n"
    "  reinit attempts: 12  tx timeouts: 0\r\n"
    "  BUSY pin: 0  DIO1 pin: 0\r\n"
)

RADIO_REPLY_OK = (
    "radio: ok  tcxo: yes\r\n"
    "  init err: tcxo=0 xtal=0 (0=ok; -1 rst/busy, -3 no chip, -7/-21 dev errors)\r\n"
    "  reinit attempts: 0  tx timeouts: 0\r\n"
    "  BUSY pin: 0  DIO1 pin: 1\r\n"
)

REPLY_FOR_CMD = {
    "status": STATUS_REPLY,
    "sensors": SENSORS_REPLY,
    "gps": GPS_REPLY,
    "radio": RADIO_REPLY_FAIL,
}


class TestDashboardRegexes(unittest.TestCase):
    """Every RE_* imported from live_dashboard must match current output."""

    def test_re_imu(self):
        m = ld.RE_IMU.search(SENSORS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("0.01", "-0.02", "1.00"))

    def test_re_gyr(self):
        m = ld.RE_GYR.search(SENSORS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("0.12", "-0.34", "0.05"))

    def test_re_baro(self):
        m = ld.RE_BARO.search(SENSORS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("99123.45", "24.53", "12.34"))

    def test_re_state(self):
        m = ld.RE_STATE.search(STATUS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), "GROUND_IDLE")

    def test_re_armed(self):
        m = ld.RE_ARMED.search(STATUS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("0", "1"))

    def test_re_altvel(self):
        m = ld.RE_ALTVEL.search(STATUS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("1.23", "-0.04"))

    def test_re_batt(self):
        m = ld.RE_BATT.search(STATUS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), "7912")

    def test_re_health_captures_all_six_fields(self):
        # Current firmware emits " chg=%d gps=%d" after the original four;
        # RE_HEALTH captures all six on new firmware.
        m = ld.RE_HEALTH.search(STATUS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("1", "1", "1", "0", "1", "1"))

    def test_re_health_still_matches_old_four_field_line(self):
        # chg/gps groups are optional so the dashboard also works against
        # pre-Phase-4 firmware that emits only the first four fields.
        m = ld.RE_HEALTH.search("health:  imu=1 baro=1 sd=0 radio=1\r\n")
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("1", "1", "0", "1", None, None))

    def test_re_pyro(self):
        m = ld.RE_PYRO.search(STATUS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("01", "00", "734"))

    def test_re_gpsfix(self):
        m = ld.RE_GPSFIX.search(GPS_REPLY)
        self.assertIsNotNone(m)
        self.assertEqual(m.groups(), ("1", "8"))

    def test_re_radio_fail(self):
        m = ld.RE_RADIO.search(RADIO_REPLY_FAIL)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), "FAIL")
        self.assertEqual(m.group(2), "no")

    def test_re_radio_ok(self):
        m = ld.RE_RADIO.search(RADIO_REPLY_OK)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(1), "ok")
        self.assertEqual(m.group(2), "yes")


class TestTxnMarkers(unittest.TestCase):
    """The dashboard's _txn end-of-reply markers, as extracted from its own
    source, must hold against both the sample replies and console.c."""

    def test_poll_commands_present(self):
        self.assertEqual(set(POLL), {"status", "sensors", "gps", "radio"})

    def test_markers_in_sample_replies(self):
        for cmd, marker in POLL.items():
            self.assertIn(marker, REPLY_FOR_CMD[cmd],
                          "marker %r missing from %r reply" % (marker, cmd))

    def test_commands_in_console_cmd_table(self):
        for cmd in POLL:
            self.assertIn('"%s"' % cmd, SRC,
                          "console.c no longer has a %r command" % cmd)


class TestConsoleSourceAnchors(unittest.TestCase):
    """Anchor substrings and line ordering in App/console.c that the
    dashboard relies on.  Editing console.c output must keep these."""

    def test_anchor_substrings_present(self):
        for anchor in ("sense=", "last_fix", "DIO1", "baro:"):
            self.assertIn(anchor, SRC,
                          "console.c lost dashboard anchor %r" % anchor)

    def test_status_health_line_has_chg_gps(self):
        self.assertIn("chg=%d gps=%d", SRC,
                      "status health line no longer ends with chg/gps")

    def test_status_marker_prints_after_health_line(self):
        # _txn("status", b"sense=") assumes the pyro line (with sense=) is
        # printed after the health line, so the reply is complete.
        self.assertLess(SRC.index("chg=%d gps=%d"), SRC.index("sense="))

    def test_radio_reinit_line_before_dio1_line(self):
        # _txn("radio", b"DIO1") assumes the reinit/txto counters line comes
        # before the BUSY/DIO1 line in cmd_radio.
        self.assertIn("reinit attempts", SRC)
        self.assertIn("tx timeouts", SRC)
        self.assertLess(SRC.index("reinit attempts"), SRC.index("DIO1 pin"))

    def test_gps_fix_line_before_last_fix_line(self):
        self.assertIn("fix=%d sats=%u", SRC)
        self.assertLess(SRC.index("fix=%d sats=%u"), SRC.index("last_fix"))

    def test_sensors_imu_line_before_baro_line(self):
        self.assertIn("ax=%s ay=%s az=%s", SRC)
        self.assertIn("baro: %s Pa", SRC)
        self.assertLess(SRC.index("ax=%s ay=%s az=%s"),
                        SRC.index("baro: %s Pa"))


if __name__ == "__main__":
    unittest.main()
