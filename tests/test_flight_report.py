"""Tests for tools/flight_report.py: synthesize a tiny LOGnnn.BIN the same way
tests/test_decode_log.py builds records (68-byte log_record_t, magic 0xA5,
CRC16 over the first 66 bytes), render a report into a tempfile, and assert the
HTML is produced, non-empty, carries the apogee value + an <svg> chart, and is
fully self-contained (no external http/https resources, no <link>/<script src>).
"""

import os
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "tools"))

import decode_log
import flight_report

REC = struct.Struct("<BBHI10f3i4B2H")
MAGIC = 0xA5


def build_rec(state, t_ms, agl_m, vel_ms, pyro_fired=0,
              ax=0.0, ay=0.0, az=1.0, lat_e7=391234567, lon_e7=-771234567):
    """One valid 68-byte log record with exact field values (real GPS coords so
    the ground-track renders too)."""
    body = REC.pack(MAGIC, state, 0, t_ms,
                    ax, ay, az, 0.0, 0.0, 0.0,
                    101325.0, 20.0, agl_m, vel_ms,
                    lat_e7, lon_e7, 12000,
                    0, 0, pyro_fired, 0,
                    7400, 0)
    crc = decode_log.crc16_ccitt(body[:decode_log.CRC_LEN])
    return body[:decode_log.CRC_LEN] + struct.pack("<H", crc)


# pad -> boost -> coast -> apogee(500 m) -> drogue(fire) -> main -> descent -> landed
FLIGHT = [
    build_rec(1, 0,     0.0,   0.0),
    build_rec(3, 1000,  50.0,  80.0, ax=16.0),          # BOOST, accel railed
    build_rec(4, 3000,  400.0, 40.0, lat_e7=391235000),
    build_rec(5, 5000,  500.0, -1.0, lat_e7=391235500),  # APOGEE (max AGL)
    build_rec(6, 5100,  499.0, -3.0, pyro_fired=0x01),   # DROGUE, first fire
    build_rec(7, 8000,  150.0, -8.0, pyro_fired=0x03, lat_e7=391234000),
    build_rec(9, 12000, 0.0,   0.0,  pyro_fired=0x03),   # LANDED
]


class TestFlightReport(unittest.TestCase):
    def _render(self, blob):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "LOG001.BIN")
            with open(path, "wb") as f:
                f.write(blob)
            out = os.path.join(d, "report.html")
            rc = flight_report.main([path, "--out", out])
            self.assertEqual(rc, 0)
            with open(out, encoding="utf-8") as f:
                return f.read()

    def test_report_is_self_contained(self):
        html = self._render(b"".join(FLIGHT))

        self.assertTrue(html.strip())                     # non-empty
        self.assertIn("500.0", html)                      # apogee value
        self.assertIn("<svg", html)                       # altitude chart
        self.assertIn("Apogee", html)

        # self-contained: no external resources anywhere.
        self.assertNotIn("http://", html)
        self.assertNotIn("https://", html)
        self.assertNotIn("<link", html)
        self.assertNotIn("<script", html)

        # GPS present in this flight -> ground-track rendered (two SVGs).
        self.assertEqual(html.count("<svg"), 2)

    def test_no_gps_notes_gracefully(self):
        # All fixes zeroed -> track skipped, note shown, still one chart SVG.
        flight = [build_rec(s, t, a, v, lat_e7=0, lon_e7=0)
                  for s, t, a, v in [(1, 0, 0.0, 0.0), (3, 1000, 300.0, 60.0),
                                     (5, 3000, 420.0, -1.0), (9, 6000, 0.0, 0.0)]]
        html = self._render(b"".join(flight))
        self.assertIn("No GPS fixes", html)
        self.assertEqual(html.count("<svg"), 1)
        self.assertIn("420.0", html)


if __name__ == "__main__":
    unittest.main()
