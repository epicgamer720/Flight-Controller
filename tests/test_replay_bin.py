#!/usr/bin/env python3
"""Test ReplaySource replaying an SD LOGnnn.BIN log (Phase 9a).

Build a tiny valid 68-byte-record .bin the same way tests/test_decode_log.py
does, replay it through ReplaySource(path=...).run_sync(), and assert the deck
sees one frame per record and that known fields (apogee alt, a pyro-fired
frame) round-trip through the bin -> wire -> json path."""

import os
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "tools"))

import decode_log
from deck.sources import ReplaySource

REC = struct.Struct("<BBHI10f3i4B2H")   # log_record_t, same as test_decode_log
MAGIC = 0xA5


def build_rec(state, t_ms, agl_m, vel_ms, pyro_fired=0,
              ax=0.0, ay=0.0, az=1.0, flags=0):
    """One valid 68-byte record with exact t_ms (copied from
    tests/test_decode_log.py)."""
    body = REC.pack(MAGIC, state, flags, t_ms,
                    ax, ay, az, 0.0, 0.0, 0.0,      # accel / gyro
                    101325.0, 20.0, agl_m, vel_ms,  # press temp agl vel
                    0, 0, 0,                         # lat lon alt_gps
                    0, 0, pyro_fired, 0,             # sats cont fired pad
                    7000, 0)                         # batt_mv, crc placeholder
    crc = decode_log.crc16_ccitt(body[:decode_log.CRC_LEN])
    return body[:decode_log.CRC_LEN] + struct.pack("<H", crc)


# Tiny flight: apogee 500 m at rec 3; drogue fires (0x01) at rec 4. az=16 g at
# boot (rec 1) rails the accelerometer -> exercises the int16 clamp guard.
FLIGHT = [
    build_rec(1, 0,    0.0,   0.0),                    # GROUND_IDLE
    build_rec(3, 1000, 200.0, 100.0, az=16.0),         # BOOST (railed accel)
    build_rec(4, 2000, 480.0, 5.0),                    # COAST
    build_rec(5, 3000, 500.0, -1.0),                   # APOGEE (max AGL)
    build_rec(6, 3100, 499.0, -3.0, pyro_fired=0x01),  # DROGUE, first fire
]


class TestReplayBin(unittest.TestCase):
    def _replay(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "LOG001.BIN")
            with open(path, "wb") as f:
                f.write(b"".join(FLIGHT))
            src = ReplaySource(path=path, speed=1e9)
            src.run_sync()
        return src.drain()

    def test_frame_count_and_fields_roundtrip(self):
        out = self._replay()
        alt_rows = out["series"]["alt"]
        # one telem frame (one alt row) per log record
        self.assertEqual(len(alt_rows), len(FLIGHT))
        # apogee alt_baro_m round-trips bin(agl_m) -> cm wire -> json
        self.assertAlmostEqual(max(r[1] for r in alt_rows), 500.0, places=2)
        # the railed 16 g accel survived the clamp (no struct error) and a
        # pyro-fired frame produced a "pyro" event
        kinds = [e["kind"] for e in out["events"]]
        self.assertIn("pyro", kinds)
        self.assertEqual(out["latest"]["pyro_fired"], 0x01)


# A CRC-valid record can still hold NaN/inf (e.g. a diverged Kalman alt) or a
# huge value; these must be clamped, not crash int()/round()/struct and abort
# the whole replay.
GARBAGE_FLIGHT = [
    build_rec(3, 0,    float("nan"), 100.0),                       # NaN agl
    build_rec(4, 1000, float("inf"), float("-inf"), az=float("nan")),
    build_rec(4, 2000, 1e12, 9e9),                                 # huge finite overflow
    build_rec(5, 3000, 500.0, -1.0),                               # a good record after
]


class TestReplayBinGarbage(unittest.TestCase):
    def test_nonfinite_records_do_not_abort_replay(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "LOG002.BIN")
            with open(path, "wb") as f:
                f.write(b"".join(GARBAGE_FLIGHT))
            src = ReplaySource(path=path, speed=1e9)
            src.run_sync()          # must NOT raise (ValueError/OverflowError/struct.error)
        out = src.drain()
        alt_rows = out["series"]["alt"]
        self.assertEqual(len(alt_rows), len(GARBAGE_FLIGHT))  # every record produced a frame
        # the good 500 m record still came through (garbage clamped, not crashed)
        self.assertTrue(any(abs(r[1] - 500.0) < 0.1 for r in alt_rows),
                        "the valid record after the garbage ones must survive")


if __name__ == "__main__":
    unittest.main()
