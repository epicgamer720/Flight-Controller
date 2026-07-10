#!/usr/bin/env python3
"""Recorder tests: session layout, JSON-literal cells, record -> replay
round-trip reproducing identical peaks, rotation, raw capture."""

import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "tools"))

from deck import recorder as recmod
from deck.recorder import Recorder, TELEM_COLUMNS
from deck.sources import ReplaySource


def synthetic_drains():
    """Run the synthetic flight and yield per-frame drain-shaped dicts so
    the recorder sees every frame (not poll-rate snapshots)."""
    src = ReplaySource(synthetic="nominal", speed=1e9)
    src.run_sync()
    out = src.drain()
    rows = out["series"]["alt"]
    latest = out["latest"]
    drains = []
    for i, r in enumerate(rows):
        d = dict(latest)
        d["t_host"] = r[0]
        d["alt_baro_m"] = r[1]
        drains.append({"latest": d,
                       "events": out["events"] if i == 0 else []})
    return drains, out


class TestRecorder(unittest.TestCase):
    def test_session_layout_and_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec = Recorder(recdir=tmp)
            session = rec.start()
            drains, original = synthetic_drains()
            for d in drains:
                rec.feed(d)
            rec.stop()

            telem = os.path.join(session, "telem.csv")
            events = os.path.join(session, "events.csv")
            self.assertTrue(os.path.isfile(telem))
            self.assertTrue(os.path.isfile(events))
            with open(telem, encoding="utf-8") as f:
                header = f.readline().strip().split(",")
            self.assertEqual(tuple(header), TELEM_COLUMNS)

            # record -> replay round-trip: identical apogee
            back = ReplaySource(path=telem, speed=1e9)
            back.run_sync()
            replayed = back.drain()
            orig_apogee = max(r[1] for r in original["series"]["alt"])
            got_apogee = max(r[1] for r in replayed["series"]["alt"]
                             if r[1] is not None)
            self.assertAlmostEqual(got_apogee, orig_apogee, places=2)

            with open(events, encoding="utf-8") as f:
                body = f.read()
            self.assertIn("state", body)

    def test_rotation(self):
        old = recmod.ROTATE_BYTES
        recmod.ROTATE_BYTES = 2000
        try:
            with tempfile.TemporaryDirectory() as tmp:
                rec = Recorder(recdir=tmp)
                session = rec.start()
                drains, _ = synthetic_drains()
                for d in drains:
                    rec.feed(d)
                rec.stop()
                names = os.listdir(session)
                self.assertIn("telem_part2.csv", names)
        finally:
            recmod.ROTATE_BYTES = old

    def test_raw_capture(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec = Recorder(recdir=tmp)
            session = rec.start()
            rec.start_raw()
            rec.feed_raw('{"gs":"boot","radio":"ok"}')
            rec.feed_raw('{"error":"crc16","len":46}')
            rec.stop()
            with open(os.path.join(session, "raw.jsonl"),
                      encoding="utf-8") as f:
                lines = f.read().splitlines()
            self.assertEqual(len(lines), 2)

    def test_no_double_start(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec = Recorder(recdir=tmp)
            a = rec.start()
            b = rec.start()
            self.assertEqual(a, b)
            rec.stop()
            self.assertFalse(rec.active)


if __name__ == "__main__":
    unittest.main()
