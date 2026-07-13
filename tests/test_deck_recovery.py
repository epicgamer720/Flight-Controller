#!/usr/bin/env python3
"""Unit tests for the deck server's recovery-bearing/distance helpers and
the pad-fix / GPS-apogee logic in Deck._update_peaks / Deck._recovery.

No server, no source, no threads: Deck(record=False) is side-effect-free
(the poll thread only starts in set_source), so we drive the real methods
with hand-built `d` dicts shaped like Source.drain() output:
    d = {"series": {...}, "latest": {flat telem dict}, "events": [{...}]}
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "tools"))

from deck.server import Deck, _bearing_deg, _haversine_m

# spherical metres per degree of latitude, used to place points a known
# distance away (matches _haversine_m's own earth radius exactly).
M_PER_DEG_LAT = _haversine_m(0.0, 0.0, 1.0, 0.0)

PAD_LAT, PAD_LON, PAD_ALT = 40.0, -105.0, 1600.0   # mid-lat, plausible pad


def _d(latest=None, events=None, series=None):
    """A Source.drain()-shaped dict. series is always present because
    _update_peaks indexes d["series"] directly."""
    return {"series": series or {}, "latest": latest or {},
            "events": events or []}


def _adiff(a, b):
    """Smallest absolute angular difference in degrees (handles 0/360)."""
    return abs((a - b + 180.0) % 360.0 - 180.0)


class TestHaversine(unittest.TestCase):
    def test_one_degree_latitude(self):
        # 1 deg of latitude ~= 111.2 km, within 0.5%.
        d = _haversine_m(0.0, 0.0, 1.0, 0.0)
        self.assertLess(abs(d - 111200.0) / 111200.0, 0.005)

    def test_short_baseline_100m(self):
        # point exactly 100 m north of the pad -> distance ~= 100 m.
        d = _haversine_m(PAD_LAT, PAD_LON,
                         PAD_LAT + 100.0 / M_PER_DEG_LAT, PAD_LON)
        self.assertAlmostEqual(d, 100.0, delta=0.5)

    def test_zero_distance_to_self(self):
        self.assertEqual(_haversine_m(PAD_LAT, PAD_LON, PAD_LAT, PAD_LON), 0.0)


class TestBearing(unittest.TestCase):
    D = 0.001   # ~100 m offset near mid-latitude

    def test_cardinals(self):
        for name, dlat, dlon, expect in (
                ("north", self.D, 0.0, 0.0),
                ("east", 0.0, self.D, 90.0),
                ("south", -self.D, 0.0, 180.0),
                ("west", 0.0, -self.D, 270.0)):
            b = _bearing_deg(PAD_LAT, PAD_LON,
                             PAD_LAT + dlat, PAD_LON + dlon)
            self.assertLess(_adiff(b, expect), 1.0,
                            "%s: got %.3f expected %.1f" % (name, b, expect))


class TestPadFixSnapshot(unittest.TestCase):
    def test_pad_fix_frozen_on_boost(self):
        deck = Deck(record=False)
        # pre-launch good fix: sets _last_fix, but no BOOST event yet.
        deck._update_peaks(_d(latest={
            "state": "GROUND_IDLE", "gps_fix": True,
            "lat_deg": PAD_LAT, "lon_deg": PAD_LON, "alt_gps_m": PAD_ALT}))
        self.assertIsNone(deck.pad_fix, "pad_fix set before BOOST event")

        # BOOST event freezes the pad fix to the earlier fix. This frame's
        # latest carries no good fix, so _last_fix cannot be overwritten.
        deck._update_peaks(_d(
            latest={"state": "BOOST"},
            events=[{"kind": "state", "text": "ARMED -> BOOST",
                     "t_host": 12.3, "seq": 5}]))
        self.assertIsNotNone(deck.pad_fix)
        self.assertAlmostEqual(deck.pad_fix["lat"], PAD_LAT)
        self.assertAlmostEqual(deck.pad_fix["lon"], PAD_LON)
        self.assertEqual(deck.tplus["launch_t_host"], 12.3)

    def test_pad_fix_fallback_when_launch_precedes_fix(self):
        # BOOST fires before any GPS fix -> pad_fix stays None (no _last_fix yet)...
        deck = Deck(record=False)
        deck._update_peaks(_d(
            latest={"state": "BOOST"},
            events=[{"kind": "state", "text": "ARMED -> BOOST",
                     "t_host": 5.0, "seq": 2}]))
        self.assertIsNone(deck.pad_fix, "no fix existed at BOOST")
        # ...then the first fix acquired AFTER launch is adopted as the pad ref.
        deck._update_peaks(_d(latest={
            "gps_fix": True, "lat_deg": PAD_LAT, "lon_deg": PAD_LON,
            "alt_gps_m": PAD_ALT}))
        self.assertIsNotNone(deck.pad_fix, "post-launch fix must establish a pad ref")
        self.assertAlmostEqual(deck.pad_fix["lat"], PAD_LAT)


class TestRecovery(unittest.TestCase):
    def _armed_deck(self):
        """Deck with pad_fix frozen at the pad."""
        deck = Deck(record=False)
        deck._update_peaks(_d(latest={
            "gps_fix": True, "lat_deg": PAD_LAT, "lon_deg": PAD_LON,
            "alt_gps_m": PAD_ALT}))
        deck._update_peaks(_d(events=[{"kind": "state",
                                       "text": "ARMED -> BOOST",
                                       "t_host": 1.0, "seq": 1}]))
        assert deck.pad_fix is not None
        return deck

    def test_recovery_100m_north(self):
        deck = self._armed_deck()
        rec = deck._recovery({
            "gps_fix": True, "lon_deg": PAD_LON,
            "lat_deg": PAD_LAT + 100.0 / M_PER_DEG_LAT})
        self.assertAlmostEqual(rec["distance_m"], 100.0, delta=1.0)
        self.assertLess(_adiff(rec["bearing_deg"], 0.0), 1.0)
        self.assertEqual(rec["pad_lat"], PAD_LAT)
        self.assertEqual(rec["pad_lon"], PAD_LON)

    def test_recovery_none_without_pad_fix(self):
        deck = Deck(record=False)     # no pad fix
        self.assertIsNone(deck._recovery({
            "gps_fix": True, "lat_deg": PAD_LAT, "lon_deg": PAD_LON}))

    def test_recovery_none_without_gps_fix(self):
        deck = self._armed_deck()
        self.assertIsNone(deck._recovery({
            "gps_fix": False, "lat_deg": PAD_LAT, "lon_deg": PAD_LON}))


class TestGpsApogee(unittest.TestCase):
    def test_apogee_gps_captures_max_agl(self):
        deck = Deck(record=False)
        deck._update_peaks(_d(latest={
            "gps_fix": True, "lat_deg": PAD_LAT, "lon_deg": PAD_LON,
            "alt_gps_m": PAD_ALT}))
        deck._update_peaks(_d(events=[{"kind": "state",
                                       "text": "ARMED -> BOOST",
                                       "t_host": 1.0, "seq": 1}]))
        # rising then falling MSL altitude; peak AGL is 300 m (1900 - 1600).
        for alt in (1650.0, 1750.0, 1900.0, 1800.0, 1650.0):
            deck._update_peaks(_d(latest={"alt_gps_m": alt}))
        self.assertAlmostEqual(deck.peaks["apogee_gps_m"], 300.0, delta=0.01)


if __name__ == "__main__":
    unittest.main()
