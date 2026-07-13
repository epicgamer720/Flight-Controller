#!/usr/bin/env python3
"""Mirror test for param_store.c's v2 flash record (28-byte slot).

No host C compiler on this box, so this transcribes the slot encoding +
newest-seq-wins selection and asserts round-trip + CRC/version validation.
Keep in lockstep with flight-controller/App/param_store.c (the param_rec_t
layout, PARAM_MAGIC/PARAM_VER/PARAM_PAYLOAD_LEN, and scan()).

Run: py -m unittest tests.test_param_store
"""
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
from telem_decode import crc16_ccitt   # same CRC16-CCITT the firmware uses

PARAM_MAGIC = 0x5250
PARAM_VER = 2
PARAM_PAYLOAD_LEN = 20
SLOT_BYTES = 28
# magic H, ver B, len B, main_alt I, apogee I, lat i, lon i, seq I, spare 2x, crc H
_BODY = "<HBBIIiiI2x"          # 26 bytes (everything the CRC covers)


def pack_slot(main_alt_cm, apogee_ms, lat_e7, lon_e7, seq,
              magic=PARAM_MAGIC, ver=PARAM_VER, length=PARAM_PAYLOAD_LEN):
    body = struct.pack(_BODY, magic, ver, length, main_alt_cm, apogee_ms,
                       lat_e7, lon_e7, seq)
    assert len(body) == SLOT_BYTES - 2, len(body)
    return body + struct.pack("<H", crc16_ccitt(body))


def slot_valid(slot):
    if len(slot) != SLOT_BYTES:
        return None
    magic, ver, length, main_alt, apogee, lat, lon, seq = struct.unpack(_BODY, slot[:-2])
    (crc,) = struct.unpack("<H", slot[-2:])
    if magic != PARAM_MAGIC or ver != PARAM_VER or length != PARAM_PAYLOAD_LEN:
        return None
    if crc16_ccitt(slot[:-2]) != crc:
        return None
    return dict(main_alt_cm=main_alt, apogee_timeout_ms=apogee,
                last_lat_e7=lat, last_lon_e7=lon, seq=seq)


def newest(slots):
    """Mirror scan(): highest-seq CRC-valid record wins."""
    best = None
    for s in slots:
        r = slot_valid(s)
        if r and (best is None or r["seq"] > best["seq"]):
            best = r
    return best


class ParamSlot(unittest.TestCase):
    def test_size(self):
        self.assertEqual(len(pack_slot(15000, 30000, 391234567, -771234567, 1)), SLOT_BYTES)
        self.assertEqual(SLOT_BYTES % 4, 0)   # word-aligned for flash programming

    def test_round_trip(self):
        s = pack_slot(15000, 22000, 391234567, -771234567, 7)
        r = slot_valid(s)
        self.assertIsNotNone(r)
        self.assertEqual(r["main_alt_cm"], 15000)
        self.assertEqual(r["apogee_timeout_ms"], 22000)
        self.assertEqual(r["last_lat_e7"], 391234567)
        self.assertEqual(r["last_lon_e7"], -771234567)   # negative lon survives
        self.assertEqual(r["seq"], 7)

    def test_bad_crc_rejected(self):
        s = bytearray(pack_slot(15000, 30000, 1, 2, 1))
        s[4] ^= 0xFF                          # corrupt main_alt
        self.assertIsNone(slot_valid(bytes(s)))

    def test_v1_record_rejected(self):
        # a v1 firmware wrote ver=1 — must be ignored after the bump
        s = pack_slot(15000, 30000, 0, 0, 1, ver=1)
        self.assertIsNone(slot_valid(s))

    def test_newest_seq_wins(self):
        slots = [pack_slot(10000, 30000, 0, 0, 1),
                 pack_slot(20000, 25000, 0, 0, 5),   # newest
                 pack_slot(15000, 30000, 0, 0, 3)]
        r = newest(slots)
        self.assertEqual(r["seq"], 5)
        self.assertEqual(r["main_alt_cm"], 20000)
        self.assertEqual(r["apogee_timeout_ms"], 25000)

    def test_no_valid_record(self):
        blank = b"\xff" * SLOT_BYTES
        self.assertIsNone(newest([blank, blank]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
