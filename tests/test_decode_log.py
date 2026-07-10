"""Tests for tools/decode_log.py: synthesize 68-byte log records per the
log_record_t layout (<BBHI10f3i4B2H, magic 0xA5, CRC16 over first 66 bytes),
then feed decode() streams with valid records, a corrupted-CRC record and
inter-record garbage, asserting decode counts and resync behavior."""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "tools"))

import decode_log

REC = struct.Struct("<BBHI10f3i4B2H")
MAGIC = 0xA5


class RowCollector(object):
    """Minimal csv.writer stand-in that captures rows."""

    def __init__(self):
        self.rows = []

    def writerow(self, row):
        self.rows.append(list(row))


def make_record(t_ms, state=1, flags=0x000D, agl_m=123.25, vel_ms=-4.5,
                lat_e7=391234567, lon_e7=-771234567, alt_gps_cm=12345,
                sats=9, pyro_cont=0x01, pyro_fired=0x00, batt_mv=7912):
    """Build one valid 68-byte record. Nudges t_ms until the record contains
    exactly one 0xA5 byte (the magic) so resync byte-counts in the tests are
    deterministic."""
    for bump in range(4096):
        body = REC.pack(MAGIC, state, flags, t_ms + bump,
                        0.01, -0.02, 1.0,          # ax ay az [g]
                        0.5, -0.25, 0.125,          # gx gy gz [dps]
                        101325.0, 21.5,             # press_pa temp_c
                        agl_m, vel_ms,
                        lat_e7, lon_e7, alt_gps_cm,
                        sats, pyro_cont, pyro_fired, 0,
                        batt_mv, 0)
        crc = decode_log.crc16_ccitt(body[:decode_log.CRC_LEN])
        rec = body[:decode_log.CRC_LEN] + struct.pack("<H", crc)
        if rec.count(bytes([MAGIC])) == 1:
            return rec
    raise AssertionError("could not build an 0xA5-unique record")


class TestRecordLayout(unittest.TestCase):
    def test_struct_matches_decode_log(self):
        self.assertEqual(REC.size, 68)
        self.assertEqual(decode_log.RECORD_LEN, 68)
        self.assertEqual(decode_log.CRC_LEN, 66)
        self.assertEqual(decode_log.RECORD_MAGIC, 0xA5)
        self.assertEqual(decode_log.REC_STRUCT.format,
                         REC.format.decode() if isinstance(REC.format, bytes)
                         else REC.format)


class TestDecodeClean(unittest.TestCase):
    def test_valid_stream(self):
        blob = b"".join(make_record(1000 * i) for i in range(5))
        rows = RowCollector()
        records, crc_bad, skipped, resyncs, tail = decode_log.decode(blob, rows)
        self.assertEqual(records, 5)
        self.assertEqual(crc_bad, 0)
        self.assertEqual(skipped, 0)
        self.assertEqual(resyncs, 0)
        self.assertEqual(tail, 0)
        self.assertEqual(len(rows.rows), 5)

    def test_field_values_decoded(self):
        rec = make_record(4242, state=3, sats=7, batt_mv=8100)
        rows = RowCollector()
        records, _, _, _, _ = decode_log.decode(rec, rows)
        self.assertEqual(records, 1)
        row = rows.rows[0]
        hdr = decode_log.CSV_HEADER
        self.assertEqual(row[hdr.index("state")], 3)
        self.assertEqual(row[hdr.index("state_name")], "BOOST")
        self.assertEqual(row[hdr.index("sats")], 7)
        self.assertEqual(row[hdr.index("batt_mv")], 8100)
        self.assertEqual(row[hdr.index("agl_m")], "123.25")
        self.assertEqual(row[hdr.index("lat_deg")], "39.1234567")
        self.assertEqual(row[hdr.index("lon_deg")], "-77.1234567")
        # flags 0x000D -> gps_fix=1, accel_sat=0, armed=1, sd_ok=1
        self.assertEqual(row[hdr.index("gps_fix")], 1)
        self.assertEqual(row[hdr.index("accel_sat")], 0)
        self.assertEqual(row[hdr.index("armed")], 1)
        self.assertEqual(row[hdr.index("sd_ok")], 1)


class TestDecodeCorruption(unittest.TestCase):
    def test_garbage_corruption_and_resync(self):
        good = [make_record(1000 * i) for i in range(4)]

        # Corrupt a payload byte (not the magic) of a copy of good[2].
        corrupt = bytearray(make_record(777000))
        corrupt[10] ^= 0xFF
        corrupt = bytes(corrupt)
        # Keep counts deterministic: corruption must not mint a fake magic.
        self.assertEqual(corrupt.count(bytes([MAGIC])), 1)
        self.assertEqual(corrupt[0], MAGIC)

        garbage = bytes([0x00, 0x11, 0x22, 0x33, 0xDE, 0xAD, 0xBE])  # no 0xA5
        trailing = bytes([0x01] * 10)  # partial-record tail

        blob = (good[0] + good[1] + garbage + good[2] + corrupt + good[3]
                + trailing)
        rows = RowCollector()
        records, crc_bad, skipped, resyncs, tail = decode_log.decode(blob, rows)

        # 4 valid records survive.
        self.assertEqual(records, 4)
        # The corrupted record: magic OK but CRC fails exactly once.
        self.assertEqual(crc_bad, 1)
        # Skipped: 7 garbage bytes + all 68 bytes of the corrupt record.
        self.assertEqual(skipped, 7 + 68)
        # Two contiguous resync runs (garbage run, corrupt-record run).
        self.assertEqual(resyncs, 2)
        # 10 trailing bytes reported as tail (partial record at EOF).
        self.assertEqual(tail, 10)
        # Records decoded in order, t_ms monotonic across the mess.
        t_col = decode_log.CSV_HEADER.index("t_ms")
        t_vals = [r[t_col] for r in rows.rows]
        self.assertEqual(t_vals, sorted(t_vals))

    def test_all_garbage(self):
        blob = bytes([0x5A, 0x3C] * 100)  # no magic anywhere
        rows = RowCollector()
        records, crc_bad, skipped, resyncs, tail = decode_log.decode(blob, rows)
        self.assertEqual(records, 0)
        self.assertEqual(crc_bad, 0)
        self.assertEqual(len(rows.rows), 0)
        self.assertEqual(skipped + tail, 200)


if __name__ == "__main__":
    unittest.main()
