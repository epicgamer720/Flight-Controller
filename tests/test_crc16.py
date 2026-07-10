"""CRC16-CCITT golden vectors + cross-check of the two Python
implementations in tools/ (decode_log.py table-based vs telem_decode.py
bit-serial port of shared/crc16.h)."""

import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "tools"))

import decode_log
import telem_decode


class TestCrc16Golden(unittest.TestCase):
    def test_check_value_123456789(self):
        # Canonical CRC-16/CCITT-FALSE check value.
        self.assertEqual(telem_decode.crc16_ccitt(b"123456789"), 0x29B1)
        self.assertEqual(decode_log.crc16_ccitt(b"123456789"), 0x29B1)

    def test_empty_is_init(self):
        self.assertEqual(telem_decode.crc16_ccitt(b""), 0xFFFF)
        self.assertEqual(decode_log.crc16_ccitt(b""), 0xFFFF)

    def test_single_bytes(self):
        # Spot values that also pin endianness/shift direction.
        self.assertEqual(telem_decode.crc16_ccitt(b"\x00"),
                         decode_log.crc16_ccitt(b"\x00"))
        self.assertEqual(telem_decode.crc16_ccitt(b"\xff"),
                         decode_log.crc16_ccitt(b"\xff"))


class TestCrc16CrossCheck(unittest.TestCase):
    def test_random_buffers_match(self):
        rng = random.Random(0xC0FFEE)  # fixed seed: deterministic
        for _ in range(200):
            n = rng.randrange(0, 128)
            buf = bytes(rng.randrange(256) for _ in range(n))
            self.assertEqual(telem_decode.crc16_ccitt(buf),
                             decode_log.crc16_ccitt(buf),
                             "mismatch on %s" % buf.hex())

    def test_result_is_16_bit(self):
        rng = random.Random(42)
        for _ in range(50):
            buf = bytes(rng.randrange(256) for _ in range(64))
            c = telem_decode.crc16_ccitt(buf)
            self.assertTrue(0 <= c <= 0xFFFF)


if __name__ == "__main__":
    unittest.main()
