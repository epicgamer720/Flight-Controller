"""Round-trip tests for tools/telem_decode.py against the wire layouts in
shared/protocol.h (telem_packet_t 48 B, command_packet_t 14 B)."""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "tools"))

import telem_decode as td


def pack_telem_raw(magic=td.LINK_MAGIC, version=td.PROTO_VERSION,
                   ptype=td.PKT_TELEMETRY, state=4, t_ms=123456,
                   lat_e7=391234567, lon_e7=-771234567,
                   alt_baro_cm=152435, alt_gps_cm=165000,
                   vel_up_cms=-1234, accel=(-25, 50, 1600),
                   gyro=(15, -100, 3), batt_mv=7912,
                   pyro_cont=0x01, pyro_fired=0x00, sats=11, flags=0x0F,
                   mcu_temp_c10=235, seq=0, last_evt_state=0, last_evt_count=0,
                   main_alt_m=150, crc=None):
    """Build a telem packet with raw struct.pack, independent of the module
    under test (only the CRC routine is shared when crc is None)."""
    body = struct.pack("<BBBBIiiiih3h3hHBBBBhHBBH",
                       magic, version, ptype, state, t_ms,
                       lat_e7, lon_e7, alt_baro_cm, alt_gps_cm, vel_up_cms,
                       accel[0], accel[1], accel[2],
                       gyro[0], gyro[1], gyro[2],
                       batt_mv, pyro_cont, pyro_fired, sats, flags,
                       mcu_temp_c10, seq, last_evt_state, last_evt_count, main_alt_m)
    if crc is None:
        crc = td.crc16_ccitt(body)
    return body + struct.pack("<H", crc)


class TestLayoutSizes(unittest.TestCase):
    def test_struct_sizes(self):
        self.assertEqual(struct.calcsize("<BBBBIiiiih3h3hHBBBBhHBBHH"), 54)
        self.assertEqual(struct.calcsize("<BBBBIIH"), 14)
        self.assertEqual(td.TELEM_LEN, 54)
        self.assertEqual(td.CMD_LEN, 14)

    def test_protocol_constants(self):
        self.assertEqual(td.LINK_MAGIC, 0x52)
        self.assertEqual(td.PROTO_VERSION, 3)
        self.assertEqual(td.PKT_COMMAND, 0x10)
        self.assertEqual(td.CMD_SET_TX_POWER, 8)


class TestParseTelem(unittest.TestCase):
    def test_round_trip_engineering_units(self):
        buf = pack_telem_raw()
        d = td.parse_telem(buf)

        # raw fields
        self.assertEqual(d["magic"], 0x52)
        self.assertEqual(d["version"], 3)
        self.assertEqual(d["type"], td.PKT_TELEMETRY)
        self.assertEqual(d["state"], 4)
        self.assertEqual(d["state_name"], "COAST")
        self.assertEqual(d["t_ms"], 123456)
        self.assertEqual(d["lat_e7"], 391234567)
        self.assertEqual(d["lon_e7"], -771234567)
        self.assertEqual(d["alt_baro_cm"], 152435)
        self.assertEqual(d["alt_gps_cm"], 165000)
        self.assertEqual(d["vel_up_cms"], -1234)
        self.assertEqual(d["accel_g_x100"], [-25, 50, 1600])
        self.assertEqual(d["gyro_dps_x10"], [15, -100, 3])
        self.assertEqual(d["batt_mv"], 7912)
        self.assertEqual(d["pyro_cont"], 0x01)
        self.assertEqual(d["pyro_fired"], 0x00)
        self.assertEqual(d["sats"], 11)
        self.assertEqual(d["flags"], 0x0F)
        self.assertEqual(d["mcu_temp_c10"], 235)

        # engineering units
        self.assertAlmostEqual(d["lat_deg"], 39.1234567, places=7)
        self.assertAlmostEqual(d["lon_deg"], -77.1234567, places=7)
        self.assertAlmostEqual(d["alt_baro_m"], 1524.35, places=6)
        self.assertAlmostEqual(d["alt_gps_m"], 1650.0, places=6)
        self.assertAlmostEqual(d["vel_up_ms"], -12.34, places=6)
        self.assertEqual(d["accel_g"], [-0.25, 0.5, 16.0])
        self.assertEqual(d["gyro_dps"], [1.5, -10.0, 0.3])
        self.assertAlmostEqual(d["batt_v"], 7.912, places=6)
        self.assertAlmostEqual(d["mcu_temp_c"], 23.5, places=6)

        # flag booleans (0x0F = fix|sat|armed|sd)
        self.assertTrue(d["gps_fix"])
        self.assertTrue(d["accel_sat"])
        self.assertTrue(d["armed"])
        self.assertTrue(d["sd_ok"])

    def test_flags_clear(self):
        d = td.parse_telem(pack_telem_raw(flags=0x00))
        self.assertFalse(d["gps_fix"] or d["accel_sat"]
                         or d["armed"] or d["sd_ok"])

    def test_bad_crc_raises(self):
        buf = bytearray(pack_telem_raw())
        buf[-1] ^= 0xFF
        with self.assertRaises(ValueError):
            td.parse_telem(bytes(buf))

    def test_corrupt_payload_raises(self):
        buf = bytearray(pack_telem_raw())
        buf[10] ^= 0x01  # payload flip -> CRC mismatch
        with self.assertRaises(ValueError):
            td.parse_telem(bytes(buf))

    def test_bad_magic_raises(self):
        with self.assertRaises(ValueError):
            td.parse_telem(pack_telem_raw(magic=0x53))

    def test_bad_version_raises(self):
        with self.assertRaises(ValueError):
            td.parse_telem(pack_telem_raw(version=1))    # v1 rejected by v2

    def test_bad_length_raises(self):
        with self.assertRaises(ValueError):
            td.parse_telem(pack_telem_raw()[:47])
        with self.assertRaises(ValueError):
            td.parse_telem(pack_telem_raw() + b"\x00")

    def test_mcu_temp_sentinel_is_none(self):
        d = td.parse_telem(pack_telem_raw(mcu_temp_c10=td.MCU_TEMP_NODATA))
        self.assertEqual(d["mcu_temp_c10"], td.MCU_TEMP_NODATA)
        self.assertIsNone(d["mcu_temp_c"])
        # build_telem default is the sentinel -> None
        self.assertIsNone(td.parse_telem(td.build_telem())["mcu_temp_c"])

    def test_build_telem_parses_back(self):
        buf = td.build_telem(state=1, t_ms=99, alt_baro_cm=-150,
                             vel_up_cms=25, sats=6, flags=td.FLAG_GPS_FIX,
                             mcu_temp_c10=418)
        self.assertEqual(len(buf), 54)
        d = td.parse_telem(buf)
        self.assertEqual(d["state_name"], "GROUND_IDLE")
        self.assertAlmostEqual(d["alt_baro_m"], -1.5, places=6)
        self.assertAlmostEqual(d["vel_up_ms"], 0.25, places=6)
        self.assertAlmostEqual(d["mcu_temp_c"], 41.8, places=6)
        self.assertTrue(d["gps_fix"])
        self.assertFalse(d["armed"])


class TestBuildCommand(unittest.TestCase):
    def test_output_verified_by_hand(self):
        buf = td.build_command(td.CMD_SET_MAIN_ALT, arg=20000,
                               nonce=0xDEADBEEF)
        self.assertEqual(len(buf), 14)

        # Re-parse by hand with struct, independent of the module.
        magic, version, ptype, cmd, arg, nonce, crc = \
            struct.unpack("<BBBBIIH", buf)
        self.assertEqual(magic, 0x52)
        self.assertEqual(version, 3)
        self.assertEqual(ptype, 0x10)          # PKT_COMMAND
        self.assertEqual(cmd, td.CMD_SET_MAIN_ALT)
        self.assertEqual(arg, 20000)
        self.assertEqual(nonce, 0xDEADBEEF)

        # CRC16-CCITT over all-but-last-2 bytes, hand-rolled here.
        c = 0xFFFF
        for b in buf[:-2]:
            c ^= b << 8
            for _ in range(8):
                c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 \
                    else (c << 1) & 0xFFFF
        self.assertEqual(crc, c)

    def test_parse_command_round_trip(self):
        buf = td.build_command(td.CMD_ARM, nonce=7)
        d = td.parse_command(buf)
        self.assertEqual(d["cmd"], td.CMD_ARM)
        self.assertEqual(d["cmd_name"], "ARM")
        self.assertEqual(d["arg"], 0)
        self.assertEqual(d["nonce"], 7)

    def test_tampered_command_raises(self):
        buf = bytearray(td.build_command(td.CMD_DISARM, nonce=1))
        buf[3] = td.CMD_ARM  # try to flip DISARM -> ARM without fixing CRC
        with self.assertRaises(ValueError):
            td.parse_command(bytes(buf))

    def test_parse_packet_dispatch(self):
        self.assertIn("cmd_name", td.parse_packet(td.build_command(0)))
        self.assertIn("state_name", td.parse_packet(pack_telem_raw()))
        with self.assertRaises(ValueError):
            td.parse_packet(b"\x52\x01\x10")


if __name__ == "__main__":
    unittest.main()
