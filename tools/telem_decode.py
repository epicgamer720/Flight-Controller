#!/usr/bin/env python3
"""telem_decode.py - parse/build the binary LoRa packets from shared/protocol.h.

Layouts (packed little-endian, byte-for-byte identical to the C structs):

telem_packet_t (48 bytes, struct fmt <BBBBIiiiih3h3hHBBBBhH):
    magic            u8      LINK_MAGIC 0x52
    version          u8      PROTO_VERSION 1
    type             u8      packet_type_t (PKT_TELEMETRY / PKT_EVENT)
    state            u8      flight_state_t
    t_ms             u32     ms since boot
    lat_e7           i32     deg * 1e7
    lon_e7           i32     deg * 1e7
    alt_baro_cm      i32     AGL, cm
    alt_gps_cm       i32     MSL, cm
    vel_up_cms       i16     vertical, cm/s
    accel_g_x100[3]  i16x3   g*100 (may rail at +/-1600 = 16 g)
    gyro_dps_x10[3]  i16x3   deg/s * 10
    batt_mv          u16
    pyro_cont        u8      continuity bitfield
    pyro_fired       u8      fired bitfield
    sats             u8
    flags            u8      FLAG_*
    mcu_temp_c10     i16     STM32 die temp * 10 (MCU_TEMP_NODATA if unread)
    crc16            u16     CRC16-CCITT over all preceding bytes

command_packet_t (14 bytes, struct fmt <BBBBIIH):
    magic u8, version u8, type u8 (PKT_COMMAND), cmd u8,
    arg u32, nonce u32, crc16 u16

CRC16-CCITT: poly 0x1021, init 0xFFFF (bit-serial port of shared/crc16.h).

Usable as a library (ground-station bridge) or as a CLI:
    python telem_decode.py <hex bytes of one packet>
e.g.
    python telem_decode.py 52011001000000002a000000 + 4 CRC hex chars
    (spaces optional; use build_command() to produce a frame with valid CRC)

No third-party dependencies (stdlib only).
"""

import struct
import sys

# ---- shared/protocol.h constants ----
LINK_MAGIC = 0x52          # 'R'
PROTO_VERSION = 3          # v3: +seq, last_evt_state/count, main_alt_m echo (54 B)
NUM_PYRO = 1
MCU_TEMP_NODATA = -32768   # mcu_temp_c10 sentinel: die-temp read failed

PKT_TELEMETRY = 0x01
PKT_EVENT = 0x02
PKT_COMMAND = 0x10
PKT_ACK = 0x11

CMD_NOP = 0
CMD_ARM = 1
CMD_DISARM = 2
CMD_TEST_FIRE = 3
CMD_SET_MAIN_ALT = 4
CMD_ZERO_BARO = 5
CMD_REBOOT = 6
CMD_ENTER_BOOTLOADER = 7
CMD_SET_TX_POWER = 8

CMD_NAMES = {
    CMD_NOP: "NOP", CMD_ARM: "ARM", CMD_DISARM: "DISARM",
    CMD_TEST_FIRE: "TEST_FIRE", CMD_SET_MAIN_ALT: "SET_MAIN_ALT",
    CMD_ZERO_BARO: "ZERO_BARO", CMD_REBOOT: "REBOOT",
    CMD_ENTER_BOOTLOADER: "ENTER_BOOTLOADER", CMD_SET_TX_POWER: "SET_TX_POWER",
}

STATE_NAMES = [
    "INIT", "GROUND_IDLE", "ARMED", "BOOST", "COAST",
    "APOGEE", "DROGUE", "MAIN", "DESCENT", "LANDED", "FAULT",
]

FLAG_GPS_FIX = 1 << 0
FLAG_ACCEL_SAT = 1 << 1
FLAG_ARMED = 1 << 2
FLAG_SD_OK = 1 << 3
FLAG_CHG_OK = 1 << 4
FLAG_GPS_OK = 1 << 5
FLAG_LOW_BATT = 1 << 6
FLAG_DEPLOY_FAIL = 1 << 7

# ---- struct layouts ----
# ...mcu_temp_c10(h) | seq(H) evt_state(B) evt_count(B) main_alt_m(H) | crc(H)
TELEM_STRUCT = struct.Struct("<BBBBIiiiih3h3hHBBBBhHBBHH")
TELEM_LEN = TELEM_STRUCT.size          # 54
CMD_STRUCT = struct.Struct("<BBBBIIH")
CMD_LEN = CMD_STRUCT.size              # 14

assert TELEM_LEN == 54, "telem_packet_t must be 54 bytes"
assert CMD_LEN == 14, "command_packet_t must be 14 bytes"


def crc16_ccitt(data):
    """CRC16-CCITT, poly 0x1021, init 0xFFFF.

    Bit-serial port of shared/crc16.h (kept structurally identical on
    purpose so it can be diffed against the C).
    """
    c = 0xFFFF
    for byte in data:
        c ^= byte << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) if (c & 0x8000) else (c << 1)
            c &= 0xFFFF
    return c


def state_name(s):
    return STATE_NAMES[s] if 0 <= s < len(STATE_NAMES) else "UNKNOWN(%d)" % s


def _check_frame(buf, expect_len, name):
    if len(buf) != expect_len:
        raise ValueError("%s: expected %d bytes, got %d"
                         % (name, expect_len, len(buf)))
    if buf[0] != LINK_MAGIC:
        raise ValueError("%s: bad magic 0x%02X (expected 0x%02X)"
                         % (name, buf[0], LINK_MAGIC))
    if buf[1] != PROTO_VERSION:
        raise ValueError("%s: bad version %d (expected %d)"
                         % (name, buf[1], PROTO_VERSION))
    stored = struct.unpack_from("<H", buf, expect_len - 2)[0]
    calc = crc16_ccitt(buf[:expect_len - 2])
    if stored != calc:
        raise ValueError("%s: bad CRC (stored 0x%04X, computed 0x%04X)"
                         % (name, stored, calc))


def parse_telem(buf):
    """Parse a 54-byte telem_packet_t. Returns a dict with both raw fields
    and engineering-unit fields. Raises ValueError on bad length / magic /
    version / CRC."""
    buf = bytes(buf)
    _check_frame(buf, TELEM_LEN, "telem_packet_t")

    (magic, version, ptype, state, t_ms,
     lat_e7, lon_e7, alt_baro_cm, alt_gps_cm,
     vel_up_cms, ax, ay, az, gx, gy, gz,
     batt_mv, pyro_cont, pyro_fired, sats, flags,
     mcu_temp_c10, seq, last_evt_state, last_evt_count, main_alt_m,
     crc) = TELEM_STRUCT.unpack(buf)

    return {
        # raw fields (exact wire values)
        "magic": magic,
        "version": version,
        "type": ptype,
        "state": state,
        "t_ms": t_ms,
        "lat_e7": lat_e7,
        "lon_e7": lon_e7,
        "alt_baro_cm": alt_baro_cm,
        "alt_gps_cm": alt_gps_cm,
        "vel_up_cms": vel_up_cms,
        "accel_g_x100": [ax, ay, az],
        "gyro_dps_x10": [gx, gy, gz],
        "batt_mv": batt_mv,
        "pyro_cont": pyro_cont,
        "pyro_fired": pyro_fired,
        "sats": sats,
        "flags": flags,
        "mcu_temp_c10": mcu_temp_c10,
        "seq": seq,
        "last_evt_state": last_evt_state,
        "last_evt_count": last_evt_count,
        "main_alt_m": main_alt_m,
        "crc16": crc,
        # engineering units
        "state_name": state_name(state),
        "last_evt_name": state_name(last_evt_state),
        "lat_deg": lat_e7 / 1e7,
        "lon_deg": lon_e7 / 1e7,
        "alt_baro_m": alt_baro_cm / 100.0,
        "alt_gps_m": alt_gps_cm / 100.0,
        "vel_up_ms": vel_up_cms / 100.0,
        "accel_g": [ax / 100.0, ay / 100.0, az / 100.0],
        "gyro_dps": [gx / 10.0, gy / 10.0, gz / 10.0],
        "batt_v": batt_mv / 1000.0,
        "mcu_temp_c": (None if mcu_temp_c10 == MCU_TEMP_NODATA
                       else mcu_temp_c10 / 10.0),
        # flag booleans
        "gps_fix": bool(flags & FLAG_GPS_FIX),
        "accel_sat": bool(flags & FLAG_ACCEL_SAT),
        "armed": bool(flags & FLAG_ARMED),
        "sd_ok": bool(flags & FLAG_SD_OK),
        "chg_ok": bool(flags & FLAG_CHG_OK),
        "gps_ok": bool(flags & FLAG_GPS_OK),
        "low_batt": bool(flags & FLAG_LOW_BATT),
        "deploy_fail": bool(flags & FLAG_DEPLOY_FAIL),
    }


def build_telem(state=0, t_ms=0, lat_e7=0, lon_e7=0, alt_baro_cm=0,
                alt_gps_cm=0, vel_up_cms=0, accel_g_x100=(0, 0, 0),
                gyro_dps_x10=(0, 0, 0), batt_mv=0, pyro_cont=0,
                pyro_fired=0, sats=0, flags=0, mcu_temp_c10=MCU_TEMP_NODATA,
                seq=0, last_evt_state=0, last_evt_count=0, main_alt_m=0,
                ptype=PKT_TELEMETRY):
    """Build a valid 54-byte telem_packet_t from raw wire values
    (useful for GS/bridge simulation and tests)."""
    ax, ay, az = accel_g_x100
    gx, gy, gz = gyro_dps_x10
    body = struct.pack("<BBBBIiiiih3h3hHBBBBhHBBH",
                       LINK_MAGIC, PROTO_VERSION, ptype, state, t_ms,
                       lat_e7, lon_e7, alt_baro_cm, alt_gps_cm,
                       vel_up_cms, ax, ay, az, gx, gy, gz,
                       batt_mv, pyro_cont, pyro_fired, sats, flags,
                       mcu_temp_c10, seq, last_evt_state, last_evt_count, main_alt_m)
    return body + struct.pack("<H", crc16_ccitt(body))


def build_command(cmd, arg=0, nonce=0):
    """Build a valid 14-byte command_packet_t (GS -> FC uplink)."""
    body = struct.pack("<BBBBII", LINK_MAGIC, PROTO_VERSION, PKT_COMMAND,
                       cmd, arg & 0xFFFFFFFF, nonce & 0xFFFFFFFF)
    return body + struct.pack("<H", crc16_ccitt(body))


def parse_command(buf):
    """Parse a 14-byte command_packet_t. Raises ValueError on bad
    length / magic / version / CRC."""
    buf = bytes(buf)
    _check_frame(buf, CMD_LEN, "command_packet_t")
    if buf[2] != PKT_COMMAND:  # firmware handle_command() drops these too
        raise ValueError("command_packet_t: bad type 0x%02X (expected 0x%02X)"
                         % (buf[2], PKT_COMMAND))
    magic, version, ptype, cmd, arg, nonce, crc = CMD_STRUCT.unpack(buf)
    return {
        "magic": magic, "version": version, "type": ptype,
        "cmd": cmd, "cmd_name": CMD_NAMES.get(cmd, "UNKNOWN(%d)" % cmd),
        "arg": arg, "nonce": nonce, "crc16": crc,
    }


def parse_packet(buf):
    """Dispatch on length: 46 -> parse_telem, 14 -> parse_command."""
    buf = bytes(buf)
    if len(buf) == TELEM_LEN:
        return parse_telem(buf)
    if len(buf) == CMD_LEN:
        return parse_command(buf)
    raise ValueError("packet length %d is neither telem (%d) nor command (%d)"
                     % (len(buf), TELEM_LEN, CMD_LEN))


def _cli(argv):
    if len(argv) < 2 or argv[1] in ("-h", "--help"):
        print("usage: %s <hex bytes of one packet>" % argv[0], file=sys.stderr)
        print("  telem_packet_t = %d bytes, command_packet_t = %d bytes"
              % (TELEM_LEN, CMD_LEN), file=sys.stderr)
        return 1

    hexstr = "".join(argv[1:]).replace(" ", "").replace(",", "")
    try:
        buf = bytes.fromhex(hexstr)
    except ValueError as e:
        print("error: bad hex input: %s" % e, file=sys.stderr)
        return 2

    try:
        fields = parse_packet(buf)
    except ValueError as e:
        print("error: %s" % e, file=sys.stderr)
        return 3

    print("%d-byte %s" % (len(buf),
          "telem_packet_t" if len(buf) == TELEM_LEN else "command_packet_t"))
    for key, val in fields.items():
        if isinstance(val, float):
            print("  %-14s %.7g" % (key, val))
        elif isinstance(val, list):
            print("  %-14s [%s]" % (key, ", ".join("%g" % v for v in val)))
        elif key in ("magic", "flags", "pyro_cont", "pyro_fired", "crc16"):
            print("  %-14s 0x%02X" % (key, val))
        else:
            print("  %-14s %s" % (key, val))
    return 0


if __name__ == "__main__":
    sys.exit(_cli(sys.argv))
