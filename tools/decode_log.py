#!/usr/bin/env python3
"""decode_log.py - decode flight-controller SD logs (LOGnnn.BIN) to CSV.

Record format: 68-byte packed little-endian log_record_t (App/app.h):
    magic      u8    0xA5 (LOG_RECORD_MAGIC)
    state      u8    flight_state_t
    flags      u16   low 8 = protocol FLAG_*, high 8 reserved
    t_ms       u32
    ax ay az   f32   [g]
    gx gy gz   f32   [dps]
    press_pa   f32
    temp_c     f32
    agl_m      f32
    vel_ms     f32
    lat_e7     i32   deg * 1e7
    lon_e7     i32   deg * 1e7
    alt_gps_cm i32   MSL
    sats       u8
    pyro_cont  u8    bitfield
    pyro_fired u8    bitfield
    pad        u8
    batt_mv    u16
    crc16      u16   CRC16-CCITT (poly 0x1021, init 0xFFFF) over first 66 bytes

Corruption handling: on bad magic or bad CRC the scanner slides forward one
byte at a time until a valid record is found; skipped bytes are counted and
reported.

Usage:
    python decode_log.py LOG001.BIN [out.csv]
(default output: input path with extension replaced by .csv)

No third-party dependencies.
"""

import csv
import os
import struct
import sys

RECORD_MAGIC = 0xA5
RECORD_LEN = 68
CRC_LEN = 66  # bytes covered by crc16

# <BBHI10f3i4B2H : 1+1+2+4 + 40 + 12 + 4 + 2+2 = 68
REC_STRUCT = struct.Struct("<BBHI10f3i4B2H")
assert REC_STRUCT.size == RECORD_LEN

STATE_NAMES = [
    "INIT", "GROUND_IDLE", "ARMED", "BOOST", "COAST",
    "APOGEE", "DROGUE", "MAIN", "DESCENT", "LANDED", "FAULT",
]

# protocol.h status flags (low byte of record flags)
FLAG_GPS_FIX = 1 << 0
FLAG_ACCEL_SAT = 1 << 1
FLAG_ARMED = 1 << 2
FLAG_SD_OK = 1 << 3

CSV_HEADER = [
    "rec", "t_ms", "t_s", "state", "state_name",
    "flags_hex", "gps_fix", "accel_sat", "armed", "sd_ok",
    "ax_g", "ay_g", "az_g",
    "gx_dps", "gy_dps", "gz_dps",
    "press_pa", "temp_c", "agl_m", "vel_ms",
    "lat_deg", "lon_deg", "alt_gps_m",
    "sats", "pyro_cont", "pyro_fired", "batt_mv",
]


def _make_crc_table():
    tbl = []
    for byte in range(256):
        c = byte << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) if (c & 0x8000) else (c << 1)
            c &= 0xFFFF
        tbl.append(c)
    return tbl


_CRC_TABLE = _make_crc_table()


def crc16_ccitt(data):
    """CRC16-CCITT, poly 0x1021, init 0xFFFF (matches shared/crc16.h)."""
    c = 0xFFFF
    for b in data:
        c = ((c << 8) & 0xFFFF) ^ _CRC_TABLE[((c >> 8) ^ b) & 0xFF]
    return c


def state_name(s):
    return STATE_NAMES[s] if s < len(STATE_NAMES) else "UNKNOWN(%d)" % s


def decode(blob, writer):
    """Scan blob, write CSV rows. Returns (records, crc_bad, skipped, resyncs, tail)."""
    n = len(blob)
    i = 0
    records = 0
    crc_bad = 0          # candidates with good magic but failed CRC
    skipped = 0          # total bytes slid past during resync
    resyncs = 0          # contiguous skip runs
    in_skip = False

    while i + RECORD_LEN <= n:
        if blob[i] != RECORD_MAGIC:
            skipped += 1
            if not in_skip:
                resyncs += 1
                in_skip = True
            i += 1
            continue
        rec = blob[i:i + RECORD_LEN]
        fields = REC_STRUCT.unpack(rec)
        crc_stored = fields[-1]
        if crc16_ccitt(rec[:CRC_LEN]) != crc_stored:
            crc_bad += 1
            skipped += 1
            if not in_skip:
                resyncs += 1
                in_skip = True
            i += 1
            continue

        in_skip = False
        (_magic, state, flags, t_ms,
         ax, ay, az, gx, gy, gz,
         press_pa, temp_c, agl_m, vel_ms,
         lat_e7, lon_e7, alt_gps_cm,
         sats, pyro_cont, pyro_fired, _pad,
         batt_mv, _crc) = fields

        writer.writerow([
            records, t_ms, "%.3f" % (t_ms / 1000.0),
            state, state_name(state),
            "0x%04X" % flags,
            (flags >> 0) & 1, (flags >> 1) & 1, (flags >> 2) & 1, (flags >> 3) & 1,
            "%.4f" % ax, "%.4f" % ay, "%.4f" % az,
            "%.3f" % gx, "%.3f" % gy, "%.3f" % gz,
            "%.2f" % press_pa, "%.2f" % temp_c,
            "%.2f" % agl_m, "%.3f" % vel_ms,
            "%.7f" % (lat_e7 / 1e7), "%.7f" % (lon_e7 / 1e7),
            "%.2f" % (alt_gps_cm / 100.0),
            sats, "0x%02X" % pyro_cont, "0x%02X" % pyro_fired, batt_mv,
        ])
        records += 1
        i += RECORD_LEN

    tail = n - i
    return records, crc_bad, skipped, resyncs, tail


def main(argv):
    if len(argv) < 2 or len(argv) > 3 or argv[1] in ("-h", "--help"):
        print("usage: %s in.bin [out.csv]" % os.path.basename(argv[0]),
              file=sys.stderr)
        return 1

    in_path = argv[1]
    out_path = argv[2] if len(argv) == 3 else os.path.splitext(in_path)[0] + ".csv"

    try:
        with open(in_path, "rb") as f:
            blob = f.read()
    except OSError as e:
        print("error: cannot read %s: %s" % (in_path, e), file=sys.stderr)
        return 2

    if not blob:
        print("error: %s is empty" % in_path, file=sys.stderr)
        return 2

    try:
        with open(out_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(CSV_HEADER)
            records, crc_bad, skipped, resyncs, tail = decode(blob, writer)
    except OSError as e:
        print("error: cannot write %s: %s" % (out_path, e), file=sys.stderr)
        return 2

    print("%s: %d bytes -> %s" % (in_path, len(blob), out_path))
    print("  records decoded : %d" % records)
    print("  crc failures    : %d (magic OK, bad CRC)" % crc_bad)
    print("  bytes skipped   : %d in %d resync run(s)" % (skipped, resyncs))
    if tail:
        print("  trailing bytes  : %d (partial record at EOF)" % tail)
    if records == 0:
        print("warning: no valid records found - wrong file or format mismatch?",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
