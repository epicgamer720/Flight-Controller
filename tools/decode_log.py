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
    python decode_log.py LOG001.BIN [out.csv]      # -> CSV
    python decode_log.py LOG001.BIN --summary       # -> flight summary
(default CSV output: input path with extension replaced by .csv)

iter_records(path) is importable and yields one decoded record dict per valid
record (magic + CRC checked, corrupt bytes skipped) for reuse by other tools.

No third-party dependencies.
"""

import argparse
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
FLAG_CHG_OK = 1 << 4
FLAG_GPS_OK = 1 << 5

# flight_state_t indices (mirror STATE_NAMES / App state machine), used by the
# --summary phase-duration math.
ST_BOOST, ST_COAST = 3, 4
ST_DROGUE, ST_MAIN, ST_DESCENT = 6, 7, 8

# ISM330DHCX rails at +/-16 g; an axis at/above this is clipped by the sensor,
# so max |accel| is only a floor when this trips (see CLAUDE.md 5.3). Tunable.
ACCEL_SAT_G = 15.9
# Drogue/first pyro should fire near apogee; firing below this fraction of the
# max AGL is flagged as a suspicious deployment. Tunable to the airframe.
DROGUE_NEAR_APOGEE_FRAC = 0.85

CSV_HEADER = [
    "rec", "t_ms", "t_s", "state", "state_name",
    "flags_hex", "gps_fix", "accel_sat", "armed", "sd_ok", "chg_ok", "gps_ok",
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


def _record_dict(rec_index, fields):
    """Turn one unpacked REC_STRUCT tuple into a record dict (raw wire values;
    lat/lon/alt stay in their integer e7/cm encodings)."""
    (_magic, state, flags, t_ms,
     ax, ay, az, gx, gy, gz,
     press_pa, temp_c, agl_m, vel_ms,
     lat_e7, lon_e7, alt_gps_cm,
     sats, pyro_cont, pyro_fired, _pad,
     batt_mv, _crc) = fields
    return {
        "rec": rec_index, "state": state, "state_name": state_name(state),
        "flags": flags, "t_ms": t_ms,
        "ax_g": ax, "ay_g": ay, "az_g": az,
        "gx_dps": gx, "gy_dps": gy, "gz_dps": gz,
        "press_pa": press_pa, "temp_c": temp_c,
        "agl_m": agl_m, "vel_ms": vel_ms,
        "lat_e7": lat_e7, "lon_e7": lon_e7, "alt_gps_cm": alt_gps_cm,
        "sats": sats, "pyro_cont": pyro_cont, "pyro_fired": pyro_fired,
        "batt_mv": batt_mv,
    }


def _iter_blob(blob, stats=None):
    """Core scanner: yield decoded record dicts from a raw blob, validating
    magic + CRC and sliding forward one byte on a bad candidate. If `stats` (a
    dict) is passed it is filled once the generator is exhausted with keys
    records/crc_bad/skipped/resyncs/tail (side channel for decode())."""
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
        if crc16_ccitt(rec[:CRC_LEN]) != fields[-1]:
            crc_bad += 1
            skipped += 1
            if not in_skip:
                resyncs += 1
                in_skip = True
            i += 1
            continue

        in_skip = False
        yield _record_dict(records, fields)
        records += 1
        i += RECORD_LEN

    if stats is not None:
        stats.update(records=records, crc_bad=crc_bad, skipped=skipped,
                     resyncs=resyncs, tail=n - i)


def iter_records(path):
    """Yield decoded log records (dicts) from a LOGnnn.BIN file, one at a time.
    Validates magic + CRC; silently skips corrupt/garbage bytes. Importable for
    reuse by other tools (summary, plotting)."""
    with open(path, "rb") as f:
        blob = f.read()
    yield from _iter_blob(blob)


def _csv_row(r):
    """Format one record dict as a CSV row (matches CSV_HEADER order)."""
    flags = r["flags"]
    return [
        r["rec"], r["t_ms"], "%.3f" % (r["t_ms"] / 1000.0),
        r["state"], r["state_name"],
        "0x%04X" % flags,
        (flags >> 0) & 1, (flags >> 1) & 1, (flags >> 2) & 1, (flags >> 3) & 1,
        (flags >> 4) & 1, (flags >> 5) & 1,
        "%.4f" % r["ax_g"], "%.4f" % r["ay_g"], "%.4f" % r["az_g"],
        "%.3f" % r["gx_dps"], "%.3f" % r["gy_dps"], "%.3f" % r["gz_dps"],
        "%.2f" % r["press_pa"], "%.2f" % r["temp_c"],
        "%.2f" % r["agl_m"], "%.3f" % r["vel_ms"],
        "%.7f" % (r["lat_e7"] / 1e7), "%.7f" % (r["lon_e7"] / 1e7),
        "%.2f" % (r["alt_gps_cm"] / 100.0),
        r["sats"], "0x%02X" % r["pyro_cont"], "0x%02X" % r["pyro_fired"],
        r["batt_mv"],
    ]


def decode(blob, writer):
    """Scan blob via _iter_blob, write CSV rows.
    Returns (records, crc_bad, skipped, resyncs, tail)."""
    stats = {}
    for r in _iter_blob(blob, stats):
        writer.writerow(_csv_row(r))
    return (stats["records"], stats["crc_bad"], stats["skipped"],
            stats["resyncs"], stats["tail"])


def summarize(records):
    """One-pass flight summary from an iterable of iter_records() dicts.

    Returns a dict with: n_records, apogee_m (max AGL), max_vel_ms (max upward
    velocity), max_accel_g (max |accel| vector magnitude) + accel_saturated,
    burn_ms/coast_ms/descent_ms phase durations (from state transitions),
    fire_t_ms/fire_agl_m (first nonzero pyro_fired), and warnings (list)."""
    apogee_m = None            # highest baro AGL seen
    max_vel_ms = None          # most-positive (upward) vertical velocity
    max_accel_g = 0.0          # max |accel| vector magnitude, g
    accel_saturated = False    # any axis hit the +/-16 g rail
    fire_t_ms = None           # first t_ms with a nonzero pyro_fired bitfield
    fire_agl_m = None          # AGL at that first fire
    main_ascending = False     # MAIN state seen while still climbing (vel > 0)
    n = 0

    # Phase durations from state transitions: record (t_ms, state) at each
    # state change; each entry's dwell runs until the next change (or the last
    # sample). Sum dwell per state, then group into burn/coast/descent.
    transitions = []
    prev_state = None
    last_t = 0

    for r in records:
        n += 1
        t_ms, state = r["t_ms"], r["state"]
        agl, vel = r["agl_m"], r["vel_ms"]
        last_t = t_ms

        if apogee_m is None or agl > apogee_m:
            apogee_m = agl
        if max_vel_ms is None or vel > max_vel_ms:
            max_vel_ms = vel

        ax, ay, az = r["ax_g"], r["ay_g"], r["az_g"]
        amag = (ax * ax + ay * ay + az * az) ** 0.5
        if amag > max_accel_g:
            max_accel_g = amag
        if (abs(ax) >= ACCEL_SAT_G or abs(ay) >= ACCEL_SAT_G
                or abs(az) >= ACCEL_SAT_G):
            accel_saturated = True

        if fire_t_ms is None and r["pyro_fired"]:
            fire_t_ms, fire_agl_m = t_ms, agl
        if state == ST_MAIN and vel > 0:
            main_ascending = True

        if state != prev_state:
            transitions.append((t_ms, state))
            prev_state = state

    phase_ms = {}
    for k, (t, st) in enumerate(transitions):
        end = transitions[k + 1][0] if k + 1 < len(transitions) else last_t
        phase_ms[st] = phase_ms.get(st, 0) + (end - t)
    burn_ms = phase_ms.get(ST_BOOST, 0)
    coast_ms = phase_ms.get(ST_COAST, 0)
    # descent = under-canopy + freefall between apogee and landing.
    descent_ms = (phase_ms.get(ST_DROGUE, 0) + phase_ms.get(ST_MAIN, 0)
                  + phase_ms.get(ST_DESCENT, 0))

    warnings = []
    # (1) first pyro fired well below apogee -> premature/late deployment.
    if (fire_agl_m is not None and apogee_m and
            fire_agl_m < DROGUE_NEAR_APOGEE_FRAC * apogee_m):
        warnings.append(
            "pyro fired at %.1f m AGL, %.0f%% below apogee (%.1f m)"
            % (fire_agl_m, 100.0 * (1.0 - fire_agl_m / apogee_m), apogee_m))
    # (2) MAIN deploy marker while still ascending -> sanity-inhibit failure.
    if main_ascending:
        warnings.append("MAIN state reached while vertical velocity > 0 "
                        "(ascending)")

    return {
        "n_records": n,
        "apogee_m": apogee_m,
        "max_vel_ms": max_vel_ms,
        "max_accel_g": max_accel_g,
        "accel_saturated": accel_saturated,
        "burn_ms": burn_ms,
        "coast_ms": coast_ms,
        "descent_ms": descent_ms,
        "phase_ms": phase_ms,
        "fire_t_ms": fire_t_ms,
        "fire_agl_m": fire_agl_m,
        "warnings": warnings,
    }


def _print_summary(s, out=None):
    out = out or sys.stdout

    def sec(v):
        return "-" if v is None else "%.2f s" % (v / 1000.0)

    def line(*a):
        print(*a, file=out)

    line("flight summary (%d records)" % s["n_records"])
    line("  apogee (max AGL) : %s"
         % ("-" if s["apogee_m"] is None else "%.1f m" % s["apogee_m"]))
    line("  max vertical vel : %s"
         % ("-" if s["max_vel_ms"] is None else "%.1f m/s" % s["max_vel_ms"]))
    sat = (" (SATURATED, >= %.0f g rail - floor only)" % ACCEL_SAT_G
           if s["accel_saturated"] else "")
    line("  max |accel|      : %.1f g%s" % (s["max_accel_g"], sat))
    line("  burn duration    : %s" % sec(s["burn_ms"]))
    line("  coast duration   : %s" % sec(s["coast_ms"]))
    line("  descent duration : %s" % sec(s["descent_ms"]))
    if s["fire_t_ms"] is None:
        line("  pyro fire        : none recorded")
    else:
        line("  pyro fire        : t=%s at %.1f m AGL"
             % (sec(s["fire_t_ms"]), s["fire_agl_m"]))
    for w in s["warnings"]:
        line("  WARNING: %s" % w)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Decode a flight-controller SD log (LOGnnn.BIN) to CSV "
                    "and/or print a flight summary.")
    ap.add_argument("input", help="input .bin log file")
    ap.add_argument("output", nargs="?",
                    help="output CSV path (default: input with .csv extension)")
    ap.add_argument("--summary", action="store_true",
                    help="print a flight summary (apogee, velocities, phase "
                         "durations, pyro timing, sanity warnings). Skips CSV "
                         "unless an output path is also given.")
    args = ap.parse_args(argv)

    # --summary alone skips CSV; give an output path to get both.
    write_csv = (not args.summary) or (args.output is not None)

    try:
        with open(args.input, "rb") as f:
            blob = f.read()
    except OSError as e:
        print("error: cannot read %s: %s" % (args.input, e), file=sys.stderr)
        return 2
    if not blob:
        print("error: %s is empty" % args.input, file=sys.stderr)
        return 2

    if write_csv:
        out_path = args.output or os.path.splitext(args.input)[0] + ".csv"
        try:
            with open(out_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(CSV_HEADER)
                records, crc_bad, skipped, resyncs, tail = decode(blob, writer)
        except OSError as e:
            print("error: cannot write %s: %s" % (out_path, e), file=sys.stderr)
            return 2
        print("%s: %d bytes -> %s" % (args.input, len(blob), out_path))
        print("  records decoded : %d" % records)
        print("  crc failures    : %d (magic OK, bad CRC)" % crc_bad)
        print("  bytes skipped   : %d in %d resync run(s)" % (skipped, resyncs))
        if tail:
            print("  trailing bytes  : %d (partial record at EOF)" % tail)
        if records == 0:
            print("warning: no valid records found - wrong file or format "
                  "mismatch?", file=sys.stderr)

    if args.summary:
        # Same core scanner as the CSV path; reuse the blob we already read.
        _print_summary(summarize(_iter_blob(blob)))

    return 0


if __name__ == "__main__":
    sys.exit(main())
