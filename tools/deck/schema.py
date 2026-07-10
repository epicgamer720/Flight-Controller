#!/usr/bin/env python3
"""schema.py - canonical Flight Deck telemetry model.

The canonical field set IS tools/telem_decode.py parse_telem(): the GS
bridge (ground-station/gs/gs.ino emit_telem) already emits exactly those
keys as one JSON object per line, so this module re-exports telem_decode's
constants and helpers instead of duplicating them.

On top of the wire fields every source adds:

    t_host   float seconds on the session clock (time.monotonic based).
             The x-axis everywhere; t_ms is never used as an axis.
    src      "fc" | "gs" | "replay"
    rssi     dBm, nullable (only the GS path measures it)
    snr      dB, nullable

and the FC-console-only extras (all nullable — the GS path never sees
them): press_pa, temp_c, pyro_sense_mv, main_alt_m, testen, log stats,
radio counters, reset cause, GPS fix age and the per-subsystem health
booleans from the `status` health line.

Events are flat records: {seq, t_host, t_ms, kind, severity, text}.
"""

import dataclasses
import os
import sys

# telem_decode lives one directory up (tools/); same insertion pattern as
# tests/test_telem_decode.py so `import deck.schema` works from anywhere.
_TOOLS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

import telem_decode as td

# ---- re-exports: telem_decode is the single source of truth ----
LINK_MAGIC = td.LINK_MAGIC
PROTO_VERSION = td.PROTO_VERSION
NUM_PYRO = td.NUM_PYRO

PKT_TELEMETRY = td.PKT_TELEMETRY
PKT_EVENT = td.PKT_EVENT
PKT_COMMAND = td.PKT_COMMAND
PKT_ACK = td.PKT_ACK

CMD_NOP = td.CMD_NOP
CMD_ARM = td.CMD_ARM
CMD_DISARM = td.CMD_DISARM
CMD_TEST_FIRE = td.CMD_TEST_FIRE
CMD_SET_MAIN_ALT = td.CMD_SET_MAIN_ALT
CMD_ZERO_BARO = td.CMD_ZERO_BARO
CMD_REBOOT = td.CMD_REBOOT
CMD_ENTER_BOOTLOADER = td.CMD_ENTER_BOOTLOADER
CMD_NAMES = td.CMD_NAMES

STATE_NAMES = td.STATE_NAMES
state_name = td.state_name

FLAG_GPS_FIX = td.FLAG_GPS_FIX
FLAG_ACCEL_SAT = td.FLAG_ACCEL_SAT
FLAG_ARMED = td.FLAG_ARMED
FLAG_SD_OK = td.FLAG_SD_OK
FLAG_CHG_OK = td.FLAG_CHG_OK
FLAG_GPS_OK = td.FLAG_GPS_OK

crc16_ccitt = td.crc16_ccitt
parse_telem = td.parse_telem
parse_command = td.parse_command
parse_packet = td.parse_packet
build_telem = td.build_telem
build_command = td.build_command
TELEM_LEN = td.TELEM_LEN
CMD_LEN = td.CMD_LEN

# ---- canonical key sets ----
# Wire keys, in parse_telem() order (gs.ino emits exactly this order).
TELEM_KEYS = tuple(td.parse_telem(td.build_telem()).keys())

# Host extensions stamped by every source.
HOST_KEYS = ("t_host", "src", "rssi", "snr")

# Derived link statistics (GS/replay paths).
LINK_KEYS = ("pkt_rate",)

# FC-console-only extras (nullable everywhere else).
CONSOLE_KEYS = (
    "press_pa", "temp_c",           # sensors: raw baro
    "pyro_sense_mv",                # status: pyro sense line
    "main_alt_m", "testen",         # status: armed line
    "log_running", "log_drops", "log_ring_hw", "log_ring_cap",
    "radio_reinit", "radio_txto", "radio_irq", "radio_tcxo",
    "reset_cause",                  # boot banner
    "gps_last_fix_ms",              # gps command
    "imu_ok", "baro_ok", "radio_ok",  # health line (sd/chg/gps_ok are
                                      # already flag booleans in TELEM_KEYS)
    "chg_stat", "chg_charging", "chg_fault",
)

ALL_KEYS = TELEM_KEYS + HOST_KEYS + LINK_KEYS + CONSOLE_KEYS

# Chart buffers every Source maintains (rows are [t_host, v, ...]).
SERIES = ("alt", "vel", "accel", "gyro", "batt", "rssi", "snr", "rate",
          "pyro")

# ---- events ----
EVENT_KINDS = ("state", "pyro", "cmd_sent", "ack", "nak", "refusal",
               "conn", "reboot", "rec", "gs_error", "fault")
SEVERITIES = ("info", "warn", "crit")


@dataclasses.dataclass
class Event:
    """One event-log record. t_ms is the FC clock when known, else None."""
    seq: int
    t_host: float
    t_ms: object          # int or None
    kind: str             # one of EVENT_KINDS
    severity: str         # one of SEVERITIES
    text: str

    def to_dict(self):
        return dataclasses.asdict(self)


def normalize_telem(d, t_host, src):
    """Flatten one telemetry-shaped dict (parse_telem output, or the GS
    JSON line which has identical keys) into (latest, rows):

    latest  the flat "latest values" dict: all input keys + t_host/src,
            with rssi/snr present (None when the frame has none)
    rows    {chart buffer: row} — each row stamped t_host first
    """
    latest = dict(d)
    latest["t_host"] = round(t_host, 3)
    latest["src"] = src
    latest.setdefault("rssi", None)
    latest.setdefault("snr", None)

    t = round(t_host, 3)
    acc = d.get("accel_g") or (None, None, None)
    gyr = d.get("gyro_dps") or (None, None, None)
    rows = {
        "alt": [t, d.get("alt_baro_m"), d.get("alt_gps_m")],
        "vel": [t, d.get("vel_up_ms")],
        "accel": [t, acc[0], acc[1], acc[2]],
        "gyro": [t, gyr[0], gyr[1], gyr[2]],
        "batt": [t, d.get("batt_v")],
    }
    if latest["rssi"] is not None:
        rows["rssi"] = [t, latest["rssi"]]
    if latest["snr"] is not None:
        rows["snr"] = [t, latest["snr"]]
    return latest, rows
