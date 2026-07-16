#!/usr/bin/env python3
"""sources.py - Flight Deck telemetry sources.

    Source           base: start()/stop()/connected/drain(since) over
                     thread-safe 130 s rolling series + latest + events
    LineIngest       the SINGLE shared GS-JSON-line classifier, used by
                     both GsJsonSource (serial) and ReplaySource (file)
    ReplaySource     .jsonl of recorded GS lines, a session telem.csv, or
                     the synthetic "nominal" flight profile
    GsJsonSource     ground-station USB JSON bridge @115200
    FcConsoleSource  FC USB console poller, a port of the Poller/_txn
                     pattern in tools/live_dashboard.py (read-only donor;
                     regexes copied + extended, never imported)

stdlib + pyserial only; pyserial is imported lazily inside the serial
sources so everything here is importable without it.
"""

import csv
import json
import random
import re
import threading
import time
from collections import deque

from . import schema
import telem_decode as td   # sys.path fixed by deck.schema import above
import decode_log           # SD LOGnnn.BIN decoder (same tools/ dir)

WINDOW_S = 130.0          # rolling series window (charts pan back 120 s)
RATE_WINDOW_S = 5.0       # packet-rate sliding window
RECONNECT_S = 2.0         # serial reopen retry period
EVENTS_MAX = 1000         # event log cap (oldest dropped)
REBOOT_TMS_MARGIN = 500   # t_ms must fall by > this to call it a reboot
_BIN_RSSI = -70.0         # SD .bin has no link quality: synthetic rssi/snr
_BIN_SNR = 9.0            # (same nominal pair the jsonl/synthetic paths use)

# FC console poll cadences (donor: live_dashboard.py, extended)
STATUS_PERIOD = 0.5
GPS_PERIOD = 5.0
RADIO_PERIOD = 5.0
TX_PERIOD = 5.0
CHARGE_PERIOD = 5.0
LOG_PERIOD = 5.0
SENSOR_PERIOD = 0.0       # flat-out; serial round-trip is the limit

# ============================================================
# FC console scrape regexes: copied from tools/live_dashboard.py and
# extended (uptime, main_alt, flags, gps detail, charge, log stat, boot
# banner, sensors sat).  Formats verified verbatim against
# flight-controller/App/console.c + App/app_main.c; tests/
# test_deck_contract.py locks every one of them to the firmware source.
# ============================================================
RE_IMU = re.compile(r"ax=(-?[\d.]+) ay=(-?[\d.]+) az=(-?[\d.]+)")
RE_IMU_SAT = re.compile(r" g\s+sat=(\d)")
RE_GYR = re.compile(r"gx=(-?[\d.]+) gy=(-?[\d.]+) gz=(-?[\d.]+)")
RE_BARO = re.compile(r"baro: (-?[\d.]+) Pa\s+(-?[\d.]+) C\s+alt=(-?[\d.]+)")
RE_STATE = re.compile(r"state:\s+(\w+)\s+t=(\d+)ms")
RE_ARMED = re.compile(r"armed:\s+(\d)\s+testen:\s*(\d)\s+main_alt:\s*(-?[\d.]+) m")
RE_ALTVEL = re.compile(r"alt:\s+(-?[\d.]+) m AGL\s+vel: (-?[\d.]+)")
RE_BATT = re.compile(r"batt:\s+(\d+) mV")
RE_FLAGS = re.compile(r"flags:\s+gps_fix=(\d) accel_sat=(\d)")
RE_HEALTH = re.compile(r"imu=(\d) baro=(\d) sd=(\d) radio=(\d)(?:\s+chg=(\d)\s+gps=(\d))?")
RE_PYRO = re.compile(r"cont=0x([0-9A-Fa-f]+) fired=0x([0-9A-Fa-f]+) sense=(\d+) mV")
RE_GPSFIX = re.compile(r"fix=(\d) sats=(\d+)")
RE_GPSPOS = re.compile(r"lat=(-?\d+\.\d{7}) lon=(-?\d+\.\d{7}) alt=(-?\d+\.\d{2}) m MSL")
RE_GPSAGE = re.compile(r"last_fix=(\d+) ms ago")
RE_RADIO = re.compile(r"radio:\s+(\w+)\s+tcxo:\s+(\S+)")
RE_RADIO_CNT = re.compile(r"reinit attempts:\s+(\d+)\s+tx timeouts:\s+(\d+)")
RE_RADIO_IRQ = re.compile(r"chip irq status:\s+0x([0-9A-Fa-f]+)")
# `tx` dashboard (bare `tx`): "power:22 dBm" / "packets: tx=.. rx=.." /
# "last rx: rssi=-72 dBm  snr=9 dB" (or "last rx: (none)").
RE_TX_POWER = re.compile(r"power:(-?\d+) dBm")
RE_TX_PKTS = re.compile(r"packets: tx=(\d+)\s+rx=(\d+)")
RE_TX_RXQ = re.compile(r"last rx: rssi=(-?\d+) dBm\s+snr=(-?\d+) dB")
# `status`: "mcu:     23.9 C (die)" is the STM32 internal die temp (fc only)
RE_MCU = re.compile(r"mcu:\s+(-?[\d.]+) C")
RE_CHARGE = re.compile(r"charger: vbat=(\d+) mV stat=0x([0-9A-Fa-f]+) charging=(\d) fault=(\d)")
RE_LOG = re.compile(r"log:\s+(running|not running)")
RE_LOGSTAT = re.compile(r"drops:\s+(\d+)\s+ring high-water:\s+(\d+)/(\d+) bytes")
RE_RESET = re.compile(r"reset:([^\r\n]+)")
BOOT_BANNER = "=== FC boot ==="


# ============================================================
# LineIngest: the one GS-JSON-line classifier
# ============================================================
class LineIngest:
    """Classify one GS bridge line: feed(line) -> (kind, payload).

    kinds: telem (type==1), event (type==2), ack (type==0x11),
    cmd_echo (parsed 14-byte command echo), uplink_echo ({"uplink":..}),
    gs_boot ({"gs":"boot",..}), gs_error ({"error":..}), junk
    (unparseable, counted, never raises).
    """

    def __init__(self):
        self.counts = {}
        self.junk = 0

    def feed(self, line):
        line = line.strip()
        if not line:
            return ("junk", None)
        try:
            obj = json.loads(line)
        except ValueError:
            self.junk += 1
            return ("junk", line)
        if not isinstance(obj, dict):
            self.junk += 1
            return ("junk", line)

        if "uplink" in obj:
            kind = "uplink_echo"
        elif "gs" in obj:
            kind = "gs_boot"
        elif "error" in obj:
            kind = "gs_error"
        else:
            ptype = obj.get("type")
            if ptype == schema.PKT_TELEMETRY:
                kind = "telem"
            elif ptype == schema.PKT_EVENT:
                kind = "event"
            elif ptype == schema.PKT_ACK:      # ACK = telemetry-shaped frame
                kind = "ack"
            elif ptype == schema.PKT_COMMAND and "cmd" in obj:
                kind = "cmd_echo"
            else:
                self.junk += 1
                return ("junk", line)
        self.counts[kind] = self.counts.get(kind, 0) + 1
        return (kind, obj)


def _mask_fire_arg(name, arg):
    """CMD_TEST_FIRE packs (passcode << 16 | channel) into arg; the
    passcode must never reach the event log or any recording."""
    if name == "CMD_TEST_FIRE" and isinstance(arg, int):
        return "****<<16|ch%d" % (arg & 0xFFFF)
    return arg


def mask_fire_raw(text):
    """Scrub the packed test-fire passcode out of a raw GS line before it
    is persisted to raw.jsonl. Non-fire lines pass through untouched."""
    if "CMD_TEST_FIRE" not in text:
        return text
    try:
        obj = json.loads(text)
    except ValueError:
        return text
    if isinstance(obj, dict) and isinstance(obj.get("arg"), int) and \
            (obj.get("uplink") == "CMD_TEST_FIRE"
             or obj.get("cmd_name") == "CMD_TEST_FIRE"):
        obj["arg"] = obj["arg"] & 0xFFFF     # keep channel, drop passcode
        obj["arg_masked"] = True
        return json.dumps(obj)
    return text


# ============================================================
# GS JSON serialization: byte-identical to gs.ino emit_telem()
# ============================================================
def gs_json_line(d, rssi, snr):
    """Serialize a parse_telem() dict exactly like gs.ino emit_telem():
    same key order, same printf precisions (%.7f lat/lon, %.2f alt/vel,
    %.2f accel, %.1f gyro, %.3f batt, rssi %.1f, snr %.2f)."""
    def b(v):
        return "true" if v else "false"
    a1 = d["accel_g_x100"]
    g1 = d["gyro_dps_x10"]
    a2 = d["accel_g"]
    g2 = d["gyro_dps"]
    # eng mcu_temp_c: null when the die-temp read failed (gs.ino matches)
    mcu_c = "null" if d.get("mcu_temp_c") is None else "%.1f" % d["mcu_temp_c"]
    return (
        '{"magic":%d,"version":%d,"type":%d,"state":%d,"t_ms":%d,'
        '"lat_e7":%d,"lon_e7":%d,'
        '"alt_baro_cm":%d,"alt_gps_cm":%d,"vel_up_cms":%d,'
        '"accel_g_x100":[%d,%d,%d],"gyro_dps_x10":[%d,%d,%d],'
        '"batt_mv":%d,"pyro_cont":%d,"pyro_fired":%d,"sats":%d,'
        '"flags":%d,"mcu_temp_c10":%d,'
        '"seq":%d,"last_evt_state":%d,"last_evt_count":%d,"main_alt_m":%d,"crc16":%d,'
        '"state_name":"%s","last_evt_name":"%s",'
        '"lat_deg":%.7f,"lon_deg":%.7f,'
        '"alt_baro_m":%.2f,"alt_gps_m":%.2f,"vel_up_ms":%.2f,'
        '"accel_g":[%.2f,%.2f,%.2f],"gyro_dps":[%.1f,%.1f,%.1f],'
        '"batt_v":%.3f,"mcu_temp_c":%s,'
        '"gps_fix":%s,"accel_sat":%s,"armed":%s,"sd_ok":%s,'
        '"chg_ok":%s,"gps_ok":%s,"low_batt":%s,"deploy_fail":%s,'
        '"rssi":%.1f,"snr":%.2f}'
        % (d["magic"], d["version"], d["type"], d["state"], d["t_ms"],
           d["lat_e7"], d["lon_e7"],
           d["alt_baro_cm"], d["alt_gps_cm"], d["vel_up_cms"],
           a1[0], a1[1], a1[2], g1[0], g1[1], g1[2],
           d["batt_mv"], d["pyro_cont"], d["pyro_fired"], d["sats"],
           d["flags"], d["mcu_temp_c10"],
           d["seq"], d["last_evt_state"], d["last_evt_count"], d["main_alt_m"], d["crc16"],
           d["state_name"], d["last_evt_name"],
           d["lat_deg"], d["lon_deg"],
           d["alt_baro_m"], d["alt_gps_m"], d["vel_up_ms"],
           a2[0], a2[1], a2[2], g2[0], g2[1], g2[2],
           d["batt_v"], mcu_c,
           b(d["gps_fix"]), b(d["accel_sat"]), b(d["armed"]), b(d["sd_ok"]),
           b(d["chg_ok"]), b(d["gps_ok"]), b(d["low_batt"]), b(d["deploy_fail"]),
           rssi, snr))


# ============================================================
# Synthetic "nominal" flight profile
# ============================================================
_SYN_BOOT_MS = 5000       # FC booted 5 s before the session starts
_SYN_G = 9.81
_SYN_BOOST_A = 25.0       # net boost accel (m/s^2) -> apogee ~399 m
_SYN_T = {"armed": 10.0, "boost": 13.0, "coast": 16.0, "apogee": 23.6,
          "drogue": 24.1, "main": 37.9, "descent": 39.0, "landed": 62.9}
_SYN_END = 67.0           # battery sag endpoint: 8200 -> 7400 mV


def _syn_alt_vel(t):
    """Piecewise closed-form altitude (m AGL) / vertical velocity (m/s)."""
    T = _SYN_T
    if t < T["boost"]:
        return 0.0, 0.0
    if t < T["coast"]:
        tau = t - T["boost"]
        return 0.5 * _SYN_BOOST_A * tau * tau, _SYN_BOOST_A * tau
    burn = T["coast"] - T["boost"]
    h_bo = 0.5 * _SYN_BOOST_A * burn * burn          # 112.5 m
    v_bo = _SYN_BOOST_A * burn                        # 75 m/s
    if t < T["drogue"]:                               # incl. APOGEE state
        tau = t - T["coast"]
        return h_bo + v_bo * tau - 0.5 * _SYN_G * tau * tau, v_bo - _SYN_G * tau
    tau_d = T["drogue"] - T["coast"]
    h_dr = h_bo + v_bo * tau_d - 0.5 * _SYN_G * tau_d * tau_d
    if t < T["main"]:
        return h_dr - 18.0 * (t - T["drogue"]), -18.0
    h_mn = h_dr - 18.0 * (T["main"] - T["drogue"])   # ~149.78 m
    if t < T["landed"]:
        return max(0.0, h_mn - 6.0 * (t - T["main"])), -6.0
    return 0.0, 0.0


def _syn_state(t):
    T = _SYN_T
    if t < T["armed"]:
        return 1                                      # GROUND_IDLE
    if t < T["boost"]:
        return 2                                      # ARMED
    if t < T["coast"]:
        return 3                                      # BOOST
    if t < T["apogee"]:
        return 4                                      # COAST
    if t < T["drogue"]:
        return 5                                      # APOGEE
    if t < T["main"]:
        return 6                                      # DROGUE
    if t < T["descent"]:
        return 7                                      # MAIN
    if t < T["landed"]:
        return 8                                      # DESCENT
    return 9                                          # LANDED


def synthetic_profile(name="nominal"):
    """Full nominal flight as GS-shaped JSON lines: 10 s pad @1 Hz ->
    ARMED -> BOOST 3 s (accel railed +/-1600, FLAG_ACCEL_SAT) -> COAST ->
    APOGEE ~400 m -> DROGUE -> MAIN @150 m -> DESCENT -> LANDED, one
    PKT_EVENT frame per transition, battery 8.2 -> 7.4 V, GPS jitter,
    RSSI fade -70 -> ~-95 dBm with altitude.  Frames are built with
    telem_decode.build_telem -> parse_telem -> gs_json_line so the binary
    codec round-trips too.  Deterministic (seeded RNG)."""
    if name != "nominal":
        raise ValueError("unknown synthetic profile %r" % name)

    ticks = [float(t) for t in range(0, 13)]           # pad+armed @1 Hz
    ticks += [k / 10.0 for k in range(130, 630)]       # flight @10 Hz
    ticks += [64.0, 65.0, 66.0, 67.0]                  # landed tail @1 Hz

    rng = random.Random(42)
    lines = []
    prev_state = None
    seq = 0
    last_evt = 0
    evt_count = 0
    for i, t in enumerate(ticks):
        st = _syn_state(t)
        alt, vel = _syn_alt_vel(t)
        if prev_state is not None and st != prev_state:   # v3 event echo
            evt_count = (evt_count + 1) & 0xFF
            last_evt = st

        if st == 3:
            accel, gyro = (12, -8, 1600), (900, -35, 25)   # railed, thrusting
        elif st in (4, 5):
            accel, gyro = (3, -2, 5), (250, -12, 8)        # ~free fall
        elif st in (6, 7, 8):
            accel, gyro = (5, -5, 95), (40, -30, 20)       # under canopy
        elif st == 9:
            accel, gyro = (0, 0, 100), (0, 0, 0)
        else:
            accel, gyro = (0, 0, 100), (1, -2, 3)

        flags = (schema.FLAG_GPS_FIX | schema.FLAG_SD_OK |
                 schema.FLAG_CHG_OK | schema.FLAG_GPS_OK)
        if st >= 2:
            flags |= schema.FLAG_ARMED
        if st == 3:
            flags |= schema.FLAG_ACCEL_SAT

        fired = 0x01 if st >= 7 else 0x00
        fields = dict(
            state=st,
            t_ms=_SYN_BOOT_MS + int(round(t * 1000)),
            lat_e7=391234567 + rng.randint(-300, 300),
            lon_e7=-771234567 + rng.randint(-300, 300),
            alt_baro_cm=int(round(alt * 100)),
            alt_gps_cm=int(round((alt + 120.0) * 100)) + rng.randint(-200, 200),
            vel_up_cms=int(round(vel * 100)),
            accel_g_x100=accel,
            gyro_dps_x10=gyro,
            batt_mv=int(round(8200 - 800.0 * t / _SYN_END)),
            pyro_cont=0x00 if fired else 0x01,
            pyro_fired=fired,
            sats=8 + (i % 3),
            flags=flags,
            # MCU die temp drifts up ~28->40 C (TX self-heating over the IMU)
            mcu_temp_c10=int(round((28.0 + 12.0 * min(t, _SYN_END) / _SYN_END)
                                   * 10)),
            main_alt_m=150,
            last_evt_state=last_evt,
            last_evt_count=evt_count,
        )
        rssi = -70.0 - 25.0 * min(alt, 400.0) / 400.0
        snr = 9.0 - 5.0 * min(alt, 400.0) / 400.0

        if prev_state is not None and st != prev_state:
            evt = td.parse_telem(td.build_telem(ptype=schema.PKT_EVENT,
                                                seq=seq & 0xFFFF, **fields))
            lines.append(gs_json_line(evt, rssi, snr))
            seq += 1
        tel = td.parse_telem(td.build_telem(ptype=schema.PKT_TELEMETRY,
                                            seq=seq & 0xFFFF, **fields))
        lines.append(gs_json_line(tel, rssi, snr))
        seq += 1
        prev_state = st
    return lines


# ============================================================
# Source base
# ============================================================
class Source:
    """Thread-safe rolling store all sources share.  Rows are stamped
    t_host (session-clock seconds) and trimmed to WINDOW_S."""

    src = "base"

    def __init__(self):
        self._lock = threading.Lock()
        self._t0 = time.monotonic()
        self._connected = False
        self._series = {k: deque() for k in schema.SERIES}
        self._latest = {"src": self.src}
        self._events = []
        self._eseq = 0
        self._counters = {}
        self._stop = threading.Event()
        self._thread = None

    # -- session clock --
    def now(self):
        return time.monotonic() - self._t0

    # -- lifecycle --
    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=3.0)
        self._thread = None

    def _run(self):
        raise NotImplementedError

    @property
    def connected(self):
        with self._lock:
            return self._connected

    # -- store helpers (methods suffixed _locked expect self._lock held) --
    def _trim_locked(self, dq, t):
        cut = t - WINDOW_S
        while dq and dq[0][0] < cut:
            dq.popleft()

    def _append_locked(self, buf, row):
        dq = self._series[buf]
        dq.append(row)
        self._trim_locked(dq, row[0])

    def _push_event_locked(self, kind, text, t_host, t_ms=None,
                           severity="info"):
        self._eseq += 1
        self._events.append(schema.Event(self._eseq, round(t_host, 3), t_ms,
                                         kind, severity, text))
        if len(self._events) > EVENTS_MAX:
            del self._events[0:len(self._events) - EVENTS_MAX]

    def _event(self, kind, text, t_host, t_ms=None, severity="info"):
        with self._lock:
            self._push_event_locked(kind, text, t_host, t_ms, severity)

    def push_event(self, kind, text, severity="info"):
        """Public event injection (command adapters, recorder, server)."""
        self._event(kind, text, self.now(), severity=severity)

    # -- consumer API --
    def drain(self, since=-1.0, event_seq=-1):
        """Incremental snapshot: series rows with t_host > since, the full
        latest dict, and events with seq > event_seq."""
        with self._lock:
            return {
                "connected": self._connected,
                "now": round(self.now(), 3),
                "series": {k: [r for r in dq if r[0] > since]
                           for k, dq in self._series.items()},
                "latest": dict(self._latest),
                "events": [e.to_dict() for e in self._events
                           if e.seq > event_seq],
                "counters": dict(self._counters),
            }


# ============================================================
# Shared GS-JSON ingestion (GsJsonSource + ReplaySource)
# ============================================================
class _JsonLineSource(Source):
    """Everything downstream of LineIngest: normalization, event
    derivation (state/pyro/ack/reboot), link stats."""

    def __init__(self):
        super().__init__()
        self.ingest = LineIngest()
        self.listeners = []      # callables (kind, obj, t_host), called
                                 # OUTSIDE the store lock (command adapters)
        self._last_t_ms = None
        self._prev_state = None
        self._prev_fired = None
        self._arrivals = deque()
        self._last_frame_t = None

    def _notify(self, kind, obj, t_host):
        for fn in list(self.listeners):
            try:
                fn(kind, obj, t_host)
            except Exception:
                pass                 # a broken listener must not kill ingest

    def _ingest_gs(self, kind, obj, t_host):
        self._ingest_gs_inner(kind, obj, t_host)
        self._notify(kind, obj, t_host)

    def _ingest_gs_inner(self, kind, obj, t_host):
        if kind == "junk":
            with self._lock:
                self._counters["junk"] = self.ingest.junk
            return
        if kind == "gs_boot":
            self._event("conn", "GS boot: radio %s" % obj.get("radio", "?"),
                        t_host)
            return
        if kind == "gs_error":
            with self._lock:
                self._counters["gs_error"] = \
                    self._counters.get("gs_error", 0) + 1
                self._push_event_locked("gs_error",
                                        "GS: %s" % obj.get("error"),
                                        t_host, severity="warn")
            return
        if kind == "uplink_echo":
            self._event("cmd_sent",
                        "uplink %s arg=%s nonce=%s tx_status=%s"
                        % (obj.get("uplink"),
                           _mask_fire_arg(obj.get("uplink"), obj.get("arg")),
                           obj.get("nonce"), obj.get("tx_status")), t_host)
            return
        if kind == "cmd_echo":
            self._event("cmd_sent",
                        "cmd echo %s arg=%s nonce=%s"
                        % (obj.get("cmd_name"),
                           _mask_fire_arg(obj.get("cmd_name"),
                                          obj.get("arg")),
                           obj.get("nonce")), t_host)
            return

        # telem / event / ack: all full telemetry-shaped frames
        latest, rows = schema.normalize_telem(obj, t_host, self.src)
        t_ms = obj.get("t_ms")
        st = obj.get("state")
        fired = obj.get("pyro_fired")
        with self._lock:
            if (self._last_t_ms is not None and t_ms is not None
                    and t_ms + REBOOT_TMS_MARGIN < self._last_t_ms):
                self._push_event_locked(
                    "reboot", "FC reboot detected (t_ms %d -> %d)"
                    % (self._last_t_ms, t_ms), t_host, t_ms=t_ms,
                    severity="warn")
            if t_ms is not None:
                self._last_t_ms = t_ms

            if (st is not None and self._prev_state is not None
                    and st != self._prev_state):
                prev = schema.state_name(self._prev_state)
                name = schema.state_name(st)
                if name == "FAULT":
                    self._push_event_locked("fault",
                                            "state %s -> FAULT" % prev,
                                            t_host, t_ms=t_ms,
                                            severity="crit")
                else:
                    self._push_event_locked("state", "%s -> %s"
                                            % (prev, name), t_host, t_ms=t_ms)
            if st is not None:
                self._prev_state = st

            if fired is not None:
                if (self._prev_fired is not None
                        and (fired & ~self._prev_fired)):
                    self._push_event_locked("pyro",
                                            "pyro fired: 0x%02X" % fired,
                                            t_host, t_ms=t_ms,
                                            severity="warn")
                self._prev_fired = fired

            if kind == "ack":
                self._push_event_locked("ack", "PKT_ACK received", t_host,
                                        t_ms=t_ms)

            self._latest.update(latest)
            for buf, row in rows.items():
                self._append_locked(buf, row)

            # link stats: rate over a sliding RATE_WINDOW_S window
            self._arrivals.append(t_host)
            cut = t_host - RATE_WINDOW_S
            while self._arrivals and self._arrivals[0] < cut:
                self._arrivals.popleft()
            rate = round(len(self._arrivals) / RATE_WINDOW_S, 2)
            self._latest["pkt_rate"] = rate
            self._append_locked("rate", [round(t_host, 3), rate])
            self._last_frame_t = t_host

            for k, v in self.ingest.counts.items():
                self._counters[k] = v

    def drain(self, since=-1.0, event_seq=-1):
        out = super().drain(since, event_seq)
        with self._lock:
            last = self._last_frame_t
        out["gap"] = None if last is None else round(self.now() - last, 3)
        return out


# ============================================================
# ReplaySource
# ============================================================
class ReplaySource(_JsonLineSource):
    """Replay recorded GS lines (.jsonl), a session telem.csv, or the
    synthetic profile.  Pacing honors inter-frame t_ms deltas x speed;
    pause()/resume() supported.  now() is the replay clock."""

    src = "replay"

    def __init__(self, path=None, synthetic=None, speed=1.0):
        super().__init__()
        if (path is None) == (synthetic is None):
            raise ValueError("give exactly one of path= or synthetic=")
        self.path = path
        self.synthetic = synthetic
        self.speed = float(speed)
        self._pause = threading.Event()
        self._replay_t = 0.0
        self._t0_ms = None

    def now(self):
        return self._replay_t

    def pause(self):
        self._pause.set()

    def resume(self):
        self._pause.clear()

    # -- input loaders --
    def _load(self):
        if self.synthetic is not None:
            return synthetic_profile(self.synthetic)
        low = self.path.lower()
        if low.endswith(".csv"):
            return self._load_csv(self.path)
        if low.endswith(".bin"):
            return self._load_bin(self.path)
        with open(self.path, encoding="utf-8", errors="replace") as f:
            return [ln for ln in f.read().splitlines() if ln.strip()]

    @staticmethod
    def _load_bin(path):
        """SD LOGnnn.BIN -> deck GS-JSON lines.  Reuse decode_log.iter_records
        (magic+CRC checked, corrupt bytes skipped) for the binary decode, map
        each record's engineering-unit fields to build_telem's RAW wire kwargs,
        then round-trip through parse_telem/gs_json_line like every other path.
        The log carries no link quality -> synthetic rssi/snr constants."""
        def clamp(v, lo, hi):
            # A NaN/inf sensor sample (a CRC-valid record can still hold one, e.g.
            # a diverged Kalman alt) or a huge finite value must never abort the
            # whole replay via int()/round() raising or struct overflow.
            try:
                v = int(round(v))
            except (ValueError, OverflowError):
                return 0
            return lo if v < lo else (hi if v > hi else v)

        def i16(v):
            return clamp(v, -32768, 32767)      # railed 16 g accel (1600) fits
        I32 = 2 ** 31
        lines = []
        for r in decode_log.iter_records(path):
            tel = td.parse_telem(td.build_telem(
                state=r["state"], t_ms=r["t_ms"],
                alt_baro_cm=clamp(r["agl_m"] * 100, -I32, I32 - 1),
                vel_up_cms=i16(r["vel_ms"] * 100),
                accel_g_x100=[i16(a * 100) for a in
                              (r["ax_g"], r["ay_g"], r["az_g"])],
                gyro_dps_x10=[i16(g * 10) for g in
                              (r["gx_dps"], r["gy_dps"], r["gz_dps"])],
                lat_e7=r["lat_e7"], lon_e7=r["lon_e7"],
                alt_gps_cm=r["alt_gps_cm"], batt_mv=r["batt_mv"],
                sats=r["sats"], pyro_cont=r["pyro_cont"],
                pyro_fired=r["pyro_fired"],
                flags=r["flags"] & 0xFF))       # log u16 -> protocol u8 FLAG_*
            lines.append(gs_json_line(tel, _BIN_RSSI, _BIN_SNR))
        return lines

    @staticmethod
    def _load_csv(path):
        """Session telem.csv -> dicts -> json.dumps -> same LineIngest.
        Cells are JSON literals where needed (lists, booleans); anything
        that doesn't json-parse stays a string; empty cells become null."""
        lines = []
        with open(path, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                d = {}
                for k, v in row.items():
                    if v is None or v == "":
                        d[k] = None
                        continue
                    try:
                        d[k] = json.loads(v)
                    except ValueError:
                        d[k] = v
                lines.append(json.dumps(d))
        return lines

    # -- replay clock: rebased t_ms of the frames --
    def _stamp(self, obj):
        t_ms = obj.get("t_ms") if isinstance(obj, dict) else None
        if t_ms is None:
            return self._replay_t          # non-telem line: no time advance
        if self._t0_ms is None:
            self._t0_ms = t_ms
        t = (t_ms - self._t0_ms) / 1000.0
        if t < self._replay_t:             # t_ms reset (FC reboot on tape)
            self._t0_ms = t_ms - int(round(self._replay_t * 1000.0))
            t = self._replay_t
        self._replay_t = t
        return t

    def _run(self):
        try:
            lines = self._load()
        except Exception as e:
            self._event("gs_error", "replay load failed: %s" % e, 0.0,
                        severity="crit")
            return
        with self._lock:
            self._connected = True
        self._event("conn", "replay started (%d lines)" % len(lines),
                    self._replay_t)
        last_t = None
        for raw in lines:
            if self._stop.is_set():
                break
            while self._pause.is_set() and not self._stop.is_set():
                self._stop.wait(0.02)
            kind, obj = self.ingest.feed(raw)
            t = self._stamp(obj)
            if last_t is not None and self.speed > 0:
                delay = (t - last_t) / self.speed
                while delay > 1e-4 and not self._stop.is_set():
                    step = min(0.05, delay)
                    self._stop.wait(step)
                    delay -= step
            last_t = t
            self._ingest_gs(kind, obj, t)
        self._event("conn", "replay finished", self._replay_t)
        with self._lock:
            self._connected = False

    def run_sync(self):
        """Run the whole replay in the caller's thread (tests / fast)."""
        self._run()


# ============================================================
# GsJsonSource
# ============================================================
class GsJsonSource(_JsonLineSource):
    """Ground-station USB JSON bridge: one JSON object per line @115200.
    Derives its own link stats (rate/gap/rssi/snr/error counters), since
    the GS emits none.  Reconnects every RECONNECT_S on serial errors; the
    port is ALWAYS closed in stop()/finally (a host process dying with
    the port open can wedge CDC endpoints, per project history)."""

    src = "gs"

    def __init__(self, com):
        super().__init__()
        self.com = com
        self._port = None
        self._rxbuf = b""
        self._txlock = threading.Lock()
        self.raw_hook = None      # callable(line): lossless .jsonl capture

    def send_line(self, line):
        """Write one stdin command line to the GS bridge. Returns False
        when the port is down (caller surfaces the failure)."""
        with self._txlock:
            p = self._port
            if p is None:
                return False
            try:
                p.write((line.rstrip("\r\n") + "\n").encode())
                return True
            except Exception:
                return False

    def _open(self):
        import serial   # lazy: importable without pyserial installed
        # write_timeout: a wedged CDC endpoint can block a Windows serial
        # write indefinitely (observed: 6+ min), so fail into reconnect.
        return serial.Serial(self.com, 115200, timeout=0.05,
                             write_timeout=0.5)

    def _close_port(self):
        p, self._port = self._port, None
        if p is not None:
            try:
                p.close()
            except Exception:
                pass
        with self._lock:
            self._connected = False

    def _feed_bytes(self, data):
        """Assemble complete lines (handles partial reads + CRLF)."""
        self._rxbuf += data
        while b"\n" in self._rxbuf:
            raw, self._rxbuf = self._rxbuf.split(b"\n", 1)
            text = raw.decode("utf-8", errors="replace")
            hook = self.raw_hook
            if hook is not None and text.strip():
                try:
                    hook(mask_fire_raw(text))   # passcode never persists
                except Exception:
                    pass          # recording must never break ingest
            kind, obj = self.ingest.feed(text)
            self._ingest_gs(kind, obj, self.now())

    def _run(self):
        try:
            while not self._stop.is_set():
                if self._port is None:
                    try:
                        self._port = self._open()
                    except Exception:
                        with self._lock:
                            self._connected = False
                        self._stop.wait(RECONNECT_S)
                        continue
                    self._rxbuf = b""
                    with self._lock:
                        self._connected = True
                    self._event("conn", "GS serial connected (%s)" % self.com,
                                self.now())
                try:
                    data = self._port.read(256)   # timeout paces the loop
                except Exception:
                    self._close_port()
                    self._event("conn",
                                "GS serial lost, retrying every %gs"
                                % RECONNECT_S, self.now(), severity="warn")
                    continue
                if data:
                    self._feed_bytes(data)
        finally:
            self._close_port()

    def stop(self):
        super().stop()
        self._close_port()   # belt and braces: never leave the port open


# ============================================================
# FcConsoleSource
# ============================================================
class _PendingCmd:
    """One queued console command; reply is the captured console output."""

    def __init__(self, line):
        self.line = line
        self.reply = None
        self.done = threading.Event()


class FcConsoleSource(Source):
    """FC USB console poller: port of live_dashboard.Poller.

    Priority ladder per _service() pass: queued command -> status 0.5 s
    -> gps 5 s -> radio 5 s -> charge 5 s -> log stat 5 s -> sensors
    flat-out.  execute()/submit() run arbitrary console lines inside the
    poller thread between transactions (single port owner); this is the
    command hook P2 builds on."""

    src = "fc"

    def __init__(self, com, clock=None, serial_factory=None):
        super().__init__()
        self.com = com
        self._clock = clock if clock is not None else time.time
        self._serial_factory = serial_factory
        self._port = None
        self._cmdq = deque()
        self._last_t_ms = None
        self._prev_state = None
        self._prev_fired = None
        self._last_status = 0.0
        self._last_gps = 0.0
        self._last_radio = 0.0
        self._last_tx = 0.0
        self._last_charge = 0.0
        self._last_log = 0.0
        self._last_sensor = 0.0

    # -- port --
    def _open(self):
        if self._serial_factory is not None:
            return self._serial_factory(self.com)
        import serial   # lazy: importable without pyserial installed
        # write_timeout: see GsJsonSource._open, wedged CDC blocks writes
        return serial.Serial(self.com, 115200, timeout=0.001,
                             write_timeout=0.5)

    def _close_port(self):
        p, self._port = self._port, None
        if p is not None:
            try:
                p.close()
            except Exception:
                pass
        with self._lock:
            self._connected = False

    # -- transactions (donor: live_dashboard.Poller._txn) --
    def _txn(self, port, cmd, marker, tmo=0.5):
        """Send cmd, return as soon as `marker` shows up in the reply."""
        port.reset_input_buffer()
        port.write((cmd + "\r\n").encode())
        deadline = self._clock() + tmo
        buf = b""
        while self._clock() < deadline:
            chunk = port.read(256)
            if chunk:
                buf += chunk
                if marker in buf:
                    tail = port.read(256)   # drain the marker line's tail:
                    if tail:                # e.g. "sense=" breaks before the
                        buf += tail         # "14168 mV" that follows it
                    break
            elif buf and marker in buf:
                break
        return buf.decode(errors="replace")

    def _exec_txn(self, port, line, tmo=2.5, quiet=0.15):
        """Generic command: capture output until it goes quiet (no known
        end marker for arbitrary commands)."""
        port.reset_input_buffer()
        port.write((line + "\r\n").encode())
        deadline = self._clock() + tmo
        buf = b""
        last_rx = self._clock()
        while self._clock() < deadline:
            chunk = port.read(256)
            if chunk:
                buf += chunk
                last_rx = self._clock()
            elif buf and (self._clock() - last_rx) > quiet:
                break
        return buf.decode(errors="replace")

    # -- command queue (P2 hook) --
    @staticmethod
    def _redact(line):
        """Never let the test-fire passcode reach the event log."""
        parts = line.split()
        if parts and parts[0].lower() == "fire" and len(parts) >= 3:
            parts[-1] = "****"
        return " ".join(parts)

    def submit(self, line):
        """Queue a console command; returns the pending handle."""
        p = _PendingCmd(line)
        self._cmdq.append(p)
        return p

    def execute(self, line, timeout=5.0):
        """Queue a console command and wait for its captured reply
        (None on timeout).  Runs in the poller thread between polls."""
        p = self.submit(line)
        p.done.wait(timeout)
        return p.reply

    # -- one poll-ladder step --
    def _service(self, port):
        now = self._clock()
        if self._cmdq:
            p = self._cmdq.popleft()
            try:
                p.reply = self._exec_txn(port, p.line)
            finally:
                p.done.set()
            self._scan_boot(p.reply, self.now())
            self._event("cmd_sent", self._redact(p.line), self.now())
            return "cmd"
        if now - self._last_status > STATUS_PERIOD:
            out = self._txn(port, "status", b"sense=")
            self._parse_status(out, self.now())
            self._last_status = self._clock()
            return "status"
        if now - self._last_gps > GPS_PERIOD:
            out = self._txn(port, "gps", b"last_fix")
            self._parse_gps(out, self.now())
            self._last_gps = self._clock()
            return "gps"
        if now - self._last_radio > RADIO_PERIOD:
            out = self._txn(port, "radio", b"DIO1")
            self._parse_radio(out, self.now())
            self._last_radio = self._clock()
            return "radio"
        if now - self._last_tx > TX_PERIOD:
            out = self._txn(port, "tx", b"cw:")
            self._parse_tx(out, self.now())
            self._last_tx = self._clock()
            return "tx"
        if now - self._last_charge > CHARGE_PERIOD:
            out = self._txn(port, "charge", b"fault=")
            self._parse_charge(out, self.now())
            self._last_charge = self._clock()
            return "charge"
        if now - self._last_log > LOG_PERIOD:
            out = self._txn(port, "log stat", b"bytes")
            self._parse_log(out, self.now())
            self._last_log = self._clock()
            return "log"
        out = self._txn(port, "sensors", b"baro:")
        self._parse_sensors(out, self.now())
        self._last_sensor = self._clock()
        return "sensors"

    def _run(self):
        try:
            while not self._stop.is_set():
                if self._port is None:
                    try:
                        self._port = self._open()
                        time.sleep(0.3)
                    except Exception:
                        self._close_port()
                        self._stop.wait(RECONNECT_S)
                        continue
                    with self._lock:
                        self._connected = True
                    self._event("conn",
                                "FC console connected (%s)" % self.com,
                                self.now())
                try:
                    self._service(self._port)
                except Exception:
                    self._close_port()
                    self._event("conn",
                                "FC console lost, retrying every %gs"
                                % RECONNECT_S, self.now(), severity="warn")
        finally:
            self._close_port()   # ALWAYS release COM (CDC-wedge history)

    def stop(self):
        super().stop()
        self._close_port()

    # -- parsers: normalize into the SAME schema keys the GS emits --
    def _scan_boot(self, out, now):
        """Boot banner => reboot event with reset cause (app_main.c)."""
        if BOOT_BANNER not in out:
            return
        m = RE_RESET.search(out)
        cause = m.group(1).strip() if m else "?"
        with self._lock:
            self._latest["reset_cause"] = cause
            self._latest["t_host"] = round(now, 3)
            self._push_event_locked("reboot", "FC rebooted (reset: %s)"
                                    % cause, now, severity="warn")
            self._last_t_ms = None       # don't double-report via t_ms

    def _parse_status(self, out, now):
        self._scan_boot(out, now)
        t = round(now, 3)
        with self._lock:
            self._latest["t_host"] = t
            m = RE_STATE.search(out)
            if m:
                name = m.group(1)
                t_ms = int(m.group(2))
                if (self._last_t_ms is not None
                        and t_ms + REBOOT_TMS_MARGIN < self._last_t_ms):
                    self._push_event_locked(
                        "reboot", "FC reboot detected (t_ms %d -> %d)"
                        % (self._last_t_ms, t_ms), now, t_ms=t_ms,
                        severity="warn")
                self._last_t_ms = t_ms
                self._latest["t_ms"] = t_ms
                self._latest["state_name"] = name
                try:
                    st = schema.STATE_NAMES.index(name)
                except ValueError:
                    st = None
                if st is not None:
                    if (self._prev_state is not None
                            and st != self._prev_state):
                        prev = schema.state_name(self._prev_state)
                        if name == "FAULT":
                            self._push_event_locked(
                                "fault", "state %s -> FAULT" % prev, now,
                                t_ms=t_ms, severity="crit")
                        else:
                            self._push_event_locked(
                                "state", "%s -> %s" % (prev, name), now,
                                t_ms=t_ms)
                    self._prev_state = st
                    self._latest["state"] = st
            m = RE_ARMED.search(out)
            if m:
                self._latest["armed"] = bool(int(m.group(1)))
                self._latest["testen"] = bool(int(m.group(2)))
                self._latest["main_alt_m"] = float(m.group(3))
            m = RE_ALTVEL.search(out)
            if m:
                self._latest["alt_baro_m"] = float(m.group(1))
                self._latest["vel_up_ms"] = float(m.group(2))
                self._append_locked("vel", [t, float(m.group(2))])
            m = RE_BATT.search(out)
            if m:
                mv = int(m.group(1))
                self._latest["batt_mv"] = mv
                self._latest["batt_v"] = mv / 1000.0
                self._append_locked("batt", [t, mv / 1000.0])
            m = RE_MCU.search(out)      # STM32 internal die temp (fc only)
            if m:
                self._latest["mcu_temp_c"] = float(m.group(1))
            m = RE_FLAGS.search(out)
            if m:
                self._latest["gps_fix"] = bool(int(m.group(1)))
                self._latest["accel_sat"] = bool(int(m.group(2)))
            m = RE_HEALTH.search(out)
            if m:
                g = m.groups()
                for key, val in zip(("imu_ok", "baro_ok", "sd_ok",
                                     "radio_ok"), g[:4]):
                    self._latest[key] = bool(int(val))
                if g[4] is not None:      # newer firmware: chg/gps too
                    self._latest["chg_ok"] = bool(int(g[4]))
                    self._latest["gps_ok"] = bool(int(g[5]))
            m = RE_PYRO.search(out)
            if m:
                cont = int(m.group(1), 16)
                fired = int(m.group(2), 16)
                sense = int(m.group(3))
                if (self._prev_fired is not None
                        and (fired & ~self._prev_fired)):
                    self._push_event_locked("pyro",
                                            "pyro fired: 0x%02X" % fired,
                                            now, severity="warn")
                self._prev_fired = fired
                self._latest["pyro_cont"] = cont
                self._latest["pyro_fired"] = fired
                self._latest["pyro_sense_mv"] = sense
                self._append_locked("pyro", [t, sense])

    def _parse_sensors(self, out, now):
        self._scan_boot(out, now)
        t = round(now, 3)
        ma, mg = RE_IMU.search(out), RE_GYR.search(out)
        mb = RE_BARO.search(out)
        ms = RE_IMU_SAT.search(out)
        with self._lock:
            self._latest["t_host"] = t
            if ma:
                acc = [float(x) for x in ma.groups()]
                self._latest["accel_g"] = acc
                self._append_locked("accel", [t] + acc)
            if mg:
                gyr = [float(x) for x in mg.groups()]
                self._latest["gyro_dps"] = gyr
                self._append_locked("gyro", [t] + gyr)
            if ms:
                self._latest["accel_sat"] = bool(int(ms.group(1)))
            if mb:
                self._latest["press_pa"] = float(mb.group(1))
                self._latest["temp_c"] = float(mb.group(2))
                # 62 Hz alt series comes from here; the (Kalman) status
                # `alt:` line owns latest["alt_baro_m"] at 2 Hz.
                self._append_locked("alt", [t, float(mb.group(3)), None])

    def _parse_gps(self, out, now):
        self._scan_boot(out, now)
        with self._lock:
            self._latest["t_host"] = round(now, 3)
            m = RE_GPSFIX.search(out)
            if m:
                self._latest["gps_fix"] = bool(int(m.group(1)))
                self._latest["sats"] = int(m.group(2))
            m = RE_GPSPOS.search(out)
            if m:
                self._latest["lat_deg"] = float(m.group(1))
                self._latest["lon_deg"] = float(m.group(2))
                self._latest["alt_gps_m"] = float(m.group(3))
                # 5 s GPS trace alongside the 62 Hz baro trace (the alt
                # chart's --s2 series; renderer is null-tolerant)
                self._append_locked("alt", [round(now, 3), None,
                                            float(m.group(3))])
            m = RE_GPSAGE.search(out)
            if m:
                self._latest["gps_last_fix_ms"] = int(m.group(1))

    def _parse_radio(self, out, now):
        self._scan_boot(out, now)
        with self._lock:
            self._latest["t_host"] = round(now, 3)
            m = RE_RADIO.search(out)
            if m:
                self._latest["radio_ok"] = (m.group(1) == "ok")
                self._latest["radio_tcxo"] = m.group(2)
            m = RE_RADIO_CNT.search(out)
            if m:
                self._latest["radio_reinit"] = int(m.group(1))
                self._latest["radio_txto"] = int(m.group(2))
            m = RE_RADIO_IRQ.search(out)
            if m:
                self._latest["radio_irq"] = m.group(1)

    def _parse_tx(self, out, now):
        """`tx` dashboard: TX power, packet counts, last-RX link quality.
        This is the FC's own radio state; the 'GS link' panel is downlink-
        as-seen-by-the-GS, so the FC-console path surfaces these instead."""
        self._scan_boot(out, now)
        with self._lock:
            self._latest["t_host"] = round(now, 3)
            m = RE_TX_POWER.search(out)
            if m:
                self._latest["tx_power_dbm"] = int(m.group(1))
            m = RE_TX_PKTS.search(out)
            if m:
                self._latest["tx_count"] = int(m.group(1))
                self._latest["rx_count"] = int(m.group(2))
            m = RE_TX_RXQ.search(out)   # absent when "last rx: (none)"
            if m:
                self._latest["rx_rssi"] = int(m.group(1))
                self._latest["rx_snr"] = int(m.group(2))

    def _parse_charge(self, out, now):
        self._scan_boot(out, now)
        with self._lock:
            self._latest["t_host"] = round(now, 3)
            m = RE_CHARGE.search(out)
            if m:
                mv = int(m.group(1))
                self._latest["batt_mv"] = mv
                self._latest["batt_v"] = mv / 1000.0
                self._latest["chg_stat"] = int(m.group(2), 16)
                self._latest["chg_charging"] = bool(int(m.group(3)))
                self._latest["chg_fault"] = bool(int(m.group(4)))

    def _parse_log(self, out, now):
        self._scan_boot(out, now)
        with self._lock:
            self._latest["t_host"] = round(now, 3)
            m = RE_LOG.search(out)
            if m:
                self._latest["log_running"] = (m.group(1) == "running")
            m = RE_LOGSTAT.search(out)
            if m:
                self._latest["log_drops"] = int(m.group(1))
                self._latest["log_ring_hw"] = int(m.group(2))
                self._latest["log_ring_cap"] = int(m.group(3))
