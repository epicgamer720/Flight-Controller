#!/usr/bin/env python3
"""server.py - Flight Deck session accumulator + local HTTP API.

Deck owns the active Source, the Recorder, the command adapters, and the
session-wide derived state (peak tiles, T+ launch clock) that must
outlive the sources' 130 s rolling window. A 20 Hz poll thread drains
the source with the server's own cursor to feed the recorder and update
peaks; HTTP clients drain independently with their own `since` cursors.

Endpoints (127.0.0.1 only, HTTP/1.1 keep-alive, no-store):

    GET  /            index.html (session token injected)
    GET  /ui/<file>   static assets from deck_ui/
    GET  /data?since=<t_host>          incremental series + latest +
                                       source/rec/peaks/tplus
    GET  /events?since=<seq>           incremental event log
    POST /cmd     {"cmd","args"}       X-Deck-Token required; BLOCKS until
                                       the adapter resolves (plan deviation:
                                       simpler than async correlation and
                                       commands are rare; the UI's data
                                       loop runs on a parallel connection)
    GET  /ports                        pyserial enumeration, is_fc flag
    POST /source  {"kind","port","file","synthetic","speed"}
    POST /record  {"on": bool}
    POST /replay  {"op":"pause"|"resume", "speed"?}
"""

import json
import math
import os
import re
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from . import commands as cmdmod
from . import schema
from .recorder import Recorder
from .sources import FcConsoleSource, GsJsonSource, ReplaySource

POLL_S = 0.05
FC_VID, FC_PID = 0x0483, 0x5740      # STM32 VCP (serial_monitor.py)


def _json_safe(o):
    """Replace non-finite floats (NaN/±Inf) with None so the payload is
    valid JSON. Python's json emits bare NaN/Infinity tokens that the
    browser's JSON.parse rejects, which would freeze the whole dashboard
    on a single bad in-flight value (e.g. a diverged Kalman velocity)."""
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {k: _json_safe(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_json_safe(v) for v in o]
    return o


def _haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance (m) between two lat/lon degree pairs."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


def _bearing_deg(lat1, lon1, lat2, lon2):
    """True bearing (deg, 0=N) from point 1 to point 2."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


class Deck:
    """Session state: active source + recorder + adapters + peaks/T+."""

    def __init__(self, recdir="recordings", record=True):
        self.token = secrets.token_hex(16)
        self.recorder = Recorder(recdir=recdir, enabled=record)
        self.source = None
        self.adapter = None
        self.kind = None
        self.port = None
        self._lock = threading.RLock()
        self._cursor = -1.0
        self._eseq = -1
        self.peaks = {"apogee_m": None, "max_vel_ms": None,
                      "max_abs_g": None, "apogee_gps_m": None}
        self.tplus = {"launch_t_host": None, "state": None}
        self.pad_fix = None       # {lat, lon, alt_gps_m} snapshot near launch
        self._last_fix = None     # most recent good GPS fix (for pad snapshot)
        self._stop = threading.Event()
        self._poll = None

    # -- source lifecycle --
    def set_source(self, kind, port=None, file=None, synthetic=None,
                   speed=1.0):
        with self._lock:
            self._teardown_source()
            if kind == "fc":
                src, adapter = (FcConsoleSource(port),
                                None)   # adapter needs src, set below
                adapter = cmdmod.ConsoleCommandAdapter(src)
            elif kind == "gs":
                src = GsJsonSource(port)
                adapter = cmdmod.GsCommandAdapter(src)
            elif kind == "replay":
                src = ReplaySource(path=file, synthetic=synthetic,
                                   speed=speed)
                adapter = None
            else:
                raise ValueError("unknown source kind %r" % kind)
            self.source, self.adapter = src, adapter
            self.kind, self.port = kind, port
            self._cursor = -1.0
            self._eseq = -1
            self.peaks = {"apogee_m": None, "max_vel_ms": None,
                          "max_abs_g": None, "apogee_gps_m": None}
            self.tplus = {"launch_t_host": None, "state": None}
            self.pad_fix = None
            self._last_fix = None
            if self.recorder.enabled and kind in ("fc", "gs"):
                if self.recorder.active:
                    self.recorder.stop()   # rotate: fresh session per source
                self.recorder.start()      # (stale high-water marks would
                src.push_event("rec", "recording -> %s"   # drop everything)
                               % self.recorder.session)
                if kind == "gs":
                    self.recorder.start_raw()
                    src.raw_hook = self.recorder.feed_raw
            src.start()
            self._ensure_poll()

    def _teardown_source(self):
        src = self.source
        if src is not None:
            try:
                src.stop()          # ALWAYS closes the port (CDC wedge)
            except Exception:
                pass
        self.source = self.adapter = None

    def _ensure_poll(self):
        if self._poll is None or not self._poll.is_alive():
            self._stop.clear()
            self._poll = threading.Thread(target=self._poll_run,
                                          daemon=True)
            self._poll.start()

    def shutdown(self):
        self._stop.set()
        if self._poll is not None and self._poll.is_alive():
            self._poll.join(timeout=2.0)
        with self._lock:
            self._teardown_source()
            self.recorder.stop()

    # -- 20 Hz accumulator: recorder + peaks + T+ --
    def _poll_run(self):
        while not self._stop.is_set():
            self.poll_once()
            self._stop.wait(POLL_S)

    def poll_once(self):
        with self._lock:
            src = self.source
            if src is None:
                return
            d = src.drain(since=self._cursor, event_seq=self._eseq)
        series = d["series"]
        for rows in series.values():
            if rows:
                self._cursor = max(self._cursor, rows[-1][0])
        if d["events"]:
            self._eseq = max(self._eseq, d["events"][-1]["seq"])
        self._update_peaks(d)
        if self.recorder.active:
            self.recorder.feed(d)

    def _update_peaks(self, d):
        p = self.peaks
        for row in d["series"].get("alt") or ():
            v = row[1]
            if v is not None and (p["apogee_m"] is None
                                  or v > p["apogee_m"]):
                p["apogee_m"] = round(v, 2)
        for row in d["series"].get("vel") or ():
            v = row[1]
            if v is not None and (p["max_vel_ms"] is None
                                  or v > p["max_vel_ms"]):
                p["max_vel_ms"] = round(v, 2)
        for row in d["series"].get("accel") or ():
            if row[1] is None:
                continue
            mag = math.sqrt(sum(x * x for x in row[1:4] if x is not None))
            if p["max_abs_g"] is None or mag > p["max_abs_g"]:
                p["max_abs_g"] = round(mag, 2)
        latest = d.get("latest") or {}
        st = latest.get("state")
        if st is not None:
            self.tplus["state"] = st
        # track the most recent good GPS fix, to freeze as the pad position
        if latest.get("gps_fix") and latest.get("lat_deg") is not None \
                and latest.get("lon_deg") is not None:
            self._last_fix = {"lat": latest["lat_deg"], "lon": latest["lon_deg"],
                              "alt_gps_m": latest.get("alt_gps_m")}
            # fallback: if we launched before ever getting a fix, adopt the first
            # post-launch fix as an approximate pad reference (better than never).
            if self.pad_fix is None and self.tplus["launch_t_host"] is not None:
                self.pad_fix = dict(self._last_fix)
        if self.tplus["launch_t_host"] is None:
            for ev in d.get("events") or ():
                if ev["kind"] == "state" and ev["text"].endswith("-> BOOST"):
                    self.tplus["launch_t_host"] = ev["t_host"]
                    if self.pad_fix is None and self._last_fix is not None:
                        self.pad_fix = dict(self._last_fix)     # freeze pad fix
                    break
        # GPS-derived apogee (MSL AGL vs pad) for the baro-vs-GPS check (7d)
        if self.pad_fix is not None and self.pad_fix.get("alt_gps_m") is not None:
            ag = latest.get("alt_gps_m")
            if ag is not None:
                agl = ag - self.pad_fix["alt_gps_m"]
                if self.peaks["apogee_gps_m"] is None or agl > self.peaks["apogee_gps_m"]:
                    self.peaks["apogee_gps_m"] = round(agl, 2)

    def _recovery(self, latest):
        """Bearing + distance from the frozen pad fix to the latest position
        (recovery aid). None until we have both a pad fix and a live fix."""
        pad = self.pad_fix          # snapshot: a concurrent set_source() may null it
        if pad is None:
            return None
        lat, lon = latest.get("lat_deg"), latest.get("lon_deg")
        if lat is None or lon is None or not latest.get("gps_fix"):
            return None
        return {
            "distance_m": round(_haversine_m(pad["lat"], pad["lon"], lat, lon), 1),
            "bearing_deg": round(_bearing_deg(pad["lat"], pad["lon"], lat, lon), 1),
            "pad_lat": pad["lat"], "pad_lon": pad["lon"],
        }

    # -- request-facing --
    def data(self, since):
        src = self.source
        if src is None:
            return {"now": 0, "source": {"kind": None, "connected": False},
                    "rec": self._rec_info(), "latest": {}, "series": {},
                    "peaks": self.peaks, "tplus": self.tplus,
                    "recovery": None, "counters": {}}
        d = src.drain(since=since)
        return {
            "now": d["now"],
            "source": {"kind": self.kind, "port": self.port,
                       "connected": d["connected"]},
            "rec": self._rec_info(),
            "latest": d["latest"],
            "series": d["series"],
            "gap": d.get("gap"),
            "counters": d["counters"],
            "peaks": self.peaks,
            "tplus": dict(self.tplus),
            "recovery": self._recovery(d.get("latest") or {}),
            "capabilities": {n: c.to_dict() for n, c
                             in cmdmod.CAPABILITIES.items()},
        }

    def events(self, since_seq):
        src = self.source
        if src is None:
            return {"events": []}
        d = src.drain(since=float("inf"), event_seq=since_seq)
        return {"events": d["events"]}

    def _rec_info(self):
        return {"on": self.recorder.active,
                "path": self.recorder.session,
                "rows": self.recorder.rows}

    def command(self, name, args):
        adapter = self.adapter
        if adapter is None:
            return cmdmod.CmdResult(
                False, refusal="no command path on this source (replay)")
        return adapter.send(name, args)

    def replay_ctl(self, op, speed=None):
        src = self.source
        if not isinstance(src, ReplaySource):
            return False
        if speed is not None:
            src.speed = float(speed)
        if op == "pause":
            src.pause()
        elif op == "resume":
            src.resume()
        return True


def list_ports():
    try:
        from serial.tools import list_ports as lp
    except Exception:
        return []
    out = []
    for p in lp.comports():
        out.append({"device": p.device, "vid": p.vid, "pid": p.pid,
                    "desc": p.description,
                    "is_fc": (p.vid == FC_VID and p.pid == FC_PID)})
    return out


def make_server(deck, ui_dir, host="127.0.0.1", port=8322):
    """Build (but do not start) the ThreadingHTTPServer."""

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        # -- helpers --
        def _send(self, code, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else \
                json.dumps(_json_safe(body)).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _authed(self):
            return self.headers.get("X-Deck-Token") == deck.token

        def _body(self):
            n = int(self.headers.get("Content-Length") or 0)
            if n <= 0 or n > 65536:
                return {}
            try:
                return json.loads(self.rfile.read(n).decode())
            except ValueError:
                return {}

        def _qs(self, name, default):
            m = re.search(r"[?&]%s=([^&]+)" % name, self.path)
            return m.group(1) if m else default

        # -- GET --
        def do_GET(self):
            if self.path.startswith("/data"):
                since = float(self._qs("since", "-1"))
                self._send(200, deck.data(since))
            elif self.path.startswith("/events"):
                seq = int(self._qs("since", "-1"))
                self._send(200, deck.events(seq))
            elif self.path.startswith("/ports"):
                self._send(200, {"ports": list_ports()})
            elif self.path == "/" or self.path.startswith(
                    ("/index", "/?")):
                self._static("index.html", inject_token=True)
            elif self.path.startswith("/ui/"):
                name = os.path.basename(self.path.split("?")[0])
                self._static(name)
            else:
                self._send(404, {"error": "not found"})

        def _static(self, name, inject_token=False):
            path = os.path.join(ui_dir, name)
            if not os.path.isfile(path):
                self._send(404, {"error": "%s missing (UI not built yet?)"
                                 % name})
                return
            with open(path, "rb") as f:
                data = f.read()
            if inject_token:
                data = data.replace(b"__DECK_TOKEN__",
                                    deck.token.encode())
                # cache-buster: WebView2 can serve stale JS across app
                # relaunches despite no-store; a fresh stamp per asset
                # mtime forces a reload whenever the UI files change
                stamp = str(int(max(
                    os.path.getmtime(os.path.join(ui_dir, n))
                    for n in os.listdir(ui_dir)
                    if os.path.isfile(os.path.join(ui_dir, n)))))
                data = data.replace(b"__DECK_V__", stamp.encode())
            ext = os.path.splitext(name)[1].lower()
            ctype = {".html": "text/html; charset=utf-8",
                     ".js": "text/javascript; charset=utf-8",
                     ".css": "text/css; charset=utf-8",
                     ".svg": "image/svg+xml"}.get(ext,
                                                  "application/octet-stream")
            self._send(200, data, ctype)

        # -- POST --
        def do_POST(self):
            if not self._authed():
                self._send(403, {"error": "bad or missing X-Deck-Token"})
                return
            body = self._body()
            if self.path.startswith("/cmd"):
                name = body.get("cmd", "")
                res = deck.command(name, body.get("args") or {})
                self._send(200, res.to_dict())
            elif self.path.startswith("/source"):
                try:
                    deck.set_source(body.get("kind"),
                                    port=body.get("port"),
                                    file=body.get("file"),
                                    synthetic=body.get("synthetic"),
                                    speed=body.get("speed", 1.0))
                    self._send(200, {"ok": True})
                except Exception as e:
                    self._send(200, {"ok": False, "error": str(e)})
            elif self.path.startswith("/record"):
                if body.get("on"):
                    deck.recorder.enabled = True
                    deck.recorder.start()
                    if deck.kind == "gs" and deck.source is not None:
                        deck.recorder.start_raw()   # lossless GS capture
                        deck.source.raw_hook = deck.recorder.feed_raw
                else:
                    deck.recorder.stop()
                self._send(200, deck._rec_info())
            elif self.path.startswith("/replay"):
                ok = deck.replay_ctl(body.get("op"), body.get("speed"))
                self._send(200, {"ok": ok})
            else:
                self._send(404, {"error": "not found"})

        def log_message(self, *a):
            pass

    return ThreadingHTTPServer((host, port), Handler)
