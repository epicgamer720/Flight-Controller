#!/usr/bin/env python3
"""recorder.py - Flight Deck ground-side session recorder.

One directory per session:

    recordings/deck_<YYYYmmdd_HHMMSS>/
        telem.csv    every telemetry sample, fixed column order
        events.csv   every event record (passcodes already redacted
                     upstream by the sources/adapters)
        raw.jsonl    lossless GS line capture (GS source only)

telem.csv cells are JSON literals (lists stay [..], booleans true/false,
missing null) so ReplaySource._load_csv() can round-trip a session
byte-faithfully. Rotation: when telem.csv passes ROTATE_BYTES a new part
file (telem_part2.csv, ...) is started and a `rec` event is logged.
"""

import csv
import json
import os
import threading
import time

from . import schema

ROTATE_BYTES = 50 * 1024 * 1024
FLUSH_S = 1.0

# fixed column order: wire keys, then host keys, then console extras
TELEM_COLUMNS = tuple(schema.TELEM_KEYS) + ("t_host", "src", "rssi",
                                            "snr") + schema.CONSOLE_KEYS
EVENT_COLUMNS = ("seq", "t_host", "t_ms", "kind", "severity", "text")


def _cell(v):
    """JSON-literal cell so the CSV round-trips through json.loads."""
    if v is None:
        return ""
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (list, tuple)):
        return json.dumps(list(v))
    if isinstance(v, str):
        return json.dumps(v)      # quoted -> json.loads gives the str back
    return repr(v) if isinstance(v, float) else str(v)


class Recorder:
    """Thread-safe: feed() runs on the server poll thread while
    feed_raw() runs on a GS source thread."""

    def __init__(self, recdir="recordings", enabled=True, clock=time.time):
        self.recdir = recdir
        self.enabled = enabled
        self._clock = clock
        self._lock = threading.RLock()
        self.session = None
        self._telem = None
        self._events = None
        self._raw = None
        self._part = 1
        self._rows = 0
        self._last_flush = 0.0
        self._last_event_seq = -1
        self._last_row_t = -1.0

    # -- lifecycle --
    def start(self, label="deck"):
        with self._lock:
            if self.session:
                return self.session
            stamp = time.strftime("%Y%m%d_%H%M%S",
                                  time.localtime(self._clock()))
            base = os.path.join(self.recdir, "%s_%s" % (label, stamp))
            path, n = base, 2
            while os.path.exists(path):     # same-second source switch
                path = "%s-%d" % (base, n)
                n += 1
            self.session = path
            os.makedirs(self.session, exist_ok=True)
            self._part = 1
            self._rows = 0
            self._last_event_seq = -1
            self._last_row_t = -1.0
            self._telem, self._telem_w = self._open_telem("telem.csv")
            self._events = open(os.path.join(self.session, "events.csv"),
                                "w", encoding="utf-8", newline="")
            self._events_w = csv.writer(self._events)
            self._events_w.writerow(EVENT_COLUMNS)
            return self.session

    def _open_telem(self, name):
        f = open(os.path.join(self.session, name), "w",
                 encoding="utf-8", newline="")
        w = csv.writer(f)          # proper quoting: list cells hold commas
        w.writerow(TELEM_COLUMNS)
        return f, w

    def start_raw(self):
        """GS sessions also capture the raw lines for lossless replay."""
        with self._lock:
            if self.session and self._raw is None:
                self._raw = open(os.path.join(self.session, "raw.jsonl"),
                                 "w", encoding="utf-8")

    def stop(self):
        with self._lock:
            for f in (self._telem, self._events, self._raw):
                if f is not None:
                    try:
                        f.flush()
                        f.close()
                    except Exception:
                        pass
            self._telem = self._events = self._raw = None
            self.session = None

    @property
    def active(self):
        return self._telem is not None

    @property
    def rows(self):
        return self._rows

    # -- feeding --
    def feed(self, drained):
        """Consume one Source.drain() result: one row per advanced
        latest-snapshot (poll-rate record; GS sessions are additionally
        lossless via raw.jsonl, and the FC's own 200 Hz SD log remains
        the authoritative onboard record); events append by seq."""
        with self._lock:
            if not self.active:
                return
            latest = drained.get("latest") or {}
            t = latest.get("t_host")
            if t is not None and t > self._last_row_t:
                self._last_row_t = t
                self._write_row(latest)
            for ev in drained.get("events") or []:
                if ev["seq"] > self._last_event_seq:
                    self._last_event_seq = ev["seq"]
                    self._write_event(ev)
            now = self._clock()
            if now - self._last_flush >= FLUSH_S:
                self._last_flush = now
                for f in (self._telem, self._events, self._raw):
                    if f is not None:
                        f.flush()

    def feed_raw(self, line):
        with self._lock:
            if self._raw is not None:
                self._raw.write(line.rstrip("\r\n") + "\n")

    def _write_row(self, latest):
        self._telem_w.writerow([_cell(latest.get(k))
                                for k in TELEM_COLUMNS])
        self._rows += 1
        if self._telem.tell() >= ROTATE_BYTES:
            self._part += 1
            self._telem.close()
            self._telem, self._telem_w = self._open_telem(
                "telem_part%d.csv" % self._part)

    def _write_event(self, ev):
        self._events_w.writerow([_cell(ev.get(k)) for k in EVENT_COLUMNS])
