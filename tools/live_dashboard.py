#!/usr/bin/env python3
"""Live bring-up dashboard for the flight controller.

Holds the FC's USB console (COM port), polls `sensors`/`status`/`gps`
continuously, and serves a live-updating dashboard on localhost.

    python tools/live_dashboard.py [--port COM9] [--http 8321]

Then open http://localhost:8321 — interactive charts (pan / zoom / pause)
over the last 2 minutes of history.
Note: while this runs it owns the COM port (serial_monitor.py can't
connect at the same time).
"""
import argparse
import json
import re
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import serial
except ImportError:
    raise SystemExit("pyserial missing — run:  python -m pip install pyserial")

WINDOW_S = 120.0          # rolling buffer window (history headroom for pan-back)
SENSOR_PERIOD = 0.0       # `sensors` cadence (poll flat-out; serial round-trip is the limit)
STATUS_PERIOD = 0.5       # `status` cadence
GPS_PERIOD = 5.0          # `gps` cadence
RADIO_PERIOD = 5.0        # `radio` cadence

RE_IMU = re.compile(r"ax=(-?[\d.]+) ay=(-?[\d.]+) az=(-?[\d.]+)")
RE_GYR = re.compile(r"gx=(-?[\d.]+) gy=(-?[\d.]+) gz=(-?[\d.]+)")
RE_BARO = re.compile(r"baro: (-?[\d.]+) Pa\s+(-?[\d.]+) C\s+alt=(-?[\d.]+)")
RE_STATE = re.compile(r"state:\s+(\w+)")
RE_ARMED = re.compile(r"armed:\s+(\d)\s+testen:\s*(\d)")
RE_ALTVEL = re.compile(r"alt:\s+(-?[\d.]+) m AGL\s+vel: (-?[\d.]+)")
RE_BATT = re.compile(r"batt:\s+(\d+) mV")
RE_HEALTH = re.compile(r"imu=(\d) baro=(\d) sd=(\d) radio=(\d)")
RE_PYRO = re.compile(r"cont=0x([0-9A-Fa-f]+) fired=0x([0-9A-Fa-f]+) sense=(\d+) mV")
RE_GPSFIX = re.compile(r"fix=(\d) sats=(\d+)")
RE_RADIO = re.compile(r"radio:\s+(\w+)\s+tcxo:\s+(\S+)")


class Poller(threading.Thread):
    def __init__(self, com):
        super().__init__(daemon=True)
        self.com = com
        self.lock = threading.Lock()
        self.t0 = time.time()
        self.connected = False
        self.imu = deque()       # [t, ax, ay, az, gx, gy, gz]
        self.baro = deque()      # [t, press_pa, temp_c, alt_m]
        self.batt = deque()      # [t, mv]
        self.pyro = deque()      # [t, sense_mv]
        self.status = {}         # latest parsed status/gps fields

    def _trim(self, dq, now):
        while dq and dq[0][0] < now - WINDOW_S - 5:
            dq.popleft()

    def _txn(self, port, cmd, marker, tmo=0.5):
        """Send cmd, return as soon as `marker` shows up in the reply."""
        port.reset_input_buffer()
        port.write((cmd + "\r\n").encode())
        deadline = time.time() + tmo
        buf = b""
        while time.time() < deadline:
            chunk = port.read(256)
            if chunk:
                buf += chunk
                if marker in buf:
                    break
            elif buf and marker in buf:
                break
        return buf.decode(errors="replace")

    def run(self):
        port = None
        last_status = last_gps = last_radio = last_sensor = 0.0
        while True:
            if port is None:
                try:
                    port = serial.Serial(self.com, 115200, timeout=0.001)
                    time.sleep(0.3)
                    with self.lock:
                        self.connected = True
                except Exception:
                    with self.lock:
                        self.connected = False
                    time.sleep(2.0)
                    continue
            try:
                now = time.time() - self.t0
                if time.time() - last_status > STATUS_PERIOD:
                    out = self._txn(port, "status", b"sense=")
                    self._parse_status(out, now)
                    last_status = time.time()
                elif time.time() - last_gps > GPS_PERIOD:
                    out = self._txn(port, "gps", b"last_fix")
                    m = RE_GPSFIX.search(out)
                    if m:
                        with self.lock:
                            self.status["gps_fix"] = int(m.group(1))
                            self.status["gps_sats"] = int(m.group(2))
                    last_gps = time.time()
                elif time.time() - last_radio > RADIO_PERIOD:
                    out = self._txn(port, "radio", b"DIO1")
                    m = RE_RADIO.search(out)
                    if m:
                        with self.lock:
                            self.status["radio_txt"] = m.group(1)
                            self.status["radio_tcxo"] = m.group(2)
                    last_radio = time.time()
                elif time.time() - last_sensor >= SENSOR_PERIOD:
                    out = self._txn(port, "sensors", b"baro:")
                    self._parse_sensors(out, now)
                    last_sensor = time.time()
                else:
                    time.sleep(0.01)
            except Exception:
                try:
                    port.close()
                except Exception:
                    pass
                port = None
                with self.lock:
                    self.connected = False

    def _parse_sensors(self, out, now):
        ma, mg = RE_IMU.search(out), RE_GYR.search(out)
        mb = RE_BARO.search(out)
        with self.lock:
            if ma and mg:
                self.imu.append([round(now, 2)] +
                                [float(x) for x in ma.groups()] +
                                [float(x) for x in mg.groups()])
                self._trim(self.imu, now)
            if mb:
                self.baro.append([round(now, 2)] + [float(x) for x in mb.groups()])
                self._trim(self.baro, now)

    def _parse_status(self, out, now):
        with self.lock:
            m = RE_STATE.search(out)
            if m:
                self.status["state"] = m.group(1)
            m = RE_ARMED.search(out)
            if m:
                self.status["armed"] = int(m.group(1))
                self.status["testen"] = int(m.group(2))
            m = RE_ALTVEL.search(out)
            if m:
                self.status["alt_m"] = float(m.group(1))
                self.status["vel_ms"] = float(m.group(2))
            m = RE_HEALTH.search(out)
            if m:
                self.status["imu_ok"], self.status["baro_ok"], \
                    self.status["sd_ok"], self.status["radio_ok"] = \
                    (int(x) for x in m.groups())
            m = RE_BATT.search(out)
            if m:
                self.batt.append([round(now, 2), int(m.group(1))])
                self._trim(self.batt, now)
            m = RE_PYRO.search(out)
            if m:
                self.status["pyro_cont"] = int(m.group(1), 16)
                self.status["pyro_fired"] = int(m.group(2), 16)
                self.pyro.append([round(now, 2), int(m.group(3))])
                self._trim(self.pyro, now)

    def snapshot(self, since=-1.0):
        """Rows newer than `since` only — the page keeps its own buffer."""
        with self.lock:
            return json.dumps({
                "connected": self.connected,
                "now": round(time.time() - self.t0, 2),
                "status": self.status,
                "imu": [r for r in self.imu if r[0] > since],
                "baro": [r for r in self.baro if r[0] > since],
                "batt": [r for r in self.batt if r[0] > since],
                "pyro": [r for r in self.pyro if r[0] > since],
            })


PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>FC Live</title>
<style>
  :root {
    --paper:#101514; --card:#1a201e; --ink:#ecf1ef; --ink-2:#a9b5b0; --ink-3:#7e8a85;
    --line:rgba(236,241,239,.13); --line-soft:rgba(236,241,239,.07); --accent:#7fb8d4;
    --good:#0ca30c; --good-text:#3fc53f; --good-wash:rgba(12,163,12,.16);
    --warn-text:#fab219; --warn-wash:rgba(250,178,25,.13);
    --crit-text:#e46262; --crit-wash:rgba(208,59,59,.16);
    --idle-text:#a9b5b0; --idle-wash:rgba(126,138,133,.16);
    --s1:#3987e5; --s2:#199e70; --s3:#c98500; --grid:#262d2b; --axis:#3a423f;
  }
  @media (prefers-color-scheme: light) {
    :root {
      --paper:#f4f6f5; --card:#fcfdfc; --ink:#131a18; --ink-2:#55615d; --ink-3:#7e8a85;
      --line:rgba(19,26,24,.12); --line-soft:rgba(19,26,24,.07); --accent:#2b6e8f;
      --good-text:#006300; --good-wash:rgba(12,163,12,.10);
      --warn-text:#7a5500; --warn-wash:rgba(250,178,25,.14);
      --crit-text:#a12626; --crit-wash:rgba(208,59,59,.10);
      --idle-text:#55615d; --idle-wash:rgba(126,138,133,.12);
      --s1:#2a78d6; --s2:#1baf7a; --s3:#eda100; --grid:#e3e5e1; --axis:#c0c5c1;
    }
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--paper);color:var(--ink);
       font-family:"Segoe UI",system-ui,-apple-system,sans-serif;line-height:1.45}
  .mono{font-family:Consolas,"Cascadia Mono",ui-monospace,monospace}
  .wrap{max-width:1460px;margin:0 auto;padding:28px 24px 48px}
  header{display:flex;flex-wrap:wrap;justify-content:space-between;align-items:baseline;
         gap:8px 24px;border-bottom:2px solid var(--ink);padding-bottom:14px}
  h1{font-size:22px;font-weight:650;margin:0;letter-spacing:-.01em}
  .eyebrow{font-family:Consolas,ui-monospace,monospace;font-size:10.5px;letter-spacing:.14em;
           text-transform:uppercase;color:var(--accent);margin:0 0 4px}
  .livechip{display:inline-flex;align-items:center;gap:8px;font-size:12.5px;font-weight:650;
            letter-spacing:.05em;text-transform:uppercase}
  .livedot{width:10px;height:10px;border-radius:50%;background:var(--idle-text)}
  .livedot.on{background:var(--good)}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:.35}}
  @media (prefers-reduced-motion: no-preference){ .livedot.on{animation:pulse 1.6s ease-in-out infinite} }
  .strip{display:flex;flex-wrap:wrap;gap:10px;margin:18px 0 6px}
  .chip{background:var(--card);border:1px solid var(--line);border-radius:6px;
        padding:8px 14px;min-width:110px}
  .chip .l{font-size:10px;letter-spacing:.1em;text-transform:uppercase;color:var(--ink-3);
           font-family:Consolas,ui-monospace,monospace}
  .chip .v{font-size:19px;font-weight:650;font-variant-numeric:tabular-nums;margin-top:1px}
  .chip .v.small{font-size:15px}
  .ok{color:var(--good-text)} .bad{color:var(--crit-text)} .meh{color:var(--warn-text)}
  h2{font-size:12.5px;font-weight:650;letter-spacing:.12em;text-transform:uppercase;
     color:var(--ink-2);margin:26px 0 12px;display:flex;align-items:center;gap:12px}
  h2::after{content:"";flex:1;height:1px;background:var(--line)}
  .toolbar{display:flex;flex-wrap:wrap;align-items:center;gap:10px 18px;margin:0 0 14px}
  .tb-group{display:flex;align-items:center;gap:8px}
  .tb-label{font-size:10px;letter-spacing:.1em;text-transform:uppercase;color:var(--ink-3);
            font-family:Consolas,ui-monospace,monospace}
  .seg{display:inline-flex;background:var(--card);border:1px solid var(--line);
       border-radius:6px;overflow:hidden}
  .seg button{appearance:none;border:0;border-right:1px solid var(--line-soft);
              background:transparent;color:var(--ink-2);
              font:600 12px "Segoe UI",system-ui,sans-serif;padding:6px 12px;cursor:pointer}
  .seg button:last-child{border-right:0}
  .seg button:hover{color:var(--ink)}
  .seg button.on{background:var(--accent);color:var(--paper)}
  .btn{appearance:none;background:var(--card);border:1px solid var(--line);border-radius:6px;
       color:var(--ink-2);font:600 12px "Segoe UI",system-ui,sans-serif;
       padding:6px 14px;cursor:pointer}
  .btn:hover{color:var(--ink)}
  .btn.live{display:inline-flex;align-items:center;gap:7px;letter-spacing:.06em;
            text-transform:uppercase;font-size:11px}
  .btn.live .ldot{width:8px;height:8px;border-radius:50%;background:var(--idle-text)}
  .btn.live.on{color:var(--good-text);border-color:var(--good-text);background:var(--good-wash)}
  .btn.live.on .ldot{background:var(--good)}
  @media (prefers-reduced-motion: no-preference){ .btn.live.on .ldot{animation:pulse 1.6s ease-in-out infinite} }
  .chart-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(480px,1fr));gap:14px}
  .chart-card{background:var(--card);border:1px solid var(--line);border-radius:6px;padding:12px 12px 6px}
  .chart-head{display:flex;justify-content:space-between;align-items:baseline;gap:12px;flex-wrap:wrap}
  .chart-title{font-size:14px;font-weight:650;margin:0}
  .chart-unit{font-size:11px;color:var(--ink-3)}
  .legend{display:flex;gap:14px;font-size:11.5px;color:var(--ink-2);margin:2px 0 2px}
  .legend .k{display:inline-flex;align-items:center;gap:6px}
  .sw{width:10px;height:3px;border-radius:2px;display:inline-block}
  .chart-box{position:relative;height:260px;user-select:none}
  .chart-box canvas{display:block;width:100%;height:100%;cursor:crosshair;touch-action:none}
  footer{margin-top:32px;padding-top:12px;border-top:1px solid var(--line);
         font-size:12px;color:var(--ink-3)}
</style>
</head>
<body>
<div class="wrap">
  <header>
    <div>
      <p class="eyebrow">Darsh Rocketry &middot; Avionics</p>
      <h1>Flight Controller — Live</h1>
    </div>
    <span class="livechip"><span class="livedot" id="dot"></span><span id="conn">connecting&hellip;</span></span>
  </header>

  <div class="strip">
    <div class="chip"><div class="l">State</div><div class="v small" id="st">—</div></div>
    <div class="chip"><div class="l">Armed</div><div class="v small" id="armed">—</div></div>
    <div class="chip"><div class="l">Battery</div><div class="v" id="batt">—</div></div>
    <div class="chip"><div class="l">Alt AGL</div><div class="v" id="alt">—</div></div>
    <div class="chip"><div class="l">Vel</div><div class="v" id="vel">—</div></div>
    <div class="chip"><div class="l">GPS</div><div class="v small" id="gps">—</div></div>
    <div class="chip"><div class="l">Health imu&middot;baro&middot;sd&middot;radio</div><div class="v small" id="health">—</div></div>
    <div class="chip"><div class="l">Radio</div><div class="v small" id="radio">—</div></div>
  </div>

  <h2 id="chart-h">Telemetry — last 10 s</h2>
  <div class="toolbar">
    <div class="tb-group">
      <span class="tb-label">Span</span>
      <div class="seg" id="seg-span">
        <button data-span="10" class="on">10s</button>
        <button data-span="30">30s</button>
        <button data-span="60">60s</button>
        <button data-span="120">120s</button>
      </div>
    </div>
    <div class="tb-group">
      <button class="btn" id="btn-pause">Pause</button>
      <button class="btn live on" id="btn-live"><span class="ldot"></span>Live</button>
    </div>
    <div class="tb-group">
      <span class="tb-label">Filter</span>
      <div class="seg" id="seg-filt">
        <button data-alpha="0" class="on">Off</button>
        <button data-alpha="0.25">Light</button>
        <button data-alpha="0.06">Strong</button>
      </div>
    </div>
  </div>

  <div class="chart-grid">
    <div class="chart-card">
      <div class="chart-head"><p class="chart-title">Accelerometer</p><span class="chart-unit">g</span></div>
      <div class="legend">
        <span class="k"><span class="sw" style="background:var(--s1)"></span>X</span>
        <span class="k"><span class="sw" style="background:var(--s2)"></span>Y</span>
        <span class="k"><span class="sw" style="background:var(--s3)"></span>Z</span>
      </div>
      <div class="chart-box" id="ch-accel"></div>
    </div>
    <div class="chart-card">
      <div class="chart-head"><p class="chart-title">Gyroscope</p><span class="chart-unit">deg/s</span></div>
      <div class="legend">
        <span class="k"><span class="sw" style="background:var(--s1)"></span>X</span>
        <span class="k"><span class="sw" style="background:var(--s2)"></span>Y</span>
        <span class="k"><span class="sw" style="background:var(--s3)"></span>Z</span>
      </div>
      <div class="chart-box" id="ch-gyro"></div>
    </div>
    <div class="chart-card">
      <div class="chart-head"><p class="chart-title">Altitude (baro)</p><span class="chart-unit">m AGL &middot; needs BMP580</span></div>
      <div class="chart-box" id="ch-alt"></div>
    </div>
    <div class="chart-card">
      <div class="chart-head"><p class="chart-title">Battery</p><span class="chart-unit">V</span></div>
      <div class="chart-box" id="ch-batt"></div>
    </div>
  </div>

  <footer>Served locally by <span class="mono">tools/live_dashboard.py</span> — it owns the COM port while running.
  Charts: drag to pan, wheel to zoom, double-click (or Live) to follow, Space pauses;
  the filter smooths the displayed trace only. 2 min of history buffered.</footer>
</div>

<script>
'use strict';

/* ---------- theme: canvas colors come from the CSS custom properties ---------- */
let THEME = {};
function loadTheme(){
  const cs = getComputedStyle(document.documentElement);
  const g = n => cs.getPropertyValue(n).trim();
  THEME = {
    series: [g('--s1'), g('--s2'), g('--s3')],
    grid: g('--grid'), axis: g('--axis'),
    ink: g('--ink'), ink3: g('--ink-3'),
    card: g('--card'), line: g('--line')
  };
}
loadTheme();
window.matchMedia('(prefers-color-scheme: dark)')
      .addEventListener('change', () => { loadTheme(); markDirty(); });

/* ---------- shared time-axis view state (all four charts) ---------- */
const view = { span: 10, end: null };   // end === null -> follow live (right edge = newest t)
const FILT = { alpha: 0 };              // EMA display filter; 0 = off
let dataNow = 0;                        // newest sample time reported by the server
const charts = [];

function latestT(){ return lastT >= 0 ? lastT : dataNow; }   // newest SAMPLE, not server clock
function viewT1(){ return view.end === null ? latestT() : view.end; }
function bufferStart(){
  let s = Infinity;
  for (const k of ['imu','baro','batt','pyro'])
    if (buf[k].length && buf[k][0][0] < s) s = buf[k][0][0];
  return s === Infinity ? latestT() : s;
}
function clampEnd(end){
  end = Math.min(end, latestT());                                  // not past newest sample
  end = Math.max(end, Math.min(latestT(), bufferStart() + view.span)); // not past history
  return end;
}
function markDirty(){ for (const c of charts) c.dirty = true; }
function goLive(){ view.end = null; syncUI(); markDirty(); }
function togglePause(){
  view.end = (view.end === null) ? latestT() : null;
  syncUI(); markDirty();
}
function setSpan(s){
  view.span = Math.min(120, Math.max(1, s));
  if (view.end !== null) view.end = clampEnd(view.end);
  syncUI(); markDirty();
}

/* first index i with rows[i][0] >= t (rows are time-ordered) */
function lowerBound(rows, t){
  let lo = 0, hi = rows.length;
  while (lo < hi){ const m = (lo + hi) >> 1; if (rows[m][0] < t) lo = m + 1; else hi = m; }
  return lo;
}
function xStep(span){
  const steps = [0.5, 1, 2, 5, 10, 15, 30, 60];
  for (const s of steps) if (s >= span / 5) return s;
  return 60;
}
function fmtOff(t, t1, liveEdge){  // axis label: offset of t from the view's right edge
  const v = t - t1;                // (stable while paused — never tied to the wall clock)
  if (v > -0.05) return liveEdge ? 'now' : '0s';
  return (Math.abs(v - Math.round(v)) < 0.05 ? Math.round(v) : v.toFixed(1)) + 's';
}

/* min/max-per-pixel-bucket decimated polyline; f = flat [t0,v0,t1,v1,...] */
function strokePts(ctx, f, X, Y, t0, span, pw){
  const n = f.length >> 1;
  if (n < 2) return false;
  ctx.beginPath();
  if (n <= pw * 2){                       // sparse enough: draw every point
    for (let i = 0; i < n; i++){
      const x = X(f[2*i]), y = Y(f[2*i+1]);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
  } else {                                 // one min + one max per pixel bucket
    const bw = span / pw;
    let started = false, b = null, minV = 0, maxV = 0, minI = 0, maxI = 0;
    const emit = i => {
      const x = X(f[2*i]), y = Y(f[2*i+1]);
      if (started) ctx.lineTo(x, y); else { ctx.moveTo(x, y); started = true; }
    };
    const flush = () => {
      if (b === null) return;
      if (minI === maxI) emit(minI);
      else if (minI < maxI){ emit(minI); emit(maxI); }
      else { emit(maxI); emit(minI); }
    };
    for (let i = 0; i < n; i++){
      const bi = Math.floor((f[2*i] - t0) / bw);
      if (bi !== b){
        flush();
        b = bi; minV = maxV = f[2*i+1]; minI = maxI = i;
      } else {
        const v = f[2*i+1];
        if (v < minV){ minV = v; minI = i; }
        if (v > maxV){ maxV = v; maxI = i; }
      }
    }
    flush();
  }
  ctx.stroke();
  return true;
}

/* ---------- canvas chart ---------- */
class CanvasChart {
  constructor(boxId, opts){
    this.opts = opts;
    this.box = document.getElementById(boxId);
    this.canvas = document.createElement('canvas');
    this.canvas.setAttribute('role', 'img');
    this.canvas.setAttribute('aria-label', opts.label);
    this.box.appendChild(this.canvas);
    this.ctx = this.canvas.getContext('2d');
    this.margin = { l: 52, r: 14, t: 10, b: 22 };
    this.w = 0; this.h = 0; this.dpr = 1;
    this.dirty = true;
    this.hover = null;        // {x,y} in CSS px, null when cursor is off
    this.dragging = false;
    this.moved = false;       // true once a drag passes the click threshold
    this.dragX = 0;
    this._raw = [];           // reused flat [t,v,...] scratch arrays
    this._flt = [];
    new ResizeObserver(() => this._resize()).observe(this.box);
    this._bind();
    charts.push(this);
  }

  _resize(){
    const r = this.box.getBoundingClientRect();
    this.w = r.width; this.h = r.height;
    this.dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.max(1, Math.round(this.w * this.dpr));
    this.canvas.height = Math.max(1, Math.round(this.h * this.dpr));
    this.dirty = true;
  }

  _plotW(){ return this.w - this.margin.l - this.margin.r; }

  _bind(){
    const cv = this.canvas;
    cv.addEventListener('pointerdown', e => {
      if (e.button !== 0) return;
      this.dragging = true; this.moved = false; this.dragX = e.offsetX;
      cv.setPointerCapture(e.pointerId);          // drags never stick
      this.dirty = true;
    });
    cv.addEventListener('pointermove', e => {
      this.hover = { x: e.offsetX, y: e.offsetY };
      if (this.dragging){
        const pw = this._plotW();
        const dx = e.offsetX - this.dragX;
        if (!this.moved){                          // a plain click must not pause follow
          if (Math.abs(dx) < 3) return;
          this.moved = true;
          if (view.end === null){ view.end = latestT(); syncUI(); }
        }
        this.dragX = e.offsetX;
        if (pw > 0){
          view.end = clampEnd(view.end - dx * view.span / pw);
          markDirty();
        }
      } else this.dirty = true;
    });
    const endDrag = e => {
      if (!this.dragging) return;
      this.dragging = false;
      try { cv.releasePointerCapture(e.pointerId); } catch (_) {}
      this.dirty = true;
    };
    cv.addEventListener('pointerup', endDrag);
    cv.addEventListener('pointercancel', endDrag);
    cv.addEventListener('pointerleave', () => { this.hover = null; this.dirty = true; });
    cv.addEventListener('dblclick', () => goLive());
    cv.addEventListener('wheel', e => {
      e.preventDefault();                          // keep the page from scrolling
      const pw = this._plotW();
      if (pw <= 0) return;
      const ns = Math.min(120, Math.max(1, view.span * Math.exp(e.deltaY * 0.0012)));
      if (view.end === null){
        view.span = ns;                            // live: right edge stays pinned
      } else {
        const frac = Math.min(1, Math.max(0, (e.offsetX - this.margin.l) / pw));
        const tCur = (view.end - view.span) + frac * view.span;  // time under cursor
        view.span = ns;
        view.end = clampEnd(tCur + (1 - frac) * ns);             // anchor it
      }
      syncUI(); markDirty();
    }, { passive: false });
  }

  draw(){
    const o = this.opts, ctx = this.ctx, w = this.w, h = this.h;
    if (!w || !h) return;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const L = this.margin.l, R = this.margin.r, T = this.margin.t;
    const pw = w - L - R, ph = h - T - this.margin.b;
    if (pw < 20 || ph < 20) return;
    const t1 = viewT1(), span = view.span, t0 = t1 - span;
    const rows = buf[o.buf];

    /* visible index range (binary search) + autoscale over visible points */
    const i0 = lowerBound(rows, t0);
    let iEnd = i0, lo = Infinity, hi = -Infinity;
    for (let i = i0; i < rows.length; i++){
      if (rows[i][0] > t1) break;
      iEnd = i + 1;
      for (let c = 0; c < o.cols.length; c++){
        const v = rows[i][o.cols[c]] * o.scale;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      }
    }
    const nVis = iEnd - i0;
    const iStart = i0 > 0 ? i0 - 1 : i0;                 // one point past each edge so
    const iStop = iEnd < rows.length ? iEnd + 1 : iEnd;  // lines reach the frame
    const drawn = iStop - iStart >= 2;                   // a segment will actually stroke
    if (nVis === 0 && drawn)                             // window inside a gap: autoscale to
      for (let i = iStart; i < iStop; i++)               // the edge points the line spans
        for (let c = 0; c < o.cols.length; c++){
          const v = rows[i][o.cols[c]] * o.scale;
          if (v < lo) lo = v;
          if (v > hi) hi = v;
        }
    let yMin = 0, yMax = 1;
    if (lo <= hi){
      const pad = Math.max((hi - lo) * 0.2, o.minSpan);
      yMin = lo - pad; yMax = hi + pad;
    }
    const X = t => L + (t - t0) / span * pw;
    const Y = v => T + (1 - (v - yMin) / (yMax - yMin)) * ph;

    /* grid + y tick labels */
    ctx.lineWidth = 1;
    ctx.font = '10px Consolas, ui-monospace, monospace';
    ctx.strokeStyle = THEME.grid;
    ctx.fillStyle = THEME.ink3;
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    for (let i = 0; i <= 4; i++){
      const v = yMin + (yMax - yMin) * i / 4, y = Y(v);
      ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(w - R, y); ctx.stroke();
      ctx.fillText(v.toFixed(o.dec), L - 6, y);
    }
    /* x labels: fixed offsets back from the right edge, values relative to now */
    ctx.textAlign = 'center'; ctx.textBaseline = 'alphabetic';
    const step = xStep(span);
    for (let k = 0; k * step <= span + 1e-9; k++){
      const x = X(t1 - k * step);
      if (x < L + 14) break;
      ctx.fillText(fmtOff(t1 - k * step, t1, view.end === null), x, h - 6);
    }
    ctx.strokeStyle = THEME.axis;
    ctx.beginPath(); ctx.moveTo(L, T + ph); ctx.lineTo(w - R, T + ph); ctx.stroke();

    /* series, clipped to the plot area */
    ctx.save();
    ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
    ctx.lineWidth = 2; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    for (let s = 0; s < o.cols.length; s++){
      const col = o.cols[s], raw = this._raw;
      raw.length = 0;
      for (let i = iStart; i < iStop; i++) raw.push(rows[i][0], rows[i][col] * o.scale);
      ctx.strokeStyle = THEME.series[s];
      if (FILT.alpha > 0){
        /* EMA seeded one full span before the window to kill the startup transient */
        const f = this._flt; f.length = 0;
        let ema = NaN;
        for (let i = lowerBound(rows, t0 - span); i < iStop; i++){
          const v = rows[i][col] * o.scale;
          ema = (ema === ema) ? ema + FILT.alpha * (v - ema) : v;
          if (i >= iStart) f.push(rows[i][0], ema);
        }
        ctx.globalAlpha = 0.25;
        strokePts(ctx, raw, X, Y, t0, span, pw);   // raw trace underneath
        ctx.globalAlpha = 1;
        strokePts(ctx, f, X, Y, t0, span, pw);     // filtered on top
      } else {
        strokePts(ctx, raw, X, Y, t0, span, pw);
      }
    }
    /* end dots on the newest visible sample */
    if (nVis > 0){
      const r = rows[iEnd - 1];
      for (let s = 0; s < o.cols.length; s++){
        ctx.fillStyle = THEME.series[s];
        ctx.beginPath();
        ctx.arc(X(r[0]), Y(r[o.cols[s]] * o.scale), 3, 0, 6.2832);
        ctx.fill();
      }
    }
    ctx.restore();

    if (!drawn){
      ctx.fillStyle = THEME.ink3;
      ctx.font = '12px "Segoe UI", system-ui, sans-serif';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(o.emptyMsg || 'no data', L + pw / 2, T + ph / 2);
    } else if (this.hover && !this.dragging){
      this._crosshair(ctx, rows, t0, t1, span, X, Y, L, R, T, pw, ph, w);
    }
  }

  _crosshair(ctx, rows, t0, t1, span, X, Y, L, R, T, pw, ph, w){
    const o = this.opts, hx = this.hover.x, hy = this.hover.y;
    if (hx < L || hx > L + pw || hy < T || hy > T + ph) return;   // off the plot: skip
    const tCur = t0 + (hx - L) / pw * span;
    let idx = lowerBound(rows, tCur);                              // nearest sample
    if (idx >= rows.length) idx = rows.length - 1;
    if (idx > 0 && tCur - rows[idx-1][0] < rows[idx][0] - tCur) idx--;
    const r = rows[idx];
    if (r[0] < t0 - span * 0.05 || r[0] > t1 + span * 0.05) return;

    ctx.strokeStyle = THEME.axis; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(hx, T); ctx.lineTo(hx, T + ph); ctx.stroke();

    ctx.save();
    ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
    for (let s = 0; s < o.cols.length; s++){
      ctx.fillStyle = THEME.series[s];
      ctx.beginPath();
      ctx.arc(X(r[0]), Y(r[o.cols[s]] * o.scale), 3.5, 0, 6.2832);
      ctx.fill();
    }
    ctx.restore();

    const lines = [(r[0] - latestT()).toFixed(2) + 's'];
    for (let s = 0; s < o.cols.length; s++)
      lines.push((o.names[s] ? o.names[s] + ' ' : '') + (r[o.cols[s]] * o.scale).toFixed(o.dec));
    ctx.font = '11px Consolas, ui-monospace, monospace';
    let bw = 0;
    for (const ln of lines) bw = Math.max(bw, ctx.measureText(ln).width);
    bw += 26;
    const lh = 15, bh = lines.length * lh + 10;
    let bx = hx + 10;
    if (bx + bw > w - R) bx = hx - 10 - bw;
    if (bx < L) bx = L;
    const by = Math.min(Math.max(hy - bh / 2, T + 2), Math.max(T + 2, T + ph - bh - 2));
    ctx.globalAlpha = 0.95; ctx.fillStyle = THEME.card;
    ctx.fillRect(bx, by, bw, bh);
    ctx.globalAlpha = 1; ctx.strokeStyle = THEME.line;
    ctx.strokeRect(bx + 0.5, by + 0.5, bw - 1, bh - 1);
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    for (let i = 0; i < lines.length; i++){
      const ly = by + 5 + i * lh + lh / 2;
      if (i > 0){
        ctx.fillStyle = THEME.series[i - 1];
        ctx.fillRect(bx + 8, ly - 1.5, 8, 3);      // series swatch
      }
      ctx.fillStyle = i === 0 ? THEME.ink3 : THEME.ink;
      ctx.fillText(lines[i], bx + (i > 0 ? 20 : 8), ly);
    }
  }
}

const chAcc = new CanvasChart('ch-accel', { buf:'imu', cols:[1,2,3], names:['X','Y','Z'],
  scale:1, minSpan:0.3, dec:2, label:'Accelerometer' });
const chGyr = new CanvasChart('ch-gyro', { buf:'imu', cols:[4,5,6], names:['X','Y','Z'],
  scale:1, minSpan:1, dec:1, label:'Gyroscope' });
const chAlt = new CanvasChart('ch-alt', { buf:'baro', cols:[3], names:['alt'],
  scale:1, minSpan:2, dec:1, label:'Baro altitude',
  emptyMsg:'no barometer yet — this chart lights up when BMP580 answers' });
const chBat = new CanvasChart('ch-batt', { buf:'batt', cols:[1], names:['V'],
  scale:0.001, minSpan:0.4, dec:2, label:'Battery volts' });

/* ---------- toolbar ---------- */
function syncUI(){
  const live = view.end === null;
  document.getElementById('btn-pause').textContent = live ? 'Pause' : 'Resume';
  document.getElementById('btn-live').className = 'btn live' + (live ? ' on' : '');
  document.querySelectorAll('#seg-span button').forEach(b => {
    b.className = Math.abs(parseFloat(b.dataset.span) - view.span) < 0.01 ? 'on' : '';
  });
  document.querySelectorAll('#seg-filt button').forEach(b => {
    b.className = Math.abs(parseFloat(b.dataset.alpha) - FILT.alpha) < 1e-6 ? 'on' : '';
  });
  const sp = view.span >= 10 ? Math.round(view.span) : Math.round(view.span * 10) / 10;
  document.getElementById('chart-h').textContent =
    'Telemetry — last ' + sp + ' s' + (live ? '' : ' (paused)');
}
document.querySelectorAll('#seg-span button').forEach(b =>
  b.addEventListener('click', () => setSpan(parseFloat(b.dataset.span))));
document.querySelectorAll('#seg-filt button').forEach(b =>
  b.addEventListener('click', () => { FILT.alpha = parseFloat(b.dataset.alpha); syncUI(); markDirty(); }));
document.getElementById('btn-pause').addEventListener('click', togglePause);
document.getElementById('btn-live').addEventListener('click', goLive);
window.addEventListener('keydown', e => {
  if (e.code !== 'Space' || e.repeat) return;
  const t = e.target;
  if (t && (t.tagName === 'BUTTON' || t.tagName === 'INPUT' ||
            t.tagName === 'TEXTAREA' || t.tagName === 'SELECT')) return;
  e.preventDefault();
  togglePause();
});

/* ---------- render loop: draw only when a chart is dirty ---------- */
function frame(){
  for (const c of charts) if (c.dirty){ c.dirty = false; c.draw(); }
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

/* ---------- status chips + data fetch (self-pacing; never setInterval) ---------- */
function setText(id,v){document.getElementById(id).textContent=v;}
function setCls(id,cls){document.getElementById(id).className='v '+cls;}

// client-side rolling buffer: each poll only transfers rows we haven't seen.
var buf={imu:[],baro:[],batt:[],pyro:[]}, lastT=-1;
function ingest(d){
  if(d.now<lastT){ buf={imu:[],baro:[],batt:[],pyro:[]}; lastT=-1; view.end=null; syncUI(); } // server restarted
  ['imu','baro','batt','pyro'].forEach(function(k){
    var rows=d[k]||[], b=buf[k];
    for(var i=0;i<rows.length;i++){ b.push(rows[i]); if(rows[i][0]>lastT)lastT=rows[i][0]; }
    var cut=d.now-125;
    while(b.length&&b[0][0]<cut)b.shift();
  });
}
function tick(){
  return fetch('/data?since='+lastT).then(function(r){return r.json();}).then(function(d){
    var prevLast=lastT;
    ingest(d);
    dataNow=d.now;
    if(lastT!==prevLast&&view.end===null)markDirty(); // repaint only on new rows while live
    if(view.end!==null){                              // keep a frozen view inside the buffer
      var ce=clampEnd(view.end);
      if(ce!==view.end){view.end=ce;markDirty();}
    }
    var dot=document.getElementById('dot');
    dot.className='livedot'+(d.connected?' on':'');
    setText('conn',d.connected?'live — COM link up':'board not found — plug in USB');
    var s=d.status||{};
    setText('st',s.state||'—');
    setText('armed',s.armed===1?'ARMED':(s.armed===0?'safe':'—'));
    document.getElementById('armed').className='v small '+(s.armed===1?'bad':'ok');
    setText('batt',s.state&&buf.batt.length?(buf.batt[buf.batt.length-1][1]/1000).toFixed(2)+' V':'—');
    setText('alt',s.alt_m!==undefined?s.alt_m.toFixed(1)+' m':'—');
    setText('vel',s.vel_ms!==undefined?s.vel_ms.toFixed(1)+' m/s':'—');
    setText('gps',s.gps_fix===1?('fix · '+s.gps_sats+' sats'):(s.gps_fix===0?'no fix':'—'));
    var h=[s.imu_ok,s.baro_ok,s.sd_ok,s.radio_ok];
    setText('health',h[0]===undefined?'—':h.map(function(v){return v?'●':'○';}).join(' '));
    var rEl=document.getElementById('radio');
    if(s.radio_txt){
      rEl.textContent=s.radio_txt==='ok'?('ok · '+(s.radio_tcxo==='yes'?'tcxo':'xtal')):'no chip';
      rEl.className='v small '+(s.radio_txt==='ok'?'ok':'bad');
    } else { rEl.textContent='—'; }
  }).catch(function(){
    document.getElementById('dot').className='livedot';
    setText('conn','dashboard server stopped');
  });
}
// self-pacing: next tick is scheduled only after the previous one finishes,
// so a slow frame can never queue up work and freeze the tab.
function loop(){ tick().then(function(){ setTimeout(loop, 20); }); }
syncUI();
loop();
</script>
</body>
</html>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM9", help="FC console COM port")
    ap.add_argument("--http", type=int, default=8321, help="HTTP port")
    args = ap.parse_args()

    poller = Poller(args.port)
    poller.start()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"   # keep-alive: no TCP handshake per poll

        def do_GET(self):
            if self.path.startswith("/data"):
                m = re.search(r"since=(-?[\d.]+)", self.path)
                since = float(m.group(1)) if m else -1.0
                body = poller.snapshot(since).encode()
                ctype = "application/json"
            elif self.path == "/" or self.path.startswith("/index"):
                body = PAGE.encode()
                ctype = "text/html; charset=utf-8"
            else:
                self.send_response(404)
                self.send_header("Content-Length", "0")  # keep-alive needs a delimited body
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):
            pass

    srv = ThreadingHTTPServer(("127.0.0.1", args.http), Handler)
    print(f"FC live dashboard: http://localhost:{args.http}  (console on {args.port})")
    print("Ctrl+C to stop.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
