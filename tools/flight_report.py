#!/usr/bin/env python3
"""flight_report.py - one-page, self-contained HTML flight report.

Input (auto-detected):
    LOGnnn.BIN                 -> decoded via decode_log.iter_records()
    recordings/deck_*/ dir     -> telem.csv (+ events.csv if present)
    a telem*.csv file          -> same recording codec

Output: a single offline .html with inline CSS, an inline SVG altitude-vs-
time chart, an inline SVG GPS ground-track, and a summary table. No external
resources and no JavaScript, so it works offline and is publishable as an
Artifact.

    python flight_report.py LOG001.BIN
    python flight_report.py recordings/deck_20260712_130000/ --out report.html

The peak/phase math is reused from decode_log.summarize(); this tool only
normalizes each source into that dict shape and owns the rendering.
"""

import argparse
import csv
import html
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # sibling tools/
import decode_log

STATE_NAMES = decode_log.STATE_NAMES
ACCEL_SAT_G = decode_log.ACCEL_SAT_G

# SVG canvas / plot geometry (px).
ALT_W, ALT_H = 760, 300
TRK_W, TRK_H = 320, 320
# Downsample cap: a 200 Hz log is thousands of points; a polyline that long
# bloats the file for no visible gain. ponytail: naive stride decimation,
# swap for min/max-per-bucket if peaks ever get clipped.
MAX_PTS = 2000


# ------------------------------------------------------------------ loaders

def _norm_from_log(r):
    """One decode_log record dict -> the common report record shape."""
    has_fix = r["lat_e7"] != 0 or r["lon_e7"] != 0
    return {
        "t_ms": r["t_ms"], "state": r["state"], "state_name": r["state_name"],
        "agl_m": r["agl_m"], "vel_ms": r["vel_ms"],
        "ax_g": r["ax_g"], "ay_g": r["ay_g"], "az_g": r["az_g"],
        "pyro_fired": r["pyro_fired"], "batt_mv": r["batt_mv"],
        "lat": r["lat_e7"] / 1e7 if has_fix else None,
        "lon": r["lon_e7"] / 1e7 if has_fix else None,
    }


def _decode_cell(v):
    """Recording CSV cell codec (mirrors deck.sources.ReplaySource._load_csv):
    JSON literals decode to their value; a bare string stays a string; empty
    becomes None."""
    if v is None or v == "":
        return None
    try:
        return json.loads(v)
    except ValueError:
        return v


def _norm_from_csv(d):
    """One decoded telem.csv row -> the common report record shape, or None if
    it lacks the numeric fields the summary needs."""
    t_ms, agl, vel = d.get("t_ms"), d.get("alt_baro_m"), d.get("vel_up_ms")
    if t_ms is None or agl is None or vel is None:
        return None
    ax, ay, az = (d.get("accel_g") or [0.0, 0.0, 0.0])[:3]
    lat, lon = d.get("lat_deg"), d.get("lon_deg")
    if not d.get("gps_fix") or (not lat and not lon):
        lat = lon = None
    st = d.get("state")
    return {
        "t_ms": t_ms, "state": st,
        "state_name": d.get("state_name") or decode_log.state_name(st or 0),
        "agl_m": agl, "vel_ms": vel,
        "ax_g": ax or 0.0, "ay_g": ay or 0.0, "az_g": az or 0.0,
        "pyro_fired": d.get("pyro_fired") or 0, "batt_mv": d.get("batt_mv"),
        "lat": lat, "lon": lon,
    }


def _load_events(path):
    """events.csv -> list of {seq,t_host,t_ms,kind,severity,text} dicts."""
    with open(path, newline="", encoding="utf-8") as f:
        return [{k: _decode_cell(v) for k, v in row.items()}
                for row in csv.DictReader(f)]


def load(path):
    """Auto-detect the source and return (records, events, title).

    ponytail: reads only telem.csv, not rotated telem_part2.csv+; add
    globbing when a session actually spans >50 MB."""
    if os.path.isdir(path):
        csv_path = os.path.join(path, "telem.csv")
        ev_path = os.path.join(path, "events.csv")
        title = os.path.basename(os.path.normpath(path))
    elif path.lower().endswith(".csv"):
        csv_path = path
        ev_path = os.path.join(os.path.dirname(path) or ".", "events.csv")
        title = os.path.basename(path)
    else:  # LOGnnn.BIN
        recs = [_norm_from_log(r) for r in decode_log.iter_records(path)]
        return recs, [], os.path.basename(path)

    with open(csv_path, newline="", encoding="utf-8") as f:
        recs = [n for n in (_norm_from_csv({k: _decode_cell(v)
                                            for k, v in row.items()})
                            for row in csv.DictReader(f)) if n is not None]
    events = _load_events(ev_path) if os.path.exists(ev_path) else []
    return recs, events, title


# ---------------------------------------------------------------- summarize

def build_summary(records):
    """decode_log.summarize() for the peaks/phases + a few report-only extras
    (apogee time, flight duration, battery range, GPS track)."""
    s = dict(decode_log.summarize(records))
    t0 = records[0]["t_ms"]
    s["t0_ms"] = t0
    s["duration_ms"] = records[-1]["t_ms"] - t0

    apogee_i = max(range(len(records)), key=lambda i: records[i]["agl_m"])
    s["apogee_t_ms"] = records[apogee_i]["t_ms"] - t0

    batts = [r["batt_mv"] for r in records if r["batt_mv"]]
    s["batt_min_mv"] = min(batts) if batts else None
    s["batt_max_mv"] = max(batts) if batts else None

    # GPS ground-track: (lat, lon) per fix, plus pad / apogee / last markers.
    track = [(r["lat"], r["lon"]) for r in records
             if r["lat"] is not None and r["lon"] is not None]
    s["track"] = track
    s["pad_ll"] = track[0] if track else None
    s["last_ll"] = track[-1] if track else None
    ap = records[apogee_i]
    s["apogee_ll"] = ((ap["lat"], ap["lon"])
                      if ap["lat"] is not None else None)
    return s


# -------------------------------------------------------------- svg helpers

def _decimate(seq):
    """Stride down to <= MAX_PTS points, always keeping the last one."""
    if len(seq) <= MAX_PTS:
        return seq
    step = len(seq) // MAX_PTS + 1
    out = seq[::step]
    if out[-1] is not seq[-1]:
        out.append(seq[-1])
    return out


def _nice_max(v):
    """Round an axis maximum up to a clean 1/2/5 x 10^n."""
    if v <= 0:
        return 10.0
    mag = 10 ** math.floor(math.log10(v))
    for m in (1, 2, 2.5, 5, 10):
        if v <= m * mag:
            return m * mag
    return 10 * mag


def _alt_chart_svg(records, s):
    """Inline SVG: altitude (m AGL) vs time (s), with apogee reference line."""
    t0 = s["t0_ms"]
    pts = _decimate([((r["t_ms"] - t0) / 1000.0, r["agl_m"]) for r in records])
    xmax = max(pts[-1][0], 1.0)
    ymax = _nice_max(max(s["apogee_m"], 1.0))
    ml, mr, mt, mb = 58, 18, 16, 36
    pw, ph = ALT_W - ml - mr, ALT_H - mt - mb

    def X(t):
        return ml + pw * (t / xmax)

    def Y(a):
        return mt + ph * (1 - max(0.0, min(a, ymax)) / ymax)

    parts = ['<svg viewBox="0 0 %d %d" class="chart" role="img" '
             'aria-label="altitude vs time">' % (ALT_W, ALT_H)]
    # horizontal gridlines + y labels
    for f in (0, .25, .5, .75, 1):
        y = mt + ph * (1 - f)
        parts.append('<line x1="%g" y1="%g" x2="%g" y2="%g" class="grid"/>'
                     % (ml, y, ml + pw, y))
        parts.append('<text x="%g" y="%g" class="ylab">%g</text>'
                     % (ml - 6, y + 4, round(ymax * f)))
    # vertical time ticks
    for k in range(6):
        t = xmax * k / 5.0
        x = X(t)
        parts.append('<line x1="%g" y1="%g" x2="%g" y2="%g" class="grid"/>'
                     % (x, mt, x, mt + ph))
        parts.append('<text x="%g" y="%g" class="xlab">%gs</text>'
                     % (x, mt + ph + 18, round(t)))
    # apogee reference line + marker
    ya = Y(s["apogee_m"])
    parts.append('<line x1="%g" y1="%g" x2="%g" y2="%g" class="apogee"/>'
                 % (ml, ya, ml + pw, ya))
    parts.append('<text x="%g" y="%g" class="aplab">apogee %.1f m</text>'
                 % (ml + 4, ya - 5, s["apogee_m"]))
    # the altitude trace
    poly = " ".join("%.1f,%.1f" % (X(t), Y(a)) for t, a in pts)
    parts.append('<polyline points="%s" class="trace"/>' % poly)
    parts.append('<circle cx="%.1f" cy="%.1f" r="4" class="apdot"/>'
                 % (X(s["apogee_t_ms"] / 1000.0), ya))
    parts.append('</svg>')
    return "".join(parts)


def _track_svg(s):
    """Inline SVG: equal-aspect GPS ground-track (east/north metres around the
    first fix), with pad / apogee / last markers. None if there's no track."""
    track = _decimate(s["track"])
    if len(track) < 2:
        return None
    lat0, lon0 = s["pad_ll"]
    k = math.cos(math.radians(lat0))

    def en(ll):  # (lat,lon) -> (east_m, north_m) around the pad
        return ((ll[1] - lon0) * 111320.0 * k, (ll[0] - lat0) * 110540.0)

    es = [en(ll) for ll in track]
    xs, ys = [e for e, _ in es], [n for _, n in es]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    margin = 30
    bw, bh = TRK_W - 2 * margin, TRK_H - 2 * margin
    # single uniform scale => equal aspect (a metre east == a metre north)
    scale = min(bw / max(maxx - minx, 1e-6), bh / max(maxy - miny, 1e-6))
    cx, cy = (minx + maxx) / 2.0, (miny + maxy) / 2.0

    def P(e, n):
        return (TRK_W / 2.0 + (e - cx) * scale,
                TRK_H / 2.0 - (n - cy) * scale)  # flip: north is up

    parts = ['<svg viewBox="0 0 %d %d" class="track" role="img" '
             'aria-label="GPS ground track">' % (TRK_W, TRK_H)]
    poly = " ".join("%.1f,%.1f" % P(e, n) for e, n in es)
    parts.append('<polyline points="%s" class="tline"/>' % poly)
    px, py = P(*en(s["pad_ll"]))
    parts.append('<circle cx="%.1f" cy="%.1f" r="5" class="pad"/>' % (px, py))
    lx, ly = P(*en(s["last_ll"]))
    parts.append('<circle cx="%.1f" cy="%.1f" r="4" class="last"/>' % (lx, ly))
    if s["apogee_ll"]:
        ax, ay = P(*en(s["apogee_ll"]))
        parts.append('<circle cx="%.1f" cy="%.1f" r="4" class="apo"/>'
                     % (ax, ay))
    parts.append('</svg>')
    return "".join(parts)


# ------------------------------------------------------------------- render

def _ms(v):
    return "&ndash;" if v is None else "%.2f s" % (v / 1000.0)


def _tile(label, value, sub=""):
    sub = '<span class="sub">%s</span>' % sub if sub else ""
    return ('<div class="tile"><div class="tval">%s</div>'
            '<div class="tlab">%s</div>%s</div>' % (value, label, sub))


def build_html(records, events, title, s):
    esc = html.escape

    sat = (' <span class="warn">SAT &ge;%.0f g</span>' % ACCEL_SAT_G
           if s["accel_saturated"] else "")
    fire = ("none" if s["fire_t_ms"] is None
            else "%.2f s @ %.1f m" % ((s["fire_t_ms"] - s["t0_ms"]) / 1000.0,
                                      s["fire_agl_m"]))
    batt = ("&ndash;" if s["batt_min_mv"] is None
            else "%.2f&ndash;%.2f V" % (s["batt_min_mv"] / 1000.0,
                                        s["batt_max_mv"] / 1000.0))

    tiles = "".join([
        _tile("Apogee", "%.1f m" % s["apogee_m"],
              "T+%.1f s" % (s["apogee_t_ms"] / 1000.0)),
        _tile("Max vertical vel", "%.1f m/s" % s["max_vel_ms"]),
        _tile("Max |accel|", "%.1f g%s" % (s["max_accel_g"], sat)),
        _tile("Flight time", "%.1f s" % (s["duration_ms"] / 1000.0)),
        _tile("Pyro fire", fire),
        _tile("Battery", batt),
    ])

    rows = "".join("<tr><th>%s</th><td>%s</td></tr>" % r for r in [
        ("Records", str(s["n_records"])),
        ("Burn (BOOST)", _ms(s["burn_ms"])),
        ("Coast", _ms(s["coast_ms"])),
        ("Descent", _ms(s["descent_ms"])),
        ("Apogee time", "T+%.2f s" % (s["apogee_t_ms"] / 1000.0)),
        ("GPS fixes", str(len(s["track"]))),
    ])

    alt_svg = _alt_chart_svg(records, s)
    trk_svg = _track_svg(s)
    trk_block = (('<div class="card"><h2>GPS ground-track</h2>%s'
                  '<div class="legend"><span class="k pad"></span>pad'
                  '<span class="k apo"></span>apogee'
                  '<span class="k last"></span>last</div></div>') % trk_svg
                 if trk_svg else
                 '<div class="card"><h2>GPS ground-track</h2>'
                 '<p class="note">No GPS fixes recorded.</p></div>')

    warn_block = ""
    if s["warnings"]:
        warn_block = ('<div class="card warns"><h2>Warnings</h2><ul>%s</ul>'
                      '</div>' % "".join("<li>%s</li>" % esc(w)
                                         for w in s["warnings"]))

    ev_block = ""
    if events:
        ev_rows = "".join(
            '<tr class="sev-%s"><td>%s</td><td>%s</td><td>%s</td></tr>'
            % (esc(str(e.get("severity") or "info")),
               ("" if e.get("t_ms") is None else "%.1fs" % (e["t_ms"] / 1000.0)),
               esc(str(e.get("kind") or "")), esc(str(e.get("text") or "")))
            for e in events)
        ev_block = ('<div class="card"><h2>Events</h2><table class="evt">'
                    '<tr><th>t_ms</th><th>kind</th><th>detail</th></tr>%s'
                    '</table></div>' % ev_rows)

    return _PAGE % {
        "title": esc(title), "css": _CSS, "tiles": tiles, "table": rows,
        "alt_svg": alt_svg, "trk_block": trk_block,
        "warn_block": warn_block, "ev_block": ev_block,
    }


# Inline stylesheet: dark-on-light card layout. No external fonts/URLs so the
# page is fully self-contained (and passes the "no http://" offline check).
_CSS = """
*{box-sizing:border-box}
body{margin:0;padding:24px;background:#eef1f5;color:#1b2733;
  font:15px/1.5 -apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif}
.wrap{max-width:840px;margin:0 auto}
h1{font-size:22px;margin:0 0 2px}
h2{font-size:14px;text-transform:uppercase;letter-spacing:.05em;
  color:#5a6b7b;margin:0 0 12px}
.meta{color:#5a6b7b;margin:0 0 20px}
.card{background:#fff;border:1px solid #dbe1e8;border-radius:10px;
  padding:18px;margin:0 0 18px;box-shadow:0 1px 2px rgba(0,0,0,.04);
  overflow-x:auto}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));
  gap:12px;margin:0 0 18px}
.tile{background:#fff;border:1px solid #dbe1e8;border-radius:10px;padding:14px}
.tval{font-size:20px;font-weight:600}
.tlab{color:#5a6b7b;font-size:12px;text-transform:uppercase;
  letter-spacing:.04em;margin-top:2px}
.sub{color:#8896a4;font-size:12px}
.warn{color:#b23b1e;font-weight:600;font-size:12px}
table{border-collapse:collapse;width:100%;font-size:14px}
th,td{text-align:left;padding:5px 10px;border-bottom:1px solid #eef1f5}
td{font-variant-numeric:tabular-nums}
.evt td:first-child{color:#8896a4;white-space:nowrap}
.warns li{color:#b23b1e}
.note{color:#8896a4}
svg.chart,svg.track{width:100%;height:auto;display:block}
.grid{stroke:#e6eaef;stroke-width:1}
.xlab,.ylab{fill:#8896a4;font-size:11px}
.xlab{text-anchor:middle}.ylab{text-anchor:end}
.trace{fill:none;stroke:#2f6fed;stroke-width:2}
.apogee{stroke:#e08a1e;stroke-width:1;stroke-dasharray:4 4}
.aplab{fill:#c8760f;font-size:11px}
.apdot{fill:#e08a1e}
.tline{fill:none;stroke:#2f6fed;stroke-width:1.5}
.pad{fill:#2e9e5b}.apo{fill:#e08a1e}.last{fill:#2f6fed}
.legend{margin-top:8px;color:#5a6b7b;font-size:12px}
.k{display:inline-block;width:10px;height:10px;border-radius:50%;
  margin:0 5px 0 14px;vertical-align:baseline}
.k.pad{background:#2e9e5b}.k.apo{background:#e08a1e}.k.last{background:#2f6fed}
"""

_PAGE = """<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Flight report - %(title)s</title>
<style>%(css)s</style></head>
<body><div class="wrap">
<h1>Flight report</h1>
<p class="meta">%(title)s</p>
<div class="tiles">%(tiles)s</div>
%(warn_block)s
<div class="card"><h2>Altitude vs time</h2>%(alt_svg)s</div>
%(trk_block)s
<div class="card"><h2>Phases &amp; details</h2><table>%(table)s</table></div>
%(ev_block)s
</div></body></html>
"""


# --------------------------------------------------------------------- main

def _default_out(path):
    if os.path.isdir(path):
        return os.path.join(path, "flight_report.html")
    return os.path.splitext(path)[0] + ".html"


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Render a self-contained one-page HTML flight report from "
                    "a LOGnnn.BIN or a recordings/deck_*/ directory.")
    ap.add_argument("input", help="LOGnnn.BIN, a deck_* recording dir, or a "
                                  "telem.csv file")
    ap.add_argument("--out", help="output .html path (default: alongside the "
                                  "input)")
    args = ap.parse_args(argv)

    try:
        records, events, title = load(args.input)
    except OSError as e:
        print("error: cannot read %s: %s" % (args.input, e), file=sys.stderr)
        return 2
    if not records:
        print("error: no usable flight records in %s" % args.input,
              file=sys.stderr)
        return 2

    s = build_summary(records)
    page = build_html(records, events, title, s)
    out = args.out or _default_out(args.input)
    with open(out, "w", encoding="utf-8") as f:
        f.write(page)
    print("%s: %d records -> %s (apogee %.1f m)"
          % (args.input, s["n_records"], out, s["apogee_m"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
