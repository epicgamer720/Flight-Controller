/* chart.js: Flight Deck canvas chart engine.
 *
 * PROVENANCE: the CanvasChart class and its helpers (view state, latestT /
 * viewT1 / clampEnd / goLive / togglePause / setSpan, lowerBound, strokePts
 * min/max decimation, crosshair tooltip, EMA display filter, loadTheme) are
 * COPIED from the PAGE string in tools/live_dashboard.py (read-only donor,
 * the bring-up tool stays byte-identical) and DIVERGE here for flight ops:
 *   - a named buffer registry (deck.js fills buffers[..]; the old single
 *     `buf` global is gone)
 *   - null-tolerant series (rows may carry null cells, e.g. GPS alt on the
 *     FC console path)
 *   - event markers: labeled ink-toned vertical ticks on the shared time
 *     axis, labels collision-stepped vertically
 *   - flight-window auto-zoom (launch_t_host − 5 s → now)
 *   - StateTimeline: a labeled per-state-visit band on the same time axis
 *     (state chip palette: status colors encode STATE here, never series)
 */
'use strict';

/* ---------- theme: canvas colors come from the CSS custom properties ---------- */
let THEME = {};
function loadTheme(){
  const cs = getComputedStyle(document.documentElement);
  const g = n => cs.getPropertyValue(n).trim();
  THEME = {
    series: [g('--s1'), g('--s2'), g('--s3')],
    grid: g('--grid'), axis: g('--axis'),
    ink: g('--ink'), ink2: g('--ink-2'), ink3: g('--ink-3'),
    card: g('--card'), line: g('--line'),
    tone: {   /* state chip palette (timeline band only, never series) */
      idle:   { fill: g('--idle-wash'), edge: g('--idle-text') },
      good:   { fill: g('--good-wash'), edge: g('--good-text') },
      warn:   { fill: g('--warn-wash'), edge: g('--warn-text') },
      crit:   { fill: g('--crit-wash'), edge: g('--crit-text') },
      flight: { fill: g('--accent-wash'), edge: g('--accent') }
    }
  };
}
loadTheme();
window.matchMedia('(prefers-color-scheme: dark)')
      .addEventListener('change', () => { loadTheme(); markDirty(); });

/* ---------- named buffer registry (deck.js fills these) ---------- */
const buffers = {};
function registerBuffer(name){
  if (!buffers[name]) buffers[name] = [];
  return buffers[name];
}
function resetBuffers(){
  for (const k in buffers) buffers[k].length = 0;
  lastT = -1;
}

/* ---------- flight-ops shared data (deck.js maintains, engine renders) ---------- */
const eventMarks = [];   // {t, label}: labeled ticks on every chart
const stateSegs = [];    // {t0, t1|null, st, name, tone}: timeline band
let launchT = null;      // tplus.launch_t_host, or null before launch

/* ---------- shared time-axis view state (all charts + timeline) ---------- */
const view = { span: 30, end: null };   // end === null -> follow live
const FILT = { alpha: 0, ghost: true }; // EMA display filter; 0 = off.
                                        // ghost: raw data drawn faintly
                                        // under the filtered trace
let refApogee = null;    // peaks.apogee_m from /data: apogee ref line
let dataNow = 0;                        // newest time reported by the server
let lastT = -1;                         // newest sample time ingested
const charts = [];

function latestT(){ return lastT >= 0 ? lastT : dataNow; }
function viewT1(){ return view.end === null ? latestT() : view.end; }
function bufferStart(){
  let s = Infinity;
  for (const k in buffers){
    const b = buffers[k];
    if (b.length && b[0][0] < s) s = b[0][0];
  }
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
/* flight-window auto-zoom: span (launch − 5 s) → now, right edge live */
function flightWindow(){
  if (launchT === null) return;
  const end = latestT();
  view.span = Math.min(125, Math.max(10, end - (launchT - 5)));
  view.end = null;                       // follow live (frozen data = frozen edge)
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
  const v = t - t1;                // (stable while paused, never tied to the wall clock)
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
    const rows = buffers[o.buf] || [];

    /* visible index range (binary search) + autoscale over visible points */
    const i0 = lowerBound(rows, t0);
    let iEnd = i0, lo = Infinity, hi = -Infinity;
    for (let i = i0; i < rows.length; i++){
      if (rows[i][0] > t1) break;
      iEnd = i + 1;
      for (let c = 0; c < o.cols.length; c++){
        const raw = rows[i][o.cols[c]];
        if (raw === null || raw === undefined) continue;
        const v = raw * o.scale;
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
          const raw = rows[i][o.cols[c]];
          if (raw === null || raw === undefined) continue;
          const v = raw * o.scale;
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

    /* apogee reference line (Altitude chart only): dashed, ink-toned;
     * a reference, not a status; skipped below 5 m (bench noise) */
    if (o.apogeeRef && refApogee !== null && refApogee >= 5){
      const y = Y(refApogee);
      if (y > T && y < T + ph){
        ctx.strokeStyle = THEME.ink3; ctx.lineWidth = 1;
        ctx.setLineDash([5, 4]);
        ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(w - R, y); ctx.stroke();
        ctx.setLineDash([]);
        ctx.font = '10px Consolas, ui-monospace, monospace';
        ctx.textAlign = 'right'; ctx.textBaseline = 'bottom';
        ctx.fillStyle = THEME.ink3;
        ctx.fillText('apogee ' + refApogee.toFixed(1) + ' m', w - R - 2, y - 2);
      }
    }

    /* series, clipped to the plot area (null cells are skipped) */
    let anyStroke = false;
    const endFilt = this._endFilt || (this._endFilt = []);
    endFilt.length = 0;
    ctx.save();
    ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
    ctx.lineWidth = 2; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
    for (let s = 0; s < o.cols.length; s++){
      const col = o.cols[s], raw = this._raw;
      raw.length = 0;
      for (let i = iStart; i < iStop; i++){
        const v = rows[i][col];
        if (v === null || v === undefined) continue;
        raw.push(rows[i][0], v * o.scale);
      }
      ctx.strokeStyle = THEME.series[s];
      if (FILT.alpha > 0){
        /* EMA seeded one full span before the window to kill the startup transient */
        const f = this._flt; f.length = 0;
        let ema = NaN;
        for (let i = lowerBound(rows, t0 - span); i < iStop; i++){
          const rv = rows[i][col];
          if (rv === null || rv === undefined) continue;
          const v = rv * o.scale;
          ema = (ema === ema) ? ema + FILT.alpha * (v - ema) : v;
          if (i >= iStart) f.push(rows[i][0], ema);
          if (i < iEnd) endFilt[s] = ema;   // filtered value at the end dot
        }
        if (FILT.ghost){
          ctx.globalAlpha = 0.25;
          anyStroke = strokePts(ctx, raw, X, Y, t0, span, pw) || anyStroke;
          ctx.globalAlpha = 1;
        }
        anyStroke = strokePts(ctx, f, X, Y, t0, span, pw) || anyStroke;
      } else {
        anyStroke = strokePts(ctx, raw, X, Y, t0, span, pw) || anyStroke;
      }
    }
    /* end dots on the newest visible non-null sample per series: on the
     * FILTERED line when the display filter is active */
    if (nVis > 0){
      for (let s = 0; s < o.cols.length; s++){
        for (let i = iEnd - 1; i >= i0; i--){
          const v = rows[i][o.cols[s]];
          if (v === null || v === undefined) continue;
          const y = (FILT.alpha > 0 && endFilt[s] !== undefined)
            ? endFilt[s] : v * o.scale;
          ctx.fillStyle = THEME.series[s];
          ctx.beginPath();
          ctx.arc(X(rows[i][0]), Y(y), 3, 0, 6.2832);
          ctx.fill();
          break;
        }
      }
    }
    ctx.restore();

    this._markers(ctx, t0, t1, X, L, T, pw, ph);

    if (!anyStroke){
      ctx.fillStyle = THEME.ink3;
      ctx.font = '12px "Segoe UI", system-ui, sans-serif';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(o.emptyMsg || 'no data', L + pw / 2, T + ph / 2);
    } else if (this.hover && !this.dragging){
      this._crosshair(ctx, rows, t0, t1, span, X, Y, L, R, T, pw, ph, w);
    }
  }

  /* event markers: ink-toned dashed vertical ticks + collision-stepped labels */
  _markers(ctx, t0, t1, X, L, T, pw, ph){
    if (!eventMarks.length) return;
    ctx.save();
    ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
    ctx.font = '9px Consolas, ui-monospace, monospace';
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    ctx.lineWidth = 1;
    const ends = [-1e9, -1e9, -1e9];       // last label end x, per level
    for (const m of eventMarks){
      if (m.t < t0 || m.t > t1) continue;
      const x = X(m.t);
      ctx.strokeStyle = THEME.ink3;
      ctx.globalAlpha = 0.5;
      ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, T + ph); ctx.stroke();
      ctx.setLineDash([]);
      let lvl = -1;                        // step labels down when they'd collide
      for (let i = 0; i < 3; i++) if (x >= ends[i] + 8){ lvl = i; break; }
      if (lvl < 0){
        lvl = 0;
        for (let i = 1; i < 3; i++) if (ends[i] < ends[lvl]) lvl = i;
      }
      ctx.globalAlpha = 0.9;
      ctx.fillStyle = THEME.ink3;
      ctx.fillText(m.label, x + 3, T + 8 + lvl * 10);
      ends[lvl] = x + 3 + ctx.measureText(m.label).width;
    }
    ctx.globalAlpha = 1;
    ctx.restore();
  }

  _crosshair(ctx, rows, t0, t1, span, X, Y, L, R, T, pw, ph, w){
    const o = this.opts, hx = this.hover.x, hy = this.hover.y;
    if (hx < L || hx > L + pw || hy < T || hy > T + ph) return;   // off the plot: skip
    const tCur = t0 + (hx - L) / pw * span;
    let idx = lowerBound(rows, tCur);                              // nearest sample
    if (idx >= rows.length) idx = rows.length - 1;
    if (idx < 0) return;
    if (idx > 0 && tCur - rows[idx-1][0] < rows[idx][0] - tCur) idx--;
    const r = rows[idx];
    if (r[0] < t0 - span * 0.05 || r[0] > t1 + span * 0.05) return;

    // When the display filter is on, the crosshair tracks the FILTERED
    // trace (the line the operator is reading), not the raw ghost.
    // EMA recomputed per series with the same seeding as draw().
    const filt = FILT.alpha > 0 ? [] : null;
    if (filt){
      for (let s = 0; s < o.cols.length; s++){
        const col = o.cols[s];
        let ema = NaN;
        for (let i = lowerBound(rows, tCur - span); i <= idx; i++){
          const rv = rows[i][col];
          if (rv === null || rv === undefined) continue;
          const v = rv * o.scale;
          ema = (ema === ema) ? ema + FILT.alpha * (v - ema) : v;
        }
        filt.push(ema === ema ? ema : null);
      }
    }

    ctx.strokeStyle = THEME.axis; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(hx, T); ctx.lineTo(hx, T + ph); ctx.stroke();

    ctx.save();
    ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
    for (let s = 0; s < o.cols.length; s++){
      const v = r[o.cols[s]];
      if (v === null || v === undefined) continue;
      const y = filt && filt[s] !== null ? filt[s] : v * o.scale;
      ctx.fillStyle = THEME.series[s];
      ctx.beginPath();
      ctx.arc(X(r[0]), Y(y), 3.5, 0, 6.2832);
      ctx.fill();
    }
    ctx.restore();

    const tag = filt ? (FILT.alpha >= 0.2 ? ' · EMA light' : ' · EMA strong')
                     : '';
    const lines = [(r[0] - latestT()).toFixed(2) + 's' + tag];
    for (let s = 0; s < o.cols.length; s++){
      const v = r[o.cols[s]];
      if (v === null || v === undefined){
        lines.push((o.names[s] ? o.names[s] + ' ' : '') + '–');
      } else if (filt && filt[s] !== null){
        lines.push((o.names[s] ? o.names[s] + ' ' : '') +
                   filt[s].toFixed(o.dec) +
                   ' (raw ' + (v * o.scale).toFixed(o.dec) + ')');
      } else {
        lines.push((o.names[s] ? o.names[s] + ' ' : '') +
                   (v * o.scale).toFixed(o.dec));
      }
    }
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

/* ---------- state timeline band: one labeled segment per state visit ----------
 * Shares the chart time axis (same l/r margins). Status colors ARE allowed
 * here, since the band encodes flight STATE, not a data series. */
class StateTimeline {
  constructor(boxId){
    this.box = document.getElementById(boxId);
    this.canvas = document.createElement('canvas');
    this.canvas.setAttribute('role', 'img');
    this.canvas.setAttribute('aria-label', 'Flight state timeline');
    this.box.appendChild(this.canvas);
    this.ctx = this.canvas.getContext('2d');
    this.margin = { l: 52, r: 14 };
    this.w = 0; this.h = 0; this.dpr = 1;
    this.dirty = true;
    new ResizeObserver(() => this._resize()).observe(this.box);
    this.canvas.addEventListener('dblclick', () => goLive());
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

  draw(){
    const ctx = this.ctx, w = this.w, h = this.h;
    if (!w || !h) return;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const L = this.margin.l, R = this.margin.r;
    const pw = w - L - R;
    if (pw < 20) return;
    const t1 = viewT1(), span = view.span, t0 = t1 - span;
    const X = t => L + (t - t0) / span * pw;

    ctx.strokeStyle = THEME.axis; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(L, h - 1.5); ctx.lineTo(w - R, h - 1.5); ctx.stroke();

    ctx.font = '10px Consolas, ui-monospace, monospace';
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    ctx.fillStyle = THEME.ink3;
    ctx.fillText('STATE', 4, h / 2);

    if (!stateSegs.length){
      ctx.textAlign = 'center';
      ctx.fillText('no state yet', L + pw / 2, h / 2);
      return;
    }
    ctx.save();
    ctx.beginPath(); ctx.rect(L, 0, pw, h); ctx.clip();
    for (const seg of stateSegs){
      const sEnd = seg.t1 === null ? t1 : seg.t1;
      if (sEnd <= t0 || seg.t0 >= t1) continue;
      const x0 = X(Math.max(seg.t0, t0));
      const x1 = X(Math.min(sEnd, t1));
      const bw = Math.max(1, x1 - x0 - 2);           // 2px surface gap between visits
      const tone = THEME.tone[seg.tone] || THEME.tone.idle;
      ctx.fillStyle = tone.fill;
      ctx.fillRect(x0 + 1, 3, bw, h - 7);
      ctx.fillStyle = tone.edge;
      ctx.fillRect(x0 + 1, 3, bw, 2);                // 2px state-toned top edge
      const tw = ctx.measureText(seg.name).width;
      if (bw > tw + 10){                             // label only when it fits
        ctx.fillStyle = THEME.ink2;
        ctx.textAlign = 'center';
        ctx.fillText(seg.name, x0 + 1 + bw / 2, (h - 2) / 2 + 1);
        ctx.textAlign = 'left';
      }
    }
    ctx.restore();
  }
}

/* ---------- GPS ground-track: equal-aspect east/north plan view ----------
 * Standalone (NOT on the shared time axis). deck.js projects GPS to metres,
 * pad-relative, and calls draw(points, apogeeIdx) on update. Colors come from
 * the same THEME custom properties as the other charts (theme-aware). */
class GroundTrack {
  constructor(canvas){
    this.canvas = canvas;
    this.canvas.setAttribute('role', 'img');
    this.canvas.setAttribute('aria-label', 'GPS ground track');
    this.ctx = this.canvas.getContext('2d');
    this.pad = 18;
  }

  /* nice 1/2/5·10^k >= a rough metre value (grid step) */
  _niceStep(x){
    if (!(x > 0)) return 1;
    const p = Math.pow(10, Math.floor(Math.log10(x))), f = x / p;
    return (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10) * p;
  }
  _fmtM(m){
    if (m >= 1000) return (m / 1000).toFixed(m >= 10000 ? 0 : 1) + ' km';
    if (m >= 1)    return Math.round(m) + ' m';
    return m.toFixed(1) + ' m';
  }

  /* bg (optional) = { img: <loaded HTMLImageElement>, halfM: <ground m> }:
   * a north-up satellite tile covering ±halfM ground metres around the pad
   * (points[0] == ground 0,0 == image centre). When present and loaded, the
   * view LOCKS to ±halfM (no auto-fit) and the tile is drawn under the track.
   * null / not-yet-loaded => unchanged auto-fit grid behaviour. */
  draw(points, apogeeIdx, bg){
    const ctx = this.ctx;
    const rect = this.canvas.getBoundingClientRect();
    const w = rect.width, h = rect.height;
    if (!w || !h) return;
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.max(1, Math.round(w * dpr));
    this.canvas.height = Math.max(1, Math.round(h * dpr));
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const P = this.pad;
    const left = P, right = w - P, top = P, bottom = h - P;
    const plotW = right - left, plotH = bottom - top;
    if (plotW < 20 || plotH < 20) return;
    const cx = (left + right) / 2, cy = (top + bottom) / 2;

    const pts = points || [];
    const n = pts.length;

    /* bounds over all points */
    let minE = Infinity, maxE = -Infinity, minN = Infinity, maxN = -Infinity;
    for (const p of pts){
      if (!p) continue;
      if (p[0] < minE) minE = p[0]; if (p[0] > maxE) maxE = p[0];
      if (p[1] < minN) minN = p[1]; if (p[1] > maxN) maxN = p[1];
    }

    /* EQUAL ASPECT: one scale from the larger span so 1 m E == 1 m N on
     * screen. MIN_SPAN floors it so 0/1/identical points never divide by 0. */
    const bgOn = !!(bg && bg.img && bg.img.complete && bg.img.naturalWidth > 0);
    const MIN_SPAN = 2;
    let midE, midN, scale;
    if (bgOn){
      /* LOCK to ±halfM ground metres centred on the pad (image centre) */
      midE = 0; midN = 0;
      scale = Math.min(plotW, plotH) / (2 * bg.halfM);
    } else if (n === 0 || minE === Infinity){
      midE = 0; midN = 0;
      scale = Math.min(plotW, plotH) / (MIN_SPAN * 8);   // default empty zoom
    } else {
      midE = (minE + maxE) / 2; midN = (minN + maxN) / 2;
      const S = Math.max(maxE - minE, maxN - minN, MIN_SPAN);
      scale = Math.min(plotW, plotH) / S;
    }
    const px = e => cx + (e - midE) * scale;
    const py = nn => cy - (nn - midN) * scale;           // north up
    const step = this._niceStep(56 / scale);

    /* background grid (metre-aligned) + border */
    ctx.save();
    ctx.beginPath(); ctx.rect(left, top, plotW, plotH); ctx.clip();
    if (bgOn){
      /* fill exactly the ±halfM square (centred on the pad) with the tile */
      const s = bg.halfM * scale;
      ctx.drawImage(bg.img, cx - s, cy - s, 2 * s, 2 * s);
    }
    ctx.strokeStyle = THEME.grid; ctx.lineWidth = 1;
    if (bgOn) ctx.globalAlpha = 0.25;          // grid drawn faintly over imagery
    const eL = Math.ceil((midE - (cx - left) / scale) / step) * step;
    for (let e = eL; px(e) <= right + 0.5; e += step){
      const x = px(e);
      ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, bottom); ctx.stroke();
    }
    const nB = Math.ceil((midN - (bottom - cy) / scale) / step) * step;
    for (let nn = nB; py(nn) >= top - 0.5; nn += step){
      const y = py(nn);
      ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(right, y); ctx.stroke();
    }

    if (bgOn) ctx.globalAlpha = 1;

    /* flight path + markers (clipped to the plot) */
    if (n > 0){
      if (n >= 2){
        ctx.lineJoin = 'round'; ctx.lineCap = 'round';
        ctx.beginPath();
        let started = false;
        for (let i = 0; i < n; i++){
          const p = pts[i]; if (!p) continue;
          const x = px(p[0]), y = py(p[1]);
          if (!started){ ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
        }
        /* over imagery: a card-toned halo under the trace so it stays legible */
        if (bgOn){ ctx.strokeStyle = THEME.card; ctx.lineWidth = 4; ctx.stroke(); }
        ctx.strokeStyle = THEME.series[0]; ctx.lineWidth = 2; ctx.stroke();
      }
      /* apogee: distinct diamond + tiny label */
      if (apogeeIdx != null && apogeeIdx >= 0 && apogeeIdx < n && pts[apogeeIdx]){
        const a = pts[apogeeIdx], x = px(a[0]), y = py(a[1]);
        ctx.fillStyle = THEME.tone.flight.edge;
        ctx.beginPath();
        ctx.moveTo(x, y - 5); ctx.lineTo(x + 5, y);
        ctx.lineTo(x, y + 5); ctx.lineTo(x - 5, y); ctx.closePath();
        ctx.fill();
        if (bgOn){ ctx.strokeStyle = THEME.card; ctx.lineWidth = 1.5; ctx.stroke(); }
        ctx.font = '9px Consolas, ui-monospace, monospace';
        ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
        ctx.fillText('apogee', x + 7, y - 3);
      }
      /* pad = points[0] (filled square) */
      const p0 = pts[0];
      if (p0){
        ctx.fillStyle = THEME.ink2;
        ctx.fillRect(px(p0[0]) - 4, py(p0[1]) - 4, 8, 8);
        if (bgOn){ ctx.strokeStyle = THEME.card; ctx.lineWidth = 1.5;
                   ctx.strokeRect(px(p0[0]) - 4, py(p0[1]) - 4, 8, 8); }
      }
      /* current = last point (filled dot) */
      const pc = pts[n - 1];
      if (pc){
        ctx.fillStyle = THEME.series[0];
        ctx.beginPath(); ctx.arc(px(pc[0]), py(pc[1]), 4, 0, 6.2832); ctx.fill();
        if (bgOn){ ctx.strokeStyle = THEME.card; ctx.lineWidth = 1.5; ctx.stroke(); }
      }
    }
    ctx.restore();

    /* border */
    ctx.strokeStyle = THEME.axis; ctx.lineWidth = 1;
    ctx.strokeRect(left + 0.5, top + 0.5, plotW - 1, plotH - 1);

    /* N arrow (top-right): orientation hint */
    const ax = right - 12, a0 = top + 12, a1 = top + 30;
    ctx.strokeStyle = THEME.ink3; ctx.fillStyle = THEME.ink3; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(ax, a1); ctx.lineTo(ax, a0); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(ax, a0 - 1); ctx.lineTo(ax - 3, a0 + 5);
    ctx.lineTo(ax + 3, a0 + 5); ctx.closePath(); ctx.fill();
    ctx.font = '9px Consolas, ui-monospace, monospace';
    ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
    ctx.fillText('N', ax, a0 - 3);

    /* scale hint: metres per grid division (bottom-left) + track extent.
     * Over imagery, the bottom-left slot carries the required Esri credit
     * (Esri tile ToS) on a card chip instead of the grid-step hint. */
    ctx.fillStyle = THEME.ink3;
    ctx.font = '10px Consolas, ui-monospace, monospace';
    ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
    if (bgOn){
      const cr = 'Imagery © Esri';
      const cw = ctx.measureText(cr).width;
      ctx.globalAlpha = 0.7; ctx.fillStyle = THEME.card;
      ctx.fillRect(left + 1, bottom - 13, cw + 6, 12);
      ctx.globalAlpha = 1; ctx.fillStyle = THEME.ink2;
      ctx.fillText(cr, left + 4, bottom - 2);
    } else {
      ctx.fillText('grid ' + this._fmtM(step), left + 2, bottom - 2);
    }
    if (n > 0){
      ctx.textAlign = 'right';
      ctx.fillText(this._fmtM(maxE - minE) + ' × ' + this._fmtM(maxN - minN),
                   right - 2, bottom - 2);
    }

    /* degenerate: no fix yet */
    if (n === 0){
      ctx.fillStyle = THEME.ink3;
      ctx.font = '12px "Segoe UI", system-ui, sans-serif';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('no GPS fix', cx, cy);
    }
  }
}

/* ---------- render loop: draw only when a chart is dirty ---------- */
function frame(){
  // guard each draw: one chart throwing must not kill the rAF loop and
  // freeze every other chart until reload
  for (const c of charts) if (c.dirty){
    c.dirty = false;
    try { c.draw(); } catch (e){ console.error('chart draw failed', e); }
  }
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
