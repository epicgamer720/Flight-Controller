/* deck.js — Flight Deck UI controller.
 *
 * Self-pacing fetch loop for /data + /events (fetch -> then -> setTimeout
 * ~50 ms; NEVER setInterval — a setInterval version froze a tab in this
 * project). Fills the named chart buffers (chart.js), wires every tile /
 * panel / rail control, mirrors the firmware interlocks from the server's
 * capability matrix, and runs the pyro hold-to-fire ladder.
 *
 * Commands POST /cmd synchronously (the server blocks until the adapter
 * resolves) on their own fetch, parallel to the data loop; the pressed
 * control shows a busy state until the verbatim result comes back.
 *
 * The session token is injected by the server into <body data-token>.
 * The test-fire passcode is never prefilled, never persisted (no
 * localStorage anywhere in this file), and cleared after every attempt.
 */
'use strict';

/* ---------- constants (mirror tools/telem_decode.py + deck/commands.py) ---------- */
const TOKEN = document.body.dataset.token || '';
const STATE_NAMES = ['INIT', 'GROUND_IDLE', 'ARMED', 'BOOST', 'COAST',
                     'APOGEE', 'DROGUE', 'MAIN', 'DESCENT', 'LANDED', 'FAULT'];
/* state chip palette tone per state (chart.js timeline uses the same map) */
const STATE_TONE = ['idle', 'good', 'warn', 'flight', 'flight', 'flight',
                    'flight', 'flight', 'flight', 'good', 'crit'];
const PAD_STATES = [0, 1, 2];           // telemetry.c on_pad()
const SERIES = ['alt', 'vel', 'accel', 'gyro', 'batt', 'rssi', 'snr', 'rate', 'pyro', 'position'];
const BATT_WARN_V = 7.0, BATT_CRIT_V = 6.6;   // 2S thresholds (tile + audio)
const MAINALT_MIN = 30, MAINALT_MAX = 2000;   // radio-path bound, UI-enforced

const $ = id => document.getElementById(id);

/* ---------- buffers + charts ---------- */
SERIES.forEach(registerBuffer);

new CanvasChart('ch-alt',   { buf:'alt',   cols:[1,2],   names:['baro','GPS'],
  scale:1, minSpan:2,   dec:1, label:'Altitude, baro and GPS',
  apogeeRef:true });
new CanvasChart('ch-vel',   { buf:'vel',   cols:[1],     names:['v'],
  scale:1, minSpan:1,   dec:1, label:'Vertical velocity' });
new CanvasChart('ch-accel', { buf:'accel', cols:[1,2,3], names:['X','Y','Z'],
  scale:1, minSpan:0.3, dec:2, label:'Accelerometer' });
new CanvasChart('ch-gyro',  { buf:'gyro',  cols:[1,2,3], names:['X','Y','Z'],
  scale:1, minSpan:1,   dec:1, label:'Gyroscope' });
new CanvasChart('ch-batt',  { buf:'batt',  cols:[1],     names:['V'],
  scale:1, minSpan:0.2, dec:2, label:'Battery volts' });
new CanvasChart('ch-rssi',  { buf:'rssi',  cols:[1],     names:['dBm'],
  scale:1, minSpan:2,   dec:0, label:'Link RSSI' });
new CanvasChart('ch-snr',   { buf:'snr',   cols:[1],     names:['dB'],
  scale:1, minSpan:1,   dec:1, label:'Link SNR' });
new CanvasChart('ch-rate',  { buf:'rate',  cols:[1],     names:['Hz'],
  scale:1, minSpan:1,   dec:1, label:'Packet rate' });
new CanvasChart('ch-pyro',  { buf:'pyro',  cols:[1],     names:['mV'],
  scale:1, minSpan:50,  dec:0, label:'Pyro sense millivolts',
  emptyMsg:'console only — no pyro sense on this source' });
new StateTimeline('timeline-box');
const groundTrack = new GroundTrack($('ground-track'));

/* ---------- client session state ---------- */
let srcKind = null, srcPort = null, connected = false;
let caps = {}, latest = {}, peaks = {}, counters = {};
let recInfo = { on: false, path: null, rows: 0 };
let gap = null;
let recovery = null;              // {distance_m, bearing_deg, pad_lat, pad_lon}
let eseq = -1, lastNow = -1;
let ackCount = 0, nakCount = 0;
let gsTesten = false;              // UI-local only — real testen unknown over radio
let replayPaused = false;
let battWarned = false, battCrited = false;
let staleAlarmed = false;         // telemetry-loss alarm hysteresis
let evFilter = 'all';
const FILTER_KINDS = {
  cmd:   ['cmd_sent', 'ack', 'nak', 'refusal'],
  link:  ['conn', 'gs_error', 'reboot', 'rec'],
  state: ['state', 'fault', 'pyro']
};

function resetClient(){
  resetBuffers();
  eventMarks.length = 0;
  stateSegs.length = 0;
  launchT = null;
  eseq = -1; lastNow = -1; dataNow = 0;
  ackCount = 0; nakCount = 0;
  gsTesten = false;
  replayPaused = false;
  battWarned = battCrited = false;
  staleAlarmed = false;
  recovery = null;
  $('ev-body').innerHTML = '';
  $('ev-count').textContent = '';
  $('pyro-result').textContent = '';
  $('out-pad').textContent = '';
  $('out-maint').textContent = '';
  $('src-out').textContent = '';
  $('passcode').value = '';
  $('btn-rp-pause').textContent = 'Pause replay';
  DeckAudio.stopFault();
  $('fault-banner').classList.remove('on');
  view.end = null;
  syncUI(); markDirty();
}

/* ---------- helpers ---------- */
function fmt(v, d){
  return (v === null || v === undefined || Number.isNaN(Number(v)))
    ? '—' : Number(v).toFixed(d);
}
function fmtTplus(){
  if (launchT === null) return 'T− —';
  const dt = dataNow - launchT;
  const sign = dt < 0 ? 'T−' : 'T+';
  const a = Math.abs(dt);
  /* pre-round to tenths so 59.95 s becomes 01:00.0, never 00:60.0 */
  const d10 = Math.round(a * 10);
  const m = Math.floor(d10 / 600), s = (d10 % 600) / 10;
  return sign + ' ' + String(m).padStart(2, '0') + ':' + (s < 10 ? '0' : '') + s.toFixed(1);
}
function stateName(st){ return STATE_NAMES[st] || ('#' + st); }

/* mirror of deck/commands.py _gate_check — the server (and firmware)
 * re-check; this only drives enable/disable + the WHY tooltip */
function cmdReason(name){
  if (!srcKind) return 'no source connected';
  if (srcKind === 'replay') return 'replay source — commands disabled';
  const cap = caps[name];
  if (!cap) return 'capabilities not loaded yet';
  if (srcKind === 'gs' && !cap.gs) return name + ' is console-only — connect the FC over USB';
  const st = latest.state;
  if (st === null || st === undefined) return 'no telemetry yet — state unknown';
  if (srcKind === 'gs' && PAD_STATES.indexOf(st) < 0)
    return 'FC is TX-only in flight — commands disabled';
  if (cap.states.indexOf(st) < 0) return name + ' not allowed in ' + stateName(st);
  if (cap.needs_disarmed && latest.armed) return name + ' requires DISARMED — disarm first';
  if (cap.needs_testen && latest.testen === false) return 'test mode is off (testen on first)';
  return null;
}

/* ---------- command POST (parallel to the data loop; busy state) ---------- */
/* a 403 means our injected token no longer matches (server restarted under
 * the open tab) — its body has none of the ok/refusal/error keys the
 * renderers look at, so synthesize one that every result path understands */
const STALE_MSG = 'session token stale — reload the page';
function postJSON(path, body){
  return fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json', 'X-Deck-Token': TOKEN },
    body: JSON.stringify(body || {})
  }).then(r => {
    if (r.status === 403)
      return { ok: false, stale: true, refusal: STALE_MSG, error: STALE_MSG };
    return r.json();
  });
}
function postCmd(name, args, btn, cb){
  if (btn){ btn.classList.add('busy'); btn.disabled = true; }
  return postJSON('/cmd', { cmd: name, args: args || {} })
    .catch(() => ({ ok: false, detail: '', refusal: 'deck server unreachable', nak: false }))
    .then(res => {
      if (btn){ btn.classList.remove('busy'); btn.disabled = false; }
      if (cb) cb(res);
      return res;
    });
}
function resultText(res){
  const parts = [];
  if (res.ok){ parts.push(res.detail || 'OK'); }
  else {
    if (res.refusal) parts.push(res.refusal);
    if (res.detail && res.detail !== res.refusal) parts.push(res.detail);
    if (!parts.length) parts.push('failed');
    if (res.nak) parts.push('(NAK)');
  }
  return parts.join('\n');
}

/* ---------- fetch loop: self-pacing, never setInterval ---------- */
function tick(){
  return Promise.all([
    fetch('/data?since=' + lastT).then(r => r.json()),
    fetch('/events?since=' + eseq).then(r => r.json())
  ]).then(([d, e]) => {
    if (lastNow >= 0 && d.now < lastNow - 0.5) resetClient();  // server/source restart
    handleEvents(e);          // events first: precise state-transition times
    handleData(d);
  }).catch(() => {
    connected = false;
    $('dot').className = 'livedot';
    $('conn').textContent = 'deck server stopped';
  });
}
function loop(){ tick().then(() => setTimeout(loop, 50)); }

/* ---------- /data ---------- */
function handleData(d){
  lastNow = d.now;

  const prevLast = lastT;
  const cut = d.now - 130;                     // client rolling window
  for (const k of SERIES){
    const rows = (d.series && d.series[k]) || [];
    const b = buffers[k];
    for (const r of rows){ b.push(r); if (r[0] > lastT) lastT = r[0]; }
    while (b.length && b[0][0] < cut) b.shift();
  }
  dataNow = d.now;
  if (lastT !== prevLast && view.end === null) markDirty();
  if (view.end !== null){
    const ce = clampEnd(view.end);
    if (ce !== view.end){ view.end = ce; markDirty(); }
  }

  const src = d.source || {};
  if ((src.kind || null) !== srcKind){
    srcKind = src.kind || null;
    onSourceKind();
  }
  srcPort = src.port || null;
  connected = !!src.connected;
  latest = d.latest || {};
  caps = d.capabilities || caps;
  peaks = d.peaks || {};
  const apo = (peaks.apogee_m === undefined) ? null : peaks.apogee_m;
  if (apo !== refApogee){ refApogee = apo; markDirty(); }
  counters = d.counters || {};
  recInfo = d.rec || recInfo;
  gap = (d.gap === undefined) ? null : d.gap;
  recovery = d.recovery || null;

  const lt = (d.tplus && d.tplus.launch_t_host !== undefined)
    ? d.tplus.launch_t_host : null;
  if (lt !== launchT){ launchT = lt; syncUI(); markDirty(); }

  battAudio(latest.batt_v);
  syncTimelineWithLatest();
  trimMarks();

  renderHeader();
  renderTiles();
  renderLink();
  renderRadio();
  renderRail();
  renderPyro();
  renderCards();
  renderGroundTrack();
}

function onSourceKind(){
  $('replay-ctl').hidden = srcKind !== 'replay';
  $('gs-testen-warn').hidden = srcKind !== 'gs';
  $('sec-radio').hidden = !(srcKind === 'fc' || srcKind === 'gs');
  $('fc-radio-panel').hidden = srcKind !== 'fc';   // FC's own LoRa radio tiles
}

/* ---------- /events ---------- */
function handleEvents(e){
  const evs = (e && e.events) || [];
  for (const ev of evs){
    if (ev.seq <= eseq) continue;
    eseq = ev.seq;
    addLogRow(ev);
    switch (ev.kind){
      /* counters count each command OUTCOME once (the log shows every
       * event): a success emits both the ingest-level "PKT_ACK received"
       * and the adapter-level "<cmd>: ACK" — only the latter counts; a
       * timeout emits an interim "… — retrying once" nak plus the final
       * "<cmd>: NAK…" / "<cmd>: no reply…" — the interim never counts. */
      case 'ack':
        if (/: ACK$/.test(ev.text || '')) ackCount++;
        break;
      case 'nak': {
        const nt = ev.text || '';
        if ((nt.indexOf(': NAK') >= 0 || nt.indexOf(': no ') >= 0) &&
            nt.indexOf('retrying once') < 0)
          nakCount++;
        DeckAudio.nakBlip();
        break;
      }
      case 'refusal': DeckAudio.nakBlip(); break;
      case 'state': onStateEvent(ev, false); break;
      case 'fault': onStateEvent(ev, true); break;
      case 'pyro': addMark(ev.t_host, 'pyro'); break;
      case 'reboot': addMark(ev.t_host, 'reboot'); break;
    }
  }
}

function onStateEvent(ev, isFault){
  const m = /->\s*(\w+)\s*$/.exec(ev.text || '');
  const to = m ? m[1] : '?';
  addMark(ev.t_host, to);
  const idx = STATE_NAMES.indexOf(to);
  if (idx >= 0) transitionSeg(ev.t_host, idx);
  if (isFault){
    $('fault-banner').classList.add('on');
    DeckAudio.startFault();
  } else {
    DeckAudio.stateChirp();
  }
}

function addMark(t, label){
  if (t === null || t === undefined) return;
  eventMarks.push({ t: t, label: label });
  markDirty();
}
function trimMarks(){
  const cut = dataNow - 130;
  while (eventMarks.length && eventMarks[0].t < cut) eventMarks.shift();
  while (stateSegs.length > 1 && stateSegs[0].t1 !== null && stateSegs[0].t1 < cut)
    stateSegs.shift();
}

/* ---------- state timeline segments ---------- */
function transitionSeg(t, st){
  const cur = stateSegs[stateSegs.length - 1];
  if (cur){
    if (cur.st === st) return;
    cur.t1 = t;
  }
  stateSegs.push({ t0: t, t1: null, st: st,
                   name: stateName(st), tone: STATE_TONE[st] || 'idle' });
  markDirty();
}
function syncTimelineWithLatest(){
  const st = latest.state;
  if (st === null || st === undefined) return;
  const cur = stateSegs[stateSegs.length - 1];
  if (!cur){
    stateSegs.push({ t0: 0, t1: null, st: st,
                     name: stateName(st), tone: STATE_TONE[st] || 'idle' });
    markDirty();
  } else if (cur.st !== st){
    transitionSeg(dataNow, st);       // fallback if a state event was missed
  }
}

/* ---------- event log pane ---------- */
function evMatch(kind){
  if (evFilter === 'all') return true;
  return (FILTER_KINDS[evFilter] || []).indexOf(kind) >= 0;
}
function addLogRow(ev){
  const body = $('ev-body');
  const tr = document.createElement('tr');
  tr.dataset.kind = ev.kind;
  const tp = launchT !== null && ev.t_host !== null
    ? (ev.t_host - launchT >= 0 ? 'T+' : 'T−') + Math.abs(ev.t_host - launchT).toFixed(1)
    : '—';
  const tdTp = document.createElement('td'); tdTp.className = 't'; tdTp.textContent = tp;
  const tdT = document.createElement('td'); tdT.className = 't';
  tdT.textContent = (ev.t_host === null ? '—' : ev.t_host.toFixed(1) + 's');
  const tdSev = document.createElement('td');
  const sev = document.createElement('span');
  sev.className = 'sev sev-' + (ev.severity || 'info');
  sev.textContent = ev.severity || 'info';
  tdSev.appendChild(sev);
  const tdK = document.createElement('td'); tdK.className = 't'; tdK.textContent = ev.kind;
  const tdTxt = document.createElement('td'); tdTxt.className = 'txt';
  tdTxt.textContent = ev.text;                      // VERBATIM, incl. refusals
  tr.append(tdTp, tdT, tdSev, tdK, tdTxt);
  tr.hidden = !evMatch(ev.kind);
  body.insertBefore(tr, body.firstChild);           // newest on top
  while (body.children.length > 400) body.removeChild(body.lastChild);
  $('ev-count').textContent = eseq >= 0 ? (eseq + ' events') : '';
}
function applyEvFilter(){
  for (const tr of $('ev-body').children) tr.hidden = !evMatch(tr.dataset.kind);
}

/* ---------- renders ---------- */
function renderHeader(){
  $('dot').className = 'livedot' + (connected ? ' on' : '');
  $('conn').textContent = srcKind
    ? srcKind + (srcPort ? ' · ' + srcPort : '') + (connected ? '' : ' — no link')
    : 'no source';
  $('tplus').textContent = fmtTplus();

  const st = latest.state;
  const chip = $('state-chip');
  if (st === null || st === undefined){
    chip.textContent = '—'; chip.className = 'stchip t-idle';
  } else {
    chip.textContent = latest.state_name || stateName(st);
    chip.className = 'stchip t-' + (STATE_TONE[st] || 'idle');
  }
  const ac = $('armed-chip');
  if (latest.armed === null || latest.armed === undefined){
    ac.textContent = '—'; ac.className = 'stchip t-idle';
  } else if (latest.armed){
    ac.textContent = '⚠ ARMED'; ac.className = 'stchip t-crit';
  } else {
    ac.textContent = '● SAFE'; ac.className = 'stchip t-good';
  }

  $('rec-dot').className = 'recdot' + (recInfo.on ? ' on' : '');
  $('rec-txt').textContent = recInfo.on
    ? 'REC · ' + (recInfo.rows || 0) + ' rows' : 'rec off';
  $('btn-rec').textContent = recInfo.on ? '■ Stop rec' : '● Record';
  $('rec-path').textContent = recInfo.path ? 'session: ' + recInfo.path : '';
}

function compass(deg){
  return ['N','NE','E','SE','S','SW','W','NW'][Math.round((((deg % 360) + 360) % 360) / 45) % 8];
}
function fmtDist(m){
  if (m === null || m === undefined) return '—';
  return m >= 1000 ? (m / 1000).toFixed(2) + ' km' : Math.round(m) + ' m';
}

/* GPS ground track (9b): project the recorded position buffer (raw lat/lon)
 * to local east/north metres around the first fix, mark the apogee (max baro
 * alt among fixes), and hand it to the equal-aspect GroundTrack canvas. */
function renderGroundTrack(){
  const rows = buffers.position;
  if (!rows || !rows.length){ groundTrack.draw([], -1); return; }
  // Origin = the frozen pad fix (recovery.pad_*, which survives the 130 s
  // rolling-window trim), so the pad marker + projection stay correct even for
  // flights longer than the window; fall back to the first buffered fix before
  // a pad is known. (apogee marker is best-effort: max baro alt among buffered
  // fixes — for a >130 s flight the true apogee fix may have aged out.)
  let lat0, lon0, padKnown = false;
  if (recovery && recovery.pad_lat != null && recovery.pad_lon != null){
    lat0 = recovery.pad_lat; lon0 = recovery.pad_lon; padKnown = true;
  } else {
    lat0 = rows[0][1]; lon0 = rows[0][2];
  }
  const k = 111320, cosLat = Math.cos(lat0 * Math.PI / 180);
  const points = rows.map(r => [(r[2] - lon0) * k * cosLat, (r[1] - lat0) * k]);
  let ai = -1, amax = -Infinity;
  for (let i = 0; i < rows.length; i++){
    const a = rows[i][3];
    if (a !== null && a !== undefined && a > amax){ amax = a; ai = i; }
  }
  if (padKnown){ points.unshift([0, 0]); if (ai >= 0) ai += 1; }  // pad at the true pad
  groundTrack.draw(points, ai);
}

function renderTiles(){
  $('t-alt').textContent = fmt(latest.alt_baro_m, 1);
  $('t-vel').textContent = fmt(latest.vel_up_ms, 1);
  $('t-temp').textContent = fmt(latest.temp_c, 1);   // baro die temp; fc only
  $('t-mcu').textContent = fmt(latest.mcu_temp_c, 1); // STM32 die temp; fc only

  const v = latest.batt_v;
  const bv = $('t-batt'), bs = $('t-batt-sub');
  bv.textContent = fmt(v, 2);
  if (v === null || v === undefined){
    bv.className = 'v'; bs.className = 'sub'; bs.textContent = 'V';
  } else if (v <= BATT_CRIT_V){
    bv.className = 'v crit'; bs.className = 'sub crit'; bs.textContent = '⚠ CRIT · V';
  } else if (v <= BATT_WARN_V){
    bv.className = 'v warn'; bs.className = 'sub warn'; bs.textContent = '⚠ LOW · V';
  } else {
    bv.className = 'v ok'; bs.className = 'sub ok'; bs.textContent = '● OK · V';
  }

  const pc = latest.pyro_cont;
  const c = $('t-cont'), cs = $('t-cont-sub');
  if (pc === null || pc === undefined){
    c.textContent = '—'; c.className = 'v small-v'; cs.textContent = 'ch1';
  } else if (pc & 1){
    c.textContent = '● CONT'; c.className = 'v small-v ok'; cs.textContent = 'ch1 e-match present';
  } else {
    c.textContent = '○ OPEN'; c.className = 'v small-v'; cs.textContent = 'ch1 no continuity';
  }

  const gf = latest.gps_fix;
  const g = $('t-gps'), gs2 = $('t-gps-sub');
  if (gf === null || gf === undefined){
    g.textContent = '—'; g.className = 'v small-v'; gs2.textContent = 'fix · sats';
  } else if (gf){
    g.textContent = '● FIX'; g.className = 'v small-v ok';
    gs2.textContent = (latest.sats === undefined ? '?' : latest.sats) + ' sats';
  } else {
    g.textContent = '○ NO FIX'; g.className = 'v small-v';
    gs2.textContent = (latest.sats === undefined ? '?' : latest.sats) + ' sats';
  }

  $('t-apogee').textContent = fmt(peaks.apogee_m, 1);
  // baro-vs-GPS apogee divergence (7d): a large delta flags a bad baro zero,
  // a stuck baro, or GPS multipath.
  const ab = peaks.apogee_m, ag = peaks.apogee_gps_m, asub = $('t-apogee-sub');
  if (ab !== null && ab !== undefined && ag !== null && ag !== undefined){
    const dv = ab - ag;
    asub.textContent = 'GPS Δ ' + (dv >= 0 ? '+' : '') + dv.toFixed(0) + ' m';
    asub.className = Math.abs(dv) > 30 ? 'sub warn' : 'sub';
  } else {
    asub.textContent = 'peak · m'; asub.className = 'sub';
  }
  $('t-maxvel').textContent = fmt(peaks.max_vel_ms, 1);
  $('t-maxg').textContent = fmt(peaks.max_abs_g, 2);
  // recovery bearing + distance from the frozen pad fix (7a)
  const rc = $('t-recovery'), rs = $('t-recovery-sub');
  if (recovery){
    rc.textContent = Math.round(recovery.bearing_deg) + '° ' + compass(recovery.bearing_deg);
    rs.textContent = fmtDist(recovery.distance_m);
    rc.className = 'v small-v ok';
  } else {
    rc.textContent = '—'; rc.className = 'v small-v'; rs.textContent = 'bearing · dist';
  }
}

function renderLink(){
  const show = srcKind === 'gs' || srcKind === 'replay';
  $('link-panel').classList.toggle('on', show);
  if (!show) return;
  $('t-rssi').textContent = fmt(latest.rssi, 0);
  $('t-snr').textContent = fmt(latest.snr, 1);
  $('t-rate').textContent = fmt(latest.pkt_rate, 1);
  $('t-acknak').textContent = ackCount + ' / ' + nakCount;
  $('t-counters').textContent =
    'junk ' + (counters.junk || 0) + ' · gs_err ' + (counters.gs_error || 0);

  /* last-packet-gap meter, cadence-aware: pad 1 Hz vs flight 10 Hz */
  const st = latest.state;
  const inFlight = st !== null && st !== undefined && st >= 3 && st <= 8;
  const warn = inFlight ? 1 : 3, crit = inFlight ? 3 : 10;
  const fill = $('gap-fill'), txt = $('gap-txt');
  if (gap === null || gap === undefined){
    fill.style.width = '0%'; fill.className = ''; txt.textContent = '—';
  } else {
    fill.style.width = Math.min(100, gap / crit * 100).toFixed(1) + '%';
    fill.className = gap > crit ? 'bad' : (gap > warn ? 'meh' : '');
    txt.textContent = gap.toFixed(1) + ' s' + (gap > crit ? ' ⚠ stale' : '');
    /* telemetry-loss alarm (7b): blip once on the rising edge into stale,
     * re-arm when the link recovers (battAudio-style hysteresis). */
    if (gap > crit){
      if (!staleAlarmed){ staleAlarmed = true; DeckAudio.linkStale(); }
    } else if (gap <= warn){
      staleAlarmed = false;
    }
  }
}

/* Radio section: TX power control on both paths.
 *  - FC (USB): live `tx` dashboard scrape — power, packets tx/rx, last-RX
 *    link quality (the 'GS link' panel is downlink-as-seen-by-the-GS).
 *  - GS (radio): the control sends CMD_SET_TX_POWER over the air; the FC
 *    doesn't report its own power back, so no live readout. */
function renderRadio(){
  if (srcKind !== 'fc' && srcKind !== 'gs') return;   // control hidden otherwise
  /* TX-power SET control gating (both paths) — command results own
   * #radio-stats, so this never overwrites them per frame. */
  const why = cmdReason('power');
  const btn = $('btn-txpower'), inp = $('txpower');
  if (!btn.classList.contains('busy')){ btn.disabled = !!why; inp.disabled = !!why; }
  $('radio-why').textContent = why ||
    (srcKind === 'gs' ? 'over-air — FC applies on its next pad RX window' : '');
  const p = latest.tx_power_dbm;
  $('txpower-cur').textContent = (p == null) ? '' : 'now ' + p + ' dBm';
  if (srcKind !== 'fc') return;   // downlink RSSI/SNR/rate live in the GS-link panel

  /* FC's own LoRa radio (read over SPI, scraped from the `tx` console cmd) */
  $('t-txpower').textContent = (p == null) ? '—' : p;
  $('t-txcount').textContent = (latest.tx_count == null) ? '—' : latest.tx_count;
  $('t-txcount-sub').textContent =
    'sent · rx ' + (latest.rx_count == null ? 0 : latest.rx_count);
  const ok = latest.radio_ok;
  const rh = $('t-radiohealth');
  rh.textContent = (ok == null) ? '—' : (ok ? '● OK' : '○ FAIL');
  rh.className = 'v small-v' + (ok ? ' ok' : (ok === false ? ' crit' : ''));
  $('t-radiohealth-sub').textContent = 'tcxo ' + (latest.radio_tcxo || '?') +
    ' · reinit ' + (latest.radio_reinit == null ? 0 : latest.radio_reinit);
  const up = $('t-uplink');
  if (latest.rx_rssi == null){
    up.textContent = '—';
    up.className = 'v small-v';
    $('fc-radio-hint').textContent =
      'FC broadcasts telemetry on cadence (1 Hz on the pad) into the air — no ' +
      'receiver needed, so Packets TX climbs on its own. RSSI/SNR here are of ' +
      'uplink commands the FC hears; downlink link quality is measured on the GS.';
  } else {
    up.textContent = latest.rx_rssi + ' / ' + (latest.rx_snr == null ? '—' : latest.rx_snr);
    up.className = 'v small-v ok';
    $('fc-radio-hint').textContent = '';
  }
}

const CMD_BTNS = [
  ['arm', 'btn-arm'], ['disarm', 'btn-disarm'], ['zero', 'btn-zero'],
  ['cal', 'btn-cal'], ['mainalt', 'btn-mainalt'],
  ['log', 'btn-log-start'], ['log', 'btn-log-stop'],
  ['reboot', 'btn-reboot'], ['bootloader', 'btn-boot'], ['wdtest', 'btn-wdtest'],
  ['sleep', 'btn-sleep']
];
function renderRail(){
  for (const [name, id] of CMD_BTNS){
    const b = $(id);
    if (b.classList.contains('busy')) continue;     // don't clobber in-flight cmd
    const why = cmdReason(name);
    b.disabled = !!why;
    b.title = why || '';
  }
  const padWhy = cmdReason('zero');                  // representative pad command
  $('pad-why').textContent = padWhy || '';

  $('mainalt-cur').textContent =
    (latest.main_alt_m === null || latest.main_alt_m === undefined)
      ? '' : 'now ' + latest.main_alt_m + ' m';
  $('log-state').textContent =
    (latest.log_running === null || latest.log_running === undefined)
      ? '' : (latest.log_running ? 'log running' : 'log stopped');

  const sw = cmdReason('servo');
  $('servo-en').disabled = !!sw;
  $('servo-en').title = sw || '';
  if (sw) $('servo-en').checked = false;
  const en = $('servo-en').checked && !sw;
  document.querySelectorAll('#servo-rows input[type=range]')
    .forEach(s => { s.disabled = !en; });
}

/* ---------- pyro interlock ladder ---------- */
function gchip(id, on, unk){
  const el = $(id);
  el.className = 'gchip' + (on ? ' on' : '') + (unk ? ' unk' : '');
}
function testActive(){
  if (srcKind === 'fc') return latest.testen === true;
  if (srcKind === 'gs') return gsTesten;
  return false;
}
function fireWhy(){
  const r = cmdReason('fire');
  if (r) return r;
  if (srcKind === 'fc' && latest.testen !== true) return 'test mode is off (testen on first)';
  if (srcKind === 'gs' && !gsTesten) return 'flip the local test-enable toggle first';
  if (!$('passcode').value) return 'enter the passcode';
  return null;
}
function renderPyro(){
  const tb = $('btn-testen');
  if (srcKind === 'gs'){
    tb.disabled = tb.classList.contains('busy');
    tb.title = 'UI-local on GS — firmware testen state unknown over radio';
    tb.textContent = 'Test-enable: ' + (gsTesten ? 'ON (local)' : 'OFF');
    $('testen-why').textContent = '';
  } else if (srcKind === 'fc'){
    const why = cmdReason('testen');
    if (!tb.classList.contains('busy')) tb.disabled = !!why;
    tb.title = why || '';
    tb.textContent = 'Test-enable: ' +
      (latest.testen === undefined || latest.testen === null
        ? '?' : (latest.testen ? 'ON' : 'OFF'));
    $('testen-why').textContent = why || '';
  } else {
    tb.disabled = true;
    tb.title = cmdReason('testen') || 'no source';
    tb.textContent = 'Test-enable: —';
    $('testen-why').textContent = tb.title;
  }

  const active = testActive();
  $('pyro-body').hidden = !active;
  if (!active) return;

  gchip('g-idle', latest.state === 1);
  gchip('g-disarm', latest.armed === false);
  gchip('g-testen', srcKind === 'fc' ? latest.testen === true : false,
        srcKind === 'gs' && gsTesten);
  gchip('g-cont', latest.pyro_cont !== null && latest.pyro_cont !== undefined
                  && (latest.pyro_cont & 1) === 1);

  const why = fireWhy();
  $('fire-hold').classList.toggle('disabled', !!why);
  $('fire-why').textContent = why || 'hold 2 s — release aborts';
}

/* hold-to-fire: pointer-only, 2 s countdown ring, release/leave aborts.
 * Deliberately NOT keyboard-activatable (div, no tabindex, no key handlers). */
const fireBtn = $('fire-hold');
let holdRAF = null, holdT0 = 0, holdTickIdx = -1;
let holdPt = null;         // last-known pointer position (client px) during a hold
function holdAbort(){
  if (holdRAF !== null){ cancelAnimationFrame(holdRAF); holdRAF = null; }
  fireBtn.style.setProperty('--p', 0);
}
function holdStep(){
  const frac = (performance.now() - holdT0) / 2000;
  if (fireWhy()){ holdAbort(); renderPyro(); return; }   // gates changed mid-hold
  if (holdPt){             // belt-and-braces: abort if the pointer slid off the pad
    const r = fireBtn.getBoundingClientRect();
    if (holdPt.x < r.left || holdPt.x > r.right ||
        holdPt.y < r.top  || holdPt.y > r.bottom){ holdAbort(); return; }
  }
  const tickIdx = Math.floor(frac * 4);                  // a tick every 0.5 s
  if (tickIdx !== holdTickIdx){ holdTickIdx = tickIdx; DeckAudio.countTick(); }
  fireBtn.style.setProperty('--p', Math.min(100, frac * 100));
  if (frac >= 1){ holdRAF = null; doFire(); return; }
  holdRAF = requestAnimationFrame(holdStep);
}
fireBtn.addEventListener('pointerdown', e => {
  if (e.button !== undefined && e.button !== 0) return;
  if (holdRAF !== null) return;
  const why = fireWhy();
  if (why){ $('fire-why').textContent = why; return; }
  /* touch pointers get IMPLICIT pointer capture on pointerdown, which
   * swallows pointerleave — release it so sliding off the pad aborts */
  try { fireBtn.releasePointerCapture(e.pointerId); } catch (_) {}
  holdPt = { x: e.clientX, y: e.clientY };
  holdT0 = performance.now();
  holdTickIdx = -1;
  holdStep();
});
fireBtn.addEventListener('pointermove', e => {
  holdPt = { x: e.clientX, y: e.clientY };
});
fireBtn.addEventListener('pointerup', holdAbort);
fireBtn.addEventListener('pointerleave', holdAbort);
fireBtn.addEventListener('pointercancel', holdAbort);

function doFire(){
  fireBtn.style.setProperty('--p', 0);
  const code = $('passcode').value;
  $('passcode').value = '';                       // cleared after EVERY attempt
  const ch = parseInt($('fire-ch').value, 10) || 1;
  fireBtn.classList.add('busy');
  DeckAudio.fireSend();
  postCmd('fire', { ch: ch, passcode: code }, null, res => {
    fireBtn.classList.remove('busy');
    $('pyro-result').textContent = resultText(res);   // verbatim detail/refusal
  });
}

/* ---------- chart-card auto-hide (buffers that never fill) ---------- */
function renderCards(){
  document.querySelectorAll('.chart-card[data-buf]').forEach(card => {
    const b = buffers[card.dataset.buf];
    const has = b && b.length > 0;
    card.hidden = !has && dataNow > 8;      // grace period, then hide until data
  });
}

/* ---------- battery audio (thresholds shared with the tile) ---------- */
function battAudio(v){
  if (v === null || v === undefined) return;
  if (v <= BATT_CRIT_V){
    if (!battCrited){ battCrited = true; battWarned = true; DeckAudio.battCrit(); }
  } else if (v <= BATT_WARN_V){
    if (!battWarned){ battWarned = true; DeckAudio.battWarn(); }
    if (v > BATT_CRIT_V + 0.15) battCrited = false;      // hysteresis re-arm
  } else {
    if (v > BATT_WARN_V + 0.15) battWarned = false;
    battCrited = false;
  }
}

/* ---------- confirm modal ---------- */
let modalYes = null;
function showConfirm(title, bodyHtml, onYes, okLabel){
  $('modal-title').textContent = title;
  $('modal-body').innerHTML = bodyHtml;      // static strings from this file only
  $('modal-ok').textContent = okLabel || 'Confirm';
  modalYes = onYes;
  $('modal-back').classList.add('on');
}
$('modal-cancel').addEventListener('click', () => {
  modalYes = null; $('modal-back').classList.remove('on');
});
$('modal-ok').addEventListener('click', () => {
  const f = modalYes; modalYes = null;
  $('modal-back').classList.remove('on');
  if (f) f();
});
$('modal-back').addEventListener('click', e => {
  if (e.target === e.currentTarget){
    modalYes = null; e.currentTarget.classList.remove('on');
  }
});

const ARM_HTML =
  '<p>Arming enables the autonomous pyro state machine. The firmware ' +
  'refuses (<span class="mono">arm refused (code)</span>) unless ALL pass:</p>' +
  '<ul>' +
  '<li><span class="mono">−1</span> — must be in GROUND_IDLE</li>' +
  '<li><span class="mono">−2</span> — IMU + baro must be healthy</li>' +
  '<li><span class="mono">−3</span> — gyro cal not done (run cal, keep still ~2 s)</li>' +
  '<li><span class="mono">−4</span> — moving (|vel| ≥ 2 m/s)</li>' +
  '<li><span class="mono">−5</span> — tilt/accel implausible (accel_sat or ||a|−1g| ≥ 0.5 g)</li>' +
  '</ul><p>Disarm stays available on the pad at all times.</p>';

/* ---------- rail wiring ---------- */
$('btn-arm').addEventListener('click', () =>
  showConfirm('ARM the vehicle?', ARM_HTML,
    () => postCmd('arm', {}, $('btn-arm'),
                  res => { $('out-pad').textContent = resultText(res); }),
    'ARM'));
$('btn-disarm').addEventListener('click', () =>
  postCmd('disarm', {}, $('btn-disarm'),
          res => { $('out-pad').textContent = resultText(res); }));
$('btn-zero').addEventListener('click', () =>
  postCmd('zero', {}, $('btn-zero'),
          res => { $('out-pad').textContent = resultText(res); }));
$('btn-cal').addEventListener('click', () =>
  postCmd('cal', {}, $('btn-cal'),
          res => { $('out-pad').textContent = resultText(res); }));
$('btn-mainalt').addEventListener('click', () => {
  const m = parseFloat($('mainalt').value);
  if (!(m >= MAINALT_MIN && m <= MAINALT_MAX)){
    $('out-pad').textContent =
      'main altitude must be ' + MAINALT_MIN + '..' + MAINALT_MAX + ' m';
    return;
  }
  postCmd('mainalt', { m: m }, $('btn-mainalt'),
          res => { $('out-pad').textContent = resultText(res); });
});

$('btn-txpower').addEventListener('click', () => {
  const dbm = parseInt($('txpower').value, 10);
  if (!(dbm >= -9 && dbm <= 22)){
    $('radio-stats').textContent = 'TX power must be -9..22 dBm';
    return;
  }
  postCmd('power', { dbm: dbm }, $('btn-txpower'),
          res => { $('radio-stats').textContent = resultText(res); });
});

$('btn-log-start').addEventListener('click', () =>
  postCmd('log', { op: 'start' }, $('btn-log-start'),
          res => { $('out-maint').textContent = resultText(res); }));
$('btn-log-stop').addEventListener('click', () =>
  postCmd('log', { op: 'stop' }, $('btn-log-stop'),
          res => { $('out-maint').textContent = resultText(res); }));

$('btn-reboot').addEventListener('click', () =>
  showConfirm('Reboot the FC?',
    '<p>The FC restarts and the USB COM port <b>drops</b> for a few seconds; ' +
    'the deck reopens it automatically. Any unsaved log file is closed by ' +
    'the firmware first.</p>',
    () => postCmd('reboot', {}, $('btn-reboot'),
                  res => { $('out-maint').textContent = resultText(res); }),
    'Reboot'));
$('btn-boot').addEventListener('click', () =>
  showConfirm('Jump to DFU bootloader?',
    '<p>The FC jumps to the STM32 system bootloader. The COM port <b>drops ' +
    'and stays down</b> until you flash (<span class="mono">dfu-util -a 0 -d ' +
    '0483:df11 …</span>) or power-cycle the board.</p>',
    () => postCmd('bootloader', {}, $('btn-boot'),
                  res => { $('out-maint').textContent = resultText(res); }),
    'Enter DFU'));
$('btn-wdtest').addEventListener('click', () =>
  showConfirm('Watchdog test?',
    '<p>The firmware halts on purpose so the IWDG resets the MCU (~a few ' +
    'seconds). The COM port <b>drops</b> briefly, then the FC reboots with ' +
    '<span class="mono">reset:IWDG</span>.</p>',
    () => postCmd('wdtest', {}, $('btn-wdtest'),
                  res => { $('out-maint').textContent = resultText(res); }),
    'Halt for WD'));
$('btn-sleep').addEventListener('click', () => {
  const raw = $('sleep-secs').value.trim();
  const secs = raw === '' ? 0 : parseInt(raw, 10);
  if (!(secs >= 0 && secs <= 36000)){
    $('out-maint').textContent = 'sleep seconds must be 0..36000 (0 = until RESET)';
    return;
  }
  const wake = secs > 0
    ? 'auto-wakes and reboots in ~' + secs + ' s'
    : 'stays asleep until you press RESET (SW1)';
  showConfirm('Sleep the FC?',
    '<p>The FC enters STOP mode: the USB console <b>drops</b> and telemetry ' +
    'stops. It ' + wake + '. Pyros are disarmed and the SD log is closed first.</p>',
    () => postCmd('sleep', { secs: secs }, $('btn-sleep'),
                  res => { $('out-maint').textContent = resultText(res); }),
    'Sleep');
});

$('btn-testen').addEventListener('click', () => {
  if (srcKind === 'gs'){
    gsTesten = !gsTesten;               // UI-local; firmware state unknown
    renderPyro();
    return;
  }
  if (srcKind !== 'fc') return;
  const turnOn = latest.testen !== true;
  const send = () => postCmd('testen', { on: turnOn }, $('btn-testen'),
    res => { $('pyro-result').textContent = resultText(res); renderPyro(); });
  if (turnOn){
    showConfirm('Enable pyro test mode?',
      '<p>Opens the test-fire ladder (GROUND_IDLE + disarmed only). The ' +
      'firmware still requires the passcode on every fire command.</p>',
      send, 'Enable');
  } else send();
});

document.querySelectorAll('#servo-rows input[type=range]').forEach(sl => {
  const val = sl.parentElement.querySelector('.servo-val');
  sl.addEventListener('input', () => { val.textContent = sl.value; });
  sl.addEventListener('change', () => {
    postCmd('servo', { ch: parseInt(sl.dataset.ch, 10), us: parseInt(sl.value, 10) },
            null, res => { $('out-maint').textContent = resultText(res); });
  });
});

/* ---------- source panel ---------- */
function refreshPorts(){
  fetch('/ports').then(r => r.json()).then(d => {
    const sel = $('port-sel');
    const cur = sel.value;
    sel.innerHTML = '';
    for (const p of (d.ports || [])){
      const o = document.createElement('option');
      o.value = p.device;
      o.textContent = p.device + (p.is_fc ? ' — FC' : '') +
                      (p.desc ? ' · ' + p.desc : '');
      sel.appendChild(o);
      if (p.is_fc && !cur) o.selected = true;
    }
    if (cur){
      for (const o of sel.options) if (o.value === cur) o.selected = true;
    }
    if (!sel.options.length){
      const o = document.createElement('option');
      o.value = ''; o.textContent = 'no COM ports found';
      sel.appendChild(o);
    }
  }).catch(() => {});
}
function postSource(body){
  $('src-out').textContent = 'switching source…';
  postJSON('/source', body).then(res => {
    if (res.ok){ resetClient(); }
    else $('src-out').textContent = 'source error: ' + (res.error || '?');
  }).catch(() => { $('src-out').textContent = 'deck server unreachable'; });
}
$('btn-ports').addEventListener('click', refreshPorts);
$('btn-conn-fc').addEventListener('click', () => {
  const p = $('port-sel').value;
  if (!p){ $('src-out').textContent = 'pick a COM port first'; return; }
  postSource({ kind: 'fc', port: p });
});
$('btn-conn-gs').addEventListener('click', () => {
  const p = $('port-sel').value;
  if (!p){ $('src-out').textContent = 'pick a COM port first'; return; }
  postSource({ kind: 'gs', port: p });
});
$('btn-replay').addEventListener('click', () => {
  let speed = 1;
  document.querySelectorAll('#seg-rspeed button').forEach(b => {
    if (b.classList.contains('on')) speed = parseFloat(b.dataset.speed);
  });
  postSource({ kind: 'replay', synthetic: 'nominal', speed: speed });
});
$('btn-rec').addEventListener('click', () => {
  postJSON('/record', { on: !recInfo.on }).then(ri => {
    if (ri && ri.stale){ $('src-out').textContent = ri.refusal; return; }
    recInfo = ri; renderHeader();
  }).catch(() => {});
});

/* replay transport */
$('btn-rp-pause').addEventListener('click', () => {
  replayPaused = !replayPaused;
  $('btn-rp-pause').textContent = replayPaused ? 'Resume replay' : 'Pause replay';
  postJSON('/replay', { op: replayPaused ? 'pause' : 'resume' }).catch(() => {});
});
document.querySelectorAll('#seg-rspeed button').forEach(b =>
  b.addEventListener('click', () => {
    document.querySelectorAll('#seg-rspeed button')
      .forEach(x => x.classList.toggle('on', x === b));
    postJSON('/replay', { op: 'speed', speed: parseFloat(b.dataset.speed) })
      .catch(() => {});
  }));

/* ---------- toolbar (donor wiring + flight window) ---------- */
function syncUI(){
  const live = view.end === null;
  $('btn-pause').textContent = live ? 'Pause' : 'Resume';
  $('btn-live').className = 'btn live' + (live ? ' on' : '');
  $('btn-fwin').disabled = launchT === null;
  document.querySelectorAll('#seg-span button').forEach(b => {
    b.className = Math.abs(parseFloat(b.dataset.span) - view.span) < 0.01 ? 'on' : '';
  });
  document.querySelectorAll('#seg-filt button').forEach(b => {
    b.className = Math.abs(parseFloat(b.dataset.alpha) - FILT.alpha) < 1e-6 ? 'on' : '';
  });
  const gh = $('btn-ghost');
  gh.className = 'btn' + (FILT.ghost ? ' on' : '');
  gh.style.display = FILT.alpha > 0 ? '' : 'none';  // only meaningful filtered
  const sp = view.span >= 10 ? Math.round(view.span) : Math.round(view.span * 10) / 10;
  $('chart-h').textContent =
    'Telemetry — last ' + sp + ' s' + (live ? '' : ' (paused)');
}
document.querySelectorAll('#seg-span button').forEach(b =>
  b.addEventListener('click', () => setSpan(parseFloat(b.dataset.span))));
document.querySelectorAll('#seg-filt button').forEach(b =>
  b.addEventListener('click', () => {
    FILT.alpha = parseFloat(b.dataset.alpha); syncUI(); markDirty();
  }));
$('btn-ghost').addEventListener('click', () => {
  FILT.ghost = !FILT.ghost; syncUI(); markDirty();
});
$('btn-pause').addEventListener('click', togglePause);
$('btn-live').addEventListener('click', goLive);
$('btn-fwin').addEventListener('click', flightWindow);
window.addEventListener('keydown', e => {
  if (e.code !== 'Space' || e.repeat) return;
  const t = e.target;
  if (t && (t.tagName === 'BUTTON' || t.tagName === 'INPUT' ||
            t.tagName === 'TEXTAREA' || t.tagName === 'SELECT')) return;
  e.preventDefault();
  togglePause();
});

/* ---------- event-log filter buttons ---------- */
document.querySelectorAll('#seg-elog button').forEach(b =>
  b.addEventListener('click', () => {
    evFilter = b.dataset.f;
    document.querySelectorAll('#seg-elog button')
      .forEach(x => x.classList.toggle('on', x === b));
    applyEvFilter();
  }));

/* ---------- header misc ---------- */
$('btn-mute').addEventListener('click', () => {
  DeckAudio.setMuted(!DeckAudio.muted);
  $('btn-mute').textContent = DeckAudio.muted ? '🔇 muted' : '🔊 sound';
});
/* theme toggle: auto → light → dark, session-only (never persisted).
 * data-theme on <html> drives the deck.css override token blocks; the
 * canvases repaint via chart.js loadTheme() + markDirty() so they re-read
 * the CSS custom properties immediately. */
const THEME_MODES = ['auto', 'light', 'dark'];
let themeMode = 'auto';
$('btn-theme').addEventListener('click', () => {
  themeMode = THEME_MODES[(THEME_MODES.indexOf(themeMode) + 1) % THEME_MODES.length];
  if (themeMode === 'auto')
    document.documentElement.removeAttribute('data-theme');
  else
    document.documentElement.setAttribute('data-theme', themeMode);
  $('btn-theme').textContent = '◑ ' + themeMode;
  loadTheme();
  markDirty();
});
$('fault-banner').addEventListener('click', () => {
  DeckAudio.stopFault();
  $('fault-banner').classList.remove('on');
});

/* ---------- go ---------- */
refreshPorts();
syncUI();
loop();
