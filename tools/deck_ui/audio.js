/* audio.js: Flight Deck alert synth. Web Audio oscillators only, no
 * external files (fully offline). The AudioContext is created/resumed on
 * the first user gesture (browser autoplay policy); until then every cue
 * is silently skipped. Mute is session-only, never persisted.
 *
 * Cues:
 *   stateChirp   two-tone rising chirp on every flight-state transition
 *   startFault   repeating two-tone alarm until acknowledged (click)
 *   battWarn     low-battery warning   (deck.js gates at <= 7.0 V, 2S)
 *   battCrit     low-battery critical  (deck.js gates at <= 6.6 V)
 *   nakBlip      short low buzz on a NAK / refusal
 *   linkStale    descending double blip when telemetry goes stale (link lost)
 *   countTick    fire-hold countdown tick (every 0.5 s of the 2 s hold)
 *   fireSend     confirmation sweep when the hold completes and the
 *                command is actually posted
 */
'use strict';

const DeckAudio = (function(){
  let ctx = null, master = null;
  let muted = false;
  let faultOn = false, faultTimer = null;
  const VOL = 0.22;

  function ensure(){
    if (ctx) return true;
    const AC = window.AudioContext || window.webkitAudioContext;
    if (!AC) return false;
    ctx = new AC();
    master = ctx.createGain();
    master.gain.value = muted ? 0 : VOL;
    master.connect(ctx.destination);
    return true;
  }
  function unlock(){                     // first user gesture unlocks audio
    if (!ensure()) return;
    if (ctx.state === 'suspended') ctx.resume();
  }
  document.addEventListener('pointerdown', unlock, { capture: true });
  document.addEventListener('keydown', unlock, { capture: true });

  function tone(freq, t0, dur, type, gain){
    if (!ctx || muted) return;
    const o = ctx.createOscillator(), g = ctx.createGain();
    o.type = type || 'sine';
    o.frequency.value = freq;
    g.gain.setValueAtTime(0, t0);
    g.gain.linearRampToValueAtTime(gain === undefined ? 0.9 : gain, t0 + 0.008);
    g.gain.setTargetAtTime(0, t0 + dur, 0.02);
    o.connect(g); g.connect(master);
    o.start(t0); o.stop(t0 + dur + 0.15);
  }
  function now(){ return ctx ? ctx.currentTime : 0; }

  return {
    get muted(){ return muted; },
    setMuted(m){
      muted = !!m;
      if (master) master.gain.value = muted ? 0 : VOL;
      if (muted) this.stopFault();
    },

    stateChirp(){
      if (!ctx || muted) return;
      const t = now();
      tone(660, t, 0.09, 'sine', 0.9);
      tone(990, t + 0.10, 0.12, 'sine', 0.9);
    },

    nakBlip(){
      if (!ctx || muted) return;
      tone(196, now(), 0.18, 'square', 0.5);
    },

    linkStale(){                  // telemetry gone quiet: descending double blip
      if (!ctx || muted) return;
      const t = now();
      tone(880, t, 0.10, 'triangle', 0.7);
      tone(440, t + 0.13, 0.22, 'triangle', 0.7);
    },

    battWarn(){
      if (!ctx || muted) return;
      const t = now();
      tone(523, t, 0.15, 'triangle', 0.85);
      tone(392, t + 0.18, 0.24, 'triangle', 0.85);
    },

    battCrit(){
      if (!ctx || muted) return;
      const t = now();
      for (let i = 0; i < 3; i++) tone(330, t + i * 0.25, 0.18, 'square', 0.85);
    },

    countTick(){
      if (!ctx || muted) return;
      tone(1250, now(), 0.03, 'square', 0.45);
    },

    fireSend(){
      if (!ctx || muted) return;
      const t = now();
      tone(700, t, 0.08, 'sine', 0.9);
      tone(1400, t + 0.09, 0.16, 'sine', 0.9);
    },

    /* FAULT alarm: repeats until stopFault() (banner click acknowledges).
     * Self-rescheduling setTimeout: each beat schedules the next only
     * after it runs, and stopFault() cancels the pending one. */
    startFault(){
      if (faultOn) return;
      faultOn = true;
      const beat = () => {
        if (!faultOn) return;
        if (ctx && !muted){
          const t = now();
          tone(880, t, 0.18, 'square', 0.9);
          tone(622, t + 0.22, 0.18, 'square', 0.9);
        }
        faultTimer = setTimeout(beat, 700);
      };
      beat();
    },
    stopFault(){
      faultOn = false;
      if (faultTimer){ clearTimeout(faultTimer); faultTimer = null; }
    },
    get faultActive(){ return faultOn; }
  };
})();
