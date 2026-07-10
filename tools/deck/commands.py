#!/usr/bin/env python3
"""commands.py - Flight Deck command layer.

    CAPABILITIES        the per-command capability matrix (UI + server
                        both consult it; every refusal carries a reason)
    ConsoleCommandAdapter  FC USB console: builds the console text line,
                        runs it via FcConsoleSource.execute(), classifies
                        the VERBATIM firmware reply
    GsCommandAdapter    GS radio uplink: stdin grammar line -> uplink echo
                        (TX confirmation) -> PKT_ACK within ACK_TIMEOUT_S
                        else inferred NAK -> exactly one retry

Safety notes (firmware remains the real gate — this layer only mirrors
it so the operator gets an honest UI):
  * fire is 1-indexed on the console, 0-indexed on the GS stdin grammar;
    the UI speaks 1-indexed everywhere and GsCommandAdapter translates.
  * mainalt is bounded 30..2000 m here (the radio path's stricter bound)
    even though the console itself would accept 10..10000.
  * the FC accepts uplink commands ONLY in pad states (INIT/GROUND_IDLE/
    ARMED, telemetry.c on_pad()); in flight it is strictly TX-only.
  * there is no NAK packet: a missing ACK after ACK_TIMEOUT_S IS the NAK.
  * the test-fire passcode is never logged, never echoed in results.
"""

import queue
import threading
import time

from . import schema

ACK_TIMEOUT_S = 2.5      # pad cadence 1 Hz + 300 ms RX window -> ~1-2 s
ECHO_TIMEOUT_S = 1.0     # GS prints the uplink echo right after TX
MAINALT_MIN_M = 30.0     # radio-path bound (fw: 30..2000); console would
MAINALT_MAX_M = 2000.0   # take 10..10000 but the UI enforces the stricter

PAD_STATES = (0, 1, 2)   # ST_INIT, ST_GROUND_IDLE, ST_ARMED (on_pad())
GROUND_STATES = (0, 1, 9)          # where maintenance cmds make sense

ARM_REFUSAL = {          # fsm_request_arm() codes (state_machine.c)
    -1: "not in GROUND_IDLE",
    -2: "IMU/baro unhealthy",
    -3: "gyro cal not done (run cal, keep still ~2 s)",
    -4: "moving (|vel| >= 2 m/s)",
    -5: "tilt/accel implausible (accel_sat or | |a|-1g | >= 0.5 g)",
}


class Capability:
    def __init__(self, name, console=True, gs=False, states=PAD_STATES,
                 needs_disarmed=False, needs_testen=False, drops_port=False,
                 confirm=False, danger=False):
        self.name = name
        self.console = console
        self.gs = gs
        self.states = states
        self.needs_disarmed = needs_disarmed
        self.needs_testen = needs_testen
        self.drops_port = drops_port     # reply-then-port-drop is success
        self.confirm = confirm           # UI must confirm before sending
        self.danger = danger             # UI renders as a guarded control

    def to_dict(self):
        return {"name": self.name, "console": self.console, "gs": self.gs,
                "states": list(self.states),
                "needs_disarmed": self.needs_disarmed,
                "needs_testen": self.needs_testen,
                "drops_port": self.drops_port, "confirm": self.confirm,
                "danger": self.danger}


CAPABILITIES = {c.name: c for c in (
    Capability("arm", gs=True, states=(1,), needs_disarmed=True,
               confirm=True, danger=True),
    Capability("disarm", gs=True),                    # never refused
    Capability("zero", gs=True, states=PAD_STATES),
    Capability("cal", states=(0, 1)),
    Capability("mainalt", gs=True, states=(0, 1, 9)),  # fw NAKs while armed
    Capability("testen", states=(1,)),
    Capability("fire", gs=True, states=(1,), needs_disarmed=True,
               needs_testen=True, confirm=True, danger=True),
    Capability("servo", states=(0, 1)),
    Capability("log", states=GROUND_STATES),
    Capability("nop", gs=True),
    Capability("reboot", gs=True, states=GROUND_STATES, drops_port=True,
               confirm=True),
    Capability("bootloader", states=(1,), needs_disarmed=True,
               drops_port=True, confirm=True),
    Capability("wdtest", states=(0, 1), needs_disarmed=True,
               drops_port=True, confirm=True),
)}


class CmdResult:
    def __init__(self, ok, detail="", refusal=None, nak=False):
        self.ok = ok
        self.detail = detail      # verbatim firmware/GS output (redacted)
        self.refusal = refusal    # decoded human reason, when known
        self.nak = nak            # GS path: inferred NAK (no ACK)

    def to_dict(self):
        return {"ok": self.ok, "detail": self.detail,
                "refusal": self.refusal, "nak": self.nak}


def _redact(text):
    """Strip anything that looks like the test-fire passcode from a
    command echo: 'fire <ch> <code>' -> 'fire <ch> ****'."""
    out = []
    for line in (text or "").splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0].lower().endswith("fire"):
            parts[2] = "****"
            line = " ".join(parts)
        out.append(line)
    return "\n".join(out)


def _gate_check(cap, latest, transport):
    """Mirror the firmware gates; returns a refusal string or None."""
    if transport == "console" and not cap.console:
        return "%s is not a console command" % cap.name
    if transport == "gs" and not cap.gs:
        return "%s is console-only — connect over USB" % cap.name
    st = latest.get("state")
    if st is None:
        return "no telemetry yet — state unknown"
    if st not in PAD_STATES and transport == "gs":
        return "FC is TX-only in flight — commands disabled"
    if st not in cap.states:
        return "%s not allowed in %s" % (cap.name, schema.state_name(st))
    if cap.needs_disarmed and latest.get("armed"):
        return "%s requires DISARMED" % cap.name
    if cap.needs_testen and latest.get("testen") is False:
        return "test mode is off (testen on first)"
    # testen None (unknown over radio) is allowed through — firmware gates.
    return None


# ============================================================
# Console adapter
# ============================================================
_OK_MARKERS = {
    "arm": ("ARMED",),
    "disarm": ("disarmed",),
    "zero": ("baro zeroed",),
    "cal": ("gyro cal started",),
    "testen": ("test mode ON", "test mode off", "testen:"),
    "fire": ("fire pulse started",),
    "mainalt": ("saved to flash", "main_alt:"),
    "servo": ("servo ",),
    "log": ("log: started", "log: closed", "log: running",
            "log: not running", "log: already running"),
    "nop": (),
    "reboot": ("rebooting",),
    "bootloader": ("byebye",),
    "wdtest": ("wdtest: halting",),
}
_FAIL_MARKERS = ("DENIED:", "refused", "err:", "usage:", "FAILED",
                 "unknown cmd")


class ConsoleCommandAdapter:
    """Runs UI commands over the FC USB console (FcConsoleSource.execute).
    One outstanding command at a time; replies classified against the
    verbatim firmware strings and surfaced unmodified (redacted)."""

    def __init__(self, source):
        self.src = source
        self._busy = threading.Lock()

    # -- command text builders (console dialect) --
    @staticmethod
    def _line(name, args):
        if name == "fire":
            return "fire %d %s" % (int(args["ch"]), args["passcode"])
        if name == "mainalt":
            return "mainalt %g" % float(args["m"])
        if name == "testen":
            return "testen %s" % ("on" if args.get("on") else "off")
        if name == "servo":
            return "servo %d %d" % (int(args["ch"]), int(args["us"]))
        if name == "log":
            return "log %s" % args.get("op", "stat")
        return name

    def send(self, name, args=None, timeout=6.0):
        args = dict(args or {})
        cap = CAPABILITIES.get(name)
        if cap is None:
            return CmdResult(False, refusal="unknown command %r" % name)
        latest = self.src.drain()["latest"]
        refusal = _gate_check(cap, latest, "console")
        if refusal is None:
            refusal = _validate_args(name, args)
        if refusal is not None:
            self.src.push_event("refusal", "%s: %s" % (name, refusal),
                                severity="warn")
            return CmdResult(False, refusal=refusal)

        if not self._busy.acquire(blocking=False):
            return CmdResult(False, refusal="another command is in flight")
        try:
            reply = self.src.execute(self._line(name, args), timeout=timeout)
        finally:
            self._busy.release()

        if reply is None or reply == "":
            if cap.drops_port:
                # reboot/bootloader/wdtest: echo may be lost in the drop
                return CmdResult(True, detail="(port dropped as expected)")
            self.src.push_event("nak", "%s: no reply (port down?)" % name,
                                severity="warn")
            return CmdResult(False, refusal="no reply from FC",
                             nak=True)

        detail = _redact(reply).strip()
        ok = any(m in reply for m in _OK_MARKERS.get(name, ()))
        failed = any(m in reply for m in _FAIL_MARKERS)
        if name == "mainalt" and "flash save FAILED" in reply:
            ok, failed = False, True
        if cap.drops_port and not failed:
            ok = True
        if ok and not failed:
            return CmdResult(True, detail=detail)

        refusal = _decode_refusal(name, reply)
        self.src.push_event("refusal", "%s: %s" % (name, refusal or detail),
                            severity="warn")
        return CmdResult(False, detail=detail, refusal=refusal)


def _validate_args(name, args):
    if name == "mainalt":
        try:
            m = float(args["m"])
        except (KeyError, TypeError, ValueError):
            return "mainalt needs a numeric altitude"
        if not (MAINALT_MIN_M <= m <= MAINALT_MAX_M):
            return ("main altitude %.10g m outside %g..%g m"
                    % (m, MAINALT_MIN_M, MAINALT_MAX_M))
    if name == "fire":
        try:
            ch = int(args["ch"])
        except (KeyError, TypeError, ValueError):
            return "fire needs a channel"
        if not (1 <= ch <= schema.NUM_PYRO):
            return "channel %d outside 1..%d" % (ch, schema.NUM_PYRO)
        code = str(args.get("passcode", ""))
        if not (1 <= len(code) <= 4) or any(
                c not in "0123456789abcdefABCDEF" for c in code):
            return "passcode must be 1-4 hex digits"
    if name == "servo":
        try:
            ch, us = int(args["ch"]), int(args["us"])
        except (KeyError, TypeError, ValueError):
            return "servo needs channel + pulse"
        if not (1 <= ch <= 4):
            return "servo channel 1..4"
        if not (500 <= us <= 2500):
            return "servo pulse 500..2500 us"
    return None


def _decode_refusal(name, reply):
    for line in reply.splitlines():
        line = line.strip()
        if line.startswith("DENIED:") or line.startswith("err:") \
                or line.startswith("usage:") or "FAILED" in line:
            return line
        if "arm refused" in line:
            import re
            m = re.search(r"arm refused \((-\d+)\)", line)
            if m:
                code = int(m.group(1))
                why = ARM_REFUSAL.get(code, "?")
                return "%s — %s" % (line, why)
            return line
    return None


# ============================================================
# GS adapter
# ============================================================
class GsCommandAdapter:
    """Runs UI commands over the GS radio uplink.

    Sequence per send: stdin line -> {"uplink":..} echo (TX confirmation;
    tx_status must be 0) -> PKT_ACK frame within ack_timeout -> success.
    No ACK = inferred NAK = exactly ONE retry (the GS bumps its own
    random-seeded nonce per TX, so the retry is replay-safe)."""

    def __init__(self, source, ack_timeout=ACK_TIMEOUT_S,
                 echo_timeout=ECHO_TIMEOUT_S):
        self.src = source
        self.ack_timeout = ack_timeout
        self.echo_timeout = echo_timeout
        self._busy = threading.Lock()
        self._q = queue.Queue()
        source.listeners.append(self._on_ingest)

    def _on_ingest(self, kind, obj, t_host):
        if kind in ("uplink_echo", "ack"):
            self._q.put((kind, obj))

    # -- command text builders (GS stdin dialect) --
    @staticmethod
    def _line(name, args):
        if name == "fire":
            # UI is 1-indexed; the GS stdin grammar is 0-indexed
            return "fire %d %s" % (int(args["ch"]) - 1, args["passcode"])
        if name == "mainalt":
            return "mainalt %g" % float(args["m"])
        return name

    def _drain_stale(self):
        try:
            while True:
                self._q.get_nowait()
        except queue.Empty:
            pass

    def _await(self, kind, timeout):
        deadline = time.monotonic() + timeout
        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                return None
            try:
                k, obj = self._q.get(timeout=remain)
            except queue.Empty:
                return None
            if k == kind:
                return obj

    def _attempt(self, name, line):
        self._drain_stale()
        if not self.src.send_line(line):
            return CmdResult(False, refusal="GS port down")
        echo = self._await("uplink_echo", self.echo_timeout)
        if echo is None:
            return CmdResult(False, refusal="no uplink echo from GS",
                             nak=True)
        if echo.get("tx_status") not in (0, None):
            return CmdResult(False, nak=True,
                             refusal="GS TX failed (tx_status=%s)"
                             % echo.get("tx_status"))
        ack = self._await("ack", self.ack_timeout)
        if ack is None:
            return CmdResult(False, nak=True,
                             refusal="no ACK within %.1fs" % self.ack_timeout)
        return CmdResult(True,
                         detail="ACK (state %s)"
                         % ack.get("state_name", "?"))

    def send(self, name, args=None):
        args = dict(args or {})
        cap = CAPABILITIES.get(name)
        if cap is None:
            return CmdResult(False, refusal="unknown command %r" % name)
        latest = self.src.drain()["latest"]
        refusal = _gate_check(cap, latest, "gs")
        if refusal is None:
            refusal = _validate_args(name, args)
        if refusal is not None:
            self.src.push_event("refusal", "%s: %s" % (name, refusal),
                                severity="warn")
            return CmdResult(False, refusal=refusal)

        if not self._busy.acquire(blocking=False):
            return CmdResult(False, refusal="another command is in flight")
        try:
            line = self._line(name, args)
            res = self._attempt(name, line)
            if res.nak and name not in ("reboot",):
                # exactly one retry with a fresh GS nonce
                self.src.push_event("nak",
                                    "%s: %s — retrying once"
                                    % (name, res.refusal), severity="warn")
                res = self._attempt(name, line)
            if res.nak:
                note = ""
                if name == "mainalt":
                    note = (" (NAK may mean applied to RAM but flash save"
                            " failed, or refused while armed)")
                elif name == "fire":
                    note = (" (testen state is unknown over radio — enable"
                            " it via the USB console before pad ops)")
                self.src.push_event("nak", "%s: NAK%s" % (name, note),
                                    severity="warn")
                res.refusal = (res.refusal or "NAK") + note
            elif res.ok:
                self.src.push_event("ack", "%s: ACK" % name)
            return res
        finally:
            self._busy.release()
