#!/usr/bin/env python3
"""Unit tests for tools/deck/commands.py — capability matrix, gate
mirroring, argument validation, verbatim refusal decode, fire-channel
index translation (console 1-indexed vs GS 0-indexed), ACK/NAK/retry
with fresh nonces, and passcode redaction. No hardware, no threads."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "tools"))

from deck import commands, schema
from deck.commands import (CAPABILITIES, CmdResult, ConsoleCommandAdapter,
                           GsCommandAdapter)


PAD_LATEST = {"state": 1, "state_name": "GROUND_IDLE", "armed": False,
              "testen": True}


class FakeConsoleSource:
    def __init__(self, latest=None, replies=None):
        self.latest = dict(PAD_LATEST if latest is None else latest)
        self.replies = replies or {}
        self.executed = []
        self.events = []

    def drain(self, since=-1.0, event_seq=-1):
        return {"latest": dict(self.latest)}

    def execute(self, line, timeout=5.0):
        self.executed.append(line)
        for prefix, reply in self.replies.items():
            if line.startswith(prefix):
                return line + "\r\n" + reply
        return line + "\r\nunknown cmd — try 'help'\r\n"

    def push_event(self, kind, text, severity="info"):
        self.events.append((kind, text, severity))


class FakeGsSource:
    """send_line() synchronously feeds the scripted responses to every
    listener, so adapter._await finds them already queued."""

    def __init__(self, latest=None, script=None, port_up=True):
        self.latest = dict(PAD_LATEST if latest is None else latest)
        self.script = list(script or [])
        self.port_up = port_up
        self.listeners = []
        self.sent = []
        self.events = []

    def drain(self, since=-1.0, event_seq=-1):
        return {"latest": dict(self.latest)}

    def send_line(self, line):
        if not self.port_up:
            return False
        self.sent.append(line)
        responses = self.script.pop(0) if self.script else []
        for kind, obj in responses:
            for fn in self.listeners:
                fn(kind, obj, 0.0)
        return True

    def push_event(self, kind, text, severity="info"):
        self.events.append((kind, text, severity))


def echo(nonce=5, tx_status=0, name="CMD_ARM"):
    return ("uplink_echo", {"uplink": name, "cmd": 1, "arg": 0,
                            "nonce": nonce, "tx_status": tx_status})


def ack(state_name="GROUND_IDLE"):
    return ("ack", {"type": schema.PKT_ACK, "state": 1,
                    "state_name": state_name})


def gs_adapter(**kw):
    src = FakeGsSource(**kw)
    return GsCommandAdapter(src, ack_timeout=0.05, echo_timeout=0.05), src


# ============================================================
# Console adapter
# ============================================================
class TestConsoleAdapter(unittest.TestCase):
    def test_arm_ok(self):
        src = FakeConsoleSource(replies={"arm": "ARMED\r\n"})
        res = ConsoleCommandAdapter(src).send("arm")
        self.assertTrue(res.ok)
        self.assertEqual(src.executed, ["arm"])

    def test_arm_refusal_decoded(self):
        src = FakeConsoleSource(replies={
            "arm": "arm refused (-3): need healthy sensors + still on pad\r\n"})
        res = ConsoleCommandAdapter(src).send("arm")
        self.assertFalse(res.ok)
        self.assertIn("gyro cal", res.refusal)

    def test_state_lockout_blocks_before_execute(self):
        src = FakeConsoleSource(latest={"state": 3, "state_name": "BOOST",
                                        "armed": True, "testen": False})
        res = ConsoleCommandAdapter(src).send("arm")
        self.assertFalse(res.ok)
        self.assertIn("not allowed in BOOST", res.refusal)
        self.assertEqual(src.executed, [])
        self.assertTrue(any(k == "refusal" for k, _, _ in src.events))

    def test_mainalt_bounds_enforced_locally(self):
        adapter = ConsoleCommandAdapter(FakeConsoleSource())
        for bad in (29.9, 2000.1, "x"):
            res = adapter.send("mainalt", {"m": bad})
            self.assertFalse(res.ok, bad)
        self.assertEqual(adapter.src.executed, [])

    def test_mainalt_ok_and_flash_fail(self):
        src = FakeConsoleSource(replies={
            "mainalt": "main_alt = 150.00 m\r\nsaved to flash\r\n"})
        res = ConsoleCommandAdapter(src).send("mainalt", {"m": 150})
        self.assertTrue(res.ok)
        self.assertIn("saved to flash", res.detail)

        src = FakeConsoleSource(replies={
            "mainalt": "main_alt = 150.00 m\r\nflash save FAILED (-3)\r\n"})
        res = ConsoleCommandAdapter(src).send("mainalt", {"m": 150})
        self.assertFalse(res.ok)
        self.assertIn("FAILED", res.refusal)

    def test_fire_denied_verbatim(self):
        src = FakeConsoleSource(replies={"fire": "DENIED: bad passcode\r\n"})
        res = ConsoleCommandAdapter(src).send(
            "fire", {"ch": 1, "passcode": "dead"})
        self.assertFalse(res.ok)
        self.assertEqual(res.refusal, "DENIED: bad passcode")

    def test_fire_ok_is_redacted(self):
        src = FakeConsoleSource(replies={
            "fire": "TEST FIRE ch1 (cont=0x01)...\r\n"
                    "fire pulse started (1000 ms)\r\n"})
        res = ConsoleCommandAdapter(src).send(
            "fire", {"ch": 1, "passcode": "52A5"})
        self.assertTrue(res.ok)
        self.assertEqual(src.executed, ["fire 1 52A5"])   # wire keeps code
        self.assertNotIn("52A5", res.detail)              # result never
        self.assertIn("****", res.detail)

    def test_fire_needs_testen(self):
        src = FakeConsoleSource(latest=dict(PAD_LATEST, testen=False))
        res = ConsoleCommandAdapter(src).send(
            "fire", {"ch": 1, "passcode": "52A5"})
        self.assertFalse(res.ok)
        self.assertIn("test mode", res.refusal)
        self.assertEqual(src.executed, [])

    def test_fire_channel_and_passcode_validation(self):
        adapter = ConsoleCommandAdapter(FakeConsoleSource())
        self.assertFalse(adapter.send("fire", {"ch": 2,
                                               "passcode": "52A5"}).ok)
        self.assertFalse(adapter.send("fire", {"ch": 1,
                                               "passcode": "zzzz"}).ok)
        self.assertFalse(adapter.send("fire", {"ch": 1,
                                               "passcode": "12345"}).ok)

    def test_servo_validation(self):
        adapter = ConsoleCommandAdapter(FakeConsoleSource(
            replies={"servo": "servo 2 -> 1500 us\r\n"}))
        self.assertTrue(adapter.send("servo", {"ch": 2, "us": 1500}).ok)
        self.assertFalse(adapter.send("servo", {"ch": 5, "us": 1500}).ok)
        self.assertFalse(adapter.send("servo", {"ch": 1, "us": 300}).ok)

    def test_power_ok(self):
        src = FakeConsoleSource(replies={"tx power": "tx power = 17 dBm\r\n"})
        res = ConsoleCommandAdapter(src).send("power", {"dbm": 17})
        self.assertTrue(res.ok)
        self.assertEqual(src.executed, ["tx power 17"])

    def test_power_bounds_enforced_locally(self):
        adapter = ConsoleCommandAdapter(FakeConsoleSource())
        for bad in (-10, 23, "x"):
            self.assertFalse(adapter.send("power", {"dbm": bad}).ok, bad)
        self.assertEqual(adapter.src.executed, [])

    def test_power_over_gs(self):
        adapter, src = gs_adapter(script=[[echo(name="SET_TX_POWER"), ack()]])
        res = adapter.send("power", {"dbm": 17})
        self.assertTrue(res.ok)
        self.assertEqual(src.sent, ["power 17"])    # GS stdin grammar

    def test_sleep_ok_and_line(self):
        src = FakeConsoleSource(replies={
            "sleep": "sleeping (STOP) ~30 s — USB drops; wakes on timer or RESET\r\n"})
        res = ConsoleCommandAdapter(src).send("sleep", {"secs": 30})
        self.assertTrue(res.ok)
        self.assertEqual(src.executed, ["sleep 30"])

    def test_sleep_default_is_until_reset(self):
        src = FakeConsoleSource(replies={"sleep": "sleeping (STOP) — press RESET\r\n"})
        res = ConsoleCommandAdapter(src).send("sleep", {})
        self.assertTrue(res.ok)
        self.assertEqual(src.executed, ["sleep 0"])       # 0 = until RESET

    def test_sleep_bounds_and_port_drop_ok(self):
        adapter = ConsoleCommandAdapter(FakeConsoleSource())
        self.assertFalse(adapter.send("sleep", {"secs": 40000}).ok)
        self.assertEqual(adapter.src.executed, [])
        # port drops before any reply -> drops_port makes it a success
        src = FakeConsoleSource()
        src.execute = lambda line, timeout=5.0: None
        res = ConsoleCommandAdapter(src).send("sleep", {"secs": 10})
        self.assertTrue(res.ok)
        self.assertIn("port dropped", res.detail)

    def test_sleep_is_console_only(self):
        adapter, src = gs_adapter()
        res = adapter.send("sleep", {"secs": 10})
        self.assertFalse(res.ok)
        self.assertIn("console-only", res.refusal)

    def test_drops_port_commands_tolerate_lost_reply(self):
        src = FakeConsoleSource(latest=dict(PAD_LATEST),
                                replies={})
        src.execute = lambda line, timeout=5.0: None   # port dropped
        res = ConsoleCommandAdapter(src).send("reboot")
        self.assertTrue(res.ok)
        self.assertIn("port dropped", res.detail)

    def test_busy_lock(self):
        adapter = ConsoleCommandAdapter(FakeConsoleSource(
            replies={"arm": "ARMED\r\n"}))
        adapter._busy.acquire()
        try:
            res = adapter.send("arm")
        finally:
            adapter._busy.release()
        self.assertFalse(res.ok)
        self.assertIn("in flight", res.refusal)


# ============================================================
# GS adapter
# ============================================================
class TestGsAdapter(unittest.TestCase):
    def test_success_echo_then_ack(self):
        adapter, src = gs_adapter(script=[[echo(nonce=41), ack()]])
        res = adapter.send("arm")
        self.assertTrue(res.ok)
        self.assertEqual(src.sent, ["arm"])
        self.assertIn("ACK", res.detail)

    def test_nak_then_retry_succeeds_with_fresh_nonce(self):
        adapter, src = gs_adapter(script=[[echo(nonce=41)],
                                          [echo(nonce=42), ack()]])
        res = adapter.send("arm")
        self.assertTrue(res.ok)
        self.assertEqual(len(src.sent), 2)          # exactly one retry
        self.assertTrue(any(k == "nak" for k, _, _ in src.events))

    def test_double_nak_gives_up_with_note(self):
        adapter, src = gs_adapter(script=[[echo()], [echo()]])
        res = adapter.send("mainalt", {"m": 150})
        self.assertFalse(res.ok)
        self.assertTrue(res.nak)
        self.assertEqual(len(src.sent), 2)
        self.assertIn("flash save", res.refusal)    # mainalt NAK ambiguity

    def test_tx_status_failure(self):
        adapter, src = gs_adapter(script=[[echo(tx_status=-1)],
                                          [echo(tx_status=-1)]])
        res = adapter.send("arm")
        self.assertFalse(res.ok)
        self.assertIn("tx_status", res.refusal)

    def test_no_uplink_echo(self):
        adapter, src = gs_adapter(script=[[], []])
        res = adapter.send("arm")
        self.assertFalse(res.ok)
        self.assertTrue(res.nak)
        self.assertEqual(len(src.sent), 2)

    def test_fire_is_zero_indexed_on_gs(self):
        adapter, src = gs_adapter(script=[[echo(name="CMD_TEST_FIRE"),
                                           ack()]])
        res = adapter.send("fire", {"ch": 1, "passcode": "52A5"})
        self.assertTrue(res.ok)
        self.assertEqual(src.sent, ["fire 0 52A5"])

    def test_fire_allowed_when_testen_unknown_over_radio(self):
        latest = dict(PAD_LATEST)
        del latest["testen"]                        # unknown over radio
        adapter, src = gs_adapter(latest=latest,
                                  script=[[echo(), ack()]])
        res = adapter.send("fire", {"ch": 1, "passcode": "52A5"})
        self.assertTrue(res.ok)                     # firmware is the gate

    def test_in_flight_lockout(self):
        adapter, src = gs_adapter(latest={"state": 4, "state_name": "COAST",
                                          "armed": True})
        res = adapter.send("disarm")
        self.assertFalse(res.ok)
        self.assertIn("TX-only in flight", res.refusal)
        self.assertEqual(src.sent, [])

    def test_console_only_commands_refused_on_gs(self):
        adapter, src = gs_adapter()
        for name in ("testen", "servo", "log", "cal", "wdtest",
                     "bootloader"):
            res = adapter.send(name, {"ch": 1, "us": 1500, "on": True})
            self.assertFalse(res.ok, name)
            self.assertIn("console-only", res.refusal)
        self.assertEqual(src.sent, [])

    def test_port_down(self):
        adapter, src = gs_adapter(port_up=False)
        res = adapter.send("arm")
        self.assertFalse(res.ok)
        self.assertIn("port down", res.refusal)

    def test_no_telemetry_yet(self):
        adapter, src = gs_adapter(latest={})
        res = adapter.send("arm")
        self.assertFalse(res.ok)
        self.assertIn("state unknown", res.refusal)


# ============================================================
# Capability matrix sanity
# ============================================================
class TestCapabilities(unittest.TestCase):
    def test_gs_set_matches_gs_stdin_grammar(self):
        gs_cmds = {n for n, c in CAPABILITIES.items() if c.gs}
        self.assertEqual(gs_cmds, {"arm", "disarm", "zero", "mainalt",
                                   "fire", "reboot", "nop", "power"})

    def test_danger_commands_require_confirm(self):
        for name in ("arm", "fire"):
            self.assertTrue(CAPABILITIES[name].confirm, name)
            self.assertTrue(CAPABILITIES[name].danger, name)

    def test_to_dict_shape(self):
        d = CAPABILITIES["fire"].to_dict()
        for key in ("name", "console", "gs", "states", "needs_disarmed",
                    "needs_testen", "confirm"):
            self.assertIn(key, d)


if __name__ == "__main__":
    unittest.main()
