#!/usr/bin/env python3
"""Unit tests for tools/deck/sources.py — LineIngest classification, the
synthetic flight profile, ReplaySource (jsonl/csv/synthetic), reboot
detection, malformed-line resilience, drain() incremental semantics, and
FcConsoleSource driven by a fake serial port + fake clock (no hardware,
no COM ports)."""

import json
import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "tools"))

import telem_decode as td
from deck import schema
from deck import sources
from deck.sources import (FcConsoleSource, LineIngest, ReplaySource,
                          gs_json_line, synthetic_profile)


def _telem_line(rssi=-70.0, snr=9.0, ptype=schema.PKT_TELEMETRY, **fields):
    d = td.parse_telem(td.build_telem(ptype=ptype, **fields))
    return gs_json_line(d, rssi, snr)


# ============================================================
# LineIngest
# ============================================================
class TestLineIngest(unittest.TestCase):
    def setUp(self):
        self.ing = LineIngest()

    def test_telem_event_ack_classified_by_type(self):
        for ptype, kind in ((schema.PKT_TELEMETRY, "telem"),
                            (schema.PKT_EVENT, "event"),
                            (schema.PKT_ACK, "ack")):
            k, obj = self.ing.feed(_telem_line(ptype=ptype, state=1))
            self.assertEqual(k, kind)
            self.assertEqual(obj["type"], ptype)

    def test_uplink_echo_boot_error(self):
        k, obj = self.ing.feed(
            '{"uplink":"CMD_ARM","cmd":1,"arg":0,"nonce":7,"tx_status":0}')
        self.assertEqual(k, "uplink_echo")
        self.assertEqual(obj["nonce"], 7)
        k, _ = self.ing.feed('{"gs":"boot","radio":"ok"}')
        self.assertEqual(k, "gs_boot")
        for err in ('{"error":"radio_init","status":-3}',
                    '{"error":"rx","status":-2,"len":0}',
                    '{"error":"crc16","len":46,"rssi":-88.5,"snr":4.25}',
                    '{"error":"bad_magic","len":46,"rssi":-88.5,"snr":4.25}'):
            k, obj = self.ing.feed(err)
            self.assertEqual(k, "gs_error")

    def test_cmd_echo(self):
        # a heard-back 14-byte command frame decoded by the GS
        k, obj = self.ing.feed(
            '{"magic":82,"version":1,"type":16,"cmd":4,"cmd_name":'
            '"CMD_SET_MAIN_ALT","arg":15000,"nonce":9,"crc16":123,'
            '"rssi":-60.0,"snr":9.5}')
        self.assertEqual(k, "cmd_echo")
        self.assertEqual(obj["cmd"], schema.CMD_SET_MAIN_ALT)

    def test_junk_never_raises(self):
        for bad in ("", "not json at all", "[1,2,3]", "42",
                    '{"type":99}', '{"no":"type"}'):
            k, _ = self.ing.feed(bad)
            self.assertEqual(k, "junk", bad)
        # empty line is junk but not counted as decode junk
        self.assertEqual(self.ing.junk, 5)

    def test_gs_serializer_roundtrip_matches_parse_telem(self):
        d = td.parse_telem(td.build_telem(
            state=4, t_ms=12345, lat_e7=391234567, lon_e7=-771234567,
            alt_baro_cm=39920, alt_gps_cm=51000, vel_up_cms=7500,
            accel_g_x100=(12, -8, 1600), gyro_dps_x10=(900, -35, 25),
            batt_mv=8123, pyro_cont=1, pyro_fired=0, sats=9,
            flags=schema.FLAG_GPS_FIX | schema.FLAG_ACCEL_SAT))
        k, obj = self.ing.feed(gs_json_line(d, -77.5, 6.25))
        self.assertEqual(k, "telem")
        for key in ("state", "t_ms", "lat_e7", "alt_baro_cm", "vel_up_cms",
                    "batt_mv", "flags", "crc16", "state_name"):
            self.assertEqual(obj[key], d[key], key)
        self.assertAlmostEqual(obj["alt_baro_m"], d["alt_baro_m"], places=2)
        self.assertAlmostEqual(obj["vel_up_ms"], d["vel_up_ms"], places=2)
        self.assertEqual(obj["accel_g_x100"], list(d["accel_g_x100"]))
        self.assertIs(obj["accel_sat"], True)
        self.assertEqual(obj["rssi"], -77.5)
        self.assertEqual(obj["snr"], 6.25)


# ============================================================
# Synthetic profile
# ============================================================
class TestSyntheticProfile(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lines = synthetic_profile("nominal")
        ing = LineIngest()
        cls.frames = []
        for ln in cls.lines:
            kind, obj = ing.feed(ln)
            cls.frames.append((kind, obj))

    def test_deterministic(self):
        self.assertEqual(self.lines, synthetic_profile("nominal"))

    def test_every_line_parses_as_gs_frame(self):
        self.assertTrue(all(k in ("telem", "event") for k, _ in self.frames))

    def test_one_event_frame_per_transition(self):
        events = [o for k, o in self.frames if k == "event"]
        # GROUND_IDLE->ARMED->BOOST->COAST->APOGEE->DROGUE->MAIN->DESCENT
        # ->LANDED = 8 transitions
        self.assertEqual(len(events), 8)
        seq = [e["state_name"] for e in events]
        self.assertEqual(seq, ["ARMED", "BOOST", "COAST", "APOGEE",
                               "DROGUE", "MAIN", "DESCENT", "LANDED"])

    def test_apogee_truth(self):
        alt = max(o["alt_baro_m"] for _, o in self.frames)
        self.assertAlmostEqual(alt, 399.2, delta=2.0)

    def test_max_velocity_truth(self):
        vel = max(o["vel_up_ms"] for _, o in self.frames)
        self.assertAlmostEqual(vel, 75.0, delta=1.5)

    def test_boost_is_railed_and_saturated(self):
        boost = [o for _, o in self.frames if o["state_name"] == "BOOST"]
        self.assertTrue(boost)
        for o in boost:
            self.assertEqual(o["accel_g_x100"][2], 1600)
            self.assertIs(o["accel_sat"], True)

    def test_battery_sag(self):
        volts = [o["batt_v"] for _, o in self.frames]
        self.assertAlmostEqual(volts[0], 8.2, delta=0.05)
        self.assertAlmostEqual(volts[-1], 7.4, delta=0.05)

    def test_rssi_fades_with_altitude(self):
        by_alt = sorted((o["alt_baro_m"], o["rssi"])
                        for _, o in self.frames)
        self.assertGreater(by_alt[0][1], by_alt[-1][1] + 15.0)

    def test_pyro_fired_from_main(self):
        for _, o in self.frames:
            if o["state"] >= 7:
                self.assertEqual(o["pyro_fired"], 0x01)
            else:
                self.assertEqual(o["pyro_fired"], 0x00)


# ============================================================
# ReplaySource
# ============================================================
class TestReplaySource(unittest.TestCase):
    def _run(self, **kw):
        src = ReplaySource(speed=1e9, **kw)
        src.run_sync()
        return src

    def test_synthetic_full_flight(self):
        src = self._run(synthetic="nominal")
        out = src.drain()
        self.assertFalse(out["connected"])       # replay finished
        alt_rows = out["series"]["alt"]
        self.assertTrue(alt_rows)
        self.assertAlmostEqual(max(r[1] for r in alt_rows), 399.2, delta=2.0)
        kinds = [e["kind"] for e in out["events"]]
        self.assertEqual(kinds.count("state"), 8)
        self.assertIn("conn", kinds)
        # replay clock = rebased t_ms: profile spans ~67 s
        self.assertAlmostEqual(out["now"], 67.0, delta=1.0)
        self.assertGreater(out["latest"]["pkt_rate"], 0)

    def test_drain_incremental_and_event_seq(self):
        src = self._run(synthetic="nominal")
        full = src.drain()
        t_last = full["series"]["alt"][-1][0]
        seq_last = full["events"][-1]["seq"]
        again = src.drain(since=t_last, event_seq=seq_last)
        self.assertEqual(again["series"]["alt"], [])
        self.assertEqual(again["events"], [])
        # a mid-point since returns only the tail
        t_mid = full["series"]["alt"][len(full["series"]["alt"]) // 2][0]
        tail = src.drain(since=t_mid)
        self.assertTrue(all(r[0] > t_mid for r in tail["series"]["alt"]))

    def test_jsonl_replay_and_reboot_detect(self):
        lines = [_telem_line(state=1, t_ms=100000),
                 _telem_line(state=1, t_ms=101000),
                 _telem_line(state=1, t_ms=2000),      # FC rebooted on tape
                 "this line is garbage {{{",
                 _telem_line(state=1, t_ms=3000)]
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "tape.jsonl")
            with open(path, "w", encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
            src = self._run(path=path)
        out = src.drain()
        kinds = [e["kind"] for e in out["events"]]
        self.assertIn("reboot", kinds)
        self.assertEqual(out["counters"].get("junk"), 1)
        # garbage never killed the replay: last frame ingested
        self.assertEqual(out["latest"]["t_ms"], 3000)

    def test_csv_replay(self):
        # session-telem.csv shape: JSON-literal cells (lists/bools/null)
        d0 = td.parse_telem(td.build_telem(state=4, t_ms=5000,
                                           alt_baro_cm=10000,
                                           vel_up_cms=2000, batt_mv=8000))
        row = {k: json.dumps(v) for k, v in d0.items()}
        row["rssi"] = "-80.0"
        row["snr"] = "5.0"
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "telem.csv")
            with open(path, "w", newline="", encoding="utf-8") as f:
                import csv as _csv
                w = _csv.DictWriter(f, fieldnames=list(row.keys()))
                w.writeheader()
                w.writerow(row)
            src = self._run(path=path)
        out = src.drain()
        self.assertEqual(out["latest"]["state_name"], "COAST")
        self.assertEqual(out["series"]["alt"][0][1], 100.0)

    def test_requires_exactly_one_input(self):
        with self.assertRaises(ValueError):
            ReplaySource()
        with self.assertRaises(ValueError):
            ReplaySource(path="x", synthetic="nominal")


# ============================================================
# FcConsoleSource — fake serial + fake clock, no hardware
# ============================================================
STATUS_REPLY = (b"status\r\n"
                b"state:   GROUND_IDLE  t=127825ms\r\n"
                b"armed:   0  testen: 1  main_alt: 150.00 m\r\n"
                b"alt:     0.33 m AGL  vel: 0.00 m/s\r\n"
                b"batt:    8057 mV\r\n"
                b"flags:   gps_fix=0 accel_sat=0\r\n"
                b"health:  imu=1 baro=1 sd=1 radio=1 chg=1 gps=1\r\n"
                b"pyro:    cont=0x01 fired=0x00 sense=14168 mV\r\n")
SENSORS_REPLY = (b"sensors\r\n"
                 b"imu:  ax=-0.03 ay=0.10 az=0.99 g  sat=0\r\n"
                 b"      gx=-0.11 gy=0.05 gz=-0.01 dps\r\n"
                 b"baro: 99468.72 Pa  32.11 C  alt=-10.93 m\r\n")
GPS_REPLY = (b"gps\r\n"
             b"gps: fix=1 sats=9\r\n"
             b"     lat=39.1234567 lon=-77.1234567 alt=120.50 m MSL\r\n"
             b"     last_fix=250 ms ago\r\n")
RADIO_REPLY = (b"radio\r\n"
               b"radio: ok  tcxo: yes\r\n"
               b"  init err: tcxo=0 xtal=0 (0=ok; -1 rst/busy, -3 no chip,"
               b" -7/-21 dev errors)\r\n"
               b"  reinit attempts: 2  tx timeouts: 5\r\n"
               b"  chip irq status: 0x0000 (bit0 TxDone, bit1 RxDone,"
               b" bit9 Timeout)\r\n"
               b"  BUSY pin: 0  DIO1 pin: 0\r\n")
CHARGE_REPLY = (b"charge\r\n"
                b"charger: vbat=8057 mV stat=0x04 charging=1 fault=0\r\n")
LOG_REPLY = (b"log stat\r\n"
             b"log: running\r\n"
             b"  drops: 3  ring high-water: 4080/65484 bytes\r\n")
MAINALT_REPLY = (b"mainalt 150\r\n"
                 b"main_alt = 150.00 m\r\n"
                 b"saved to flash\r\n")
BOOT_STATUS_REPLY = (b"\r\n=== FC boot ===\r\n"
                     b"reset: IWDG PIN\r\n" + STATUS_REPLY)


class FakeClock:
    """Monotonic fake clock: every call advances a little so quiet/timeout
    loops always terminate deterministically."""

    def __init__(self, t=1000.0, step=0.005):
        self.t = t
        self.step = step

    def __call__(self):
        self.t += self.step
        return self.t


class FakePort:
    def __init__(self, replies):
        self.replies = replies
        self.buf = b""
        self.closed = False
        self.written = []

    def reset_input_buffer(self):
        self.buf = b""

    def write(self, data):
        line = data.decode().strip()
        self.written.append(line)
        for prefix, reply in self.replies.items():
            if line.startswith(prefix):
                self.buf += reply
                return
        self.buf += (line + "\r\nunknown cmd '%s' \xe2\x80\x94 try 'help'\r\n"
                     % line.split()[0]).encode("utf-8", "replace") \
            if line else b""

    def read(self, n):
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def close(self):
        self.closed = True


REPLIES = {"status": STATUS_REPLY, "sensors": SENSORS_REPLY,
           "gps": GPS_REPLY, "radio": RADIO_REPLY, "charge": CHARGE_REPLY,
           "log stat": LOG_REPLY, "mainalt": MAINALT_REPLY}


def make_src(replies=REPLIES):
    clock = FakeClock()
    port = FakePort(replies)
    src = FcConsoleSource("FAKE", clock=clock,
                          serial_factory=lambda com: port)
    return src, port, clock


class TestFcConsoleLadder(unittest.TestCase):
    """Drive _service() synchronously — deterministic ladder behavior."""

    def _quiesce(self, src, clock):
        """Mark every cadence timer freshly-served."""
        now = clock.t
        src._last_status = src._last_gps = src._last_radio = now
        src._last_charge = src._last_log = now

    def test_ladder_priority_order(self):
        src, port, clock = make_src()
        port2 = port
        # all timers stale -> status first, then gps, radio, charge, log,
        # then sensors flat-out
        order = [src._service(port2) for _ in range(7)]
        self.assertEqual(order[:5], ["status", "gps", "radio", "charge",
                                     "log"])
        self.assertEqual(order[5:], ["sensors", "sensors"])

    def test_status_parse_full(self):
        src, port, clock = make_src()
        src._service(port)                       # status
        latest = src.drain()["latest"]
        self.assertEqual(latest["state_name"], "GROUND_IDLE")
        self.assertEqual(latest["state"], 1)
        self.assertEqual(latest["t_ms"], 127825)
        self.assertIs(latest["armed"], False)
        self.assertIs(latest["testen"], True)
        self.assertEqual(latest["main_alt_m"], 150.0)
        self.assertEqual(latest["alt_baro_m"], 0.33)
        self.assertEqual(latest["batt_mv"], 8057)
        self.assertIs(latest["gps_fix"], False)
        for k in ("imu_ok", "baro_ok", "sd_ok", "radio_ok", "chg_ok",
                  "gps_ok"):
            self.assertIs(latest[k], True, k)
        self.assertEqual(latest["pyro_sense_mv"], 14168)

    def test_sensors_gps_radio_charge_log_parse(self):
        src, port, clock = make_src()
        for _ in range(6):
            src._service(port)
        latest = src.drain()["latest"]
        self.assertEqual(latest["accel_g"], [-0.03, 0.10, 0.99])
        self.assertEqual(latest["gyro_dps"], [-0.11, 0.05, -0.01])
        self.assertEqual(latest["press_pa"], 99468.72)
        self.assertEqual(latest["temp_c"], 32.11)
        self.assertEqual(latest["lat_deg"], 39.1234567)
        self.assertEqual(latest["lon_deg"], -77.1234567)
        self.assertEqual(latest["alt_gps_m"], 120.5)
        self.assertEqual(latest["gps_last_fix_ms"], 250)
        self.assertIs(latest["radio_ok"], True)
        self.assertEqual(latest["radio_tcxo"], "yes")
        self.assertEqual(latest["radio_reinit"], 2)
        self.assertEqual(latest["radio_txto"], 5)
        self.assertEqual(latest["chg_stat"], 0x04)
        self.assertIs(latest["chg_charging"], True)
        self.assertIs(latest["log_running"], True)
        self.assertEqual(latest["log_drops"], 3)
        self.assertEqual(latest["log_ring_hw"], 4080)
        self.assertEqual(latest["log_ring_cap"], 65484)
        series = src.drain()["series"]
        self.assertTrue(series["accel"])
        self.assertTrue(series["alt"])
        self.assertIsNone(series["alt"][0][2])   # no GPS column on console

    def test_execute_runs_first_and_captures_reply(self):
        src, port, clock = make_src()
        self._quiesce(src, clock)
        pending = src.submit("mainalt 150")
        kind = src._service(port)
        self.assertEqual(kind, "cmd")
        self.assertTrue(pending.done.is_set())
        self.assertIn("saved to flash", pending.reply)

    def test_fire_passcode_redacted_in_events(self):
        src, port, clock = make_src()
        self._quiesce(src, clock)
        src.submit("fire 1 52A5")
        src._service(port)
        texts = [e["text"] for e in src.drain()["events"]
                 if e["kind"] == "cmd_sent"]
        self.assertTrue(texts)
        self.assertIn("fire 1 ****", texts[0])
        self.assertNotIn("52A5", " ".join(texts))

    def test_reboot_via_tms_decrease_and_boot_banner(self):
        src, port, clock = make_src()
        src._service(port)                       # t_ms = 127825
        port.replies = dict(REPLIES)
        port.replies["status"] = BOOT_STATUS_REPLY.replace(
            b"t=127825ms", b"t=1500ms")
        src._last_status = 0.0                   # force another status
        src._service(port)
        out = src.drain()
        kinds = [e["kind"] for e in out["events"]]
        self.assertIn("reboot", kinds)
        self.assertEqual(out["latest"]["reset_cause"], "IWDG PIN")

    def test_state_transition_event(self):
        src, port, clock = make_src()
        src._service(port)                       # GROUND_IDLE
        port.replies = dict(REPLIES)
        port.replies["status"] = STATUS_REPLY.replace(
            b"GROUND_IDLE  t=127825ms", b"ARMED  t=130000ms")
        src._last_status = 0.0
        src._service(port)
        ev = [e for e in src.drain()["events"] if e["kind"] == "state"]
        self.assertTrue(ev)
        self.assertIn("GROUND_IDLE -> ARMED", ev[-1]["text"])


class TestFcConsoleThreaded(unittest.TestCase):
    """One end-to-end lifecycle pass through the real thread."""

    def test_start_poll_stop_closes_port(self):
        port = FakePort(REPLIES)
        src = FcConsoleSource("FAKE", serial_factory=lambda com: port)
        src.start()
        try:
            deadline = time.time() + 3.0
            while time.time() < deadline:
                out = src.drain()
                if out["connected"] and out["series"]["accel"]:
                    break
                time.sleep(0.02)
            out = src.drain()
            self.assertTrue(out["connected"])
            self.assertTrue(out["series"]["accel"])
        finally:
            src.stop()
        self.assertTrue(port.closed)
        self.assertFalse(src.connected)


# ============================================================
# schema.normalize_telem
# ============================================================
class TestNormalize(unittest.TestCase):
    def test_rows_shape_and_nullable_rssi(self):
        d = td.parse_telem(td.build_telem(state=3, alt_baro_cm=1000,
                                          vel_up_cms=500, batt_mv=8000))
        latest, rows = schema.normalize_telem(d, 12.3456, "gs")
        self.assertEqual(latest["t_host"], 12.346)
        self.assertEqual(latest["src"], "gs")
        self.assertIsNone(latest["rssi"])
        self.assertEqual(rows["alt"][:2], [12.346, 10.0])
        self.assertEqual(len(rows["accel"]), 4)
        self.assertNotIn("rssi", rows)           # no row when frame has none


if __name__ == "__main__":
    unittest.main()
