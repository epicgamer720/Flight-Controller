#!/usr/bin/env python3
"""Server tests: /data incremental semantics + peaks/tplus, /events,
/cmd token auth, /record toggle, /replay control, static serving with
token injection, run against a real ThreadingHTTPServer on an ephemeral
port, fed by the synthetic replay source."""

import http.client
import json
import os
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "tools"))

from deck.server import Deck, make_server


class ServerFixture:
    def __init__(self, ui_dir, record=False, recdir=None):
        self.deck = Deck(recdir=recdir or "recordings", record=record)
        self.srv = make_server(self.deck, ui_dir, port=0)   # ephemeral
        self.port = self.srv.server_address[1]
        self.thread = threading.Thread(target=self.srv.serve_forever,
                                       daemon=True)
        self.thread.start()

    def request(self, method, path, body=None, token=None):
        conn = http.client.HTTPConnection("127.0.0.1", self.port,
                                          timeout=10)
        headers = {"Content-Type": "application/json"}
        if token:
            headers["X-Deck-Token"] = token
        conn.request(method, path,
                     json.dumps(body) if body is not None else None,
                     headers)
        resp = conn.getresponse()
        data = resp.read()
        conn.close()
        try:
            return resp.status, json.loads(data)
        except ValueError:
            return resp.status, data

    def close(self):
        self.deck.shutdown()
        self.srv.shutdown()


class TestServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        ui = os.path.join(cls.tmp.name, "ui")
        os.makedirs(ui)
        with open(os.path.join(ui, "index.html"), "w") as f:
            f.write("<html><body data-token=\"__DECK_TOKEN__\">deck"
                    "</body></html>")
        cls.fx = ServerFixture(ui)
        # fast synthetic session, fully ingested before assertions
        cls.fx.deck.set_source("replay", synthetic="nominal", speed=1e9)
        deadline = time.time() + 10
        while time.time() < deadline:
            cls.fx.deck.poll_once()
            st, d = cls.fx.request("GET", "/data?since=-1")
            if d["series"]["alt"] and not d["source"]["connected"]:
                break                     # replay finished
            time.sleep(0.05)

    @classmethod
    def tearDownClass(cls):
        cls.fx.close()
        cls.tmp.cleanup()

    def test_data_shape_and_peaks(self):
        st, d = self.fx.request("GET", "/data?since=-1")
        self.assertEqual(st, 200)
        self.assertEqual(d["source"]["kind"], "replay")
        self.assertAlmostEqual(d["peaks"]["apogee_m"], 399.2, delta=2.0)
        self.assertAlmostEqual(d["peaks"]["max_vel_ms"], 75.0, delta=1.5)
        self.assertGreaterEqual(d["peaks"]["max_abs_g"], 16.0)
        self.assertIsNotNone(d["tplus"]["launch_t_host"])
        self.assertIn("capabilities", d)
        self.assertIn("fire", d["capabilities"])

    def test_data_incremental(self):
        st, full = self.fx.request("GET", "/data?since=-1")
        t_last = full["series"]["alt"][-1][0]
        st, tail = self.fx.request("GET", "/data?since=%s" % t_last)
        self.assertEqual(tail["series"]["alt"], [])
        # peaks survive the since-cursor (session state, not window)
        self.assertAlmostEqual(tail["peaks"]["apogee_m"],
                               full["peaks"]["apogee_m"])

    def test_events_incremental(self):
        st, all_ev = self.fx.request("GET", "/events?since=-1")
        self.assertTrue(all_ev["events"])
        last = all_ev["events"][-1]["seq"]
        st, none = self.fx.request("GET", "/events?since=%d" % last)
        self.assertEqual(none["events"], [])

    def test_cmd_requires_token(self):
        st, d = self.fx.request("POST", "/cmd", {"cmd": "arm"})
        self.assertEqual(st, 403)
        st, d = self.fx.request("POST", "/cmd", {"cmd": "arm"},
                                token="wrong")
        self.assertEqual(st, 403)

    def test_cmd_on_replay_refused(self):
        st, d = self.fx.request("POST", "/cmd", {"cmd": "arm"},
                                token=self.fx.deck.token)
        self.assertEqual(st, 200)
        self.assertFalse(d["ok"])
        self.assertIn("replay", d["refusal"])

    def test_replay_control(self):
        st, d = self.fx.request("POST", "/replay",
                                {"op": "pause", "speed": 2.0},
                                token=self.fx.deck.token)
        self.assertEqual(st, 200)
        self.assertTrue(d["ok"])
        self.fx.request("POST", "/replay", {"op": "resume"},
                        token=self.fx.deck.token)

    def test_static_token_injection(self):
        st, body = self.fx.request("GET", "/")
        self.assertEqual(st, 200)
        self.assertIn(self.fx.deck.token.encode(), body)
        self.assertNotIn(b"__DECK_TOKEN__", body)

    def test_missing_static_hint(self):
        st, d = self.fx.request("GET", "/ui/nope.js")
        self.assertEqual(st, 404)

    def test_ports_endpoint(self):
        st, d = self.fx.request("GET", "/ports")
        self.assertEqual(st, 200)
        self.assertIsInstance(d["ports"], list)


class TestRecordToggle(unittest.TestCase):
    def test_record_toggle_endpoint(self):
        with tempfile.TemporaryDirectory() as tmp:
            ui = os.path.join(tmp, "ui")
            os.makedirs(ui)
            open(os.path.join(ui, "index.html"), "w").write("x")
            fx = ServerFixture(ui, record=False, recdir=tmp)
            try:
                fx.deck.set_source("replay", synthetic="nominal",
                                   speed=1e9)
                st, d = fx.request("POST", "/record", {"on": True},
                                   token=fx.deck.token)
                self.assertTrue(d["on"])
                self.assertTrue(d["path"])
                st, d = fx.request("POST", "/record", {"on": False},
                                   token=fx.deck.token)
                self.assertFalse(d["on"])
            finally:
                fx.close()


if __name__ == "__main__":
    unittest.main()
