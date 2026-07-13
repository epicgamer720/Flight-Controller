#!/usr/bin/env python3
"""Flight Deck — offline flight-operations dashboard.

    py tools/flight_deck.py                          # FC console, auto-port
    py tools/flight_deck.py --source fc --port COM9
    py tools/flight_deck.py --source gs --port COM7
    py tools/flight_deck.py --source replay --synthetic --speed 4
    py tools/flight_deck.py --source replay --replay recordings/.../raw.jsonl
    py tools/flight_deck.py --browser                # no pywebview window

Native window via pywebview (Edge WebView2). --browser opens the default
browser instead (also the fallback when WebView2 is missing). All state
is local: 127.0.0.1 only, no internet.

Clean shutdown (CDC-wedge mitigation): the serial port is closed on
every exit path — window close, Ctrl+C, SIGTERM, atexit — the FC's USB
CDC TX can stick if a host process dies holding the port.
"""

import argparse
import atexit
import os
import signal
import sys
import threading
import time
import webbrowser

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from deck.server import Deck, list_ports, make_server

UI_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "deck_ui")

WEBVIEW2_HELP = """\
pywebview could not start an Edge WebView2 window.
Fix: install the WebView2 runtime (ships with Edge on Win10/11):
  https://developer.microsoft.com/microsoft-edge/webview2/
or run with --browser to use your default browser instead."""


def auto_fc_port():
    for p in list_ports():
        if p["is_fc"]:
            return p["device"]
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--source", choices=("fc", "gs", "replay"),
                    default="fc")
    ap.add_argument("--port", help="COM port (fc/gs); fc auto-detects")
    ap.add_argument("--http", type=int, default=8322)
    ap.add_argument("--browser", action="store_true",
                    help="open in the default browser (no pywebview)")
    ap.add_argument("--replay", help="recorded raw.jsonl or telem.csv")
    ap.add_argument("--synthetic", nargs="?", const="nominal",
                    help="synthetic flight profile (default: nominal)")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="replay speed factor")
    ap.add_argument("--recdir", default="recordings")
    ap.add_argument("--no-record", action="store_true")
    args = ap.parse_args()

    deck = Deck(recdir=args.recdir, record=not args.no_record)

    port = args.port
    if args.source == "fc" and port is None:
        port = auto_fc_port()
        if port is None:
            print("no FC found on USB (VID 0483 PID 5740) — plug it in or"
                  " pass --port; starting disconnected on COM9 fallback")
            port = "COM9"
    if args.source == "gs" and port is None:
        ap.error("--source gs needs --port")
    if args.source == "replay" and not (args.replay or args.synthetic):
        args.synthetic = "nominal"

    deck.set_source(args.source, port=port, file=args.replay,
                    synthetic=args.synthetic, speed=args.speed)

    srv = make_server(deck, UI_DIR, port=args.http)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    # cache-bust the top-level nav URL, not just the sub-assets: WebView2's
    # persistent disk cache serves a stale index.html across relaunches
    # (ignores no-store), which pins the OLD ?v= stamps on the JS/CSS. A
    # fresh query per launch forces a new document fetch every relaunch.
    url = "http://127.0.0.1:%d/?v=%d" % (args.http, int(time.time()))
    print("Flight Deck: %s  (source=%s%s)"
          % (url, args.source, " " + port if port else ""))
    if deck.recorder.session:
        print("recording -> %s" % deck.recorder.session)

    done = threading.Event()

    def shutdown(*_):
        if done.is_set():
            return
        done.set()
        deck.shutdown()          # stops sources -> closes serial ports
        srv.shutdown()

    atexit.register(shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        signal.signal(signal.SIGTERM, shutdown)
    except (OSError, ValueError):
        pass

    if args.browser:
        webbrowser.open(url)
        print("Ctrl+C to stop.")
        try:
            done.wait()
        except KeyboardInterrupt:
            pass
        shutdown()
        return

    try:
        import webview
        w = webview.create_window("Flight Deck", url, width=1500,
                                  height=950, min_size=(1100, 700))
        w.events.closing += shutdown
        webview.start()          # blocks until the window closes
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print("pywebview failed: %s\n%s" % (e, WEBVIEW2_HELP))
        webbrowser.open(url)
        print("Ctrl+C to stop.")
        try:
            done.wait()
        except KeyboardInterrupt:
            pass
    shutdown()


if __name__ == "__main__":
    main()
