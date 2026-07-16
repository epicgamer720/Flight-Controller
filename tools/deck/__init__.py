"""deck: Flight Deck (offline flight-operations dashboard) package.

P1 contents:
    schema.py   canonical telemetry model = telem_decode.parse_telem keys
                + host extensions (t_host/src/rssi/snr + console extras)
    sources.py  Source base, LineIngest (shared GS-JSON line classifier),
                FcConsoleSource, GsJsonSource, ReplaySource (+ synthetic
                "nominal" flight profile)

tools/live_dashboard.py is a read-only donor (its Poller/_txn pattern and
regexes are ported, never imported); tools/telem_decode.py is imported as
the single source of truth for the wire format.
"""

__all__ = ["schema", "sources"]
