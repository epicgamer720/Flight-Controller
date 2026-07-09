#!/usr/bin/env python3
"""serial_monitor.py - console for the FC USB-CDC port (and future GS bridge).

Features:
  * Lists serial ports; STM32 virtual COM ports (VID 0x0483, PID 0x5740)
    are highlighted. With no --port, auto-opens if exactly one FC is found.
  * Prints received data line-buffered with HH:MM:SS.mmm timestamps.
  * Forwards stdin lines to the port (commands to the FC console / GS bridge).
  * Ctrl+C exits cleanly.

Usage:
    python serial_monitor.py                 # auto-detect FC, or list ports
    python serial_monitor.py --list
    python serial_monitor.py --port COM7 [--baud 115200] [--eol lf|crlf|cr]

Baud is accepted for compatibility but irrelevant for USB-CDC (the FC ignores
the host's line coding).

Requires pyserial (the only third-party dependency).
"""

import argparse
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
    if not hasattr(serial, "Serial"):
        raise ImportError("wrong 'serial' package installed")
except ImportError:
    sys.stderr.write(
        "This tool needs pyserial. Install it with:\n"
        "    pip install pyserial\n"
        "(If a package named 'serial' is installed, remove it first:\n"
        "    pip uninstall serial && pip install pyserial)\n")
    sys.exit(1)

FC_VID = 0x0483  # STMicroelectronics
FC_PID = 0x5740  # STM32 Virtual COM Port (USB-CDC)

EOL = {"lf": b"\n", "crlf": b"\r\n", "cr": b"\r"}

IDLE_FLUSH_S = 1.0    # print a partial line after this much RX silence
MAX_LINE = 4096       # force-flush pathological unterminated lines


def is_fc(p):
    return p.vid == FC_VID and p.pid == FC_PID


def enumerate_ports():
    return sorted(list_ports.comports(), key=lambda p: p.device)


def print_ports(ports):
    if not ports:
        print("No serial ports found.")
        return
    print("Available ports:")
    for p in ports:
        if p.vid is not None and p.pid is not None:
            ids = "%04X:%04X" % (p.vid, p.pid)
        else:
            ids = "----:----"
        mark = "  <== FC (STM32 USB-CDC)" if is_fc(p) else ""
        print("  %-12s [%s] %s%s" % (p.device, ids, p.description or "", mark))


def timestamp():
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + ".%03d" % int(t % 1 * 1000)


def emit(line_bytes):
    text = line_bytes.decode("utf-8", errors="replace").rstrip("\r")
    print("[%s] %s" % (timestamp(), text))


def rx_loop(ser, stop):
    """Reader thread: line-buffer serial RX and print with timestamps."""
    buf = bytearray()
    last_rx = time.monotonic()
    while not stop.is_set():
        try:
            data = ser.read(ser.in_waiting or 1)  # timeout=0.1 set at open
        except (serial.SerialException, OSError):
            if buf:
                emit(bytes(buf))
            print("[%s] *** port disconnected (Ctrl+C or Enter to exit) ***"
                  % timestamp())
            stop.set()
            return
        if data:
            buf += data
            last_rx = time.monotonic()
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                emit(bytes(buf[:nl]))
                del buf[:nl + 1]
            if len(buf) > MAX_LINE:
                emit(bytes(buf))
                buf.clear()
        elif buf and time.monotonic() - last_rx > IDLE_FLUSH_S:
            emit(bytes(buf))  # unterminated output (e.g. a prompt)
            buf.clear()
    if buf:
        emit(bytes(buf))


def tx_loop(ser, stop, eol):
    """Main thread: forward stdin lines to the port until EOF/Ctrl+C."""
    while not stop.is_set():
        line = sys.stdin.readline()
        if line == "":            # stdin EOF: stay open for RX only
            while not stop.is_set():
                time.sleep(0.2)
            return
        if stop.is_set():
            return
        try:
            ser.write(line.rstrip("\r\n").encode("utf-8", errors="replace") + eol)
        except (serial.SerialException, OSError):
            print("[%s] *** write failed - port gone ***" % timestamp())
            stop.set()
            return


def pick_port(args):
    if args.port:
        return args.port
    ports = enumerate_ports()
    fcs = [p for p in ports if is_fc(p)]
    if len(fcs) == 1:
        print("Auto-selected %s (%s)" % (fcs[0].device, fcs[0].description or "FC"))
        return fcs[0].device
    print_ports(ports)
    if len(fcs) > 1:
        print("\nMultiple FC ports found - choose one with --port <name>.")
    else:
        print("\nNo FC (0483:5740) port found - specify one with --port <name>.")
    return None


def main():
    ap = argparse.ArgumentParser(
        description="Serial console for the flight-controller USB-CDC port / GS bridge.")
    ap.add_argument("-p", "--port", help="serial port (e.g. COM7, /dev/ttyACM0)")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="baud rate (ignored by USB-CDC, default 115200)")
    ap.add_argument("--eol", choices=sorted(EOL), default="lf",
                    help="line ending appended to sent lines (default lf)")
    ap.add_argument("-l", "--list", action="store_true",
                    help="list serial ports and exit")
    args = ap.parse_args()

    if args.list:
        print_ports(enumerate_ports())
        return 0

    port = pick_port(args)
    if port is None:
        return 1

    try:
        ser = serial.Serial(port, args.baud, timeout=0.1)
    except (serial.SerialException, OSError) as e:
        print("error: cannot open %s: %s" % (port, e), file=sys.stderr)
        return 2

    print("Connected to %s @ %d. Type commands + Enter to send; Ctrl+C to quit."
          % (port, args.baud))

    stop = threading.Event()
    rx = threading.Thread(target=rx_loop, args=(ser, stop), daemon=True)
    rx.start()

    rc = 0
    try:
        tx_loop(ser, stop, EOL[args.eol])
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        rx.join(timeout=1.0)
        try:
            ser.close()
        except Exception:
            pass
        print("\nClosed %s." % port)
    return rc


if __name__ == "__main__":
    sys.exit(main())
