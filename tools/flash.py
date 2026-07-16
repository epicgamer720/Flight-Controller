#!/usr/bin/env python3
"""Flash the flight controller (STM32F722) over USB DFU.

    py tools/flash.py                       # flash build/fc.bin (already in DFU)
    py tools/flash.py --port COM9           # trigger bootloader jump, then flash
    py tools/flash.py --build --port COM9   # make, trigger, flash
    py tools/flash.py --no-trigger          # skip the console trigger

The FC has no SWD debugger (CLAUDE.md 5.1). It is flashed via the STM32
built-in USB DFU bootloader. Enter it either by holding BOOT + tapping
RESET, or by sending the `bootloader` console command over the FC's
USB-CDC port (pass --port), which soft-jumps to DFU. Then dfu-util
writes build/fc.bin to flash base 0x08000000 and leaves DFU (device
resets into the new firmware).
"""

import argparse
import os
import subprocess
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DEFAULT_BIN = os.path.join("flight-controller", "build", "fc.bin")
DEFAULT_DFU_UTIL = r"C:/Users/14437/toolchains/dfu-util/dfu-util-0.9-win64/dfu-util.exe"
DFU_ID = "0483:df11"

# CLAUDE.md 5.1 build command (run from flight-controller/).
MAKE = r"C:/Users/14437/toolchains/build-tools/bin/make.exe"
GCC_PATH = "C:/Users/14437/toolchains/arm-gcc/bin"

CANNOT_OPEN_TIP = (
    'if dfu-util reports "cannot open" / permission error, replug the USB '
    "cable while holding BOOT (SW2), then rerun with --no-trigger.")


def run(cmd, **kw):
    """Print then run a subprocess, streaming its output. Returns rc."""
    print("+ " + " ".join(cmd))
    return subprocess.run(cmd, **kw).returncode


def build():
    rc = run([MAKE, "-j", "GCC_PATH=" + GCC_PATH], cwd=os.path.join(REPO_ROOT, "flight-controller"))
    if rc != 0:
        sys.exit("build failed (make rc=%d), not flashing." % rc)


def trigger_bootloader(port):
    """Send the `bootloader` console command so the FC jumps to DFU."""
    try:
        import serial  # pyserial
    except ImportError:
        sys.exit("--port needs pyserial (pip install pyserial), or enter DFU "
                 "manually (hold BOOT + tap RESET) and pass --no-trigger.")
    print("triggering DFU: sending 'bootloader' to %s" % port)
    try:
        with serial.Serial(port, 115200, timeout=1) as s:
            s.write(b"bootloader\r\n")
            s.flush()
    except Exception as e:  # serial.SerialException + friends
        sys.exit("could not open %s (%s). Enter DFU manually (hold BOOT + tap "
                 "RESET) and pass --no-trigger." % (port, e))
    print("waiting for DFU re-enumeration...")
    time.sleep(2)


def wait_for_dfu(dfu_util, tries=8):
    """Poll `dfu-util -l` until the 0483:df11 device shows up."""
    for i in range(tries):
        try:
            out = subprocess.run([dfu_util, "-l"], capture_output=True,
                                 text=True).stdout
        except FileNotFoundError:
            sys.exit("dfu-util not found at %s, pass --dfu-util." % dfu_util)
        if DFU_ID in out:
            print("DFU device %s present." % DFU_ID)
            return
        time.sleep(1)
    sys.exit("no DFU device (%s) after %d s. Is the FC in bootloader mode?\n%s"
             % (DFU_ID, tries, CANNOT_OPEN_TIP))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("--bin", default=DEFAULT_BIN,
                    help="firmware .bin (resolved from repo root; default %s)"
                    % DEFAULT_BIN)
    ap.add_argument("--port", help="FC USB-CDC COM port; sends the 'bootloader'"
                    " command to jump to DFU before flashing")
    ap.add_argument("--dfu-util", default=DEFAULT_DFU_UTIL,
                    help="path to dfu-util.exe")
    ap.add_argument("--no-trigger", action="store_true",
                    help="skip the console trigger; assume already in DFU")
    ap.add_argument("--build", action="store_true",
                    help="run the make firmware build first")
    args = ap.parse_args()

    if args.build:
        build()

    binpath = args.bin if os.path.isabs(args.bin) else os.path.join(REPO_ROOT, args.bin)
    if not os.path.isfile(binpath):
        sys.exit("firmware not found: %s (build it, or pass --bin / --build)." % binpath)
    print("flashing %s (%d bytes)" % (binpath, os.path.getsize(binpath)))

    if args.port and not args.no_trigger:
        trigger_bootloader(args.port)
    elif not args.no_trigger:
        print("no --port: assuming the FC is already in DFU (hold BOOT + tap "
              "RESET). Pass --port to trigger it over USB-CDC.")

    wait_for_dfu(args.dfu_util)

    rc = run([args.dfu_util, "-a", "0", "-d", DFU_ID,
              "-s", "0x08000000:leave", "-D", binpath])
    if rc != 0:
        sys.exit("dfu-util failed (rc=%d).\n%s" % (rc, CANNOT_OPEN_TIP))
    print("done. FC left DFU and reset into the new firmware.")


if __name__ == "__main__":
    main()
