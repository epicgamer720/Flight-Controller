# Ground Station (RP2040 + SX1262 + PE4259)

Compile-only skeleton of the ground-station firmware (CLAUDE.md §6):

- **RX**: IRQ-driven (DIO1) continuous LoRa receive of FC packets; each frame is
  validated (magic / version / CRC16-CCITT) and emitted as **one line of JSON**
  on USB-CDC Serial. Field names match `tools/telem_decode.py parse_telem()`
  output, plus `rssi` / `snr` from RadioLib.
- **TX**: operator commands typed on the same serial port become
  `command_packet_t` uplinks with a monotonically increasing, random-start
  nonce. Pad-only enforcement stays on the FC — the GS just sends.
- Link params and packet structs come **directly from `shared/protocol.h`**
  (single source of truth, §3/§7) via a `-I<repo>/shared` include flag.

```
ground-station/
  gs/
    gs.ino       sketch (Arduino-Pico + RadioLib)
    gs_pins.h    pin map — ALL -1 SENTINELS until extracted from the schematic
  README.md      this file
```

## 1. Install the toolchain

```sh
# arduino-cli: https://arduino.github.io/arduino-cli/latest/installation/
#   Windows: download the .zip, put arduino-cli.exe on PATH
#   (or: winget install ArduinoSA.CLI)

arduino-cli config init
arduino-cli config add board_manager.additional_urls \
  https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli core update-index
arduino-cli core install rp2040:rp2040     # Earle Philhower Arduino-Pico core
arduino-cli lib install RadioLib           # SX1262 driver (6.x/7.x API used)
```

## 2. Compile

Run from the repo root (`C:\Users\14437\OneDrive\Desktop\flight controller`).
The repo path contains a **space**, so the `-I` flag needs quoting at *two*
levels: your shell must hand arduino-cli the literal value

```
compiler.cpp.extra_flags=-I"C:/Users/14437/OneDrive/Desktop/flight controller/shared"
```

as **one argument with the inner double quotes intact** — arduino-cli splits
the expanded compiler recipe on spaces *except inside double quotes*, so the
inner quotes are what keep the path together as a single `-I` argument to gcc.
(Use forward slashes: a trailing `shared\"` backslash would escape the quote.)

**Git Bash** (`\` line continuations are bash-only — for cmd.exe use `^` or put
it all on one line):

```sh
arduino-cli compile -b rp2040:rp2040:rpipico --warnings default \
  --build-property "compiler.cpp.extra_flags=-I\"C:/Users/14437/OneDrive/Desktop/flight controller/shared\"" \
  --output-dir ground-station/gs/build \
  ground-station/gs
```

**PowerShell 5.1** (does *not* escape embedded quotes for native commands, so
they must be backslash-escaped manually even inside single quotes):

```powershell
arduino-cli compile -b rp2040:rp2040:rpipico --warnings default `
  --build-property 'compiler.cpp.extra_flags=-I\"C:/Users/14437/OneDrive/Desktop/flight controller/shared\"' `
  --output-dir ground-station\gs\build `
  ground-station\gs
```

> `--warnings default` matters: arduino-cli's default warning level is `none`,
> which compiles with `-w` and **silently suppresses the intentional
> `#warning`** in `gs_pins.h`. Without it the only pin-sentinel guard you'll
> see is the runtime `{"error": ...}` refusal on the serial port.

> Quoting rationale worked out from arduino-cli's recipe splitter + PS 5.1
> native-argument rules; **not yet verified by an actual compile** — arduino-cli
> was not installed on this machine when the skeleton was written. If the build
> fails with `cannot find protocol.h` or gcc sees `controller/shared"` as an
> input file, the quoting was mangled by the shell — try the other shell's form.

**Expected output**: the build **succeeds** but prints

```
warning: gs_pins.h: pin map not yet extracted from the GS KiCad schematic ...
```

That `#warning` is intentional (see §5) and stays until `gs_pins.h` is filled in.

## 3. Flash

**BOOTSEL (no tools needed):** hold the BOOTSEL button while plugging in USB;
a `RPI-RP2` drive appears; copy `ground-station/gs/build/gs.ino.uf2` onto it.
The board reboots into the firmware.

**picotool:**

```sh
picotool load -f ground-station/gs/build/gs.ino.uf2
picotool reboot
```

## 4. Use

Open the USB-CDC serial port (any baud; e.g. 115200). Output is line-delimited
JSON — one object per received packet (telemetry / event / ack, keyed by
`type`), or `{"error": ...}` lines for CRC/length failures and status.

Commands (one per line, newline-terminated):

| Command | Uplink | `arg` encoding | FC-side gate |
|---|---|---|---|
| `arm` | CMD_ARM | 0 | pad + sensors healthy |
| `disarm` | CMD_DISARM | 0 | — |
| `mainalt <m>` | CMD_SET_MAIN_ALT | meters × 100 (cm), 30–2000 m | refused while armed; persisted to flash |
| `zero` | CMD_ZERO_BARO | 0 | pad only |
| `reboot` | CMD_REBOOT | 0 | — |
| `fire <ch> <hexcode>` | CMD_TEST_FIRE | `passcode<<16 \| channel` | ST_GROUND_IDLE + disarmed + `testen` + passcode + fresh nonce |
| `nop` | CMD_NOP | 0 | — |
| `help` | (local) | — | — |

Each uplink is transmitted **once** and reported as an `{"uplink": ...}` JSON
line with the nonce and RadioLib TX status; the FC's ACK (a 46-byte frame with
`type` = 0x11) shows up as a normal JSON line when it arrives. Uplinks only
land while the FC is on the pad — in flight the FC is TX-only (§4) and the
command is simply lost (resend from the pad).

## 5. What is blocked, and how to unblock

Hardware bring-up is **blocked** on two things (compile is not):

1. **GS pin map extraction** — every pin in `gs/gs_pins.h` is a `-1` sentinel.
   Per CLAUDE.md §0 the KiCad schematic is the source of truth and GPIO numbers
   must never be guessed. *Unblock*: parse the GS `.kicad_sch` / netlist,
   fill in NSS / DIO1 / NRESET / BUSY / SCK / MOSI / MISO (RP2040 GPIO
   numbers), confirm the pins land on one SPI block (`SPI` vs `SPI1`, see the
   comment in `gs_pins.h`), and set `GS_RADIO_TCXO` (0 = XTAL, 1 = TCXO via
   DIO3) from the schematic — the current 0 is a placeholder, not an answer.
   Rebuild; the `#warning` disappears and the runtime refusal loop lifts.
2. **FC radio hardware** — the flight controller currently reports `radio=0`
   on the bench (see `radiodbg` console triage). Until the FC's SX1262
   transmits, the GS has nothing to receive. *Unblock*: fix/verify the FC
   radio, then confirm the link with the FC on pad cadence (1 Hz).

Also before range tests: verify +22 dBm at 915 MHz against regulatory limits
(CLAUDE.md §10).
