# Rocketry Avionics — bench quickstart

Flight controller firmware for an **STM32F722RET6** board (`flight-controller/`) plus a
planned **RP2040 ground station**, linked over **915 MHz LoRa** (SX1262 on both ends).
`shared/protocol.h` is the wire contract for both boards; `CLAUDE.md` has the full
design context and safety rules; `docs/PINMAP.md` is the authoritative pin map.

## Build

No SWD on this board — everything goes through USB (CDC console + DFU flashing).

```
cd flight-controller
C:/Users/14437/toolchains/build-tools/bin/make.exe -j GCC_PATH=C:/Users/14437/toolchains/arm-gcc/bin
```

Outputs `build/fc.elf`, `build/fc.hex`, `build/fc.bin`.

## Flash (USB DFU)

1. Enter the bootloader: type `bootloader` on the console, **or** hold BOOT (SW2),
   tap RESET (SW1), release BOOT.
2. Flash:

```
dfu-util -l                                              # should list 0483:df11
dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D build/fc.bin
```

If dfu-util says **"cannot open"**: unplug USB and replug it *while holding BOOT*.
(`STM32_Programmer_CLI -c port=usb1 -w build\fc.bin 0x08000000 -v -g 0x08000000`
works too — see `docs/PINMAP.md`.)

## Console commands

115200-8N1 USB-CDC (baud ignored). The authoritative list is `help` on the board
(`App/console.c`, `cmd_help`); snapshot:

| Command | Does |
|---|---|
| `help` | command list |
| `status` | state / armed / alt / vel / batt / flags / health |
| `sensors` | raw IMU + baro |
| `gps` | last GPS fix |
| `servo <1-4> <500-2500>` | set servo pulse (µs) |
| `arm` / `disarm` | arming gate / disarm |
| `testen <on\|off>` | pad test mode (GROUND_IDLE only; always boots **off**) |
| `fire <ch> <hex-code>` | pad test fire (testen + passcode gated) |
| `zero` | zero baro AGL |
| `mainalt <m>` | set main deploy altitude — persisted to flash |
| `log <stat\|start\|stop>` | SD logging (stat shows drops + ring high-water) |
| `cal` | pad gyro bias cal (~2 s, keep still) |
| `radio` | radio health + re-init / TX-timeout counters |
| `radiodbg` | raw SX1262 hardware probe (BUSY/reset/SPI bytes) |
| `i2cscan` | scan I2C1 for devices |
| `charge` | BQ25883 charger status |
| `bootloader` | reboot into USB DFU |
| `reboot` | reset MCU |
| `wdtest` | deliberate hang — IWDG must reset the board in ~4 s |

## Tools (`tools/`)

`py -m pip install -r requirements.txt` (pyserial for the serial tools, pywebview for Flight Deck's native window).

| Tool | Use |
|---|---|
| `py tools/flight_deck.py` | **Flight Deck** — the flight-ops dashboard (see below) |
| `py tools/serial_monitor.py` | timestamped console; auto-detects the FC COM port (`--list`, `--port COM7`) |
| `py tools/live_dashboard.py [--port COM9] [--http 8321]` | simple bring-up charts at http://localhost:8321. **Owns the COM port while running** — stop it before serial_monitor or flashing |
| `py tools/decode_log.py LOG001.BIN [out.csv]` | SD binary log → CSV (68-B records, CRC-checked, resyncs past corruption) |
| `py tools/telem_decode.py <hex>` | parse/build LoRa packets from `shared/protocol.h`; importable by the GS bridge |

## Flight Deck (`tools/flight_deck.py`)

Offline mission-control app in a native window (pywebview/WebView2; `--browser`
falls back to the default browser). Fully local — 127.0.0.1 only.

```
py tools/flight_deck.py                          # FC over USB, auto-detects the port
py tools/flight_deck.py --source gs --port COM7  # ground-station JSON bridge
py tools/flight_deck.py --source replay --synthetic --speed 4   # no hardware needed
py tools/flight_deck.py --source replay --replay recordings/deck_.../raw.jsonl
```

- **Instruments**: T+ clock (starts at BOOST), state timeline, alt/vel/accel/
  gyro/battery charts with labeled event markers + pan/zoom/pause + flight-window
  button, peak tiles (apogee, max vel, max |g|), continuity/battery/GPS tiles,
  GS link panel (RSSI/SNR/packet rate/gap/ACK-NAK), scrolling event log with
  verbatim firmware refusals, audio alerts (state, FAULT, low battery), theme
  toggle.
- **Commands** are interlocked in the UI *and* re-checked by the firmware (the
  firmware is always the real gate): arm/disarm confirmations, mainalt bounded
  30–2000 m (the radio path's stricter bound), test-fire behind test-enable →
  passcode (never stored, never logged — masked even in recordings) → 2 s
  hold-to-fire. In flight the FC is TX-only, so every command control disables
  itself with the reason shown.
- **Recording**: each live session writes `recordings/deck_<stamp>/telem.csv` +
  `events.csv` (+ lossless `raw.jsonl` on GS sessions); replay any of them with
  `--source replay --replay <file>`. FC sessions are poll-rate on the ground —
  the FC's own 200 Hz SD log remains the authoritative flight record.
- **Owns its COM port** while running (like live_dashboard). It closes the port
  on every exit path; if a hard-killed host process ever wedges the FC's USB
  console (port opens but no replies), tap RESET or replug USB.

## Tests

```
py -m unittest discover -s tests
```

Stdlib-only: CRC16 golden vectors, log-record decode/resync, telemetry packet
round-trips, a Python port of the Kalman gate, and a console↔dashboard contract
test that fails loudly if `App/console.c` output drifts from what
`tools/live_dashboard.py` scrapes.

## BOR option byte (one-time, manual)

```
# board in DFU mode first (BOOT + RESET)
STM32_Programmer_CLI -c port=usb1 -ob displ          # check current BOR_LEV first
STM32_Programmer_CLI -c port=usb1 -ob BOR_LEV=0      # 0b00 = BOR Level 3 ≈ 2.7 V
```

Sets brown-out reset to ~2.7 V. **The F7 encoding is inverted-looking** (RM0431,
FLASH_OPTCR bits 3:2): value **0 = Level 3 (~2.7 V)** … value **3 = BOR off** —
which is also the shipping default, so a fresh chip displays `BOR_LEV: 0x3`.
Without it, a battery sag or pyro-fire droop can leave the F7 running erratically
(corrupting flash/SD writes) instead of resetting cleanly. Option bytes live
outside program flash, so this survives reflashing — do it once per board.

## Firmware notes

- **IWDG ~4 s** is armed at the end of init and refreshed once per superloop pass;
  any hang reboots the MCU (reset cause is printed in the boot banner). `wdtest`
  proves it end-to-end.
- **`main_alt` persists** in flash **sector 7 @ `0x08060000`** (append-record store,
  `App/param_store.c`). Code flash is capped at **384 KB** in the linker so an
  oversized image fails at link instead of clobbering the store. Only `main_alt`
  is persisted — `testen` always boots off by design.
- **64 KB SD log ring** (~4.8 s of 200 Hz records); `log stat` reports drops and
  ring high-water.
- `log start` failure codes: `-1` no card, `-2` link, `-3` mount, `-4` scan,
  `-5` slots, `-6`/`-7` open.
