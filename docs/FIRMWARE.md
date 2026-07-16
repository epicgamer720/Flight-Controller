# Rocketry Avionics: bench quickstart

Flight controller firmware for an **STM32F722RET6** board (`flight-controller/`) plus a
planned **RP2040 ground station**, linked over **915 MHz LoRa** (SX1262 on both ends).
`shared/protocol.h` is the wire contract for both boards; `CLAUDE.md` has the full
design context and safety rules; `docs/PINMAP.md` is the authoritative pin map.

> **Status:** SIL/host-tested, builds clean, and flashed + bench-verified on hardware,
> but **not yet flight-tested.** Flight thresholds in `App/app_config.h` are still defaults;
> tune them (and the mount knobs flagged below) to the motor/airframe before flying.

## Build

No SWD on this board, so everything goes through USB (CDC console + DFU flashing).

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
works too; see `docs/PINMAP.md`.)

## Console commands

115200-8N1 USB-CDC (baud ignored). The authoritative list is `help` on the board
(`App/console.c`, `cmd_help`); snapshot:

| Command | Does |
|---|---|
| `help` | command list |
| `status` | state / armed / alt / vel / batt / **MCU die-temp** / flags / health / pyro |
| `preflight` | GO/NO-GO checklist before arming (read-only, nothing is pulsed) |
| `sensors` | raw IMU + baro |
| `gps` | last GPS fix + persisted last-landing fix |
| `servo <1-4> <500-2500>` | set servo pulse (µs) |
| `arm` / `disarm` | arming gate (prints refusal code `-1`..`-8`) / disarm |
| `testen <on\|off>` | pad test mode (GROUND_IDLE only; always boots **off**) |
| `fire <ch> <hex-code>` | pad test fire (testen + passcode gated) |
| `zero` | zero baro AGL |
| `mainalt <m>` | set main deploy altitude (persisted to flash) |
| `apogee <ms>` | set apogee / backup-deploy timer (per-motor); persisted to flash |
| `log <stat\|start\|stop>` | SD logging (stat shows drops + ring high-water) |
| `cal` | pad gyro bias cal (~2 s, keep still; rejected if it moves) |
| `radio` | radio health + re-init / TX-timeout counters |
| `radiodbg` | raw SX1262 hardware probe (BUSY/reset/SPI bytes) |
| `tx [power\|mon\|cw\|frame]` | TX dashboard; set power / packet monitor / bench CW / force a frame |
| `i2cscan` | scan I2C1 for devices |
| `charge` | BQ25883 charger status |
| `bootloader` | reboot into USB DFU |
| `reboot` | reset MCU |
| `wdtest` | deliberate hang; IWDG must reset the board in ~4 s |
| `sleep [s]` | STOP-mode low power (ground + disarmed); wakes on RTC timer or RESET; 0/none = until RESET |

## Tools (`tools/`)

`py -m pip install -r requirements.txt` (pyserial for the serial tools, pywebview for Flight Deck's native window).

| Tool | Use |
|---|---|
| `py tools/flight_deck.py` | **Flight Deck**, the flight-ops dashboard (see below) |
| `py tools/serial_monitor.py` | timestamped console; auto-detects the FC COM port (`--list`, `--port COM7`) |
| `py tools/live_dashboard.py [--port COM9] [--http 8321]` | simple bring-up charts at http://localhost:8321. **Owns the COM port while running**, so stop it before serial_monitor or flashing |
| `py tools/decode_log.py LOG001.BIN [out.csv]` | SD binary log → CSV (68-B records, CRC-checked, resyncs past corruption) |
| `py tools/telem_decode.py <hex>` | parse/build LoRa packets from `shared/protocol.h`; importable by the GS bridge |

## Flight Deck (`tools/flight_deck.py`)

Offline mission-control app in a native window (pywebview/WebView2; `--browser`
falls back to the default browser). Fully local: 127.0.0.1 only.

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
  passcode (never stored, never logged; masked even in recordings) → 2 s
  hold-to-fire. In flight the FC is TX-only, so every command control disables
  itself with the reason shown.
- **Recording**: each live session writes `recordings/deck_<stamp>/telem.csv` +
  `events.csv` (+ lossless `raw.jsonl` on GS sessions); replay any of them with
  `--source replay --replay <file>`. FC sessions are poll-rate on the ground;
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
FLASH_OPTCR bits 3:2): value **0 = Level 3 (~2.7 V)** … value **3 = BOR off**,
which is also the shipping default, so a fresh chip displays `BOR_LEV: 0x3`.
Without it, a battery sag or pyro-fire droop can leave the F7 running erratically
(corrupting flash/SD writes) instead of resetting cleanly. Option bytes live
outside program flash, so this survives reflashing; do it once per board.

## Safety & flight logic

Every pyro decision is made in `App/state_machine.c` and only there: radio commands can
never fire in flight (CLAUDE.md §2). Not flight-tested yet; the thresholds are defaults.

- **Arming gate** (`fsm_request_arm`) requires *all* of: `ST_GROUND_IDLE`, IMU + baro
  healthy, gyro cal done, near-zero velocity, plausible |accel|, **pyro continuity**,
  **battery ≥ `ARM_MIN_VBAT_MV` (7.0 V)**, and **vertical orientation**. On refusal it
  returns a code the console prints in English: `-1` not idle, `-2` imu/baro, `-3` gyro
  cal, `-4` moving, `-5` accel implausible, `-6` no pyro continuity, `-7` battery low,
  `-8` not vertical. Orientation keys off `ARM_UP_AXIS`/`ARM_UP_SIGN` (default +Z, a
  **mount knob**); a wrong axis safely *refuses* with `-8`, it never misfires. `preflight`
  shows the same checks as a GO/NO-GO list without arming.
- **Fault-independent backup deploy**: once launched, the single charge fires at
  `apogee_timeout_ms` after launch **even in `ST_FAULT`** with dead sensors: it bypasses
  the armed gate through one controlled path (`pyro_fire_backup`). Fires at most once,
  never on the pad and never after landing; a deploy pulse is aborted **only** by a ground
  disarm, never truncated by an in-flight fault. This is the anti-lawn-dart last line; set
  `apogee` to just past expected apogee per motor (clamped to `[MAX_BURN_MS, 120000]` so it
  can't fire under thrust).
- **Servo main-release** (this board has one pyro channel, fired at apogee; a servo releases
  the main): held SAFE through all ascent and through `ST_FAULT`, released only at the guarded
  DROGUE→MAIN point (descending AND AGL ≤ `main_alt`, upward velocity inhibited). Parked SAFE
  at boot before the first control pass. `SERVO_SAFE_US`/`SERVO_RELEASE_US` are **mount knobs**.
- **Sensor guards**: gyro cal is rejected if the vehicle moves during it (per-axis gyro
  peak-to-peak + accel-stray band → auto-restart, so a bad bias never latches); IMU reads
  detect a frozen/stuck bus (rail bytes, or N byte-identical bursts) and fail, tripping the
  0.5 s IMU-fail debounce → `ST_FAULT`; baro altitude is temperature-compensated (pad temp
  latched at `zero`).
- **Fault handling**: a *ground* fault (never launched) self-heals back to `ST_INIT` once
  both sensors read healthy again (re-runs gyro cal + baro zero); any *in-flight* fault
  latches: logging + telemetry keep running, pyros safe, the backup timer still armed.
- **Anti-replay**: 32-deep nonce dedup ring with **no** monotonic floor. The GS reseeds its
  nonce randomly on restart, so a floor would permanently lock out a restarted GS.

## Firmware notes

- **IWDG ~4 s** is armed at the end of init and refreshed once per superloop pass;
  any hang reboots the MCU (reset cause is printed in the boot banner). `wdtest`
  proves it end-to-end.
- **Config persists** in flash **sector 7 @ `0x08060000`** (append-record store **v2**,
  `App/param_store.c`): 28-byte slots holding `main_alt`, the runtime `apogee_timeout`,
  and the last landing GPS fix (recovery aid). Migrates cleanly from the old 16-byte v1
  (mismatched records are skipped, then appended past). Code flash is capped at **384 KB**
  in the linker so an oversized image fails at link instead of clobbering the store.
  **Never persisted** by design: `testen` (boots off), gyro bias, baro zero; all re-done
  each pad session.
- **Telemetry is protocol v3** (`PROTO_VERSION 3`, 54-byte `telem_packet_t` in
  `shared/protocol.h`): per-frame `seq` (host computes packet-delivery ratio), a last-event
  echo (`last_evt_state` + `last_evt_count`, so a dropped `PKT_EVENT` still surfaces), and a
  `main_alt` read-back; plus `FLAG_LOW_BATT` and `FLAG_DEPLOY_FAIL` status bits. FC and GS
  must compile the same `protocol.h`. `CMD_SET_TX_POWER` sets TX power over the air (pad-only,
  like every uplink command).
- **64 KB SD log ring** (~4.8 s of 200 Hz records); `log stat` reports drops and
  ring high-water.
- `log start` failure codes: `-1` no card, `-2` link, `-3` mount, `-4` scan,
  `-5` slots, `-6`/`-7` open.

## Status LED legend (WS2812 on PC13, needs the 5 V rail)

| State | Color |
|---|---|
| INIT | dim white |
| GROUND_IDLE | rainbow cycle with GPS fix, dim green without; red if armed |
| ARMED | solid red |
| BOOST / COAST | magenta |
| APOGEE / DROGUE | cyan |
| MAIN / DESCENT | amber |
| LANDED | bright green recovery strobe (~1/s) |
| FAULT | bright red recovery strobe (~1/s) |
| SD not logging | blue flash once per second (overlays any state) |

Bench test (console, pad + disarmed only): `led <r> <g> <b>` shows a raw
color for 5 s; `led bright <0-100>` sets a global brightness scale (all
patterns, persists until reboot); `led cycle [period_ms]` steps through
every flight state so you can preview the mapping above; `led state
<name|0-10>` holds on one specific state's pattern (e.g. `led state
LANDED`); `led effect solid|blink|breathe|strobe <r> <g> <b> [period_ms]`
and `led effect rainbow [period_ms]` run a bench-only pattern; `led auto`
/ `led effect off` resumes the real flight-state pattern. Arming
force-clears any active test/effect so it can never mask the ARMED
indicator. Flight Deck (`tools/flight_deck.py`) exposes all of these as
a Status LED bench panel when connected over USB.
