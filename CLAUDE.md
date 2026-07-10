# CLAUDE.md — Rocketry Avionics Firmware & Software

Authoritative context for **two firmware codebases** that talk over a 915 MHz LoRa link:

| Board | MCU | Radio | Role |
|---|---|---|---|
| **Flight Controller (FC)** | STM32F722RET6 (LQFP64) | Wio-SX1262 module | Sensor fusion, flight state machine, pyro firing, SD logging, downlink telemetry |
| **Ground Station (GS)** | RP2040 | SX1262 (bare chip) + PE4259 RF switch | RX telemetry, USB-serial bridge to host, command uplink |

---

## 0. FIRST ACTIONS — READ BEFORE WRITING ANY PERIPHERAL CODE

The **KiCad schematics are the source of truth**, not this file. Before generating driver/init code:

1. **Parse the netlists** (`.kicad_sch` / exported netlist) with Python to build the authoritative pin map and part list. The pin tables below mark known pins as ✅ and unknowns as `TODO`. Do not invent GPIO numbers for `TODO` rows — extract them.
2. **Resolve these hardware questions from the schematic** (they change firmware behavior):
   - **TCXO vs XTAL** on *each* SX1262 (FC module and GS chip). This determines whether you configure DIO3 as TCXO supply. Getting it wrong = radio dead or large frequency error. See §3.
   - **Pyro channel count** and their gate GPIOs + continuity-sense ADC pins.
   - **SD card interface**: SDIO (1/4-bit) vs SPI.
   - **Battery voltage sense**: ADC divider pin, or read from BQ25883 / fuel gauge over I²C.
   - **Servo output count** and timer channels.
   - GPS and barometer **part numbers** (not in this doc) → picks the driver/protocol.
3. Confirm anything ambiguous with the user rather than guessing. They actively check firmware against the schematic.

---

## 1. Repo layout

```
/avionics
  /flight-controller     # STM32F722 firmware
  /ground-station        # RP2040 firmware
  /shared
    protocol.h           # SINGLE source of truth for link params + packet structs (§7)
    crc16.h
  /tools
    decode_log.py        # binary SD-log decoder
    serial_monitor.py    # host-side GS console / GUI
  CLAUDE.md              # this file
```

`shared/protocol.h` is compiled **identically** by both boards. Both MCUs are little-endian ARM, so packed structs transfer byte-for-byte.

---

## 2. ⚠️ SAFETY — PYROTECHNICS (read first, non-negotiable)

This vehicle fires e-matches/charges. Firmware bugs here cause real-world detonation. Hard rules:

- **Boot-safe**: pyro gate GPIOs must be driven **LOW** as the very first thing after reset, before any other init, and configured as push-pull outputs (not floating). Never leave a pyro pin in a default/floating state during startup.
- **Disarmed by default.** The vehicle boots into `ST_INIT` → `ST_GROUND_IDLE`, pyros inhibited. Firing is only possible after an explicit arm transition.
- **Arming gate**: require *all* of: explicit ARM command/switch + sensors healthy + on-pad sanity (near-zero velocity, plausible orientation). Log the arm event.
- **Never fire in flight from a command.** `CMD_TEST_FIRE` is honored **only** in `ST_GROUND_IDLE`, only while disarmed-but-test-enabled, and requires a matching passcode in the command `arg`. In-flight pyro events come *only* from the autonomous state machine.
- **Continuity sense must not fire.** Use the high-side/low-current sense path; never pulse the gate to "test."
- **Sanity-inhibit deployments**: do not fire main if vertical velocity is upward; do not fire drogue before a minimum time/altitude after launch; ignore a single-sample apogee — require debounce + a backup timer.
- **Fire pulse**: assert gate HIGH for `FIRE_PULSE_MS` (~1000 ms), then LOW. Confirm by continuity going open. Optional single retry if continuity persists.
- Treat **accel saturation** (±16 g, see §5.3) as "definitely thrusting," never as a numeric value, when gating boost-phase logic.

---

## 3. Radio link contract (MUST match on both ends)

All of these live as `#define`s in `shared/protocol.h`. **Any mismatch between FC and GS = no link, silently.**

| Parameter | Default | Notes |
|---|---|---|
| Frequency | **915.0 MHz** | US 902–928 ISM. Keep both ends identical and in-band. |
| Bandwidth | 250 kHz | Wider = faster, shorter range. |
| Spreading factor | SF8 | SF7 fast/short, SF9–10 slow/long. Tune for range. |
| Coding rate | 4/5 | 4/8 = more robust, lower rate. |
| Sync word | private (`0x12` in RadioLib) | Must match; isolates your link. |
| Preamble | 8 symbols | |
| TX power | +22 dBm | SX1262 max; verify module limit + regulatory. |
| LoRa HW CRC | ON | Plus our payload CRC16 (§4). |

### 3.1 SX1262 setup gotchas (cause ~90% of "radio won't work")

- **TCXO (DIO3)** — *Per board, verify from schematic.*
  - If the part has a **TCXO**: you **must** power it via DIO3 with the correct voltage and a startup delay (RadioLib `radio.setTCXO(volts, delayMs)`; sx126x driver: `SetDIO3AsTcxoCtrl`). The Wio-SX1262 module is TCXO-based — confirm voltage (commonly 1.8 V).
  - If it has a plain **32 MHz XTAL**: do **not** enable TCXO control or the radio init fails.
- **DIO2 as RF switch** — both boards: `SetDIO2AsRfSwitchCtrl(true)` (RadioLib: `radio.setDio2AsRfSwitch(true)`). DIO2 drives TX/RX antenna switching automatically.
  - **FC (Wio-SX1262)**: the module's internal switch is driven by DIO2. The host MCU pin **PB2 / RF_SW1 must stay high-impedance** (configure as input/analog, never drive it). ✅ from project history.
  - **GS (PE4259)**: DIO2 drives the PE4259 control line; high = TX path. Verify PE4259 control polarity/truth table against the schematic — firmware action is the same (`SetDIO2AsRfSwitchCtrl(true)`), but confirm the switch sees the right level.
- **DIO1** = primary IRQ line (TxDone/RxDone/Timeout). Wire it to an MCU interrupt-capable GPIO and use IRQ-driven RX, not polling, on the GS.
- **NRESET / BUSY**: implement proper reset and **always wait on BUSY** before issuing commands. Skipping BUSY handling causes intermittent failures.
- **CS lines (FC)**: SPI1 is **shared** by the IMU (CS = PA4) and the radio (CS = PA9). Drive **both CS high (deselected) before first SPI use.** LORA_CS has a 10K boot pull-up but still set it high explicitly in firmware.

---

## 4. Telemetry / command protocol

Half-duplex link. Scheme:

- **Downlink-primary.** FC transmits a telemetry frame on a fixed cadence (e.g., 10 Hz in flight, 1 Hz on pad). GS listens continuously.
- **Uplink commands** (arm/disarm/test-fire/config) are sent by the GS **only while the FC is on the pad / `ST_GROUND_IDLE`**, where the FC opens a short RX window between TX frames. **Once in flight the FC is TX-only** — never sacrifice telemetry for an RX window during ascent/descent.
- Every packet is fixed-size, packed, little-endian, ends with **CRC16-CCITT** over all preceding bytes. Drop packets failing magic/version/CRC.

See §7 for the exact structs. Key fields downlinked: state, `t_ms`, GPS lat/lon/alt/sats, baro AGL, vertical velocity, 3-axis accel (may be clipped), gyro, battery mV, pyro continuity + fired bitfields, status flags (gps_fix, accel_sat, armed).

---

## 5. Flight Controller firmware (STM32F722RET6)

### 5.1 Toolchain, build, flash

- **No SWD on this board** (PA13/PA14 unconnected). You **cannot attach a debugger.** Therefore:
  - Implement a **USB-CDC console** (PA11/PA12 = USB_DM/DP, OTG_FS) for config, sensor dumps, and `CMD_ENTER_BOOTLOADER`.
  - Flashing is via the **STM32 built-in USB DFU bootloader** (system memory). Enter it with BOOT0 high at reset **or** a software jump to system memory (implement `CMD_ENTER_BOOTLOADER`/console cmd → jump `0x1FF00000`). Ensure BOOT0 is accessible (jumper/test point) as a fallback.
  - Flash command:
    ```
    dfu-util -l                      # confirm 0483:df11
    dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D build/fc.bin
    ```
    If dfu-util says "cannot open", replug USB while holding BOOT.
- **Build system**: `arm-none-eabi-gcc` + **Make** (CubeMX-generated Makefile, committed). Build from `flight-controller/`:
  ```
  C:/Users/14437/toolchains/build-tools/bin/make.exe -j GCC_PATH=C:/Users/14437/toolchains/arm-gcc/bin
  ```
  Outputs `build/fc.elf/.hex/.bin`. The base project was generated once with STM32CubeMX (216 MHz sysclk + 48 MHz USB; SPI1, USART6 GPS, I²C1 sensors, SDMMC1, TIM2 servo PWM, USB_OTG_FS CDC); drivers + flight logic live in `App/`.
- Linker: flash base `0x08000000`, 512 KB flash / 256 KB RAM on RET6. **FLASH is capped at 384 KB in the .ld**: sector 7 (`0x08060000`, 128 KB) is the `main_alt` param store (`App/param_store.c`) — code overlapping it fails at link. Top 16 B of RAM (`0x2003FFF0`) are reserved for the DFU handoff magic (`App/dfu.c` + `SystemInit`, kept in lockstep).

### 5.2 Pin map (✅ known / `TODO` extract from schematic)

> **Resolved:** the full pin map was extracted from the KiCad netlist — see **`docs/PINMAP.md`** (authoritative; it wins over the planning sketch below on any conflict).

| Function | Pin | Status |
|---|---|---|
| IMU (ISM330DHCX) CS | PA4 | ✅ |
| LoRa (Wio-SX1262) CS | PA9 | ✅ |
| SPI1 SCK / MISO / MOSI | TODO (typical PA5/PA6/PA7) | confirm |
| LoRa BUSY / DIO1 / NRESET | TODO | confirm — DIO1 must be IRQ-capable |
| RF_SW1 (keep **high-Z**) | PB2 | ✅ do not drive |
| USB_DM / USB_DP | PA11 / PA12 | ✅ fixed silicon (OTG_FS) |
| SWD (unused) | PA13 / PA14 | ✅ NC — no debugger |
| GPS UART TX/RX | TODO | + part #/protocol (u-blox→UBX) |
| Barometer (I²C or SPI) | TODO | + part # |
| BQ25883 I²C + BQ_INT | TODO | INT has pull-up ✅ |
| Pyro gate ×N | TODO | boot LOW, push-pull |
| Pyro continuity sense ×N | TODO | ADC |
| Servo PWM ×N | TODO | timer channels |
| SD card (SDIO/SPI) | TODO | |
| Battery voltage sense | TODO | ADC divider or via charger |

### 5.3 Sensors — board-specific cautions

- **IMU = ISM330DHCX, ±16 g max.** High-power boost will **saturate** the accelerometer. Never use peak accel as a quantitative value during boost; use it only as a boolean "thrusting." Set the `accel_sat` flag when any axis rails. Apogee detection must **not** depend on accel integration.
- **Gyro thermal coupling**: U7 (IMU) sits directly above the LoRa module. TX bursts create a thermal gradient → gyro bias drift. Mitigations: do **gyro bias calibration on the pad while stationary** just before launch; don't rely on long gyro integration for flight-critical decisions; be aware bias can shift during sustained TX.
- **Shared SPI1 — different SPI modes**: **SX1262 requires SPI mode 0 (mandatory).** ST MEMS commonly run mode 3; some support mode 0. **Verify ISM330DHCX mode in the datasheet.** If they differ, reconfigure CPOL/CPHA per transaction (or confirm mode 0 works for both — cleanest). Serialize bus access: never assert both CS; in a superloop this is natural, with RTOS use a mutex. Use ~8 MHz SCK to stay within both parts.
- **Barometer is primary for apogee/altitude** (accel saturates, gyro drifts). Run a small 1-D Kalman (state = altitude, vertical velocity), updated by baro; optionally use accel as the control input **only when not saturated**.

### 5.4 Flight state machine

```
ST_INIT → ST_GROUND_IDLE → ST_ARMED → ST_BOOST → ST_COAST
        → ST_APOGEE → ST_DROGUE → ST_MAIN → ST_DESCENT → ST_LANDED
  (any state on sensor loss / impossible reading) -> ST_FAULT
```

Detection (all thresholds = tunable constants):

- **Launch**: `|accel| > LAUNCH_G` (~3 g) sustained `LAUNCH_DEBOUNCE_MS` (~150 ms) to reject handling. Because accel may already be saturated, also accept "baro AGL rising > N m." → `ST_BOOST`.
- **Burnout**: axial accel drops below `BURNOUT_G` (≈0) for a debounce, **or** `MAX_BURN_MS` elapsed. → `ST_COAST`.
- **Apogee**: filtered vertical velocity ≤ 0 for `APOGEE_DEBOUNCE` samples **and** altitude decreasing; backup `APOGEE_TIMEOUT` from launch. Fire **drogue** (or main if single-deploy). → `ST_DROGUE`.
- **Main deploy**: descending **and** baro AGL ≤ `MAIN_ALT` (e.g., 150 m). Inhibit if velocity upward. Fire **main**. → `ST_MAIN`.
- **Landed**: `|vertical vel| < LAND_VEL` and altitude stable for `LAND_DEBOUNCE` (~10 s). → `ST_LANDED`: stop pyros, **raise GPS telemetry rate for recovery**, sound buzzer.
- **ST_FAULT**: sensor loss / impossible state → safe pyros, keep logging + telemetry.

Emit a `PKT_EVENT` on every state transition and pyro action.

### 5.5 Logging (SD)

- Log raw IMU + baro + GPS + state at high rate (target 100–500 Hz) as **fixed binary records** to a file; periodic flush; flush + close on `ST_LANDED`.
- Use SDIO + DMA with a ring buffer so logging never blocks the control loop. (If SD is on SPI it shares nothing with SPI1 only if it's a different bus — confirm.)
- Implemented: **64 KB RAM ring** (`LOG_RING_BYTES`, ~4.8 s of 200 Hz 68-B records) drained to SDMMC1; `log stat` reports drops + ring high-water; flush (not close) on `ST_FAULT`.
- Provide `tools/decode_log.py` to turn the binary log into CSV.

### 5.6 Power / charger (BQ25883)

- I²C 2-cell Li-ion charger. At init optionally set charge current/voltage; then mostly **monitor**: charging/fault/present status, watch BQ_INT. Surface battery mV in telemetry (ADC divider or charger/fuel-gauge register — confirm source).

### 5.7 Scheduling

Bare-metal superloop with a timer tick is sufficient and most deterministic for this. Suggested rates: control/state-machine + sensor sample 100–500 Hz, telemetry TX 1 Hz pad / 10 Hz flight, log flush ~1–5 Hz, charger poll ~1 Hz. If using an RTOS, isolate the pyro/state task at highest priority and never let logging/telemetry preempt it.

An **IWDG (~4 s)** is active: armed at the end of `app_init()` (after the slow inits), refreshed once per superloop pass plus at the SD liveness point. Any hang reboots the MCU; the reset cause is printed in the boot banner and logged. `wdtest` on the console proves it end-to-end.

---

## 6. Ground Station firmware (RP2040)

### 6.1 Toolchain, build, flash

- **Recommended: Arduino-Pico (Earle Philhower core) + RadioLib** — fast development, mature SX1262 support, and it compiles `shared/protocol.h` so packet/param definitions are reused. (Alternative: Pico SDK C/C++ + a portable sx126x driver.)
- Build with `arduino-cli` or PlatformIO. Flash via **BOOTSEL** (hold button, plug USB, drag the `.uf2`) or `picotool load -f fw.uf2`. SWD via picoprobe optional if those pins are exposed.
- **W25Q128 is the RP2040 boot flash** (program storage, 16 MB) — the RP2040 boots from it via QSPI. It is *not* spare data storage by default. You may carve out a LittleFS region for optional telemetry logging, but the simplest robust design streams to the host instead.

### 6.2 Role

- Continuously **RX telemetry** (IRQ-driven via DIO1), validate magic/version/CRC, and **bridge to the host over USB-CDC serial** as line-delimited JSON or CSV for a laptop GUI/`serial_monitor.py`.
- Accept operator commands from the host (stdin) → build `command_packet_t` → **TX uplink** (honored by FC only on pad, §4).
- Optional: drive a status LED / display if present on the board (not in the known BOM — check schematic).
- Radio config identical to §3; set `setDio2AsRfSwitch(true)`. **TCXO vs XTAL**: determine from the GS schematic — XTAL → no `setTCXO`; TCXO → configure DIO3 supply.

### 6.3 GS pin map (`TODO` from schematic)

| Function | Pin | Status |
|---|---|---|
| SX1262 SPI (SCK/MOSI/MISO/CS) | TODO | |
| SX1262 BUSY / DIO1 / NRESET | TODO | DIO1 → IRQ |
| PE4259 control | via SX1262 **DIO2** | ✅ (firmware: DIO2 RF switch) |
| QSPI flash (W25Q128) | dedicated QSPI pins | boot flash |

---

## 7. `shared/protocol.h` (drop into /shared, compile on both boards)

The checked-in `shared/protocol.h` is the source of truth; this listing is a reference and may lag it (e.g. the added `FLAG_CHG_OK`/`FLAG_GPS_OK` health flags).

```c
#pragma once
#include <stdint.h>
#include <stddef.h>

/* ---- Link parameters (MUST match FC and GS) ---- */
#define LORA_FREQ_HZ      915000000UL
#define LORA_BW_KHZ       250.0f
#define LORA_SF           8
#define LORA_CR           5          /* 4/5 */
#define LORA_SYNC_WORD    0x12       /* RadioLib private */
#define LORA_PREAMBLE     8
#define LORA_TX_DBM       22
/* TCXO: set per-board after confirming hardware. e.g. 1.8 V, 5 ms */
#define LORA_TCXO_V       1.8f
#define LORA_TCXO_DELAYMS 5

/* ---- Protocol ---- */
#define LINK_MAGIC    0x52   /* 'R' */
#define PROTO_VERSION 1
#define NUM_PYRO      1      /* confirmed from schematic: gate PB13, sense PC1 */

typedef enum {
    PKT_TELEMETRY = 0x01,
    PKT_EVENT     = 0x02,   /* state change / pyro fired / fault */
    PKT_COMMAND   = 0x10,   /* GS -> FC */
    PKT_ACK       = 0x11,
} packet_type_t;

typedef enum {
    ST_INIT = 0, ST_GROUND_IDLE, ST_ARMED, ST_BOOST, ST_COAST,
    ST_APOGEE, ST_DROGUE, ST_MAIN, ST_DESCENT, ST_LANDED, ST_FAULT
} flight_state_t;

/* status flags bitfield */
#define FLAG_GPS_FIX   (1u<<0)
#define FLAG_ACCEL_SAT (1u<<1)
#define FLAG_ARMED     (1u<<2)
#define FLAG_SD_OK     (1u<<3)

typedef struct __attribute__((packed)) {
    uint8_t  magic;          /* LINK_MAGIC */
    uint8_t  version;        /* PROTO_VERSION */
    uint8_t  type;           /* packet_type_t */
    uint8_t  state;          /* flight_state_t */
    uint32_t t_ms;           /* ms since boot */
    int32_t  lat_e7;         /* deg * 1e7 */
    int32_t  lon_e7;         /* deg * 1e7 */
    int32_t  alt_baro_cm;    /* AGL, cm */
    int32_t  alt_gps_cm;     /* MSL, cm */
    int16_t  vel_up_cms;     /* vertical, cm/s */
    int16_t  accel_g_x100[3];/* g*100; may rail at +/-1600 (16 g) */
    int16_t  gyro_dps_x10[3];/* deg/s *10 */
    uint16_t batt_mv;
    uint8_t  pyro_cont;      /* continuity bitfield, 1 = continuity */
    uint8_t  pyro_fired;     /* fired bitfield */
    uint8_t  sats;
    uint8_t  flags;          /* FLAG_* */
    uint16_t crc16;          /* CRC16-CCITT over all preceding bytes */
} telem_packet_t;

typedef enum {
    CMD_NOP = 0, CMD_ARM, CMD_DISARM, CMD_TEST_FIRE,
    CMD_SET_MAIN_ALT, CMD_ZERO_BARO, CMD_REBOOT, CMD_ENTER_BOOTLOADER
} command_id_t;

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  type;     /* PKT_COMMAND */
    uint8_t  cmd;      /* command_id_t */
    uint32_t arg;      /* e.g. main altitude cm, or TEST_FIRE passcode+channel */
    uint32_t nonce;    /* anti-replay; FC rejects repeats */
    uint16_t crc16;
} command_packet_t;

/* CRC16-CCITT (poly 0x1021, init 0xFFFF) */
static inline uint16_t crc16_ccitt(const uint8_t *d, size_t n) {
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        c ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++)
            c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}
```

Validation on RX: check `magic`, `version`, then `crc16_ccitt(buf, len-2) == crc`. Reject otherwise. `CMD_TEST_FIRE` handler on the FC must check state == `ST_GROUND_IDLE`, test-enabled, and a valid passcode/channel in `arg`, and a fresh `nonce`.

---

## 8. Build & flash quick reference

**FC (STM32F722, DFU only):**
```
cd flight-controller
C:/Users/14437/toolchains/build-tools/bin/make.exe -j GCC_PATH=C:/Users/14437/toolchains/arm-gcc/bin
# enter bootloader: console "bootloader" cmd, or hold BOOT (SW2) + tap RESET (SW1)
dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D build/fc.bin
# "cannot open"? replug USB while holding BOOT
```

**GS (RP2040):**
```
arduino-cli compile -b rp2040:rp2040:rpipico ground-station
# hold BOOTSEL, plug USB, then:
cp ground-station/build/*.uf2 /Volumes/RPI-RP2   # or: picotool load -f gs.uf2
```

---

## 9. Known gotchas / lessons (from project history)

- SX1262 needs **`SetDIO2AsRfSwitchCtrl(true)`** on both boards; **PB2/RF_SW1 stays high-Z** on the FC; PE4259 is DIO2-driven on the GS.
- SX1262 **TCXO via DIO3** must be configured if the part has a TCXO; verify per board.
- Shared **SPI1** (IMU PA4 / LoRa PA9): serialize access, reconcile SPI mode (radio = mode 0 mandatory), drive both CS high at boot.
- **ISM330DHCX ±16 g** saturates on boost — boolean-only during thrust; baro is primary for apogee.
- IMU sits over the radio → **gyro bias drifts during TX**; calibrate on pad, don't over-integrate.
- **No SWD** → debug via USB-CDC console + telemetry; flash via DFU; implement a soft jump-to-bootloader.
- RP2040 **W25Q128 = boot flash**, not free storage.
- Pyros **boot LOW, disarmed**; deployments come only from the state machine; `CMD_TEST_FIRE` is pad-only + passcode-gated.
- Keep FC/GS radio params and `protocol.h` **byte-identical**.

---

## 10. Open items (resolve, then implement)

- [x] Extract full pin map + part numbers from KiCad netlist (all `TODO` rows) — **done**, see `docs/PINMAP.md`.
- [x] Confirm **TCXO vs XTAL** on FC module — **confirmed on hardware**: Wio-SX1262 is TCXO (1.8 V via DIO3; init succeeds on the TCXO path). GS chip: still TODO from the GS schematic.
- [x] Confirm **pyro channel count** + gate/continuity pins; set `NUM_PYRO` — **done**: 1 channel (gate PB13, sense PC1), `NUM_PYRO 1`.
- [x] Confirm SD interface (SDIO vs SPI), servo count/timers, battery sense source — **done**: SDMMC1 4-bit; servos ×4 on TIM2 CH1–4; battery via BQ25883 ADC (I²C).
- [x] Identify GPS + barometer parts → choose drivers/protocols — baro **done**: BMP580 (I²C1). GPS: external NMEA module on USART6 (J8 JST-GH; generic GGA/RMC autobaud driver — exact module part not identified).
- [ ] Tune flight thresholds (`LAUNCH_G`, `MAIN_ALT`, debounces, timers) to the motor/airframe.
- [ ] Confirm 915 MHz TX power vs regulatory (Part 15 / amateur 33 cm) before range tests.
