# Ground Station firmware (RP2040)

Scoped context for the GS board. Loads when working under `ground-station/`. The radio link contract, telemetry/command protocol, and `shared/protocol.h` live in the root `CLAUDE.md` (§3, §4, §7) and apply identically to both boards.

## Toolchain, build, flash

- **Recommended: Arduino-Pico (Earle Philhower core) + RadioLib**: fast development, mature SX1262 support, and it compiles `shared/protocol.h` so packet/param definitions are reused. (Alternative: Pico SDK C/C++ + a portable sx126x driver.)
- Build with `arduino-cli` or PlatformIO. Flash via **BOOTSEL** (hold button, plug USB, drag the `.uf2`) or `picotool load -f fw.uf2`. SWD via picoprobe optional if those pins are exposed.
- **W25Q128 is the RP2040 boot flash** (program storage, 16 MB); the RP2040 boots from it via QSPI. It is *not* spare data storage by default. You may carve out a LittleFS region for optional telemetry logging, but the simplest robust design streams to the host instead.

## Role

- Continuously **RX telemetry** (IRQ-driven via DIO1), validate magic/version/CRC, and **bridge to the host over USB-CDC serial** as line-delimited JSON or CSV for a laptop GUI/`serial_monitor.py`.
- Accept operator commands from the host (stdin) → build `command_packet_t` → **TX uplink** (honored by FC only on pad, see root §4).
- Optional: drive a status LED / display if present on the board (not in the known BOM; check schematic).
- Radio config identical to root §3; set `setDio2AsRfSwitch(true)`. **TCXO vs XTAL**: determine from the GS schematic: XTAL → no `setTCXO`; TCXO → configure DIO3 supply.

## GS pin map (`TODO` from schematic)

| Function | Pin | Status |
|---|---|---|
| SX1262 SPI (SCK/MOSI/MISO/CS) | TODO | |
| SX1262 BUSY / DIO1 / NRESET | TODO | DIO1 → IRQ |
| PE4259 control | via SX1262 **DIO2** | ✅ (firmware: DIO2 RF switch) |
| QSPI flash (W25Q128) | dedicated QSPI pins | boot flash |

## Deferred / untested work (needs real GS hardware)

The GS board is an **unbuilt skeleton** (RP2040 + SX1262, no PCB yet). Everything below is a **documented plan only**: **none of it is compiled or tested here**, and it stays blocked on real GS hardware plus the GS schematic (§0). The radio params and packet structs do **not** change: both boards keep compiling the same `shared/protocol.h` (now **v3**: 54-byte `telem_packet_t`, 14-byte `command_packet_t`), so anything added here must stay byte-identical to the FC. Each item below is marked **untested**.

1. **Named dev-board pin profile**: **untested.**
   - *What:* a concrete example GPIO set for bringing the GS up on a hand-wired **Pico + SX1262 breakout**, kept as an inert named profile in `gs/gs_pins.h` (the "NAMED DEV-BOARD PROFILE" comment block below the live `-1` sentinels).
   - *Why:* lets bring-up start on a breakout before the GS PCB exists, without guessing the real board's pins. The live defines stay `-1` sentinels so `GS_PINS_VALID` keeps the sketch in its runtime-refusal loop until the actual schematic is parsed (§0); the profile numbers are a wiring *choice* for a self-wired breakout, not a schematic fact, so the block is commentary, not active `#define`s.

2. **TCXO/XTAL runtime auto-fallback + link-liveness heartbeat**: **untested.**
   - *What:* at bring-up, try the TCXO path first (DIO3 supply at `LORA_TCXO_V`, `LORA_TCXO_DELAYMS`); if `radio.begin()` / `setTCXO()` fails, re-init with TCXO control OFF (plain 32 MHz XTAL). Then run a heartbeat: if no CRC-valid RX for N seconds, report `link_lost` to the host and re-arm the radio (`startReceive()`, or a full re-`begin()` if the chip is wedged).
   - *Why:* the GS crystal type is **unknown from the schematic** (root §10 open item); today `GS_RADIO_TCXO` is a hard placeholder, and the wrong choice = dead radio or large frequency error. Auto-fallback removes the guess; the heartbeat recovers a silently wedged radio instead of going dark.

3. **Reliable uplink (bounded retransmit)**: **untested.**
   - *What:* on an operator command, transmit the `command_packet_t` repeatedly across successive TX/RX cycles (bounded retry count / timeout) and stop as soon as the FC's ACK for that `nonce` is seen; report give-up to the host otherwise.
   - *Why:* the FC opens only a **short RX window between pad TX frames** (root §4), so the single-shot `send_cmd()` uplink gs.ino does today is easily missed. Retransmitting across windows raises the odds the command lands. The anti-replay `nonce` stays **fixed across the retries of one command** so the FC dedups the duplicates (root §7); a *new* command uses a fresh `nonce`.

4. **ACK/NAK echo to host**: **untested.**
   - *What:* when the FC's ACK (a `PKT_ACK`, telemetry-shaped frame) arrives, emit a host line carrying the command id + `nonce` + accepted/rejected result + the uplink RSSI/SNR, keyed to the pending uplink from item 3.
   - *Why:* the operator currently only sees "transmitted once" with no confirmation the command was honored. An explicit ACK/NAK with link quality tells them whether it landed, and lets item 3 stop retransmitting.

5. **GS-local TX-power control (`gspower <dbm>`)**: **untested.**
   - *What:* a stdin command `gspower <-9..22>` (plus a dashboard control) that sets the **GS** SX1262 output power via `radio.setOutputPower(dbm)`, a local radio setting with **no uplink**.
   - *Why:* distinct from the existing `power <dbm>` command, which uplinks `CMD_SET_TX_POWER` to change the **FC's** TX power. `gspower` tunes the GS's own transmit level for uplink range / regulatory headroom (root §10), mirroring the FC-side `tx power` console command.
