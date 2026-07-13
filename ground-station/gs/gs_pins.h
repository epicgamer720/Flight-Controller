#pragma once
/* ============================================================
 * gs_pins.h — Ground Station pin map (RP2040 + bare SX1262 + PE4259)
 *
 * ALL PINS ARE -1 SENTINELS ON PURPOSE. The KiCad schematic is the
 * source of truth (CLAUDE.md §0) — NEVER guess GPIO numbers. Parse
 * the GS .kicad_sch / exported netlist, fill in every define below,
 * then rebuild. Until then the sketch compiles (with a #warning)
 * but refuses to run at startup with a JSON error on USB-CDC.
 *
 * Every value below is an RP2040 GPIO number (0..29), NOT a physical
 * package pin.
 * ============================================================ */

/* SX1262 SPI chip select. Schematic net: RP2040 GPIO -> SX1262 NSS. */
#define GS_PIN_LORA_NSS     -1

/* SX1262 DIO1 — primary IRQ line (RxDone/TxDone/Timeout). Schematic net:
 * SX1262 DIO1 -> RP2040 GPIO. Any RP2040 GPIO is interrupt-capable. */
#define GS_PIN_LORA_DIO1    -1

/* SX1262 NRESET (active-low reset). Schematic net: RP2040 GPIO -> NRESET. */
#define GS_PIN_LORA_NRESET  -1

/* SX1262 BUSY — RadioLib waits on this before every command (§3.1;
 * skipping BUSY = intermittent failures). Schematic net: BUSY -> GPIO. */
#define GS_PIN_LORA_BUSY    -1

/* SPI clock: RP2040 GPIO -> SX1262 SCK.
 * NOTE RP2040 pin muxing: SCK/MOSI/MISO must all belong to the SAME SPI
 * block. SPI0: SCK = GPIO 2/6/18, TX(MOSI) = 3/7/19, RX(MISO) = 0/4/16.
 * SPI1: SCK = GPIO 10/14, TX = 11/15, RX = 8/12.
 * If the schematic lands on SPI1 GPIOs, change `SPI` to `SPI1` in gs.ino. */
#define GS_PIN_SPI_SCK      -1

/* SPI MOSI (RP2040 "TX"): RP2040 GPIO -> SX1262 MOSI. */
#define GS_PIN_SPI_MOSI     -1

/* SPI MISO (RP2040 "RX"): SX1262 MISO -> RP2040 GPIO. */
#define GS_PIN_SPI_MISO     -1

/* PE4259 RF switch: NO GPIO define — the switch control is driven by
 * SX1262 DIO2 (gs.ino calls radio.setDio2AsRfSwitch(true), §3.1/§6.3).
 * Verify PE4259 control polarity against the schematic (DIO2 high = TX). */

/* ------------------------------------------------------------
 * TCXO vs XTAL on the GS SX1262 — VERIFY FROM THE SCHEMATIC (§3.1).
 *   0 = plain 32 MHz XTAL: DIO3 TCXO control stays OFF
 *       (enabling TCXO on an XTAL part makes radio init fail).
 *   1 = TCXO powered from DIO3: configured at LORA_TCXO_V with a
 *       LORA_TCXO_DELAYMS startup delay (values from shared/protocol.h).
 * Getting this wrong = dead radio or large frequency error.
 * The default 0 below is a PLACEHOLDER, not a confirmed answer.
 * ------------------------------------------------------------ */
#ifndef GS_RADIO_TCXO
#define GS_RADIO_TCXO 0
#endif

/* ---- sentinel guard: compile-time warning + runtime refusal flag ---- */
#if (GS_PIN_LORA_NSS < 0) || (GS_PIN_LORA_DIO1 < 0) || \
    (GS_PIN_LORA_NRESET < 0) || (GS_PIN_LORA_BUSY < 0) || \
    (GS_PIN_SPI_SCK < 0) || (GS_PIN_SPI_MOSI < 0) || (GS_PIN_SPI_MISO < 0)
#warning "gs_pins.h: pin map not yet extracted from the GS KiCad schematic (sentinel -1 pins) - sketch will refuse to run"
#define GS_PINS_VALID 0
#else
#define GS_PINS_VALID 1
#endif

/* ============================================================
 * NAMED DEV-BOARD PROFILE (DEFERRED / UNTESTED) —
 *   "Pico + generic SX1262 breakout", hand-wired for early bring-up.
 *
 * This is an EXAMPLE wiring recipe, not the GS PCB, and it is NOT active:
 * it is commentary only, so it cannot flip GS_PINS_VALID and let the
 * sketch touch the radio on bogus pins. On a hand-wired breakout YOU pick
 * the RP2040 GPIOs, so the numbers below are a CHOICE, not a schematic
 * fact — but the real GS board's pins still come from ITS schematic (§0).
 * To try it on a breakout: copy these values up into the -1 defines above
 * and rebuild. Do NOT apply it to the GS PCB. Never compiled or run here.
 *
 * All GPIOs below sit on RP2040 SPI0 (see the muxing note above):
 *   GS_PIN_SPI_SCK      2   SPI0 SCK
 *   GS_PIN_SPI_MOSI     3   SPI0 TX  -> SX1262 MOSI
 *   GS_PIN_SPI_MISO     4   SPI0 RX  <- SX1262 MISO
 *   GS_PIN_LORA_NSS     5   -> SX1262 NSS      / * TODO: confirm from schematic * /
 *   GS_PIN_LORA_NRESET  6   -> SX1262 NRESET   / * TODO: confirm from schematic * /
 *   GS_PIN_LORA_BUSY    7   <- SX1262 BUSY     / * TODO: confirm from schematic * /
 *   GS_PIN_LORA_DIO1    8   <- SX1262 DIO1     / * TODO: confirm from schematic * /
 * PE4259 RF switch: no GPIO — driven by SX1262 DIO2 (setDio2AsRfSwitch(true)).
 * GS_RADIO_TCXO: still unknown on the GS chip — the TCXO/XTAL auto-fallback
 * in ground-station/CLAUDE.md ("Deferred / untested work", item 2) resolves it
 * at runtime rather than hard-coding a guess here.
 * ============================================================ */
