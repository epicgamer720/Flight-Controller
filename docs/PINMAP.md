# Flight Controller pin map (authoritative)

Source: `C:\Users\14437\Downloads\fe\Flight controler.kicad_sch`, netlist exported
2026-07-07 with `kicad-cli sch export netlist --format kicadxml`. This resolves all
`TODO` rows in CLAUDE.md §5.2. The board's IMU is **ISM330DHCX** (matches CLAUDE.md;
the December 2025 board rev in `Desktop\Flight-Controller` used ICM-45686 + no radio;
that rev is obsolete).

| Function | Pin | Notes |
|---|---|---|
| IMU ISM330DHCX CS | PA4 | 10K pull-up (R23) |
| SPI1 SCK / MISO / MOSI | PA5 / PA6 / PA7 | shared IMU + LoRa, mode 0, 6.75 MHz |
| IMU INT1 / INT2 | PC4 / PB0 | inputs (polled) |
| LoRa Wio-SX1262 CS | PA9 | 10K pull-up (R24) |
| LoRa DIO1 (IRQ) | PA10 | EXTI15_10, rising |
| LoRa BUSY | PB1 | input |
| LoRa NRESET | PA15 | output, active low |
| LoRa RF_SW1 | PB2 | **keep high-Z (analog)**, driven by module DIO2 |
| Barometer BMP580 | I2C1 PB6/PB7 | 4.7K pull-ups; INT = PB9; addr 0x46/0x47 |
| Charger BQ25883 | I2C1 PB6/PB7 | addr 0x6B; INT = PB5 (10K PU); ~CE = PB8 (low = charge) |
| GPS (JST-GH J8) | USART6 TX=PC6 RX=PC7 | PPS = PA8; ESD PRTR5V0U2X; ext. module |
| Pyro 1 gate | **PB13** | AO3400A low-side (Q2); 100R gate R, 10K pulldown; **boot LOW** |
| Pyro 1 continuity | **PC1** = ADC123_IN11 | divider 100K(R29)/10K(R30) from J7.2; VBAT/11 when continuous |
| Pyro terminal | J7 | J7.1 = BAT+, J7.2 = FET drain |
| (dangling) | PC0 | schematic label `PYRO_CON_1` unrouted, unused |
| Servo 1–4 | PA0/PA1/PA2/PA3 | TIM2 CH1–4, AF1, 50 Hz |
| SD card | SDMMC1 4-bit | D0=PC8 D1=PC9 D2=PC10 D3=PC11 CLK=PC12 CMD=PD2 (all 10K PU) |
| SD card detect | PB3 | internal pull-up, LOW = card present |
| USB-C | PA11=DM PA12=DP | device (5.1K CC pulldowns); **PA9 VBUS sensing OFF** (PA9 = LoRa CS!) |
| Status LED WS2812B-2020 | PC13 | via 100R; PC13 is low-drive, so it's bit-banged, best-effort |
| Buttons | SW1=NRST, SW2=BOOT0 | BOOT0 + reset = USB DFU bootloader |
| HSE / LSE | PH0/PH1 25 MHz, PC14/PC15 32.768 kHz | |
| Battery sense | n/a | none to MCU; read VBAT via BQ25883 ADC (I2C) |
| SWD | PA13/PA14 | unconnected (DFU + USB console only) |

## Clocks
216 MHz SYSCLK (HSE 25 MHz, PLLM=25 N=432 P=2), overdrive, APB1 54 MHz, APB2 108 MHz,
PLLQ=9 → 48 MHz for USB + SDMMC.

## Flash procedure (Windows, this machine)
```
# board in DFU: hold BOOT (SW2), tap RESET (SW1), release BOOT (enumerates 0483:DF11)
cd flight-controller
C:\Users\14437\toolchains\build-tools\bin\make.exe -j GCC_PATH=C:\Users\14437\toolchains\arm-gcc\bin
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" `
    -c port=usb1 -w build\fc.bin 0x08000000 -v -g 0x08000000
```
Software re-entry: `bootloader` command on the USB console (or CMD_ENTER_BOOTLOADER).
