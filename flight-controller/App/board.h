#pragma once
/* ============================================================
 * board.h: authoritative hardware map for the Flight Controller
 * Extracted from KiCad netlist (Downloads/fe/Flight controler.kicad_sch,
 * exported 2026-07-07 via kicad-cli). DO NOT invent pins; if it is not
 * here, it is not connected on the board.
 *
 * MCU: STM32F722RET6, LQFP64. HSE 25 MHz (X322525MOB4SI), LSE 32.768 kHz.
 * No SWD (PA13/PA14 unconnected); debug via USB-CDC, flash via DFU.
 * ============================================================ */
#include "stm32f7xx_hal.h"

/* ---- IMU: ISM330DHCX (U7) on shared SPI1, mode 0, <=10 MHz ---- */
#define IMU_CS_PORT        GPIOA
#define IMU_CS_PIN         GPIO_PIN_4    /* /ISM_CS, 10K pull-up R23 */
#define IMU_INT1_PORT      GPIOC
#define IMU_INT1_PIN       GPIO_PIN_4    /* /ISM_INT1 */
#define IMU_INT2_PORT      GPIOB
#define IMU_INT2_PIN       GPIO_PIN_0    /* /ISM_INT2 */

/* ---- LoRa: Wio-SX1262 module (U3) on shared SPI1, mode 0 (mandatory) ---- */
#define LORA_CS_PORT       GPIOA
#define LORA_CS_PIN        GPIO_PIN_9    /* /LORA_CS, 10K pull-up R24 */
#define LORA_IRQ_PORT      GPIOA
#define LORA_IRQ_PIN       GPIO_PIN_10   /* /LORA_IRQ = DIO1, EXTI15_10 */
#define LORA_BUSY_PORT     GPIOB
#define LORA_BUSY_PIN      GPIO_PIN_1    /* /LORA_BUSY */
#define LORA_RST_PORT      GPIOA
#define LORA_RST_PIN       GPIO_PIN_15   /* /LORA_RST, active low */
/* /LORA_RF_SW1 = PB2: driven internally by the module via DIO2.
 * MUST stay high-impedance (analog mode). NEVER configure as output. */
#define LORA_RFSW_PORT     GPIOB
#define LORA_RFSW_PIN      GPIO_PIN_2

/* ---- SPI1 (shared IMU + LoRa): SCK=PA5 MISO=PA6 MOSI=PA7, AF5 ----
 * APB2 = 108 MHz; prescaler /16 => 6.75 MHz (within both parts' limits). */

/* ---- Barometer: BMP580 (U4) on I2C1 (PB6=SCL PB7=SDA, 4.7K pull-ups) ---- */
#define BMP_INT_PORT       GPIOB
#define BMP_INT_PIN        GPIO_PIN_9    /* /BMP_INT */
/* I2C addresses to probe: 0x46 then 0x47 (7-bit) */

/* ---- Charger: BQ25883 (U2) on I2C1, 7-bit addr 0x6B ---- */
#define BQ_INT_PORT        GPIOB
#define BQ_INT_PIN         GPIO_PIN_5    /* /BQ_INT, 10K pull-up, open-drain */
#define BQ_CE_PORT         GPIOB
#define BQ_CE_PIN          GPIO_PIN_8    /* /BQ_CE: ~CE, LOW = charge enabled */

/* ---- GPS: external module on JST-GH J8, USART6 AF8 ---- */
#define GPS_TX_PORT        GPIOC
#define GPS_TX_PIN         GPIO_PIN_6    /* /USART6_TX (FC->GPS) */
#define GPS_RX_PORT        GPIOC
#define GPS_RX_PIN         GPIO_PIN_7    /* /USART6_RX (GPS->FC) */
#define GPS_PPS_PORT       GPIOA
#define GPS_PPS_PIN        GPIO_PIN_8    /* /PPS (input) */

/* ---- Pyro: ONE channel. Low-side AO3400A (Q2), terminal J7 off BAT+ ----
 * Gate: 100R series (R27) + 10K pulldown (R28). Fire = drive HIGH.
 * Continuity: J7.2 -100K(R29)-> PC1 -10K(R30)-> GND divider (ratio 1/11).
 * With e-match fitted and gate OFF, PC1 ~= VBAT/11 (e.g. 8.2V -> ~0.75V).
 * NOTE: schematic label PYRO_CON_1 on PC0 is dangling (unrouted), unused. */
#define PYRO1_GATE_PORT    GPIOB
#define PYRO1_GATE_PIN     GPIO_PIN_13   /* /PYRO_2 net: boot-safe LOW, push-pull */
#define PYRO1_SENSE_PORT   GPIOC
#define PYRO1_SENSE_PIN    GPIO_PIN_1    /* /PYRO_CON_2 = ADC123_IN11 */
#define PYRO1_SENSE_ADC_CH ADC_CHANNEL_11
#define PYRO_SENSE_DIV     11.0f         /* (100K+10K)/10K */

/* ---- Servos: 4x on TIM2 CH1..CH4 (AF1), 50 Hz ---- */
#define SERVO1_PORT        GPIOA
#define SERVO1_PIN         GPIO_PIN_0    /* TIM2_CH1 */
#define SERVO2_PORT        GPIOA
#define SERVO2_PIN         GPIO_PIN_1    /* TIM2_CH2 */
#define SERVO3_PORT        GPIOA
#define SERVO3_PIN         GPIO_PIN_2    /* TIM2_CH3 */
#define SERVO4_PORT        GPIOA
#define SERVO4_PIN         GPIO_PIN_3    /* TIM2_CH4 */
#define NUM_SERVO          4

/* ---- SD card: SDMMC1 4-bit (all lines 10K pulled up) ---- */
/* D0=PC8 D1=PC9 D2=PC10 D3=PC11 CLK=PC12 CMD=PD2, AF12 */
#define SD_CD_PORT         GPIOB
#define SD_CD_PIN          GPIO_PIN_3    /* card detect, closed-to-GND when inserted: use internal pull-up, LOW = card present */

/* ---- USB: OTG_FS device, USB-C with 5.1K CC pulldowns (UFP) ---- */
/* DM=PA11 DP=PA12, AF10 */

/* ---- Status LED: single WS2812B-2020 data-in from PC13 via 100R ----
 * PC13 is a low-drive (~2 MHz) pin: bit-banged with DWT delays, best effort. */
#define LED_PORT           GPIOC
#define LED_PIN            GPIO_PIN_13

/* ---- Buttons: SW1 = NRST, SW2 = BOOT0 (10K pulldown R22); hardware only. */

/* ---- Battery: no MCU ADC divider; read VBAT via BQ25883 internal ADC. ---- */

/* ---- Peripheral handles (defined in main.c) ---- */
extern SPI_HandleTypeDef  hspi1;
extern I2C_HandleTypeDef  hi2c1;
extern SD_HandleTypeDef   hsd1;
extern TIM_HandleTypeDef  htim2;
extern UART_HandleTypeDef huart6;
extern ADC_HandleTypeDef  hadc1;
extern PCD_HandleTypeDef  hpcd_USB_OTG_FS;
