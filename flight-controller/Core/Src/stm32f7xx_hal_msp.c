/* ============================================================
 * stm32f7xx_hal_msp.c: MSP init/deinit for the FC board rev.
 * Pin assignments come from board.h (KiCad netlist); do not invent.
 * NOTE: HAL_PCD_MspInit lives in App/usb/usbd_conf.c, not here.
 * ============================================================ */
#include "main.h"
#include "app.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* ---------------- Global MSP ---------------- */
void HAL_MspInit(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
}

/* ---------------- SPI1: shared IMU (CS PA4) + LoRa (CS PA9) ----------------
 * PA4 is a plain GPIO chip-select configured in main.c, not AF here. */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  GPIO_InitTypeDef g = {0};
  if (hspi->Instance == SPI1)
  {
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* PA5=SCK PA6=MISO PA7=MOSI */
    g.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &g);
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    __HAL_RCC_SPI1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
  }
}

/* ---------------- I2C1: BMP580 + BQ25883 ---------------- */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef g = {0};
  RCC_PeriphCLKInitTypeDef pclk = {0};
  if (hi2c->Instance == I2C1)
  {
    /* Kernel clock = PCLK1 (54 MHz), matches Timing in main.c. */
    pclk.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    pclk.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK)
      Error_Handler();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* PB6=SCL PB7=SDA, open-drain, external 4.7K pull-ups */
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);

    __HAL_RCC_I2C1_CLK_ENABLE();
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    __HAL_RCC_I2C1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
  }
}

/* ---------------- SDMMC1: SD card, 4-bit capable ----------------
 * Kernel clock (48 MHz CLK48) is already selected in SystemClock_Config();
 * do not override it here. All lines have external 10K pull-ups. */
void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
  GPIO_InitTypeDef g = {0};
  if (hsd->Instance == SDMMC1)
  {
    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /* PC8=D0 PC9=D1 PC10=D2 PC11=D3 PC12=CK */
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOC, &g);
    /* PD2=CMD */
    g.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &g);
  }
}

void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd)
{
  if (hsd->Instance == SDMMC1)
  {
    __HAL_RCC_SDMMC1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
                         | GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
  }
}

/* ---------------- USART6: GPS module on J8 ---------------- */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef g = {0};
  RCC_PeriphCLKInitTypeDef pclk = {0};
  if (huart->Instance == USART6)
  {
    /* Kernel clock = PCLK2 (108 MHz), the reset default, made explicit. */
    pclk.PeriphClockSelection = RCC_PERIPHCLK_USART6;
    pclk.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK)
      Error_Handler();

    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /* PC6=TX (FC->GPS) */
    g.Pin = GPIO_PIN_6;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;      /* <= 115200 baud */
    g.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &g);
    /* PC7=RX: weak pull-up so an unplugged GPS connector does not leave the
     * line floating and spam the RX interrupt with noise. */
    g.Pin = GPIO_PIN_7;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &g);

    HAL_NVIC_SetPriority(USART6_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    HAL_NVIC_DisableIRQ(USART6_IRQn);
    __HAL_RCC_USART6_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7);
  }
}

/* ---------------- ADC1: pyro continuity sense on PC1 (IN11) ---------------- */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef g = {0};
  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /* PC1 analog, also set in main.c MX_GPIO_Init; harmless repeat. */
    g.Pin = PYRO1_SENSE_PIN;
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PYRO1_SENSE_PORT, &g);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC1_CLK_DISABLE();
    /* Leave PC1 in analog mode (safe/high-Z). */
  }
}

/* ---------------- TIM2: 4x servo PWM (main.c uses HAL_TIM_PWM_Init) -------- */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim_pwm)
{
  if (htim_pwm->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim_pwm)
{
  if (htim_pwm->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_DISABLE();
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef g = {0};
  if (htim->Instance == TIM2)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /* PA0=CH1 PA1=CH2 PA2=CH3 PA3=CH4, 50 Hz servo PWM */
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &g);
  }
}
