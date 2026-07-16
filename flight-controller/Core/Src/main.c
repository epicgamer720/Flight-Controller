/* ============================================================
 * main.c: Flight Controller (STM32F722RET6)
 * Boot order is safety-critical (CLAUDE.md §2): the pyro gate is
 * forced LOW as the very first action after reset, before HAL,
 * clocks, or any other init.
 * ============================================================ */
#include "main.h"
#include "app.h"

SPI_HandleTypeDef  hspi1;
I2C_HandleTypeDef  hi2c1;
SD_HandleTypeDef   hsd1;
TIM_HandleTypeDef  htim2;
UART_HandleTypeDef huart6;
ADC_HandleTypeDef  hadc1;
/* hpcd_USB_OTG_FS lives in App/usb/usbd_conf.c */

void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);

int main(void)
{
  /* SAFETY: pyro gate LOW, push-pull, before anything else. */
  pyro_boot_safe();

  MPU_Config();
  SCB_EnableICache();
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_SDMMC1_SD_Init();   /* may fail with no card; app handles */

  app_init();
  while (1)
  {
    app_loop();
  }
}

/* 216 MHz SYSCLK from 25 MHz HSE; 48 MHz PLLQ for USB + SDMMC.
 * APB1 = 54 MHz (max), APB2 = 108 MHz. Overdrive required at 216 MHz. */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;            /* 25 MHz / 25 = 1 MHz VCO in */
  RCC_OscInitStruct.PLL.PLLN = 432;           /* 432 MHz VCO out */
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; /* 216 MHz SYSCLK */
  RCC_OscInitStruct.PLL.PLLQ = 9;             /* 48 MHz USB/SDMMC */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
    Error_Handler();

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48|RCC_PERIPHCLK_SDMMC1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    Error_Handler();
}

static void MX_SPI1_Init(void)
{
  /* Shared bus: ISM330DHCX + SX1262, both SPI mode 0, GPIO CS.
   * APB2 108 MHz / 16 = 6.75 MHz. */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
    Error_Handler();
}

static void MX_I2C1_Init(void)
{
  /* Kernel clock = PCLK1 54 MHz. PRESC=13 -> 3.857 MHz tick;
   * SCLL=19/SCLH=17 -> ~97 kHz. Conservative for BMP580 + BQ25883. */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0xD0421113;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    Error_Handler();
}

static void MX_SDMMC1_SD_Init(void)
{
  /* HAL_SD_Init is called lazily by datalog_init() so a missing card
   * doesn't hang boot; here we only fill the handle. 48 MHz kernel;
   * ClockDiv 0 -> 24 MHz after init phase. */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
}

static void MX_TIM2_Init(void)
{
  /* 4x servo PWM, 50 Hz. Timer clock = 108 MHz; PSC->1 MHz; ARR 20000. */
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 108 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    Error_Handler();
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;                  /* center */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    Error_Handler();
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;             /* gps.c re-inits during autobaud */
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
    Error_Handler();
}

static void MX_ADC1_Init(void)
{
  /* Single software-triggered conversion of CH11 (PC1, pyro continuity).
   * 480-cycle sampling: source is a 100K/10K divider (high impedance). */
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    Error_Handler();
  sConfig.Channel = PYRO1_SENSE_ADC_CH;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Both SPI CS lines HIGH (deselected) before first bus use (CLAUDE.md §3.1) */
  HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LORA_CS_PORT, LORA_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);   /* not in reset */
  HAL_GPIO_WritePin(BQ_CE_PORT, BQ_CE_PIN, GPIO_PIN_RESET);       /* ~CE low = charging allowed */
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Pin = IMU_CS_PIN;   HAL_GPIO_Init(IMU_CS_PORT, &g);
  g.Pin = LORA_CS_PIN;  HAL_GPIO_Init(LORA_CS_PORT, &g);
  g.Pin = LORA_RST_PIN; HAL_GPIO_Init(LORA_RST_PORT, &g);
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin = BQ_CE_PIN;    HAL_GPIO_Init(BQ_CE_PORT, &g);
  g.Pin = LED_PIN;      HAL_GPIO_Init(LED_PORT, &g);

  /* PB2 / RF_SW1: module-internal DIO2 domain, keep high-Z (analog). */
  g.Mode = GPIO_MODE_ANALOG;
  g.Pull = GPIO_NOPULL;
  g.Pin = LORA_RFSW_PIN; HAL_GPIO_Init(LORA_RFSW_PORT, &g);
  g.Pin = GPIO_PIN_0;    HAL_GPIO_Init(GPIOC, &g);   /* PC0: dangling PYRO_CON_1 label */
  g.Pin = PYRO1_SENSE_PIN; HAL_GPIO_Init(PYRO1_SENSE_PORT, &g); /* PC1 ADC in */

  /* Plain inputs */
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  g.Pin = LORA_BUSY_PIN; HAL_GPIO_Init(LORA_BUSY_PORT, &g);
  g.Pin = IMU_INT1_PIN;  HAL_GPIO_Init(IMU_INT1_PORT, &g);
  g.Pin = IMU_INT2_PIN;  HAL_GPIO_Init(IMU_INT2_PORT, &g);
  g.Pin = BQ_INT_PIN;    HAL_GPIO_Init(BQ_INT_PORT, &g);   /* external 10K pull-up */
  g.Pin = BMP_INT_PIN;   HAL_GPIO_Init(BMP_INT_PORT, &g);
  g.Pin = GPS_PPS_PIN;   HAL_GPIO_Init(GPS_PPS_PORT, &g);
  g.Pull = GPIO_PULLUP;
  g.Pin = SD_CD_PIN;     HAL_GPIO_Init(SD_CD_PORT, &g);    /* LOW = card present */

  /* LoRa DIO1 rising-edge IRQ */
  g.Mode = GPIO_MODE_IT_RISING;
  g.Pull = GPIO_NOPULL;
  g.Pin = LORA_IRQ_PIN;  HAL_GPIO_Init(LORA_IRQ_PORT, &g);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  HAL_MPU_Disable();
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
  /* Pyro gate LOW even on fatal error, then hang (no SWD: LED signals). */
  HAL_GPIO_WritePin(PYRO1_GATE_PORT, PYRO1_GATE_PIN, GPIO_PIN_RESET);
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
