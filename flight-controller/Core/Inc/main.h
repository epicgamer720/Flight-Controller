#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "board.h"

void Error_Handler(void);
void SystemClock_Config(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif
