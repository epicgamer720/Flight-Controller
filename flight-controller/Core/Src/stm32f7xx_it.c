/* ============================================================
 * stm32f7xx_it.c — interrupt handlers, Flight Controller board rev.
 * ISRs are minimal (flag/ring work only, never blocking).
 * SAFETY (CLAUDE.md §2): every fault/NMI path forces the pyro gate
 * LOW with a direct BSRR write before hanging — no HAL dependency.
 * ============================================================ */
#include "main.h"
#include "stm32f7xx_it.h"
#include "app.h"

/* Direct register write: safe in any fault context, cannot itself fault
 * once GPIOB clock is on (gate pin is configured first thing in main). */
static inline void pyro_gate_force_low(void)
{
  PYRO1_GATE_PORT->BSRR = (uint32_t)PYRO1_GATE_PIN << 16U;
}

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/

void NMI_Handler(void)
{
  pyro_gate_force_low();
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  pyro_gate_force_low();
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  pyro_gate_force_low();
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  pyro_gate_force_low();
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  pyro_gate_force_low();
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/* LoRa DIO1 (PA10, rising edge) — SX1262 TxDone/RxDone/Timeout. */
void EXTI15_10_IRQHandler(void)
{
  if (__HAL_GPIO_EXTI_GET_IT(LORA_IRQ_PIN) != 0U)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(LORA_IRQ_PIN);
    radio_irq();
  }
}

/* GPS on USART6: raw register-level RX-ring handler in gps.c.
 * Deliberately NOT HAL_UART_IRQHandler (no HAL RX state machine). */
void USART6_IRQHandler(void)
{
  gps_uart_irq();
}

/* USB-CDC device on OTG_FS (PA11/PA12). */
void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}
