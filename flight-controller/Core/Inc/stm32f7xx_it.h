/* ============================================================
 * stm32f7xx_it.h — interrupt handler prototypes (FC board rev)
 * ============================================================ */
#ifndef __STM32F7xx_IT_H
#define __STM32F7xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Cortex-M7 exception handlers */
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/* Peripheral interrupt handlers */
void EXTI15_10_IRQHandler(void);   /* LoRa DIO1 on PA10 -> radio_irq() */
void USART6_IRQHandler(void);      /* GPS -> gps_uart_irq() */
void OTG_FS_IRQHandler(void);      /* USB-CDC device */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F7xx_IT_H */
