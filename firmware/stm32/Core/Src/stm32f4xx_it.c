/*
 * Interrupt handlers. SVC, PendSV and SysTick are implemented by the
 * FreeRTOS Cortex-M4F port (mapped by name in FreeRTOSConfig.h).
 */

#include "board.h"

/* ---- Cortex-M4 faults ---------------------------------------------------
 * Captured so a debugger can inspect them; the IWDG then resets the board
 * and the next boot reports the reset cause on the console.
 */
volatile uint32_t g_fault_cfsr;
volatile uint32_t g_fault_hfsr;
volatile uint32_t g_fault_mmfar;
volatile uint32_t g_fault_bfar;

static void capture_fault(void)
{
  g_fault_cfsr  = SCB->CFSR;
  g_fault_hfsr  = SCB->HFSR;
  g_fault_mmfar = SCB->MMFAR;
  g_fault_bfar  = SCB->BFAR;
  board_fatal();
}

void NMI_Handler(void)        { capture_fault(); }
void HardFault_Handler(void)  { capture_fault(); }
void MemManage_Handler(void)  { capture_fault(); }
void BusFault_Handler(void)   { capture_fault(); }
void UsageFault_Handler(void) { capture_fault(); }
void DebugMon_Handler(void)   { }

/* ---- Peripherals -------------------------------------------------------- */
void USART1_IRQHandler(void)   { HAL_UART_IRQHandler(&huart_uplink); }
void USART2_IRQHandler(void)   { HAL_UART_IRQHandler(&huart_console); }
void I2C1_EV_IRQHandler(void)  { HAL_I2C_EV_IRQHandler(&hi2c_sensor); }
void I2C1_ER_IRQHandler(void)  { HAL_I2C_ER_IRQHandler(&hi2c_sensor); }
void CAN1_TX_IRQHandler(void)  { HAL_CAN_IRQHandler(&hcan); }
void CAN1_RX0_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan); }
void CAN1_SCE_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan); }
