/*
 * FreeRTOS IoT Environmental Monitor with CAN bus
 * Target: NUCLEO-F446RE (STM32F446RE, Cortex-M4F @ 180 MHz)
 *
 * main() only brings up clocks and peripherals; everything else happens in
 * FreeRTOS tasks created by app_start() (App/Src/app.c).
 */

#include "app.h"
#include "board.h"

int main(void)
{
  /* Flash prefetch/caches, NVIC priority group 4 (4 bits preemption, 0 sub),
   * and the TIM6 HAL timebase (see hal_timebase_tim6.c). */
  HAL_Init();

  reset_cause_t  rst = board_reset_cause();   /* latch before anything can clear it */
  clock_source_t clk = board_clock_init();

  board_gpio_init();
  board_uart_init();
  board_i2c_init();
  board_can_init();

  app_start(clk, rst);   /* never returns */
}
