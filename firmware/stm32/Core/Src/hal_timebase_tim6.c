/*
 * HAL timebase on TIM6.
 *
 * FreeRTOS owns SysTick, so the HAL tick (HAL_Delay, HAL driver timeouts)
 * runs from the basic timer TIM6 at 1 kHz instead. This overrides the weak
 * HAL_InitTick()/HAL_SuspendTick()/HAL_ResumeTick() in stm32f4xx_hal.c.
 */

#include "board.h"

static TIM_HandleTypeDef htim6;

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef clk;
  uint32_t flash_latency;

  __HAL_RCC_TIM6_CLK_ENABLE();
  HAL_RCC_GetClockConfig(&clk, &flash_latency);

  /* APB1 timers run at 2 x PCLK1 whenever the APB1 prescaler is not 1. */
  uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
  if (clk.APB1CLKDivider != RCC_HCLK_DIV1) {
    tim_clk *= 2U;
  }

  htim6.Instance               = TIM6;
  htim6.Init.Prescaler         = (tim_clk / 1000000U) - 1U;  /* 1 MHz counter */
  htim6.Init.Period            = (1000000U / 1000U) - 1U;     /* 1 kHz update */
  htim6.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
    return HAL_ERROR;
  }

  if (TickPriority < (1UL << __NVIC_PRIO_BITS)) {
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority, 0U);
    uwTickPrio = TickPriority;
  } else {
    return HAL_ERROR;
  }
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

  return HAL_TIM_Base_Start_IT(&htim6);
}

void HAL_SuspendTick(void)
{
  __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void)
{
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
}

void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
}
