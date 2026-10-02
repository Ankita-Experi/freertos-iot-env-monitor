#include "board.h"

UART_HandleTypeDef huart_console;
UART_HandleTypeDef huart_uplink;
I2C_HandleTypeDef  hi2c_sensor;
CAN_HandleTypeDef  hcan;
IWDG_HandleTypeDef hiwdg;

/* ===========================================================================
 * Clocks
 * ======================================================================== */

static HAL_StatusTypeDef configure_pll(uint32_t source)
{
  RCC_OscInitTypeDef osc = {0};

  if (source == RCC_PLLSOURCE_HSE) {
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_BYPASS;      /* 8 MHz MCO from ST-LINK */
    osc.PLL.PLLM       = 4;                   /* 8 MHz / 4  = 2 MHz VCO input */
  } else {
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLM            = 8;              /* 16 MHz / 8 = 2 MHz VCO input */
  }
  osc.PLL.PLLState  = RCC_PLL_ON;
  osc.PLL.PLLSource = source;
  osc.PLL.PLLN      = 180;                    /* VCO = 360 MHz */
  osc.PLL.PLLP      = RCC_PLLP_DIV2;          /* SYSCLK = 180 MHz */
  osc.PLL.PLLQ      = 2;
  osc.PLL.PLLR      = 2;
  return HAL_RCC_OscConfig(&osc);
}

clock_source_t board_clock_init(void)
{
  clock_source_t src = CLOCK_SRC_HSE;

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /*
   * Prefer HSE: CAN needs an accurate bit clock (HSI is only +/-1% over
   * temperature, which eats most of the CAN oscillator tolerance budget).
   * If the MCO solder bridge is open on this Nucleo, fall back to HSI so the
   * board still boots, and report it over the console.
   */
  if (configure_pll(RCC_PLLSOURCE_HSE) != HAL_OK) {
    src = CLOCK_SRC_HSI_FALLBACK;
    if (configure_pll(RCC_PLLSOURCE_HSI) != HAL_OK) {
      board_fatal();
    }
  }

  /* 180 MHz requires the regulator over-drive mode. */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    board_fatal();
  }

  RCC_ClkInitTypeDef clk = {0};
  clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;      /* HCLK  = 180 MHz */
  clk.APB1CLKDivider = RCC_HCLK_DIV4;        /* PCLK1 =  45 MHz */
  clk.APB2CLKDivider = RCC_HCLK_DIV2;        /* PCLK2 =  90 MHz */
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
    board_fatal();
  }
  return src;
}

/* ===========================================================================
 * GPIO
 * ======================================================================== */

void board_gpio_init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Pin   = LED_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_PORT, &g);
}

void board_led_toggle(void)
{
  HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
}

/* ===========================================================================
 * UART
 * ======================================================================== */

static void uart_config(UART_HandleTypeDef *h, USART_TypeDef *inst, uint32_t baud)
{
  h->Instance          = inst;
  h->Init.BaudRate     = baud;
  h->Init.WordLength   = UART_WORDLENGTH_8B;
  h->Init.StopBits     = UART_STOPBITS_1;
  h->Init.Parity       = UART_PARITY_NONE;
  h->Init.Mode         = UART_MODE_TX_RX;
  h->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  h->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(h) != HAL_OK) {
    board_fatal();
  }
}

void board_uart_init(void)
{
  uart_config(&huart_console, USART2, 115200);
  uart_config(&huart_uplink,  USART1, 115200);
}

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
  GPIO_InitTypeDef g = {0};
  g.Mode  = GPIO_MODE_AF_PP;
  g.Pull  = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF7_USART1; /* same AF number for USART1/2 */

  if (h->Instance == USART2) {
    __HAL_RCC_USART2_CLK_ENABLE();
    g.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_NVIC_SetPriority(USART2_IRQn, IRQ_PRIO_UART, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  } else if (h->Instance == USART1) {
    __HAL_RCC_USART1_CLK_ENABLE();
    g.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_NVIC_SetPriority(USART1_IRQn, IRQ_PRIO_UART, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
}

/* ===========================================================================
 * I2C (BME280)
 * ======================================================================== */

void board_i2c_init(void)
{
  hi2c_sensor.Instance             = I2C1;
  hi2c_sensor.Init.ClockSpeed      = 400000;           /* fast mode */
  hi2c_sensor.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c_sensor.Init.OwnAddress1     = 0;
  hi2c_sensor.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c_sensor.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c_sensor.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c_sensor.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c_sensor) != HAL_OK) {
    board_fatal();
  }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *h)
{
  if (h->Instance != I2C1) {
    return;
  }
  GPIO_InitTypeDef g = {0};
  g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
  g.Mode      = GPIO_MODE_AF_OD;
  g.Pull      = GPIO_PULLUP;  /* weak; most BME280 breakouts also fit 10k pull-ups */
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &g);

  __HAL_RCC_I2C1_CLK_ENABLE();
  /* Recover from a slave holding SDA low after a mid-transfer MCU reset. */
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();

  HAL_NVIC_SetPriority(I2C1_EV_IRQn, IRQ_PRIO_I2C, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, IRQ_PRIO_I2C, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

/* ===========================================================================
 * CAN1 @ 500 kbit/s
 *
 *   f_tq   = PCLK1 / Prescaler = 45 MHz / 5 = 9 MHz  (tq = 111.1 ns)
 *   bit    = SYNC(1) + BS1(15) + BS2(2) = 18 tq = 2.000 us  -> 500.0 kbit/s
 *   sample = (1 + 15) / 18 = 88.9 %
 *   SJW    = 2 tq
 *
 * tools/can_bittiming.py enumerates the alternatives. The closest match to the
 * commonly recommended 87.5 % is 15 tq / 86.7 % (prescaler 6); 18 tq / 88.9 %
 * was chosen instead for finer resynchronisation resolution (111 ns vs 133 ns
 * per tq). Either works -- every node on the bus just needs a similar point.
 * ======================================================================== */

void board_can_init(void)
{
  hcan.Instance                  = CAN1;
  hcan.Init.Prescaler            = 5;
  hcan.Init.SyncJumpWidth        = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1             = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2             = CAN_BS2_2TQ;
#if defined(CAN_SELF_TEST)
  hcan.Init.Mode                 = CAN_MODE_SILENT_LOOPBACK;
#else
  hcan.Init.Mode                 = CAN_MODE_NORMAL;
#endif
  hcan.Init.TimeTriggeredMode    = DISABLE;
  hcan.Init.AutoBusOff           = ENABLE;   /* hardware recovers after 128 x 11 recessive bits */
  hcan.Init.AutoWakeUp           = DISABLE;
  hcan.Init.AutoRetransmission   = ENABLE;
  hcan.Init.ReceiveFifoLocked    = DISABLE;  /* overwrite oldest on overrun (counted) */
  hcan.Init.TransmitFifoPriority = ENABLE;   /* send mailboxes in request order */
  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    board_fatal();
  }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *h)
{
  if (h->Instance != CAN1) {
    return;
  }
  __HAL_RCC_CAN1_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOA, &g);

  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, IRQ_PRIO_CAN_RX, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  HAL_NVIC_SetPriority(CAN1_TX_IRQn, IRQ_PRIO_CAN_TX, 0);
  HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, IRQ_PRIO_CAN_SCE, 0);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
}

/* ===========================================================================
 * Independent watchdog: LSI ~32 kHz / 64 = 500 Hz, reload 2000 -> ~4 s.
 * Only the health task refreshes it, and only when every task has checked in.
 * ======================================================================== */

void board_iwdg_init(void)
{
  /* Freeze the watchdog while the core is halted by the debugger. */
  __HAL_DBGMCU_FREEZE_IWDG();

  hiwdg.Instance       = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload    = 2000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
    board_fatal();
  }
}

void board_iwdg_refresh(void)
{
  HAL_IWDG_Refresh(&hiwdg);
}

/* ===========================================================================
 * Reset cause
 * ======================================================================== */

reset_cause_t board_reset_cause(void)
{
  static reset_cause_t cached = RESET_CAUSE_UNKNOWN;
  static int latched = 0;

  if (!latched) {
    /* Order matters: a POR also sets PINRSTF, a BOR also sets PORRSTF. */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))      cached = RESET_CAUSE_IWDG;
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) cached = RESET_CAUSE_WWDG;
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))  cached = RESET_CAUSE_SOFTWARE;
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) cached = RESET_CAUSE_LOW_POWER;
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))  cached = RESET_CAUSE_BROWNOUT;
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))  cached = RESET_CAUSE_POWER_ON;
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))  cached = RESET_CAUSE_PIN;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    latched = 1;
  }
  return cached;
}

const char *board_reset_cause_str(reset_cause_t cause)
{
  switch (cause) {
    case RESET_CAUSE_POWER_ON:  return "power-on";
    case RESET_CAUSE_PIN:       return "NRST pin";
    case RESET_CAUSE_SOFTWARE:  return "software";
    case RESET_CAUSE_IWDG:      return "IWDG timeout";
    case RESET_CAUSE_WWDG:      return "WWDG timeout";
    case RESET_CAUSE_BROWNOUT:  return "brown-out";
    case RESET_CAUSE_LOW_POWER: return "low-power";
    default:                    return "unknown";
  }
}

/* ===========================================================================
 * Fatal error: interrupts off, fast LED blink forever. The IWDG (if running)
 * will reset the MCU within ~4 s, and the next boot reports "IWDG timeout".
 * ======================================================================== */

void board_fatal(void)
{
  __disable_irq();
  for (;;) {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
    for (volatile uint32_t i = 0; i < 2000000UL; i++) {
      __NOP();
    }
  }
}
