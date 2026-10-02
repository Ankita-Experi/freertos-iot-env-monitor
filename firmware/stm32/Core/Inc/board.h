#ifndef BOARD_H
#define BOARD_H

/*
 * Board support for NUCLEO-F446RE.
 *
 *  Function           Peripheral  Pins            Nucleo header
 *  -----------------  ----------  --------------  --------------------------
 *  Debug console      USART2      PA2 TX/PA3 RX   ST-LINK virtual COM port
 *  ESP32 uplink       USART1      PA9 TX/PA10 RX  D8 / D2
 *  BME280 sensor      I2C1        PB8 SCL/PB9 SDA D15 / D14
 *  CAN transceiver    CAN1        PA12 TX/PA11 RX CN10-12 / CN10-14
 *  Heartbeat LED      GPIO        PA5             LD2 (green)
 *
 * Clock: 8 MHz HSE bypass from ST-LINK MCO -> PLL -> 180 MHz SYSCLK.
 *        APB1 = 45 MHz (CAN1, I2C1, USART2), APB2 = 90 MHz (USART1).
 */

#include "stm32f4xx_hal.h"

/* ---- NVIC priorities (4 bits, NVIC_PRIORITYGROUP_4) ---------------------
 * Every IRQ below calls FreeRTOS *FromISR() APIs, so all must be numerically
 * >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5).
 */
#define IRQ_PRIO_CAN_RX       6   /* highest: never lose a received frame */
#define IRQ_PRIO_CAN_TX       7
#define IRQ_PRIO_CAN_SCE      7
#define IRQ_PRIO_I2C          8
#define IRQ_PRIO_UART         9
#define IRQ_PRIO_HAL_TICK     15  /* TIM6 timebase for HAL_Delay/timeouts */

/* ---- Pins ---------------------------------------------------------------- */
#define LED_GPIO_PORT   GPIOA
#define LED_PIN         GPIO_PIN_5

/* ---- Peripheral handles (defined in board.c) ---------------------------- */
extern UART_HandleTypeDef huart_console; /* USART2 */
extern UART_HandleTypeDef huart_uplink;  /* USART1 */
extern I2C_HandleTypeDef  hi2c_sensor;   /* I2C1   */
extern CAN_HandleTypeDef  hcan;          /* CAN1   */
extern IWDG_HandleTypeDef hiwdg;

typedef enum {
  CLOCK_SRC_HSE = 0,
  CLOCK_SRC_HSI_FALLBACK = 1,
} clock_source_t;

/* Reset cause latched at boot from RCC->CSR (cleared afterwards). */
typedef enum {
  RESET_CAUSE_UNKNOWN = 0,
  RESET_CAUSE_POWER_ON,
  RESET_CAUSE_PIN,
  RESET_CAUSE_SOFTWARE,
  RESET_CAUSE_IWDG,
  RESET_CAUSE_WWDG,
  RESET_CAUSE_BROWNOUT,
  RESET_CAUSE_LOW_POWER,
} reset_cause_t;

clock_source_t board_clock_init(void);
void           board_gpio_init(void);
void           board_uart_init(void);
void           board_i2c_init(void);
void           board_can_init(void);
void           board_iwdg_init(void);
void           board_iwdg_refresh(void);
reset_cause_t  board_reset_cause(void);
const char    *board_reset_cause_str(reset_cause_t cause);
void           board_led_toggle(void);
void           board_fatal(void) __attribute__((noreturn));

#endif /* BOARD_H */
