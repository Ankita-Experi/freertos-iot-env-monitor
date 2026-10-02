#ifndef I2C_BUS_H
#define I2C_BUS_H

/*
 * Interrupt-driven I2C register access for FreeRTOS tasks.
 *
 * The calling task starts a HAL_I2C_Mem_*_IT() transfer and then blocks on a
 * binary semaphore. The I2C completion/error ISR gives the semaphore
 * (xSemaphoreGiveFromISR) -- the classic ISR-to-task signalling pattern --
 * so the CPU is free for other tasks for the ~250 us a BME280 burst read takes.
 */

#include <stdint.h>

#include "stm32f4xx_hal.h"

void i2c_bus_init(I2C_HandleTypeDef *hi2c);

/* addr7 is the 7-bit device address. Return 0 on success, -1 on NACK/bus error/timeout. */
int i2c_bus_mem_read(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len, uint32_t timeout_ms);
int i2c_bus_mem_write(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint16_t len, uint32_t timeout_ms);

typedef struct {
  uint32_t transfers;
  uint32_t errors;     /* NACK, arbitration lost, bus error */
  uint32_t timeouts;   /* no completion IRQ in time -> peripheral re-initialised */
} i2c_bus_stats_t;

void i2c_bus_get_stats(i2c_bus_stats_t *out);

#endif /* I2C_BUS_H */
