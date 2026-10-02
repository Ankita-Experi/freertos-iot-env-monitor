#include "i2c_bus.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

static I2C_HandleTypeDef *s_hi2c;
static SemaphoreHandle_t  s_mutex;    /* one transfer at a time */
static SemaphoreHandle_t  s_done;     /* given by ISR on completion or error */
static volatile int       s_result;   /* 0 = ok, -1 = error; written by ISR before give */
static i2c_bus_stats_t    s_stats;

void i2c_bus_init(I2C_HandleTypeDef *hi2c)
{
  s_hi2c  = hi2c;
  s_mutex = xSemaphoreCreateMutex();
  s_done  = xSemaphoreCreateBinary();
  configASSERT(s_mutex != NULL && s_done != NULL);
  vQueueAddToRegistry(s_mutex, "i2c_mtx");
}

void i2c_bus_get_stats(i2c_bus_stats_t *out)
{
  taskENTER_CRITICAL();
  *out = s_stats;
  taskEXIT_CRITICAL();
}

static void recover_bus(void)
{
  /* A stuck transfer leaves the HAL state machine busy; a full re-init
   * (which also pulses the peripheral reset in HAL_I2C_MspInit) recovers it. */
  HAL_I2C_DeInit(s_hi2c);
  HAL_I2C_Init(s_hi2c);
}

static int transfer(int is_read, uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len,
                    uint32_t timeout_ms)
{
  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
    return -1;
  }

  (void)xSemaphoreTake(s_done, 0); /* drop any late completion from a timed-out transfer */
  s_result = -1;
  s_stats.transfers++;

  uint16_t dev = (uint16_t)(addr7 << 1);
  HAL_StatusTypeDef st = is_read
    ? HAL_I2C_Mem_Read_IT(s_hi2c, dev, reg, I2C_MEMADD_SIZE_8BIT, buf, len)
    : HAL_I2C_Mem_Write_IT(s_hi2c, dev, reg, I2C_MEMADD_SIZE_8BIT, buf, len);

  int rc = -1;
  if (st != HAL_OK) {
    s_stats.errors++;
    if (st == HAL_BUSY) {
      recover_bus();
    }
  } else if (xSemaphoreTake(s_done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
    s_stats.timeouts++;
    recover_bus();
  } else if (s_result != 0) {
    s_stats.errors++;
  } else {
    rc = 0;
  }

  xSemaphoreGive(s_mutex);
  return rc;
}

int i2c_bus_mem_read(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
  return transfer(1, addr7, reg, buf, len, timeout_ms);
}

int i2c_bus_mem_write(uint8_t addr7, uint8_t reg, const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
  /* HAL takes a non-const pointer but does not modify the buffer on writes. */
  return transfer(0, addr7, reg, (uint8_t *)(uintptr_t)buf, len, timeout_ms);
}

/* ---- HAL callbacks (ISR context) --------------------------------------- */

static void complete_from_isr(I2C_HandleTypeDef *h, int result)
{
  if (h != s_hi2c || s_done == NULL) {
    return;
  }
  s_result = result;
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(s_done, &woken);
  portYIELD_FROM_ISR(woken);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *h) { complete_from_isr(h, 0); }
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *h) { complete_from_isr(h, 0); }
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h)     { complete_from_isr(h, -1); }
