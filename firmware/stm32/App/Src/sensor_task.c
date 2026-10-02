/*
 * Sensor task: periodic BME280 acquisition.
 *
 * Uses xTaskDelayUntil() (absolute wake times) rather than vTaskDelay() so the
 * sample period does not drift by the time spent measuring and logging.
 * Each sample fans out to the CAN task (as a packed frame) and to the uplink
 * task (as a structured sample). Producers never block on a full queue --
 * a stale sample is worth less than the next on-time one -- they count the drop.
 */

#include "app.h"
#include "bme280.h"
#include "can_bus.h"
#include "i2c_bus.h"
#include "log.h"

#define TAG "sensor"
#define I2C_TIMEOUT_MS        20u
#define REINIT_AFTER_ERRORS   3u

static int bus_read(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len)
{
  (void)ctx;
  return i2c_bus_mem_read(BME280_I2C_ADDR, reg, buf, len, I2C_TIMEOUT_MS);
}

static int bus_write(void *ctx, uint8_t reg, uint8_t value)
{
  (void)ctx;
  return i2c_bus_mem_write(BME280_I2C_ADDR, reg, &value, 1, I2C_TIMEOUT_MS);
}

static void delay_ms(uint32_t ms)
{
  vTaskDelay(pdMS_TO_TICKS(ms) ? pdMS_TO_TICKS(ms) : 1);
}

static void publish(const env_sample_t *s)
{
  can_frame_t f = { .std_id = CAN_ID_ENV_DATA, .dlc = 8 };
  protocol_pack_env(s, f.data);
  if (xQueueSend(q_can_tx, &f, 0) != pdTRUE) {
    g_queue_drops++;
  }

  uplink_msg_t m = { .kind = UPLINK_ENV, .u.env = *s };
  if (xQueueSend(q_uplink, &m, 0) != pdTRUE) {
    g_queue_drops++;
  }
}

void sensor_task(void *arg)
{
  (void)arg;

  bme280_t dev = {
    .read = bus_read, .write = bus_write, .delay_ms = delay_ms, .ctx = NULL,
    .osrs_t = BME280_OSRS_X1, .osrs_p = BME280_OSRS_X1, .osrs_h = BME280_OSRS_X1,
  };

  int      initialised = 0;
  uint32_t consecutive_errors = 0;
  uint32_t seq = 0;
  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    xEventGroupSetBits(eg_heartbeat, HB_SENSOR);

    if (!initialised) {
      bme280_status_t st = bme280_init(&dev);
      if (st == BME280_OK) {
        initialised = 1;
        consecutive_errors = 0;
        LOGI(TAG, "BME280 found at 0x%02X, conversion <= %lu ms",
             BME280_I2C_ADDR, (unsigned long)bme280_max_conversion_ms(&dev));
      } else {
        g_sensor_ok = 0;
        g_sensor_errors++;
        LOGW(TAG, "BME280 init failed (%d)%s", (int)st,
             (st == BME280_E_CHIP_ID) ? " - wrong chip ID (BMP280?)" : " - check wiring/pull-ups");
      }
    }

    if (initialised) {
      bme280_data_t d;
      bme280_status_t st = bme280_measure(&dev, &d);
      if (st == BME280_OK) {
        env_sample_t s = {
          .seq                = seq++,
          .temperature_c_x100 = d.temperature_c_x100,
          .humidity_rh_x100   = d.humidity_rh_x100,
          .pressure_pa        = d.pressure_pa,
          .timestamp_ms       = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS,
        };
        consecutive_errors = 0;
        g_sensor_ok = 1;
        publish(&s);

        char t[16], h[16];
        LOGI(TAG, "#%lu T=%sC RH=%s%% P=%luPa", (unsigned long)s.seq,
             app_fmt_x100(s.temperature_c_x100, t),
             app_fmt_x100((int32_t)s.humidity_rh_x100, h),
             (unsigned long)s.pressure_pa);
      } else {
        g_sensor_errors++;
        LOGW(TAG, "measurement failed (%d)", (int)st);
        if (++consecutive_errors >= REINIT_AFTER_ERRORS) {
          LOGE(TAG, "%lu consecutive errors - re-initialising sensor",
               (unsigned long)consecutive_errors);
          g_sensor_ok = 0;
          initialised = 0;
        }
      }
    }

    uint32_t period = g_sample_period_ms;
    if (xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(period)) == pdFALSE) {
      /* Fell behind (e.g. after a long I2C recovery): resynchronise instead of bursting. */
      last_wake = xTaskGetTickCount();
    }
  }
}
