#include "app.h"

#include <stdio.h>
#include <stdlib.h>

#include "can_bus.h"
#include "i2c_bus.h"
#include "log.h"
#include "uart_port.h"

QueueHandle_t      q_can_tx;
QueueHandle_t      q_can_rx;
QueueHandle_t      q_uplink;
QueueSetHandle_t   qs_can;
EventGroupHandle_t eg_heartbeat;

TaskHandle_t h_task_sensor;
TaskHandle_t h_task_can;
TaskHandle_t h_task_uplink;
TaskHandle_t h_task_health;

volatile uint32_t g_sample_period_ms = SAMPLE_PERIOD_DEFAULT_MS;
volatile uint8_t  g_sensor_ok;
volatile uint32_t g_sensor_errors;
volatile uint32_t g_queue_drops;
volatile uint8_t  g_task_stalled;

env_sample_t  g_last_sample;
node_status_t g_last_status;

clock_source_t g_clock_source;
reset_cause_t  g_reset_cause;

const char *app_fmt_x100(int32_t v, char buf[16])
{
  uint32_t a = (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
  snprintf(buf, 16, "%s%lu.%02lu", (v < 0) ? "-" : "", (unsigned long)(a / 100u),
           (unsigned long)(a % 100u));
  return buf;
}

static void create_task(TaskFunction_t fn, const char *name, uint32_t stack_words,
                        UBaseType_t prio, TaskHandle_t *handle)
{
  BaseType_t ok = xTaskCreate(fn, name, (configSTACK_DEPTH_TYPE)stack_words, NULL, prio, handle);
  configASSERT(ok == pdPASS);
}

void app_start(clock_source_t clk, reset_cause_t rst)
{
  g_clock_source = clk;
  g_reset_cause  = rst;

  /* Boot banner goes out by polling, before any kernel object exists. */
  LOGI("boot", "FreeRTOS IoT environmental monitor  (built " __DATE__ " " __TIME__ ")");
  LOGI("boot", "SYSCLK %lu MHz from %s, reset cause: %s",
       (unsigned long)(SystemCoreClock / 1000000u),
       (clk == CLOCK_SRC_HSE) ? "HSE 8 MHz" : "HSI (HSE failed!)",
       board_reset_cause_str(rst));
#if defined(CAN_SELF_TEST)
  LOGW("boot", "CAN in silent-loopback self-test mode");
#endif

  /* Start the watchdog only now: from here, the health task must keep it fed. */
  board_iwdg_init();

  /* ---- Kernel objects --------------------------------------------------- */
  log_init();
  uart_port_init();
  i2c_bus_init(&hi2c_sensor);

  q_can_tx     = xQueueCreate(Q_CAN_TX_LEN, sizeof(can_frame_t));
  q_can_rx     = xQueueCreate(Q_CAN_RX_LEN, sizeof(can_frame_t));
  q_uplink     = xQueueCreate(Q_UPLINK_LEN, sizeof(uplink_msg_t));
  eg_heartbeat = xEventGroupCreate();
  configASSERT(q_can_tx && q_can_rx && q_uplink && eg_heartbeat);

  vQueueAddToRegistry(q_can_tx, "q_can_tx");
  vQueueAddToRegistry(q_can_rx, "q_can_rx");
  vQueueAddToRegistry(q_uplink, "q_uplink");

  /* A queue set must be sized for every item its members can hold, and the
   * members must be empty when added. */
  qs_can = xQueueCreateSet(Q_CAN_TX_LEN + Q_CAN_RX_LEN);
  configASSERT(qs_can != NULL);
  xQueueAddToSet(q_can_tx, qs_can);
  xQueueAddToSet(q_can_rx, qs_can);

  /* ---- Tasks ------------------------------------------------------------ */
  create_task(can_task,    "can",    STACK_CAN,    PRIO_CAN,    &h_task_can);
  create_task(sensor_task, "sensor", STACK_SENSOR, PRIO_SENSOR, &h_task_sensor);
  create_task(uplink_task, "uplink", STACK_UPLINK, PRIO_UPLINK, &h_task_uplink);
  create_task(health_task, "health", STACK_HEALTH, PRIO_HEALTH, &h_task_health);

  vTaskStartScheduler();

  /* Only reached if the idle task could not be allocated. */
  board_fatal();
}

/* ===========================================================================
 * FreeRTOS hooks
 * ======================================================================== */

volatile const char *g_assert_file;
volatile int         g_assert_line;
volatile char        g_overflow_task[configMAX_TASK_NAME_LEN];

void app_assert_failed(const char *file, int line)
{
  g_assert_file = file;
  g_assert_line = line;
  board_fatal();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
  (void)task;
  for (int i = 0; i < configMAX_TASK_NAME_LEN && name[i] != '\0'; i++) {
    g_overflow_task[i] = name[i];
  }
  board_fatal();
}

void vApplicationMallocFailedHook(void)
{
  board_fatal();
}
