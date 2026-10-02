#ifndef APP_H
#define APP_H

/*
 * Task layout
 *
 *   Task     Prio  Stack  Period / trigger           Talks to
 *   -------  ----  -----  -------------------------  ------------------------------
 *   can      4     256 w  q_can_tx | q_can_rx (set)  bxCAN mailboxes, sensor period
 *   sensor   3     384 w  xTaskDelayUntil, 1 s def.  I2C (IT + binary sem) -> queues
 *   uplink   2     384 w  q_uplink                   USART1 -> ESP32 (mutex)
 *   health   1     384 w  1 s, or task notification  IWDG, CAN status, console
 *
 * The CAN task has the highest priority so an incoming command is handled
 * within one tick; the sensor task outranks the slow UART work so sampling
 * jitter stays bounded even when the uplink is congested.
 */

#include <stdint.h>

#include "FreeRTOS.h"
#include "event_groups.h"
#include "queue.h"
#include "task.h"

#include "board.h"
#include "protocol.h"

#define PRIO_CAN     (tskIDLE_PRIORITY + 4)
#define PRIO_SENSOR  (tskIDLE_PRIORITY + 3)
#define PRIO_UPLINK  (tskIDLE_PRIORITY + 2)
#define PRIO_HEALTH  (tskIDLE_PRIORITY + 1)

#define STACK_CAN     256u   /* words */
#define STACK_SENSOR  384u
#define STACK_UPLINK  384u
#define STACK_HEALTH  384u

#define Q_CAN_TX_LEN  8u
#define Q_CAN_RX_LEN  8u
#define Q_UPLINK_LEN  4u

#define SAMPLE_PERIOD_DEFAULT_MS 1000u

/* Heartbeat bits each monitored task sets on every loop iteration. */
#define HB_SENSOR  (1u << 0)
#define HB_CAN     (1u << 1)
#define HB_UPLINK  (1u << 2)
#define HB_ALL     (HB_SENSOR | HB_CAN | HB_UPLINK)

typedef enum { UPLINK_ENV, UPLINK_STATUS } uplink_kind_t;

typedef struct {
  uplink_kind_t kind;
  union {
    env_sample_t  env;
    node_status_t status;
  } u;
} uplink_msg_t;

/* ---- Shared kernel objects (created in app_start) ---------------------- */
extern QueueHandle_t      q_can_tx;     /* can_frame_t   */
extern QueueHandle_t      q_can_rx;     /* can_frame_t   */
extern QueueHandle_t      q_uplink;     /* uplink_msg_t  */
extern QueueSetHandle_t   qs_can;       /* q_can_tx + q_can_rx */
extern EventGroupHandle_t eg_heartbeat;

extern TaskHandle_t h_task_sensor;
extern TaskHandle_t h_task_can;
extern TaskHandle_t h_task_uplink;
extern TaskHandle_t h_task_health;

/* ---- Shared state ------------------------------------------------------- */
extern volatile uint32_t g_sample_period_ms;  /* written by CAN task, read by sensor task */
extern volatile uint8_t  g_sensor_ok;
extern volatile uint32_t g_sensor_errors;
extern volatile uint32_t g_queue_drops;       /* any producer that found a queue full */

extern clock_source_t g_clock_source;
extern reset_cause_t  g_reset_cause;

void app_start(clock_source_t clk, reset_cause_t rst) __attribute__((noreturn));

/* Task entry points */
void sensor_task(void *arg);
void can_task(void *arg);
void uplink_task(void *arg);
void health_task(void *arg);

/* Format 0.01-unit fixed point as "[-]I.FF" without pulling in printf float support. */
const char *app_fmt_x100(int32_t v, char buf[16]);

#endif /* APP_H */
