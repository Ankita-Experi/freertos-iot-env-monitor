#ifndef BME280_H
#define BME280_H

/*
 * Minimal, platform-independent Bosch BME280 driver.
 *
 * - Forced-mode measurements (sensor sleeps between samples: ~0.1 uA idle).
 * - Integer compensation exactly as in the BME280 datasheet (rev 1.6, sec. 4.2.3):
 *   32-bit temperature/humidity, 64-bit pressure. No floating point.
 * - Bus access is injected through function pointers, so the same code runs
 *   on the STM32 (interrupt-driven I2C under FreeRTOS) and in host unit tests.
 */

#include <stdint.h>

#define BME280_CHIP_ID          0x60u

#define BME280_REG_CALIB_00     0x88u  /* dig_T1 .. dig_H1, 26 bytes (0x88-0xA1) */
#define BME280_REG_CHIP_ID      0xD0u
#define BME280_REG_RESET        0xE0u
#define BME280_REG_CALIB_26     0xE1u  /* dig_H2 .. dig_H6, 7 bytes (0xE1-0xE7) */
#define BME280_REG_CTRL_HUM     0xF2u
#define BME280_REG_STATUS       0xF3u
#define BME280_REG_CTRL_MEAS    0xF4u
#define BME280_REG_CONFIG       0xF5u
#define BME280_REG_DATA         0xF7u  /* press[3] temp[3] hum[2] = 8 bytes */

#define BME280_RESET_CMD        0xB6u
#define BME280_STATUS_MEASURING 0x08u
#define BME280_STATUS_IM_UPDATE 0x01u

#define BME280_CALIB1_LEN       26u
#define BME280_CALIB2_LEN       7u
#define BME280_DATA_LEN         8u

typedef enum {
  BME280_OK = 0,
  BME280_E_BUS = -1,       /* read/write callback reported failure */
  BME280_E_CHIP_ID = -2,   /* wrong device at this address (BMP280 = 0x58) */
  BME280_E_TIMEOUT = -3,   /* conversion or NVM copy did not finish in time */
  BME280_E_PARAM = -4,
} bme280_status_t;

/* Oversampling settings; x1 everywhere is the datasheet "weather monitoring" profile. */
typedef enum {
  BME280_OSRS_SKIP = 0, BME280_OSRS_X1 = 1, BME280_OSRS_X2 = 2,
  BME280_OSRS_X4 = 3,   BME280_OSRS_X8 = 4, BME280_OSRS_X16 = 5,
} bme280_osrs_t;

typedef int  (*bme280_read_fn)(void *ctx, uint8_t reg, uint8_t *buf, uint16_t len);
typedef int  (*bme280_write_fn)(void *ctx, uint8_t reg, uint8_t value);
typedef void (*bme280_delay_ms_fn)(uint32_t ms);

typedef struct {
  uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
  uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3; int16_t dig_P4; int16_t dig_P5;
  int16_t  dig_P6; int16_t dig_P7; int16_t dig_P8; int16_t dig_P9;
  uint8_t  dig_H1; int16_t dig_H2; uint8_t dig_H3; int16_t dig_H4; int16_t dig_H5;
  int8_t   dig_H6;
} bme280_calib_t;

typedef struct {
  bme280_read_fn     read;
  bme280_write_fn    write;
  bme280_delay_ms_fn delay_ms;
  void              *ctx;
  bme280_osrs_t      osrs_t, osrs_p, osrs_h;
  bme280_calib_t     calib;
} bme280_t;

typedef struct {
  uint32_t adc_P;   /* 20-bit */
  uint32_t adc_T;   /* 20-bit */
  uint32_t adc_H;   /* 16-bit */
} bme280_raw_t;

typedef struct {
  int32_t  temperature_c_x100;  /* 0.01 degC      e.g. 2508 = 25.08 degC  */
  uint32_t humidity_rh_x100;    /* 0.01 %RH       e.g. 4612 = 46.12 %RH   */
  uint32_t pressure_pa;         /* 1 Pa           e.g. 100653 Pa          */
  uint32_t pressure_q24_8;      /* raw datasheet output, Pa in Q24.8      */
  uint32_t humidity_q22_10;     /* raw datasheet output, %RH in Q22.10    */
} bme280_data_t;

/* Soft-reset, check chip ID, load calibration and apply oversampling settings. */
bme280_status_t bme280_init(bme280_t *dev);

/* Trigger one forced-mode conversion, wait for it, read and compensate. */
bme280_status_t bme280_measure(bme280_t *dev, bme280_data_t *out);

/* Worst-case conversion time in ms for the configured oversampling (datasheet 9.1). */
uint32_t bme280_max_conversion_ms(const bme280_t *dev);

/* ---- Pure functions (no I/O) -- exposed for unit testing --------------- */
void bme280_parse_calib(bme280_calib_t *c, const uint8_t c1[BME280_CALIB1_LEN],
                        const uint8_t c2[BME280_CALIB2_LEN]);
void bme280_parse_raw(bme280_raw_t *raw, const uint8_t data[BME280_DATA_LEN]);
void bme280_compensate(const bme280_calib_t *c, const bme280_raw_t *raw, bme280_data_t *out);

#endif /* BME280_H */
