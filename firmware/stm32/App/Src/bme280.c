#include "bme280.h"

#include <stddef.h>

/* ---- Little-endian helpers --------------------------------------------- */
static uint16_t le_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static int16_t  le_s16(const uint8_t *p) { return (int16_t)le_u16(p); }

/* ======================================================================== */

void bme280_parse_calib(bme280_calib_t *c, const uint8_t c1[BME280_CALIB1_LEN],
                        const uint8_t c2[BME280_CALIB2_LEN])
{
  /* 0x88..0x9F: temperature and pressure words, 0xA1: dig_H1 (0xA0 unused). */
  c->dig_T1 = le_u16(&c1[0]);
  c->dig_T2 = le_s16(&c1[2]);
  c->dig_T3 = le_s16(&c1[4]);
  c->dig_P1 = le_u16(&c1[6]);
  c->dig_P2 = le_s16(&c1[8]);
  c->dig_P3 = le_s16(&c1[10]);
  c->dig_P4 = le_s16(&c1[12]);
  c->dig_P5 = le_s16(&c1[14]);
  c->dig_P6 = le_s16(&c1[16]);
  c->dig_P7 = le_s16(&c1[18]);
  c->dig_P8 = le_s16(&c1[20]);
  c->dig_P9 = le_s16(&c1[22]);
  c->dig_H1 = c1[25];

  /*
   * 0xE1..0xE7. dig_H4/dig_H5 are signed 12-bit values packed across three
   * bytes, sharing 0xE5 nibbles. The MSB byte must be sign-extended BEFORE
   * shifting -- getting this wrong gives humidity readings that are fine at
   * room temperature and drift badly elsewhere.
   *   dig_H4 = 0xE4[7:0] << 4 | 0xE5[3:0]
   *   dig_H5 = 0xE6[7:0] << 4 | 0xE5[7:4]
   */
  c->dig_H2 = le_s16(&c2[0]);
  c->dig_H3 = c2[2];
  c->dig_H4 = (int16_t)(((int16_t)(int8_t)c2[3] * 16) | (int16_t)(c2[4] & 0x0F));
  c->dig_H5 = (int16_t)(((int16_t)(int8_t)c2[5] * 16) | (int16_t)(c2[4] >> 4));
  c->dig_H6 = (int8_t)c2[6];
}

void bme280_parse_raw(bme280_raw_t *raw, const uint8_t d[BME280_DATA_LEN])
{
  /* press_msb, press_lsb, press_xlsb[7:4], temp_msb, temp_lsb, temp_xlsb[7:4], hum_msb, hum_lsb */
  raw->adc_P = ((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | ((uint32_t)d[2] >> 4);
  raw->adc_T = ((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | ((uint32_t)d[5] >> 4);
  raw->adc_H = ((uint32_t)d[6] << 8)  |  (uint32_t)d[7];
}

/* ---- Datasheet compensation formulas ----------------------------------- */

static int32_t compensate_t_fine(const bme280_calib_t *c, int32_t adc_T)
{
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)c->dig_T1 << 1))) * ((int32_t)c->dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)c->dig_T1)) *
                    ((adc_T >> 4) - ((int32_t)c->dig_T1))) >> 12) *
                  ((int32_t)c->dig_T3)) >> 14;
  return var1 + var2;
}

/* Returns pressure in Pa as unsigned Q24.8 (e.g. 24674867 = 96386.2 Pa). */
static uint32_t compensate_pressure(const bme280_calib_t *c, int32_t adc_P, int32_t t_fine)
{
  int64_t var1 = ((int64_t)t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)c->dig_P6;
  var2 = var2 + ((var1 * (int64_t)c->dig_P5) * 131072);           /* << 17 */
  var2 = var2 + (((int64_t)c->dig_P4) * 34359738368LL);            /* << 35 */
  var1 = ((var1 * var1 * (int64_t)c->dig_P3) >> 8) + ((var1 * (int64_t)c->dig_P2) * 4096);
  var1 = ((((int64_t)1) << 47) + var1) * ((int64_t)c->dig_P1) >> 33;
  if (var1 == 0) {
    return 0; /* avoid division by zero (sensor not calibrated) */
  }
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)c->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)c->dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)c->dig_P7) << 4);
  return (uint32_t)p;
}

/* Returns humidity in %RH as unsigned Q22.10 (e.g. 47445 = 46.333 %RH). */
static uint32_t compensate_humidity(const bme280_calib_t *c, int32_t adc_H, int32_t t_fine)
{
  int32_t v = t_fine - ((int32_t)76800);
  v = (((((adc_H << 14) - (((int32_t)c->dig_H4) << 20) - (((int32_t)c->dig_H5) * v)) +
         ((int32_t)16384)) >> 15) *
       (((((((v * ((int32_t)c->dig_H6)) >> 10) *
            (((v * ((int32_t)c->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
          ((int32_t)2097152)) * ((int32_t)c->dig_H2) + 8192) >> 14));
  v = v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)c->dig_H1)) >> 4);
  v = (v < 0) ? 0 : v;
  v = (v > 419430400) ? 419430400 : v;
  return (uint32_t)(v >> 12);
}

void bme280_compensate(const bme280_calib_t *c, const bme280_raw_t *raw, bme280_data_t *out)
{
  int32_t t_fine = compensate_t_fine(c, (int32_t)raw->adc_T);

  out->temperature_c_x100 = (t_fine * 5 + 128) >> 8;
  out->pressure_q24_8     = compensate_pressure(c, (int32_t)raw->adc_P, t_fine);
  out->humidity_q22_10    = compensate_humidity(c, (int32_t)raw->adc_H, t_fine);

  out->pressure_pa      = (out->pressure_q24_8 + 128u) >> 8;                  /* round */
  out->humidity_rh_x100 = (out->humidity_q22_10 * 100u + 512u) >> 10;         /* round */
}

/* ======================================================================== */

uint32_t bme280_max_conversion_ms(const bme280_t *dev)
{
  /* Datasheet 9.1, t_measure,max in us:
   * 1250 + 2300*osT + (2300*osP + 575) + (2300*osH + 575), skip -> term omitted. */
  static const uint8_t os_factor[] = {0, 1, 2, 4, 8, 16};
  uint32_t us = 1250u + 2300u * os_factor[dev->osrs_t];
  if (dev->osrs_p != BME280_OSRS_SKIP) us += 2300u * os_factor[dev->osrs_p] + 575u;
  if (dev->osrs_h != BME280_OSRS_SKIP) us += 2300u * os_factor[dev->osrs_h] + 575u;
  return (us + 999u) / 1000u;
}

bme280_status_t bme280_init(bme280_t *dev)
{
  if (dev == NULL || dev->read == NULL || dev->write == NULL || dev->delay_ms == NULL) {
    return BME280_E_PARAM;
  }

  uint8_t id = 0;
  if (dev->read(dev->ctx, BME280_REG_CHIP_ID, &id, 1) != 0) return BME280_E_BUS;
  if (id != BME280_CHIP_ID) return BME280_E_CHIP_ID;

  if (dev->write(dev->ctx, BME280_REG_RESET, BME280_RESET_CMD) != 0) return BME280_E_BUS;
  dev->delay_ms(3); /* t_startup = 2 ms */

  /* Wait for the NVM calibration copy to finish (im_update cleared). */
  uint8_t status = BME280_STATUS_IM_UPDATE;
  for (int tries = 0; (status & BME280_STATUS_IM_UPDATE) != 0; tries++) {
    if (tries >= 10) return BME280_E_TIMEOUT;
    dev->delay_ms(2);
    if (dev->read(dev->ctx, BME280_REG_STATUS, &status, 1) != 0) return BME280_E_BUS;
  }

  uint8_t c1[BME280_CALIB1_LEN];
  uint8_t c2[BME280_CALIB2_LEN];
  if (dev->read(dev->ctx, BME280_REG_CALIB_00, c1, sizeof c1) != 0) return BME280_E_BUS;
  if (dev->read(dev->ctx, BME280_REG_CALIB_26, c2, sizeof c2) != 0) return BME280_E_BUS;
  bme280_parse_calib(&dev->calib, c1, c2);

  /* IIR filter off, standby irrelevant in forced mode. */
  if (dev->write(dev->ctx, BME280_REG_CONFIG, 0x00) != 0) return BME280_E_BUS;
  /* ctrl_hum only takes effect after a subsequent write to ctrl_meas (datasheet 5.4.3). */
  if (dev->write(dev->ctx, BME280_REG_CTRL_HUM, (uint8_t)dev->osrs_h) != 0) return BME280_E_BUS;
  uint8_t meas = (uint8_t)((dev->osrs_t << 5) | (dev->osrs_p << 2)); /* mode = sleep */
  if (dev->write(dev->ctx, BME280_REG_CTRL_MEAS, meas) != 0) return BME280_E_BUS;

  return BME280_OK;
}

bme280_status_t bme280_measure(bme280_t *dev, bme280_data_t *out)
{
  if (dev == NULL || out == NULL) return BME280_E_PARAM;

  uint8_t meas = (uint8_t)((dev->osrs_t << 5) | (dev->osrs_p << 2) | 0x01u); /* forced */
  if (dev->write(dev->ctx, BME280_REG_CTRL_MEAS, meas) != 0) return BME280_E_BUS;

  uint32_t budget = bme280_max_conversion_ms(dev);
  dev->delay_ms(budget);

  /* Normally already done; poll a little longer before declaring a timeout. */
  uint8_t status = 0;
  for (int tries = 0;; tries++) {
    if (dev->read(dev->ctx, BME280_REG_STATUS, &status, 1) != 0) return BME280_E_BUS;
    if ((status & BME280_STATUS_MEASURING) == 0) break;
    if (tries >= 10) return BME280_E_TIMEOUT;
    dev->delay_ms(1);
  }

  /* Burst-read all 8 data bytes in one transaction so T/P/H belong to the same sample. */
  uint8_t d[BME280_DATA_LEN];
  if (dev->read(dev->ctx, BME280_REG_DATA, d, sizeof d) != 0) return BME280_E_BUS;

  bme280_raw_t raw;
  bme280_parse_raw(&raw, d);
  bme280_compensate(&dev->calib, &raw, out);
  return BME280_OK;
}
