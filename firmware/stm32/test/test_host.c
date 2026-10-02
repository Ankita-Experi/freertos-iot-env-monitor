/*
 * Host-side unit tests for the hardware-independent firmware modules.
 *
 *   make -C firmware/stm32/test     (or: cmake/ctest, see README)
 *
 * BME280: the integer compensation is checked against
 *   (a) the worked example in the Bosch BMP280 datasheet (sec. 3.12), whose
 *       temperature/pressure formulas and coefficients are identical, and
 *   (b) the datasheet's floating-point reference formulas, swept across the
 *       sensor's full operating range.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bme280.h"
#include "protocol.h"

static int g_failures;
static int g_checks;

#define CHECK(cond, ...)                                              \
  do {                                                                \
    g_checks++;                                                       \
    if (!(cond)) {                                                    \
      g_failures++;                                                   \
      printf("FAIL %s:%d: ", __FILE__, __LINE__);                     \
      printf(__VA_ARGS__);                                            \
      printf("\n");                                                   \
    }                                                                 \
  } while (0)

/* ---- Datasheet floating-point reference --------------------------------- */

static double ref_t_fine(const bme280_calib_t *c, double adc_T)
{
  double v1 = (adc_T / 16384.0 - c->dig_T1 / 1024.0) * c->dig_T2;
  double v2 = (adc_T / 131072.0 - c->dig_T1 / 8192.0);
  v2 = v2 * v2 * c->dig_T3;
  return v1 + v2;
}

static double ref_pressure(const bme280_calib_t *c, double adc_P, double t_fine)
{
  double v1 = t_fine / 2.0 - 64000.0;
  double v2 = v1 * v1 * c->dig_P6 / 32768.0;
  v2 = v2 + v1 * c->dig_P5 * 2.0;
  v2 = v2 / 4.0 + c->dig_P4 * 65536.0;
  v1 = (c->dig_P3 * v1 * v1 / 524288.0 + c->dig_P2 * v1) / 524288.0;
  v1 = (1.0 + v1 / 32768.0) * c->dig_P1;
  if (v1 == 0.0) return 0;
  double p = 1048576.0 - adc_P;
  p = (p - v2 / 4096.0) * 6250.0 / v1;
  v1 = c->dig_P9 * p * p / 2147483648.0;
  v2 = p * c->dig_P8 / 32768.0;
  return p + (v1 + v2 + c->dig_P7) / 16.0;
}

static double ref_humidity(const bme280_calib_t *c, double adc_H, double t_fine)
{
  double h = t_fine - 76800.0;
  h = (adc_H - (c->dig_H4 * 64.0 + c->dig_H5 / 16384.0 * h)) *
      (c->dig_H2 / 65536.0 * (1.0 + c->dig_H6 / 67108864.0 * h * (1.0 + c->dig_H3 / 67108864.0 * h)));
  h = h * (1.0 - c->dig_H1 * h / 524288.0);
  if (h > 100.0) h = 100.0;
  if (h < 0.0) h = 0.0;
  return h;
}

/* ---- Calibration fixture ------------------------------------------------ */

/* T/P coefficients from the BMP280 datasheet example; H coefficients typical of a real part. */
static const bme280_calib_t k_calib = {
  .dig_T1 = 27504, .dig_T2 = 26435, .dig_T3 = -1000,
  .dig_P1 = 36477, .dig_P2 = -10685, .dig_P3 = 3024, .dig_P4 = 2855, .dig_P5 = 140,
  .dig_P6 = -7, .dig_P7 = 15500, .dig_P8 = -14600, .dig_P9 = 6000,
  .dig_H1 = 75, .dig_H2 = 362, .dig_H3 = 0, .dig_H4 = 313, .dig_H5 = 50, .dig_H6 = 30,
};

/* Serialise a calibration struct into the sensor's register image. */
static void calib_to_regs(const bme280_calib_t *c, uint8_t c1[26], uint8_t c2[7])
{
  const uint16_t w[12] = {
    c->dig_T1, (uint16_t)c->dig_T2, (uint16_t)c->dig_T3, c->dig_P1, (uint16_t)c->dig_P2,
    (uint16_t)c->dig_P3, (uint16_t)c->dig_P4, (uint16_t)c->dig_P5, (uint16_t)c->dig_P6,
    (uint16_t)c->dig_P7, (uint16_t)c->dig_P8, (uint16_t)c->dig_P9,
  };
  memset(c1, 0, 26);
  for (int i = 0; i < 12; i++) {
    c1[2 * i] = (uint8_t)(w[i] & 0xFF);
    c1[2 * i + 1] = (uint8_t)(w[i] >> 8);
  }
  c1[25] = c->dig_H1;
  c2[0] = (uint8_t)((uint16_t)c->dig_H2 & 0xFF);
  c2[1] = (uint8_t)((uint16_t)c->dig_H2 >> 8);
  c2[2] = c->dig_H3;
  c2[3] = (uint8_t)(((uint16_t)c->dig_H4 >> 4) & 0xFF);
  c2[4] = (uint8_t)((c->dig_H4 & 0x0F) | ((c->dig_H5 & 0x0F) << 4));
  c2[5] = (uint8_t)(((uint16_t)c->dig_H5 >> 4) & 0xFF);
  c2[6] = (uint8_t)c->dig_H6;
}

/* ---- Tests -------------------------------------------------------------- */

static void test_datasheet_example(void)
{
  bme280_raw_t raw = { .adc_T = 519888, .adc_P = 415148, .adc_H = 0 };
  bme280_data_t d;
  bme280_compensate(&k_calib, &raw, &d);

  CHECK(d.temperature_c_x100 == 2508, "T = %ld, expected 2508 (25.08 C)", (long)d.temperature_c_x100);
  double p = d.pressure_q24_8 / 256.0;
  CHECK(fabs(p - 100653.27) < 0.5, "P = %.2f Pa, expected 100653.27", p);
  CHECK(d.pressure_pa == 100653, "P rounded = %lu", (unsigned long)d.pressure_pa);
}

static void test_calib_parsing_roundtrip(void)
{
  /* Include negative H4/H5 to exercise the 12-bit sign extension. */
  bme280_calib_t in = k_calib;
  in.dig_H4 = -123;
  in.dig_H5 = -2047;
  in.dig_H6 = -30;

  uint8_t c1[26], c2[7];
  calib_to_regs(&in, c1, c2);
  bme280_calib_t out;
  bme280_parse_calib(&out, c1, c2);
  CHECK(memcmp(&in, &out, sizeof in) == 0, "calibration round-trip mismatch (H4=%d H5=%d H6=%d)",
        out.dig_H4, out.dig_H5, out.dig_H6);
}

static void test_raw_parsing(void)
{
  /* adc_P = 0x655AC, adc_T = 0x7EED0, adc_H = 0x6B4D */
  const uint8_t d[8] = { 0x65, 0x5A, 0xC0, 0x7E, 0xED, 0x00, 0x6B, 0x4D };
  bme280_raw_t r;
  bme280_parse_raw(&r, d);
  CHECK(r.adc_P == 0x655AC, "adc_P = 0x%lX", (unsigned long)r.adc_P);
  CHECK(r.adc_T == 0x7EED0, "adc_T = 0x%lX", (unsigned long)r.adc_T);
  CHECK(r.adc_H == 0x6B4D,  "adc_H = 0x%lX", (unsigned long)r.adc_H);
}

static void test_against_float_reference(void)
{
  double worst_t = 0, worst_p = 0, worst_h = 0;

  for (uint32_t adc_T = 380000; adc_T <= 640000; adc_T += 6500) {       /* ~ -38..+76 C */
    for (uint32_t adc_P = 250000; adc_P <= 500000; adc_P += 12500) {
      for (uint32_t adc_H = 20000; adc_H <= 50000; adc_H += 1500) {
        bme280_raw_t raw = { .adc_T = adc_T, .adc_P = adc_P, .adc_H = adc_H };
        bme280_data_t d;
        bme280_compensate(&k_calib, &raw, &d);

        double tf = ref_t_fine(&k_calib, adc_T);
        double t  = tf / 5120.0;
        double p  = ref_pressure(&k_calib, adc_P, tf);
        double h  = ref_humidity(&k_calib, adc_H, tf);

        double et = fabs(d.temperature_c_x100 / 100.0 - t);
        double ep = fabs(d.pressure_q24_8 / 256.0 - p);
        double eh = fabs(d.humidity_q22_10 / 1024.0 - h);
        if (et > worst_t) worst_t = et;
        if (p > 30000 && p < 110000 && ep > worst_p) worst_p = ep;  /* sensor's spec range */
        if (eh > worst_h) worst_h = eh;
      }
    }
  }
  printf("  int vs float worst-case error: T %.3f C, P %.2f Pa, RH %.3f %%\n", worst_t, worst_p, worst_h);
  CHECK(worst_t <= 0.011, "temperature error %.4f C", worst_t);
  CHECK(worst_p <= 1.0,   "pressure error %.3f Pa", worst_p);
  CHECK(worst_h <= 0.01,  "humidity error %.4f %%RH", worst_h);
}

static void test_conversion_time(void)
{
  bme280_t dev = { .osrs_t = BME280_OSRS_X1, .osrs_p = BME280_OSRS_X1, .osrs_h = BME280_OSRS_X1 };
  CHECK(bme280_max_conversion_ms(&dev) == 10, "x1/x1/x1 -> %lu ms, datasheet 9.3 ms",
        (unsigned long)bme280_max_conversion_ms(&dev));
  dev.osrs_p = BME280_OSRS_X16;
  CHECK(bme280_max_conversion_ms(&dev) == 44, "x1/x16/x1 -> %lu ms",
        (unsigned long)bme280_max_conversion_ms(&dev));
}

static void test_can_env_roundtrip(void)
{
  env_sample_t in = { .seq = 0x1234, .temperature_c_x100 = -1575, .humidity_rh_x100 = 4612,
                      .pressure_pa = 101325 };
  uint8_t d[8];
  protocol_pack_env(&in, d);
  const uint8_t expect[8] = { 0xD9, 0xF9, 0x04, 0x12, 0xCD, 0x8B, 0x01, 0x34 };
  CHECK(memcmp(d, expect, 8) == 0, "ENV frame bytes %02X %02X %02X %02X %02X %02X %02X %02X",
        d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);

  env_sample_t out;
  protocol_unpack_env(d, &out);
  CHECK(out.temperature_c_x100 == -1575 && out.humidity_rh_x100 == 4612 &&
        out.pressure_pa == 101325 && out.seq == 0x34, "ENV unpack mismatch");

  /* Saturation instead of wrap-around. */
  in.temperature_c_x100 = 99999;
  in.humidity_rh_x100 = 20000;
  protocol_pack_env(&in, d);
  protocol_unpack_env(d, &out);
  CHECK(out.temperature_c_x100 == 32767 && out.humidity_rh_x100 == 10000, "ENV saturation");
}

static void test_can_commands(void)
{
  can_command_t c;
  const uint8_t set500[3] = { CAN_CMD_SET_PERIOD, 0xF4, 0x01 };
  CHECK(protocol_parse_command(0x200, set500, 3, &c) == 0 && c.period_ms == 500, "set period 500");
  CHECK(protocol_parse_command(0x20F, set500, 3, &c) == 0, "0x20F accepted");
  CHECK(protocol_parse_command(0x210, set500, 3, &c) != 0, "0x210 rejected");
  CHECK(protocol_parse_command(0x200, set500, 2, &c) != 0, "short DLC rejected");

  const uint8_t set50[3] = { CAN_CMD_SET_PERIOD, 50, 0 };
  CHECK(protocol_parse_command(0x200, set50, 3, &c) != 0, "50 ms out of range rejected");

  const uint8_t req[1] = { CAN_CMD_REQUEST_STATUS };
  CHECK(protocol_parse_command(0x201, req, 1, &c) == 0 && c.id == CAN_CMD_REQUEST_STATUS, "status req");

  const uint8_t bad[1] = { 0x7F };
  CHECK(protocol_parse_command(0x200, bad, 1, &c) != 0, "unknown command rejected");
}

static void test_uplink_lines(void)
{
  char buf[UPLINK_LINE_MAX];
  env_sample_t s = { .seq = 42, .temperature_c_x100 = 2345, .humidity_rh_x100 = 4120,
                     .pressure_pa = 101325 };
  size_t n = protocol_format_env_line(&s, buf, sizeof buf);
  /* XOR of "ENV,42,2345,4120,101325" */
  uint8_t cs = protocol_nmea_checksum("ENV,42,2345,4120,101325", 23);
  char expect[64];
  snprintf(expect, sizeof expect, "$ENV,42,2345,4120,101325*%02X\r\n", cs);
  CHECK(n == strlen(expect) && strcmp(buf, expect) == 0, "ENV line '%s'", buf);

  node_status_t st = { .flags = 0x21, .tec = 8, .rec = 0, .free_heap_bytes = 17000, .uptime_s = 3600 };
  n = protocol_format_status_line(&st, buf, sizeof buf);
  CHECK(n > 0 && strncmp(buf, "$STA,21,8,0,17000,3600*", 23) == 0, "STA line '%s'", buf);

  char tiny[16];
  CHECK(protocol_format_env_line(&s, tiny, sizeof tiny) == 0, "too-small buffer rejected");
}

int main(void)
{
  printf("Running host unit tests\n");
  test_datasheet_example();
  test_calib_parsing_roundtrip();
  test_raw_parsing();
  test_against_float_reference();
  test_conversion_time();
  test_can_env_roundtrip();
  test_can_commands();
  test_uplink_lines();
  printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
