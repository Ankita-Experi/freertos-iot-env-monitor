/*
 * ESP32 Wi-Fi gateway: STM32 UART -> HTTP -> ThingSpeak
 *
 * Reads checksummed lines from the STM32 node on UART2 (GPIO16 RX / GPIO17 TX),
 * averages the samples received in each upload window and posts them to a
 * ThingSpeak channel. See docs/protocols.md for the line format.
 *
 * ThingSpeak fields:
 *   field1 temperature  [degC]     window mean
 *   field2 humidity     [%RH]      window mean
 *   field3 pressure     [hPa]      window mean
 *   field4 lost samples            seq gaps seen in the window (UART or node drops)
 *   field5 node flags              last STA flags byte (decimal)
 *   field6 CAN TEC                 last STA transmit error counter
 *   field7 node uptime  [h]
 *
 * Board: any ESP32 dev kit (Arduino core 2.x or 3.x). Copy secrets.h.example
 * to secrets.h and fill in Wi-Fi credentials and the channel write key.
 */

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>

#include "secrets.h"  // WIFI_SSID, WIFI_PASSWORD, THINGSPEAK_WRITE_KEY

// ---- Configuration ---------------------------------------------------------
static constexpr int      NODE_RX_PIN       = 16;      // <- STM32 PA9  (USART1 TX)
static constexpr int      NODE_TX_PIN       = 17;      // -> STM32 PA10 (USART1 RX)
static constexpr uint32_t NODE_BAUD         = 115200;
static constexpr uint32_t UPLOAD_PERIOD_MS  = 20000;   // ThingSpeak free tier: >= 15 s
static constexpr uint32_t NODE_SILENT_MS    = 10000;   // warn if no valid line for this long
static constexpr size_t   LINE_MAX          = 96;
static constexpr char     THINGSPEAK_URL[]  = "http://api.thingspeak.com/update";

HardwareSerial NodeSerial(2);

// ---- Window accumulator ---------------------------------------------------
struct Window {
  int64_t  temp_sum = 0;    // 0.01 degC
  int64_t  rh_sum = 0;      // 0.01 %RH
  int64_t  press_sum = 0;   // Pa
  uint32_t count = 0;
  uint32_t lost = 0;
  void reset() { *this = Window(); }
};

struct NodeStatus {
  bool     valid = false;
  uint8_t  flags = 0;
  uint8_t  tec = 0;
  uint8_t  rec = 0;
  uint32_t free_heap = 0;
  uint32_t uptime_s = 0;
};

static Window     g_window;
static NodeStatus g_status;
static bool       g_have_seq = false;
static uint32_t   g_last_seq = 0;
static uint32_t   g_last_valid_line_ms = 0;
static uint32_t   g_bad_checksum = 0;
static uint32_t   g_bad_format = 0;

// ---- Line parsing -----------------------------------------------------------

static int hexval(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Validates "$<body>*HH" in place; on success, NUL-terminates and returns body.
static char *validate_nmea(char *line)
{
  size_t n = strlen(line);
  if (n < 5 || line[0] != '$' || line[n - 3] != '*') return nullptr;
  int hi = hexval(line[n - 2]), lo = hexval(line[n - 1]);
  if (hi < 0 || lo < 0) return nullptr;

  uint8_t cs = 0;
  for (size_t i = 1; i < n - 3; i++) cs ^= (uint8_t)line[i];
  if (cs != (uint8_t)((hi << 4) | lo)) {
    g_bad_checksum++;
    return nullptr;
  }
  line[n - 3] = '\0';
  return &line[1];
}

static void handle_env(const char *args)
{
  unsigned long seq, rh, press;
  long temp;
  if (sscanf(args, "%lu,%ld,%lu,%lu", &seq, &temp, &rh, &press) != 4) {
    g_bad_format++;
    return;
  }

  // Sequence gaps = samples the node took but we never received.
  if (g_have_seq) {
    uint32_t expected = g_last_seq + 1;
    if ((uint32_t)seq != expected) {
      uint32_t gap = (uint32_t)seq - expected;
      if (gap < 1000) g_window.lost += gap;   // otherwise: node rebooted, seq restarted
    }
  }
  g_have_seq = true;
  g_last_seq = (uint32_t)seq;

  g_window.temp_sum  += temp;
  g_window.rh_sum    += (long)rh;
  g_window.press_sum += (long)press;
  g_window.count++;

  Serial.printf("ENV #%lu  T=%.2f C  RH=%.2f %%  P=%.2f hPa\n",
                seq, temp / 100.0, rh / 100.0, press / 100.0);
}

static void handle_status(const char *args)
{
  unsigned flags, tec, rec;
  unsigned long heap, uptime;
  if (sscanf(args, "%x,%u,%u,%lu,%lu", &flags, &tec, &rec, &heap, &uptime) != 5) {
    g_bad_format++;
    return;
  }
  g_status = { true, (uint8_t)flags, (uint8_t)tec, (uint8_t)rec, (uint32_t)heap, (uint32_t)uptime };
  Serial.printf("STA flags=0x%02X TEC=%u REC=%u heap=%lu up=%lus\n", flags, tec, rec, heap, uptime);
}

static void handle_line(char *line)
{
  char *body = validate_nmea(line);
  if (body == nullptr) return;

  g_last_valid_line_ms = millis();
  if (strncmp(body, "ENV,", 4) == 0) {
    handle_env(body + 4);
  } else if (strncmp(body, "STA,", 4) == 0) {
    handle_status(body + 4);
  } else {
    g_bad_format++;
  }
}

static void poll_node_uart()
{
  static char   buf[LINE_MAX];
  static size_t len = 0;
  static bool   overflow = false;

  while (NodeSerial.available()) {
    char c = (char)NodeSerial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!overflow && len > 0) {
        buf[len] = '\0';
        handle_line(buf);
      }
      len = 0;
      overflow = false;
    } else if (c == '$') {
      len = 0;               // resynchronise on start-of-line
      overflow = false;
      buf[len++] = c;
    } else if (len < LINE_MAX - 1) {
      buf[len++] = c;
    } else {
      overflow = true;       // discard the rest of an over-long line
    }
  }
}

// ---- Wi-Fi ------------------------------------------------------------------

static void wifi_maintain()
{
  static uint32_t next_attempt_ms = 0;
  static uint32_t backoff_ms = 1000;

  if (WiFi.status() == WL_CONNECTED) {
    backoff_ms = 1000;
    return;
  }
  if ((int32_t)(millis() - next_attempt_ms) < 0) return;

  Serial.printf("Wi-Fi: connecting to '%s'...\n", WIFI_SSID);
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  next_attempt_ms = millis() + backoff_ms;
  backoff_ms = min<uint32_t>(backoff_ms * 2, 60000);   // exponential backoff, capped at 60 s
}

// ---- Upload -----------------------------------------------------------------

static void upload_window()
{
  if (g_window.count == 0) {
    Serial.println("upload: no samples in window, skipping");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("upload: Wi-Fi down, window kept for next attempt");
    return;
  }

  const double n = (double)g_window.count;
  String body = String("api_key=") + THINGSPEAK_WRITE_KEY +
                "&field1=" + String(g_window.temp_sum / n / 100.0, 2) +
                "&field2=" + String(g_window.rh_sum / n / 100.0, 2) +
                "&field3=" + String(g_window.press_sum / n / 100.0, 2) +
                "&field4=" + String(g_window.lost);
  if (g_status.valid) {
    body += "&field5=" + String(g_status.flags) +
            "&field6=" + String(g_status.tec) +
            "&field7=" + String(g_status.uptime_s / 3600.0, 2);
  }

  HTTPClient http;
  http.setTimeout(5000);
  http.begin(THINGSPEAK_URL);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int code = http.POST(body);
  String resp = http.getString();
  http.end();

  // ThingSpeak returns the new entry id, or "0" if the update was rejected (e.g. rate limit).
  if (code == 200 && resp.toInt() > 0) {
    Serial.printf("upload: ok, entry %s (%lu samples, %lu lost)\n", resp.c_str(),
                  (unsigned long)g_window.count, (unsigned long)g_window.lost);
    g_window.reset();
  } else {
    Serial.printf("upload: failed (HTTP %d, body '%s'), window kept\n", code, resp.c_str());
  }
}

// ---- Arduino entry points ---------------------------------------------------

void setup()
{
  Serial.begin(115200);
  NodeSerial.begin(NODE_BAUD, SERIAL_8N1, NODE_RX_PIN, NODE_TX_PIN);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  Serial.println("\nESP32 gateway: STM32 UART -> ThingSpeak");
  g_last_valid_line_ms = millis();
}

void loop()
{
  static uint32_t last_upload_ms = 0;
  static uint32_t last_silence_warn_ms = 0;

  poll_node_uart();
  wifi_maintain();

  uint32_t now = millis();
  if (now - last_upload_ms >= UPLOAD_PERIOD_MS) {
    last_upload_ms = now;
    upload_window();
    if (g_bad_checksum || g_bad_format) {
      Serial.printf("uart: %lu bad checksums, %lu malformed lines so far\n",
                    (unsigned long)g_bad_checksum, (unsigned long)g_bad_format);
    }
  }

  if (now - g_last_valid_line_ms > NODE_SILENT_MS && now - last_silence_warn_ms > NODE_SILENT_MS) {
    last_silence_warn_ms = now;
    Serial.println("uart: no valid data from STM32 - check GPIO16 <- PA9 and common GND");
  }

  delay(2);
}
