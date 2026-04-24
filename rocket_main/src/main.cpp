#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <SD.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <RTClib.h>
#include <Adafruit_NeoPixel.h>

// 引入感測器函式庫
#include "Adafruit_SHT31.h"
#include "Adafruit_BMP3XX.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_ADXL375.h"
#include "ICM42688.h"
#include "error_codes.h"

#ifndef ARDUINO_USB_MODE
#define ARDUINO_USB_MODE 0
#endif
#ifndef ARDUINO_USB_CDC_ON_BOOT
#define ARDUINO_USB_CDC_ON_BOOT 0
#endif
#if ARDUINO_USB_CDC_ON_BOOT != 0
#error "USB CDC must be disabled. Use UART0 (COM) for telemetry."
#endif

// On ESP32-S3 with ARDUINO_USB_CDC_ON_BOOT=0, Serial maps to UART0 (COM).
#define GS_SERIAL_PORT Serial

// Set to 1 for debug logs over UART0. Keep 0 when UART0 is used for binary packets.
#define DEBUG_SERIAL 0
#if DEBUG_SERIAL
#define DBG_PRINTF(...) GS_SERIAL_PORT.printf(__VA_ARGS__)
#define DBG_PRINTLN(msg) GS_SERIAL_PORT.println(msg)
#else
#define DBG_PRINTF(...) do {} while (0)
#define DBG_PRINTLN(msg) do {} while (0)
#endif

// --- 腳位定義 (對應硬體表) ---
#define I2C_A_SDA 20
#define I2C_A_SCL 19
#define I2C_B_SDA 9
#define I2C_B_SCL 10

#define UART_GPS_TX 35
#define UART_GPS_RX 45
#define UART_GPS_PPS 47
#define UART_GPS_BAUD 230400

#define UART_SUB_TX 17
#define UART_SUB_RX 18
#define UART_SUB_BAUD 115200

#define LORA_SCK 42
#define LORA_MISO 41
#define LORA_MOSI 40
#define LORA_NSS 39
#define LORA_RST 38
#define LORA_DIO0 37
#define LORA_DIO1 36
#define LORA_FREQ 433250000
#define LORA_SYNC_WORD 0x12
#define LORA_TX_POWER 22
#define LORA_BW 125E3
#define LORA_SF 9
#define LORA_CR 7
#define LORA_PREAMBLE_LEN 8

#define SD_CS 11
#define SD_MOSI 12
#define SD_SCK 13
#define SD_MISO 14

#define BUZZER_PIN 8
#define BUZZER_CH 0
#define BUZZER_RES 10

#define WS2812_PIN 48
#define WS2812_COUNT 1

#define BATTERY_ADC_PIN 4
#define DS3231_SQW_PIN 3
#define WATER_DETECT_PIN 15
#define SAFETY_SWITCH_PIN 21
#define SEPARATION_SERVO_PIN 16
#define CAMERA_A_TRIGGER_PIN 5
#define CAMERA_B_TRIGGER_PIN 6

#define NOTE_DO 262
#define NOTE_RE 294
#define NOTE_MI 330
#define NOTE_SOL 392
#define NOTE_HDO 523

// --- 建立感測器物件 ---
Adafruit_SHT31 sht31(&Wire1);
Adafruit_BMP3XX bmp;
Adafruit_ADXL375 accel_a(12345, &Wire);
Adafruit_ADXL375 accel_b(12345, &Wire1);
Adafruit_ADXL375 *accel = &accel_a;
ICM42688 imu_a_68(Wire, 0x68);   // IMU42688 地址通常是 0x68 或 0x69
ICM42688 imu_a_69(Wire, 0x69);
ICM42688 imu_b_68(Wire1, 0x68);
ICM42688 imu_b_69(Wire1, 0x69);
ICM42688 *imu = &imu_a_68;
uint8_t imu_addr = 0x68;
const char *imu_bus_label = "I2C_A";
const char *adxl_bus_label = "I2C_A";
RTC_DS3231 rtc;
Adafruit_NeoPixel status_led(WS2812_COUNT, WS2812_PIN, NEO_GRB + NEO_KHZ800);
HardwareSerial GPSSerial(1);
HardwareSerial BackupSerial(2);
SPIClass loraSPI(HSPI);
#if defined(VSPI)
SPIClass sdSPI(VSPI);
#else
SPIClass sdSPI(FSPI);
#endif
File sd_log;
bool lora_ready = false;
bool sd_ready = false;

// --- 狀態旗標 (檢查誰活著) ---
bool status_sht = false;
bool status_bmp = false;
bool status_adxl = false;
bool status_imu = false;
bool rtc_ready = false;

// 與地面站一致的飛行狀態定義
static const uint8_t kStateTest = 0;
static const uint8_t kStateIdle = 1;
static const uint8_t kStatePreflight = 2;
static const uint8_t kStateAscent = 3;
static const uint8_t kStateApogee = 4;
static const uint8_t kStateDescent = 5;
static const uint8_t kStateLanded = 6;
static const uint8_t kStateAbort = 99;

// --- 固定長度封包格式（47 bytes） ---
static const uint8_t kFrameLen = 47;
static const uint8_t kFrameHeader0 = 0x55;
static const uint8_t kFrameHeader1 = 0xAA;
static const float kG = 9.80665f;
static const float kBatteryRTop = 200000.0f;  // ohms (to battery)
static const float kBatteryRBottom = 100000.0f; // ohms (to GND)
static const float kBatteryDivider = kBatteryRBottom / (kBatteryRTop + kBatteryRBottom);
// Set to 0 to disable low-battery error.
static const uint16_t kBatteryLowMv = 0;

// Flight state machine thresholds (document-defined)
static const float kLiftoffAccelG = 3.0f;
static const float kLiftoffVSpeedMs = 10.0f;
static const float kDrogueAltMinM = 3000.0f;
static const float kDrogueVSpeedMaxMs = 5.0f;
static const float kMainAltMaxM = 500.0f;
static const float kMainVSpeedMaxMs = 5.0f;
static const float kLandingAltMaxM = 30.0f;
static const float kLandingVSpeedMaxMs = 5.0f;
static const float kApogeeDetectVSpeedMs = -0.5f;
static const uint32_t kLiftoffHoldMs = 500;
static const uint32_t kDrogueHoldMs = 500;
static const uint32_t kMainHoldMs = 500;
static const uint32_t kLandingHoldMs = 500;
static const uint32_t kAfterApogeeMs = 500;
static const float kDrogueTimeS = 25.0f;
static const float kMainTimeS = 190.0f;
static const uint8_t kRtcSyncLineLen = 64;
static const uint32_t kGroundStationBaud = 115200;
static const bool kEnableBmp390 = false;
static const bool kEnableStartupTone = false;
static const int16_t kInvalidBaroAltDm = -32768;
static const uint32_t kI2cClockHz = 100000;
static const uint32_t kI2cClockFallbackHz1 = 50000;
static const uint32_t kI2cClockFallbackHz2 = 25000;
static const uint16_t kI2cTimeoutMs = 20;
static const uint8_t kAdxlAltAddress = 0x1D;
// 0 = keep retrying forever until sensors respond.
static const uint32_t kSensorRetryWindowMs = 0;
static const uint32_t kSensorRetryIntervalMs = 1000;
static const bool kEnableBackupHeartbeat = false;
static const bool kEnableSetupAsciiDiagnostics = true;
static const uint32_t kDeferredInitStepGapMs = 5;
static const uint32_t kGpsSentenceTimeoutMs = 3000;
static const uint32_t kGpsBaudRotateIntervalMs = 4000;
static const uint16_t kGpsCourseMinSpeedDms = 20; // 2.0 m/s
static const uint32_t kGpsBaudCandidates[] = {
  UART_GPS_BAUD, 115200, 57600, 38400, 9600
};
static const uint8_t kGpsBaudCandidateCount = static_cast<uint8_t>(
  sizeof(kGpsBaudCandidates) / sizeof(kGpsBaudCandidates[0])
);
static const uint32_t kLoraRetryIntervalMs = 2000;
static const uint8_t kLoraTxFailReinitThreshold = 3;

enum BootInitStage : uint8_t {
  kBootInitI2c = 0,
  kBootInitLoRa,
  kBootInitSd,
  kBootInitRtc,
  kBootInitEnv,
  kBootInitMotion,
  kBootInitDone,
};

static int32_t gps_lat_raw = 0;
static int32_t gps_lon_raw = 0;
static int16_t gps_alt_dm = 0;
static int16_t gps_speed_dms = 0;
static uint16_t gps_course_ddeg = 0;
static uint8_t gps_sat_count = 0;
static uint32_t gps_last_fix_ms = 0;
static uint32_t gps_last_sentence_ms = 0;
static uint32_t gps_last_course_ms = 0;
static uint32_t gps_last_baud_switch_ms = 0;
static uint8_t gps_baud_idx = 0;
static const uint32_t kGpsTimeoutMs = 2000;
static bool gps_fix_valid = false;
static bool gps_course_valid = false;

static uint8_t flight_state = kStateIdle;
static uint32_t liftoff_ms = 0;
static bool apogee_seen = false;
static uint32_t apogee_ms = 0;
static uint32_t liftoff_cond_ms = 0;
static uint32_t drogue_cond_ms = 0;
static uint32_t main_cond_ms = 0;
static uint32_t landing_cond_ms = 0;
static bool main_deployed = false;
static uint32_t last_heartbeat_ms = 0;
static float alt_zero_m = 0.0f;
static bool alt_zero_set = false;
static HardwareSerial &ground_station_serial = GS_SERIAL_PORT;
static uint32_t sensor_init_window_start_ms = 0;
static uint32_t last_sensor_retry_ms = 0;
static uint32_t i2c_a_runtime_clock_hz = kI2cClockHz;
static uint32_t i2c_b_runtime_clock_hz = kI2cClockHz;
static BootInitStage boot_init_stage = kBootInitI2c;
static uint32_t last_deferred_init_ms = 0;
static uint32_t last_lora_retry_ms = 0;
static uint8_t lora_tx_fail_count = 0;

static void write_sd_csv_header() {
  if (!sd_log) {
    return;
  }
  sd_log.println("time_ms,rtc,lat,lon,gps_alt,baro_alt,speed,heading,sat,roll,pitch,accx,accy,accz,gyro_x,gyro_y,gyro_z,battery,servo_power_mv,servo_angle_deg,temp,hum,pressure_pa,status,error,water");
}

static void write_sd_csv_line(uint32_t time_ms,
                              uint32_t rtc_unix,
                              float lat_deg,
                              float lon_deg,
                              float gps_alt_m,
                              float baro_alt_m,
                              float speed_ms,
                              float heading_deg,
                              uint8_t sat,
                              float roll_deg,
                              float pitch_deg,
                              float accx_g,
                              float accy_g,
                              float accz_g,
                              float gyro_x_dps,
                              float gyro_y_dps,
                              float gyro_z_dps,
                              float battery_v,
                              float servo_power_mv,
                              float servo_angle_deg,
                              float temp_c,
                              float hum,
                              float pressure_pa,
                              uint8_t status,
                              uint8_t error,
                              uint8_t water) {
  if (!sd_log) {
    return;
  }
  char line[256];
  int n = snprintf(line, sizeof(line),
                   "%lu,%lu,%.7f,%.7f,%.1f,%.1f,%.1f,%.1f,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.3f,%.0f,%.1f,%.2f,%.1f,%.0f,%u,%u,%u\n",
                   static_cast<unsigned long>(time_ms),
                   static_cast<unsigned long>(rtc_unix),
                   lat_deg, lon_deg, gps_alt_m, baro_alt_m, speed_ms, heading_deg, sat,
                   roll_deg, pitch_deg, accx_g, accy_g, accz_g, gyro_x_dps, gyro_y_dps, gyro_z_dps,
                   battery_v, servo_power_mv, servo_angle_deg, temp_c, hum, pressure_pa,
                   status, error, water);
  if (n > 0) {
    sd_log.write(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(n));
  }
}

static inline void write_u16_le(uint8_t *buf, int idx, uint16_t v) {
  buf[idx] = static_cast<uint8_t>(v & 0xFF);
  buf[idx + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

static inline void write_u32_le(uint8_t *buf, int idx, uint32_t v) {
  buf[idx] = static_cast<uint8_t>(v & 0xFF);
  buf[idx + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  buf[idx + 2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  buf[idx + 3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

static inline void write_i16_le(uint8_t *buf, int idx, int16_t v) {
  write_u16_le(buf, idx, static_cast<uint16_t>(v));
}

static inline void write_i32_le(uint8_t *buf, int idx, int32_t v) {
  write_u32_le(buf, idx, static_cast<uint32_t>(v));
}

static inline int16_t clamp_i16(long v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return static_cast<int16_t>(v);
}

static inline uint16_t clamp_u16(long v) {
  if (v < 0) return 0;
  if (v > 65535) return 65535;
  return static_cast<uint16_t>(v);
}

static bool condition_hold(bool cond, uint32_t &start_ms, uint32_t now_ms, uint32_t hold_ms) {
  if (cond) {
    if (start_ms == 0) {
      start_ms = now_ms;
    }
    return (now_ms - start_ms) >= hold_ms;
  }
  start_ms = 0;
  return false;
}

static uint16_t read_battery_mv() {
  uint32_t adc_mv = analogReadMilliVolts(BATTERY_ADC_PIN);
  if (adc_mv == 0) {
    return 0;
  }
  float batt_v = (adc_mv / 1000.0f) / kBatteryDivider;
  return clamp_u16(lroundf(batt_v * 1000.0f));
}

static float parse_nmea_coord(const char *field) {
  if (!field || !*field) {
    return NAN;
  }
  double val = atof(field);
  int deg = static_cast<int>(val / 100.0);
  double minutes = val - (deg * 100.0);
  double dec = deg + minutes / 60.0;
  return static_cast<float>(dec);
}

static bool parse_nmea_course_ddeg(const char *field, uint16_t &course_ddeg) {
  if (!field || !*field) {
    return false;
  }
  float course_deg = static_cast<float>(atof(field));
  if (isnan(course_deg)) {
    return false;
  }
  long course = lroundf(course_deg * 10.0f);
  while (course < 0) {
    course += 3600;
  }
  while (course >= 3600) {
    course -= 3600;
  }
  course_ddeg = static_cast<uint16_t>(course);
  return true;
}

static int hex_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static bool nmea_checksum_ok(const char *line) {
  if (!line || line[0] != '$') {
    return false;
  }
  const char *star = strchr(line, '*');
  if (!star || star[1] == '\0' || star[2] == '\0') {
    return false;
  }
  uint8_t sum = 0;
  for (const char *p = line + 1; p < star; ++p) {
    sum ^= static_cast<uint8_t>(*p);
  }
  int hi = hex_nibble(star[1]);
  int lo = hex_nibble(star[2]);
  if (hi < 0 || lo < 0) {
    return false;
  }
  uint8_t expected = static_cast<uint8_t>((hi << 4) | lo);
  return sum == expected;
}

static void handle_gps_line(char *line) {
  if (!line || line[0] != '$') {
    return;
  }
  if (!nmea_checksum_ok(line)) {
    return;
  }
  const uint32_t now_ms = millis();
  gps_last_sentence_ms = now_ms;

  char *save = nullptr;
  char *type = strtok_r(line, ",", &save);
  if (!type) {
    return;
  }
  bool is_rmc = (strstr(type, "RMC") != nullptr);
  bool is_gga = (strstr(type, "GGA") != nullptr);
  if (!is_rmc && !is_gga) {
    return;
  }

  if (is_rmc) {
    // RMC: time, status, lat, N/S, lon, E/W, speed(knots), ...
    char *time_f = strtok_r(nullptr, ",", &save);
    char *status_f = strtok_r(nullptr, ",", &save);
    char *lat_f = strtok_r(nullptr, ",", &save);
    char *ns_f = strtok_r(nullptr, ",", &save);
    char *lon_f = strtok_r(nullptr, ",", &save);
    char *ew_f = strtok_r(nullptr, ",", &save);
    char *speed_f = strtok_r(nullptr, ",", &save);
    char *course_f = strtok_r(nullptr, ",", &save);
    (void)time_f;
    if (status_f && status_f[0] == 'A') {
      bool got_fix_payload = false;
      float lat = parse_nmea_coord(lat_f);
      float lon = parse_nmea_coord(lon_f);
      if (!isnan(lat) && ns_f && ns_f[0] == 'S') {
        lat = -lat;
      }
      if (!isnan(lon) && ew_f && ew_f[0] == 'W') {
        lon = -lon;
      }
      if (!isnan(lat) && !isnan(lon)) {
        gps_lat_raw = static_cast<int32_t>(lroundf(lat * 1e7f));
        gps_lon_raw = static_cast<int32_t>(lroundf(lon * 1e7f));
        got_fix_payload = true;
      }
      if (speed_f && *speed_f) {
        float speed_kn = static_cast<float>(atof(speed_f));
        float speed_ms = speed_kn * 0.514444f;
        gps_speed_dms = clamp_i16(lroundf(speed_ms * 10.0f));
        got_fix_payload = true;
      }
      uint16_t course_ddeg = 0;
      if (parse_nmea_course_ddeg(course_f, course_ddeg)) {
        gps_course_ddeg = course_ddeg;
        gps_course_valid = true;
        gps_last_course_ms = now_ms;
        got_fix_payload = true;
      }
      if (got_fix_payload) {
        gps_last_fix_ms = now_ms;
        gps_fix_valid = true;
      }
    } else {
      gps_fix_valid = false;
      gps_speed_dms = 0;
    }
    return;
  }

  if (is_gga) {
    // GGA: time, lat, N/S, lon, E/W, fix, sats, hdop, alt(m), ...
    char *time_f = strtok_r(nullptr, ",", &save);
    char *lat_f = strtok_r(nullptr, ",", &save);
    char *ns_f = strtok_r(nullptr, ",", &save);
    char *lon_f = strtok_r(nullptr, ",", &save);
    char *ew_f = strtok_r(nullptr, ",", &save);
    char *fix_f = strtok_r(nullptr, ",", &save);
    char *sat_f = strtok_r(nullptr, ",", &save);
    char *hdop_f = strtok_r(nullptr, ",", &save);
    char *alt_f = strtok_r(nullptr, ",", &save);
    (void)time_f;
    (void)hdop_f;
    if (sat_f && *sat_f) {
      int sat_raw = atoi(sat_f);
      if (sat_raw < 0) {
        sat_raw = 0;
      } else if (sat_raw > 255) {
        sat_raw = 255;
      }
      gps_sat_count = static_cast<uint8_t>(sat_raw);
    }
    int fix_quality = (fix_f && *fix_f) ? atoi(fix_f) : 0;
    if (fix_quality > 0) {
      bool got_fix_payload = false;
      float lat = parse_nmea_coord(lat_f);
      float lon = parse_nmea_coord(lon_f);
      if (!isnan(lat) && ns_f && ns_f[0] == 'S') {
        lat = -lat;
      }
      if (!isnan(lon) && ew_f && ew_f[0] == 'W') {
        lon = -lon;
      }
      if (!isnan(lat) && !isnan(lon)) {
        gps_lat_raw = static_cast<int32_t>(lroundf(lat * 1e7f));
        gps_lon_raw = static_cast<int32_t>(lroundf(lon * 1e7f));
        got_fix_payload = true;
      }
      got_fix_payload = got_fix_payload || (sat_f && *sat_f);
      if (alt_f && *alt_f) {
        float alt_m = static_cast<float>(atof(alt_f));
        gps_alt_dm = clamp_i16(lroundf(alt_m * 10.0f));
        got_fix_payload = true;
      }
      if (got_fix_payload) {
        gps_last_fix_ms = now_ms;
        gps_fix_valid = true;
      }
    } else {
      gps_fix_valid = false;
    }
  }
}

static void process_gps_stream() {
  static char line[256];
  static uint16_t idx = 0;
  static bool dropping_line = false;
  while (GPSSerial.available()) {
    char c = static_cast<char>(GPSSerial.read());
    if (c == '\n') {
      if (!dropping_line) {
        line[idx] = '\0';
      }
      if (!dropping_line && idx > 0) {
        handle_gps_line(line);
      }
      idx = 0;
      dropping_line = false;
      continue;
    }
    if (c == '\r') {
      continue;
    }
    if (dropping_line) {
      continue;
    }
    if (idx < static_cast<uint16_t>(sizeof(line) - 1)) {
      line[idx++] = c;
    } else {
      // Drop oversized sentence and wait for next newline to re-sync.
      dropping_line = true;
      idx = 0;
    }
  }
}

static void set_gps_uart_baud(uint32_t baud) {
  GPSSerial.end();
  delay(2);
  GPSSerial.begin(baud, SERIAL_8N1, UART_GPS_RX, UART_GPS_TX);
}

static void maybe_rotate_gps_baud(uint32_t now_ms) {
  bool sentence_recent = false;
  if (gps_last_sentence_ms > 0) {
    sentence_recent = (now_ms - gps_last_sentence_ms) < kGpsBaudRotateIntervalMs;
  }
  if (sentence_recent) {
    return;
  }
  if ((now_ms - gps_last_baud_switch_ms) < kGpsBaudRotateIntervalMs) {
    return;
  }
  gps_baud_idx = static_cast<uint8_t>((gps_baud_idx + 1) % kGpsBaudCandidateCount);
  uint32_t next_baud = kGpsBaudCandidates[gps_baud_idx];
  set_gps_uart_baud(next_baud);
  gps_last_baud_switch_ms = now_ms;
  gps_last_sentence_ms = now_ms;
  gps_fix_valid = false;
  gps_course_valid = false;
  gps_sat_count = 0;
  gps_speed_dms = 0;

  if (kEnableSetupAsciiDiagnostics) {
    ground_station_serial.print("BOOT:GPS_BAUD=");
    ground_station_serial.println(next_baud);
  }
}

static void handle_rtc_sync_stream(Stream &port, char *line, uint8_t &idx) {
  while (port.available()) {
    char c = static_cast<char>(port.read());
    if (c == '\n' || c == '\r') {
      if (idx == 0) {
        continue;
      }
      line[idx] = '\0';
      if (strncmp(line, "RTC_SYNC:", 9) == 0) {
        uint32_t epoch = static_cast<uint32_t>(strtoul(line + 9, nullptr, 10));
        if (epoch > 0 && rtc_ready) {
          rtc.adjust(DateTime(epoch));
          DBG_PRINTF("RTC synced: %lu\n", static_cast<unsigned long>(epoch));
        }
      }
      idx = 0;
    } else {
      if (idx < kRtcSyncLineLen - 1) {
        line[idx++] = c;
      }
    }
  }
}

static void handle_serial_rtc_sync() {
  static char gs_line[kRtcSyncLineLen];
  static uint8_t gs_idx = 0;
  handle_rtc_sync_stream(ground_station_serial, gs_line, gs_idx);
}

static void emit_setup_diag(const char *tag) {
  if (!kEnableSetupAsciiDiagnostics || !tag) {
    return;
  }
  ground_station_serial.print("BOOT:");
  ground_station_serial.println(tag);
}

static void emit_setup_diag_u32(const char *tag, uint32_t value) {
  if (!kEnableSetupAsciiDiagnostics || !tag) {
    return;
  }
  ground_station_serial.print("BOOT:");
  ground_station_serial.print(tag);
  ground_station_serial.print("=");
  ground_station_serial.println(value);
}

static inline float inv_sqrt(float x) {
  return 1.0f / sqrtf(x);
}

static bool i2c_probe(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return bus.endTransmission() == 0;
}

static uint8_t i2c_count_devices(TwoWire &bus) {
#if DEBUG_SERIAL
  uint8_t count = 0;
  for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
    if (i2c_probe(bus, addr)) {
      count++;
    }
  }
  return count;
#else
  // Fast-path for boot speed: probe only known sensor addresses when debug is off.
  static const uint8_t known_addrs[] = {0x44, 0x76, 0x77, 0x53, kAdxlAltAddress, 0x68, 0x69};
  uint8_t count = 0;
  for (size_t i = 0; i < sizeof(known_addrs); i++) {
    if (i2c_probe(bus, known_addrs[i])) {
      count++;
    }
  }
  return count;
#endif
}

static void i2c_scan_print(TwoWire &bus, const char *label);

static void i2c_bus_recover(int sda_pin, int scl_pin) {
  pinMode(sda_pin, INPUT_PULLUP);
  pinMode(scl_pin, INPUT_PULLUP);
  delay(1);
  if (digitalRead(sda_pin) == HIGH && digitalRead(scl_pin) == HIGH) {
    return;
  }

  pinMode(scl_pin, OUTPUT_OPEN_DRAIN);
  digitalWrite(scl_pin, HIGH);
  for (uint8_t i = 0; i < 18 && digitalRead(sda_pin) == LOW; i++) {
    digitalWrite(scl_pin, LOW);
    delayMicroseconds(5);
    digitalWrite(scl_pin, HIGH);
    delayMicroseconds(5);
  }

  pinMode(sda_pin, OUTPUT_OPEN_DRAIN);
  digitalWrite(sda_pin, LOW);
  delayMicroseconds(5);
  digitalWrite(scl_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(sda_pin, HIGH);
  delayMicroseconds(5);

  pinMode(sda_pin, INPUT_PULLUP);
  pinMode(scl_pin, INPUT_PULLUP);
}

static uint8_t init_i2c_bus(TwoWire &bus, int sda_pin, int scl_pin, const char *label) {
  static const uint32_t kI2cClockCandidates[] = {
    kI2cClockHz, kI2cClockFallbackHz1, kI2cClockFallbackHz2
  };
  uint8_t count = 0;
  bool recovered = false;
  uint32_t selected_clock = kI2cClockHz;

  for (uint8_t pass = 0; pass < 2 && count == 0; pass++) {
    if (pass == 1) {
      i2c_bus_recover(sda_pin, scl_pin);
      recovered = true;
    }
    for (size_t i = 0; i < sizeof(kI2cClockCandidates) / sizeof(kI2cClockCandidates[0]); i++) {
      bus.end();
      delay(2);
      bus.setPins(sda_pin, scl_pin);
      bus.begin(sda_pin, scl_pin);
      bus.setClock(kI2cClockCandidates[i]);
      bus.setTimeOut(kI2cTimeoutMs);
      count = i2c_count_devices(bus);
      if (count > 0) {
        selected_clock = kI2cClockCandidates[i];
        break;
      }
    }
  }

  if (count == 0) {
    // Keep bus initialized at primary clock even when no device was found.
    bus.end();
    delay(2);
    bus.setPins(sda_pin, scl_pin);
    bus.begin(sda_pin, scl_pin);
    bus.setClock(kI2cClockHz);
    bus.setTimeOut(kI2cTimeoutMs);
    selected_clock = kI2cClockHz;
  } else if (kEnableSetupAsciiDiagnostics) {
    char tag[24];
    snprintf(tag, sizeof(tag), "%s_CLK", label ? label : "I2C");
    emit_setup_diag_u32(tag, selected_clock);
    if (recovered) {
      snprintf(tag, sizeof(tag), "%s_RECOVER", label ? label : "I2C");
      emit_setup_diag(tag);
    }
  }

  if (sda_pin == I2C_A_SDA && scl_pin == I2C_A_SCL) {
    i2c_a_runtime_clock_hz = selected_clock;
  } else if (sda_pin == I2C_B_SDA && scl_pin == I2C_B_SCL) {
    i2c_b_runtime_clock_hz = selected_clock;
  }

  i2c_scan_print(bus, label);
  return count;
}

static void i2c_scan_print(TwoWire &bus, const char *label) {
#if !DEBUG_SERIAL
  (void)bus;
  (void)label;
  return;
#else
  DBG_PRINTF("%s scan:\n", label);
  bool found_any = false;
  for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
    if (i2c_probe(bus, addr)) {
      DBG_PRINTF("  - 0x%02X\n", addr);
      found_any = true;
    }
  }
  if (!found_any) {
    DBG_PRINTLN("  (no devices)");
  }
#endif
}

static bool init_adxl_sensor() {
  uint8_t adxl_addr = ADXL375_ADDRESS;
  accel = &accel_a;
  adxl_bus_label = "I2C_A";

  if (i2c_probe(Wire, ADXL375_ADDRESS)) {
    accel = &accel_a;
    adxl_bus_label = "I2C_A";
    adxl_addr = ADXL375_ADDRESS;
  } else if (i2c_probe(Wire, kAdxlAltAddress)) {
    accel = &accel_a;
    adxl_bus_label = "I2C_A";
    adxl_addr = kAdxlAltAddress;
  } else if (i2c_probe(Wire1, ADXL375_ADDRESS)) {
    accel = &accel_b;
    adxl_bus_label = "I2C_B";
    adxl_addr = ADXL375_ADDRESS;
  } else if (i2c_probe(Wire1, kAdxlAltAddress)) {
    accel = &accel_b;
    adxl_bus_label = "I2C_B";
    adxl_addr = kAdxlAltAddress;
  }

  status_adxl = accel->begin(adxl_addr);
  if (status_adxl) {
    DBG_PRINTF("ADXL375 OK on %s\n", adxl_bus_label);
  } else {
    DBG_PRINTLN("ADXL375 init failed on both I2C_A/I2C_B");
  }
  return status_adxl;
}

static bool init_imu_sensor() {
  bool imu_present = false;
  if (i2c_probe(Wire, 0x68)) {
    imu = &imu_a_68;
    imu_addr = 0x68;
    imu_bus_label = "I2C_A";
    imu_present = true;
  } else if (i2c_probe(Wire, 0x69)) {
    imu = &imu_a_69;
    imu_addr = 0x69;
    imu_bus_label = "I2C_A";
    imu_present = true;
  } else if (i2c_probe(Wire1, 0x68)) {
    imu = &imu_b_68;
    imu_addr = 0x68;
    imu_bus_label = "I2C_B";
    imu_present = true;
  } else if (i2c_probe(Wire1, 0x69)) {
    imu = &imu_b_69;
    imu_addr = 0x69;
    imu_bus_label = "I2C_B";
    imu_present = true;
  }

  if (!imu_present) {
    status_imu = false;
    DBG_PRINTLN("ICM42688 not detected on 0x68/0x69");
    return false;
  }

  int imu_status = imu->begin();
  if (imu_status < 0) {
    status_imu = false;
    DBG_PRINTF("ICM42688 init failed on %s @0x%02X\n", imu_bus_label, imu_addr);
  } else {
    status_imu = true;
    DBG_PRINTF("ICM42688 OK on %s @0x%02X\n", imu_bus_label, imu_addr);
    // 設定 IMU 範圍 (火箭通常需要最大範圍)
    imu->setAccelFS(ICM42688::gpm16); // ±16g
    imu->setGyroFS(ICM42688::dps2000); // ±2000 dps
  }

  // ICM42688 library sets I2C clock to 400kHz internally.
  // Force both buses back to our stable system-wide clock.
  Wire.setClock(i2c_a_runtime_clock_hz);
  Wire1.setClock(i2c_b_runtime_clock_hz);
  return status_imu;
}

static void retry_delayed_sensor_init(uint32_t now_ms) {
  if (status_adxl && status_imu) {
    return;
  }
  if (sensor_init_window_start_ms == 0) {
    return;
  }
  if (kSensorRetryWindowMs > 0 &&
      now_ms - sensor_init_window_start_ms > kSensorRetryWindowMs) {
    return;
  }
  if (now_ms - last_sensor_retry_ms < kSensorRetryIntervalMs) {
    return;
  }
  last_sensor_retry_ms = now_ms;

  init_i2c_bus(Wire, I2C_A_SDA, I2C_A_SCL, "I2C_A");
  init_i2c_bus(Wire1, I2C_B_SDA, I2C_B_SCL, "I2C_B");

  if (!status_adxl) {
    init_adxl_sensor();
  }
  if (!status_imu) {
    init_imu_sensor();
  }
}

static void init_lora_subsystem() {
  pinMode(LORA_NSS, OUTPUT);
  digitalWrite(LORA_NSS, HIGH);
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);
  pinMode(LORA_DIO0, INPUT);
  pinMode(LORA_DIO1, INPUT);
  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setSPI(loraSPI);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  lora_ready = LoRa.begin(LORA_FREQ);
  if (lora_ready) {
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setPreambleLength(LORA_PREAMBLE_LEN);
    LoRa.enableCrc();
    lora_tx_fail_count = 0;
    DBG_PRINTLN("LoRa OK");
  } else {
    DBG_PRINTLN("LoRa init failed");
  }
  emit_setup_diag(lora_ready ? "LORA_OK" : "LORA_FAIL");
}

static bool lora_send_frame(const uint8_t *frame, size_t len) {
  if (!lora_ready || !frame || len == 0) {
    return false;
  }
  if (LoRa.beginPacket() == 0) {
    return false;
  }
  size_t written = LoRa.write(frame, len);
  int end_ok = LoRa.endPacket();
  return (written == len) && (end_ok == 1);
}

static void maintain_lora_link(uint32_t now_ms) {
  if (lora_ready) {
    return;
  }
  if (boot_init_stage != kBootInitDone) {
    return;
  }
  if ((now_ms - last_lora_retry_ms) < kLoraRetryIntervalMs) {
    return;
  }
  last_lora_retry_ms = now_ms;
  init_lora_subsystem();
  if (lora_ready) {
    emit_setup_diag("LORA_RECOVERED");
  }
}

static void init_sd_subsystem() {
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  sd_ready = SD.begin(SD_CS, sdSPI);
  if (sd_ready) {
    sd_log = SD.open("/telemetry.csv", FILE_APPEND);
    if (!sd_log) {
      sd_ready = false;
    } else if (sd_log.size() == 0) {
      write_sd_csv_header();
    }
  }
  emit_setup_diag(sd_ready ? "SD_OK" : "SD_FAIL");
}

static void init_rtc_subsystem() {
  rtc_ready = rtc.begin(&Wire1);
  if (rtc_ready) {
    if (rtc.lostPower()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
  }
  emit_setup_diag(rtc_ready ? "RTC_OK" : "RTC_FAIL");
}

static void init_env_subsystem() {
  status_sht = sht31.begin(0x44);

  status_bmp = false;
  if (kEnableBmp390) {
    status_bmp = bmp.begin_I2C();
    if (status_bmp) {
      bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
      bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
      bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    }
  }
  emit_setup_diag(status_sht ? "SHT_OK" : "SHT_FAIL");
}

static void init_motion_subsystem(uint32_t now_ms) {
  init_adxl_sensor();
  init_imu_sensor();
  sensor_init_window_start_ms = now_ms;
  last_sensor_retry_ms = now_ms;
  emit_setup_diag((status_adxl && status_imu) ? "IMU_ADXL_OK" : "IMU_ADXL_RETRY");
}

static void run_deferred_boot_init(uint32_t now_ms) {
  if (boot_init_stage == kBootInitDone) {
    return;
  }
  if ((now_ms - last_deferred_init_ms) < kDeferredInitStepGapMs) {
    return;
  }
  last_deferred_init_ms = now_ms;

  switch (boot_init_stage) {
    case kBootInitI2c: {
      uint8_t i2c_a_count = init_i2c_bus(Wire, I2C_A_SDA, I2C_A_SCL, "I2C_A");
      uint8_t i2c_b_count = init_i2c_bus(Wire1, I2C_B_SDA, I2C_B_SCL, "I2C_B");
      emit_setup_diag_u32("I2C_A_DEV", i2c_a_count);
      emit_setup_diag_u32("I2C_B_DEV", i2c_b_count);
      emit_setup_diag((i2c_a_count + i2c_b_count) > 0 ? "I2C_READY" : "I2C_NO_DEV");
      boot_init_stage = kBootInitLoRa;
      break;
    }
    case kBootInitLoRa:
      init_lora_subsystem();
      boot_init_stage = kBootInitSd;
      break;
    case kBootInitSd:
      init_sd_subsystem();
      boot_init_stage = kBootInitRtc;
      break;
    case kBootInitRtc:
      init_rtc_subsystem();
      boot_init_stage = kBootInitEnv;
      break;
    case kBootInitEnv:
      init_env_subsystem();
      boot_init_stage = kBootInitMotion;
      break;
    case kBootInitMotion:
      init_motion_subsystem(now_ms);
      boot_init_stage = kBootInitDone;
      emit_setup_diag("BOOT_INIT_DONE");
      break;
    case kBootInitDone:
    default:
      break;
  }
}

static void buzzerTone(int freq, int duration_ms) {
  if (freq <= 0) {
    ledcWriteTone(BUZZER_CH, 0);
    delay(duration_ms);
    return;
  }
  ledcWriteTone(BUZZER_CH, freq);
  delay(duration_ms);
  ledcWriteTone(BUZZER_CH, 0);
}

static void set_status_led(uint8_t state) {
  uint32_t color = status_led.Color(0, 0, 0);
  switch (state) {
    case kStateTest:
      color = status_led.Color(255, 255, 255);
      break;
    case kStateIdle:
      color = status_led.Color(0, 0, 255);
      break;
    case kStatePreflight:
      color = status_led.Color(0, 255, 255);
      break;
    case kStateAscent:
      color = status_led.Color(0, 255, 0);
      break;
    case kStateApogee:
      color = status_led.Color(255, 200, 0);
      break;
    case kStateDescent:
      color = status_led.Color(255, 80, 0);
      break;
    case kStateLanded:
      color = status_led.Color(120, 60, 0);
      break;
    case kStateAbort:
      color = status_led.Color(255, 0, 0);
      break;
    default:
      color = status_led.Color(32, 32, 32);
      break;
  }
  status_led.setPixelColor(0, color);
  status_led.show();
}

static void madgwick_update_imu(
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float dt,
    float &q0, float &q1, float &q2, float &q3) {
  // Madgwick IMU update (gyro rad/s, accel normalized)
  const float beta = 0.15f;
  float recip_norm;
  float s0, s1, s2, s3;
  float q_dot1, q_dot2, q_dot3, q_dot4;

  if (ax == 0.0f && ay == 0.0f && az == 0.0f) {
    return;
  }

  recip_norm = inv_sqrt(ax * ax + ay * ay + az * az);
  ax *= recip_norm;
  ay *= recip_norm;
  az *= recip_norm;

  float _2q0 = 2.0f * q0;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _4q0 = 4.0f * q0;
  float _4q1 = 4.0f * q1;
  float _4q2 = 4.0f * q2;
  float _8q1 = 8.0f * q1;
  float _8q2 = 8.0f * q2;
  float q0q0 = q0 * q0;
  float q1q1 = q1 * q1;
  float q2q2 = q2 * q2;
  float q3q3 = q3 * q3;

  s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
  s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
  s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
  s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

  float s_norm = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
  if (s_norm == 0.0f) {
    return;
  }
  recip_norm = inv_sqrt(s_norm);
  s0 *= recip_norm;
  s1 *= recip_norm;
  s2 *= recip_norm;
  s3 *= recip_norm;

  q_dot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
  q_dot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - beta * s1;
  q_dot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - beta * s2;
  q_dot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - beta * s3;

  q0 += q_dot1 * dt;
  q1 += q_dot2 * dt;
  q2 += q_dot3 * dt;
  q3 += q_dot4 * dt;

  recip_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recip_norm;
  q1 *= recip_norm;
  q2 *= recip_norm;
  q3 *= recip_norm;
}

void setup() {
  ground_station_serial.begin(kGroundStationBaud);
  delay(1);
  emit_setup_diag("SETUP_ENTER");
  ground_station_serial.print("HB\n");

  pinMode(SAFETY_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SEPARATION_SERVO_PIN, OUTPUT);
  digitalWrite(SEPARATION_SERVO_PIN, LOW);
  pinMode(CAMERA_A_TRIGGER_PIN, OUTPUT);
  digitalWrite(CAMERA_A_TRIGGER_PIN, LOW);
  pinMode(CAMERA_B_TRIGGER_PIN, OUTPUT);
  digitalWrite(CAMERA_B_TRIGGER_PIN, LOW);

  status_led.begin();
  status_led.setBrightness(32);
  set_status_led(kStateIdle);

  pinMode(WATER_DETECT_PIN, INPUT);
  pinMode(DS3231_SQW_PIN, INPUT_PULLUP);

  ledcSetup(BUZZER_CH, 2000, BUZZER_RES);
  ledcAttachPin(BUZZER_PIN, BUZZER_CH);

  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

  gps_baud_idx = 0;
  gps_last_sentence_ms = 0;
  gps_last_fix_ms = 0;
  gps_last_course_ms = 0;
  gps_fix_valid = false;
  gps_course_valid = false;
  gps_last_baud_switch_ms = millis();
  set_gps_uart_baud(kGpsBaudCandidates[gps_baud_idx]);
  emit_setup_diag_u32("GPS_BAUD", kGpsBaudCandidates[gps_baud_idx]);
  pinMode(UART_GPS_PPS, INPUT);

  if (kEnableBackupHeartbeat) {
    BackupSerial.begin(UART_SUB_BAUD, SERIAL_8N1, UART_SUB_RX, UART_SUB_TX);
  }

  boot_init_stage = kBootInitI2c;
  last_deferred_init_ms = millis();
  sensor_init_window_start_ms = 0;
  last_sensor_retry_ms = 0;
  last_lora_retry_ms = 0;
  lora_tx_fail_count = 0;

  emit_setup_diag("SETUP_DONE");
  ground_station_serial.print("HB\n");
}

void loop() {
  uint32_t time_ms = millis();
  static bool loop_enter_reported = false;
  if (!loop_enter_reported) {
    emit_setup_diag("LOOP_ENTER");
    loop_enter_reported = true;
  }
  run_deferred_boot_init(time_ms);
  maintain_lora_link(time_ms);
  if (kEnableBackupHeartbeat && time_ms - last_heartbeat_ms >= 100) {
    BackupSerial.write('H');
    BackupSerial.write('B');
    BackupSerial.write('\n');
    last_heartbeat_ms = time_ms;
  }
  handle_serial_rtc_sync();
  process_gps_stream();
  if (gps_last_fix_ms > 0 && time_ms - gps_last_fix_ms > kGpsTimeoutMs) {
    gps_fix_valid = false;
    gps_speed_dms = 0;
  }
  if (gps_last_course_ms > 0 && time_ms - gps_last_course_ms > kGpsSentenceTimeoutMs) {
    gps_course_valid = false;
  }
  maybe_rotate_gps_baud(time_ms);
  retry_delayed_sensor_init(time_ms);

  if (kEnableStartupTone && boot_init_stage == kBootInitDone) {
    static bool startup_tone_done = false;
    if (!startup_tone_done) {
      // Keep tone out of setup so loop can start immediately.
      delay(1000);
      buzzerTone(NOTE_DO, 120);
      delay(20);
      buzzerTone(NOTE_MI, 120);
      delay(20);
      buzzerTone(NOTE_SOL, 120);
      delay(20);
      buzzerTone(NOTE_HDO, 120);
      startup_tone_done = true;
    }
  }

  float temp_c = 0.0f;
  float hum = 0.0f;
  if (status_sht) {
    temp_c = sht31.readTemperature();
    hum = sht31.readHumidity();
  }

  float pressure_pa = 0.0f;
  float baro_alt_m = 0.0f;
  bool baro_valid = false;
  if (kEnableBmp390 && status_bmp && bmp.performReading()) {
    pressure_pa = bmp.pressure;
    baro_alt_m = bmp.readAltitude(1013.25);
    baro_valid = true;
  }

  float accx_g = 0.0f;
  float accy_g = 0.0f;
  float accz_g = 0.0f;
  if (status_adxl) {
    sensors_event_t event;
    accel->getEvent(&event);
    float adxl_x = event.acceleration.x / kG;
    float adxl_y = event.acceleration.y / kG;
    float adxl_z = event.acceleration.z / kG;
    // Rocket axes: X=right, Y=forward (launch), Z=up/nose.
    accx_g = -adxl_y;  // Rocket X = -ADXL Y
    accy_g = adxl_x;   // Rocket Y = ADXL X
    accz_g = adxl_z;   // Rocket Z = ADXL Z
  }

  float roll_deg = 0.0f;
  float pitch_deg = 0.0f;
  uint16_t heading_ddeg = 0;
  bool heading_from_imu = false;
  float gyro_x_dps = 0.0f;
  float gyro_y_dps = 0.0f;
  float gyro_z_dps = 0.0f;
  static float q0 = 1.0f;
  static float q1 = 0.0f;
  static float q2 = 0.0f;
  static float q3 = 0.0f;
  static uint32_t last_fusion_ms = 0;
  static bool fusion_init = false;
  static uint32_t last_imu_print_ms = 0;
  if (status_imu) {
    imu->getAGT();
    float gx_raw = imu->gyrX();
    float gy_raw = imu->gyrY();
    float gz_raw = imu->gyrZ();
    float ax_raw = imu->accX();
    float ay_raw = imu->accY();
    float az_raw = imu->accZ();
    gyro_x_dps = gx_raw;
    gyro_y_dps = gy_raw;
    gyro_z_dps = gz_raw;

    if (time_ms - last_imu_print_ms >= 500) {
      DBG_PRINTF("ICM acc[g]=%.3f,%.3f,%.3f gyro[dps]=%.3f,%.3f,%.3f\n",
                 ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);
      last_imu_print_ms = time_ms;
    }

    if (!fusion_init) {
      // Map to fusion axes so roll=Z, pitch=X, yaw=Y.
      float ax_body = az_raw;
      float ay_body = ax_raw;
      float az_body = ay_raw;
      float roll_acc = atan2f(ay_body, az_body);
      float pitch_acc = atan2f(-ax_body, sqrtf(ay_body * ay_body + az_body * az_body));
      float cr = cosf(roll_acc * 0.5f);
      float sr = sinf(roll_acc * 0.5f);
      float cp = cosf(pitch_acc * 0.5f);
      float sp = sinf(pitch_acc * 0.5f);
      q0 = cr * cp;
      q1 = sr * cp;
      q2 = cr * sp;
      q3 = -sr * sp;
      last_fusion_ms = time_ms;
      fusion_init = true;
    } else {
      float dt = (time_ms - last_fusion_ms) / 1000.0f;
      if (dt > 0.0f && dt < 1.0f) {
        // Map to fusion axes so roll=Z, pitch=X, yaw=Y.
        float ax_body = az_raw;
        float ay_body = ax_raw;
        float az_body = ay_raw;
        float gx = gz_raw;
        float gy = gx_raw;
        float gz = gy_raw;
        madgwick_update_imu(
            gx * 0.0174532925f,
            gy * 0.0174532925f,
            gz * 0.0174532925f,
            ax_body, ay_body, az_body,
            dt,
            q0, q1, q2, q3
        );
      }
      last_fusion_ms = time_ms;
    }
  }

  if (fusion_init) {
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    roll_deg = atan2f(sinr_cosp, cosr_cosp) * 57.2957795f;

    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    pitch_deg = asinf(sinp) * 57.2957795f;

    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    float yaw_deg = atan2f(siny_cosp, cosy_cosp) * 57.2957795f;
    if (yaw_deg < 0.0f) {
      yaw_deg += 360.0f;
    }
    static bool yaw_zero_set = false;
    static float yaw_zero = 0.0f;
    if (!yaw_zero_set) {
      yaw_zero = yaw_deg;
      yaw_zero_set = true;
    }
    yaw_deg -= yaw_zero;
    if (yaw_deg < 0.0f) {
      yaw_deg += 360.0f;
    }
    static float yaw_hold = 0.0f;
    static bool yaw_hold_init = false;
    const float yaw_lock_deg = 80.0f;
    if (!yaw_hold_init) {
      yaw_hold = yaw_deg;
      yaw_hold_init = true;
    }
    if (fabsf(pitch_deg) < yaw_lock_deg) {
      yaw_hold = yaw_deg;
    }
    heading_ddeg = clamp_u16(lroundf(yaw_hold * 10.0f));
    heading_from_imu = true;
  }

  if (!heading_from_imu &&
      gps_fix_valid &&
      gps_course_valid &&
      gps_speed_dms >= static_cast<int16_t>(kGpsCourseMinSpeedDms)) {
    heading_ddeg = gps_course_ddeg;
  }

  // Low-pass filter to reduce roll/pitch jitter.
  static float roll_filt = 0.0f;
  static float pitch_filt = 0.0f;
  static bool filt_init = false;
  const float smooth_alpha = 0.2f;
  if (!filt_init) {
    roll_filt = roll_deg;
    pitch_filt = pitch_deg;
    filt_init = true;
  } else {
    roll_filt = roll_filt + smooth_alpha * (roll_deg - roll_filt);
    pitch_filt = pitch_filt + smooth_alpha * (pitch_deg - pitch_filt);
  }
  roll_deg = roll_filt;
  pitch_deg = pitch_filt;

  float accel_g = sqrtf(accx_g * accx_g + accy_g * accy_g + accz_g * accz_g);

  float alt_m = 0.0f;
  bool alt_valid = false;
  if (baro_valid) {
    alt_m = baro_alt_m;
    alt_valid = true;
  } else if (gps_fix_valid) {
    alt_m = gps_alt_dm / 10.0f;
    alt_valid = true;
  }

  static float last_alt_m = 0.0f;
  static uint32_t last_alt_ms = 0;
  static float vert_speed_ms = 0.0f;
  static bool vert_speed_valid = false;
  if (alt_valid) {
    if (last_alt_ms != 0 && time_ms > last_alt_ms) {
      float dt = (time_ms - last_alt_ms) / 1000.0f;
      if (dt > 0.01f && dt < 1.0f) {
        float vs_inst = (alt_m - last_alt_m) / dt;
        if (!vert_speed_valid) {
          vert_speed_ms = vs_inst;
        } else {
          const float vs_alpha = 0.2f;
          vert_speed_ms = vert_speed_ms + vs_alpha * (vs_inst - vert_speed_ms);
        }
        vert_speed_valid = true;
      }
    }
    last_alt_m = alt_m;
    last_alt_ms = time_ms;
  }
  if (last_alt_ms > 0 && (time_ms - last_alt_ms) > 1000) {
    vert_speed_valid = false;
  }

  float alt_rel_m = alt_m;
  if (alt_zero_set) {
    alt_rel_m = alt_m - alt_zero_m;
  }

  uint8_t water_detected = digitalRead(WATER_DETECT_PIN) ? 1 : 0;
  bool safety_armed = (digitalRead(SAFETY_SWITCH_PIN) == HIGH);
  float flight_time_s = (liftoff_ms > 0) ? (time_ms - liftoff_ms) / 1000.0f : 0.0f;

  switch (flight_state) {
    case kStateIdle: {
      liftoff_ms = 0;
      apogee_seen = false;
      apogee_ms = 0;
      main_deployed = false;
      alt_zero_set = false;
      liftoff_cond_ms = 0;
      drogue_cond_ms = 0;
      main_cond_ms = 0;
      landing_cond_ms = 0;
      if (safety_armed) {
        flight_state = kStatePreflight;
      }
      break;
    }
    case kStatePreflight: {
      if (!alt_zero_set && alt_valid) {
        alt_zero_m = alt_m;
        alt_zero_set = true;
      }
      if (!safety_armed) {
        flight_state = kStateIdle;
        break;
      }
      bool liftoff_cond = (status_adxl && accel_g > kLiftoffAccelG) ||
                          (vert_speed_valid && vert_speed_ms > kLiftoffVSpeedMs);
      if (condition_hold(liftoff_cond, liftoff_cond_ms, time_ms, kLiftoffHoldMs)) {
        liftoff_ms = time_ms;
        apogee_seen = false;
        apogee_ms = 0;
        main_deployed = false;
        drogue_cond_ms = 0;
        main_cond_ms = 0;
        landing_cond_ms = 0;
        flight_state = kStateAscent;
      }
      break;
    }
    case kStateAscent: {
      if (!apogee_seen && vert_speed_valid && vert_speed_ms < kApogeeDetectVSpeedMs) {
        apogee_seen = true;
        apogee_ms = time_ms;
        drogue_cond_ms = 0;
        flight_state = kStateApogee;
        break;
      }
      break;
    }
    case kStateApogee: {
      bool after_apogee = apogee_seen && (time_ms - apogee_ms >= kAfterApogeeMs);
      bool drogue_cond = ((alt_valid && alt_rel_m > kDrogueAltMinM) ||
                          (flight_time_s > kDrogueTimeS));
      if (after_apogee && condition_hold(drogue_cond, drogue_cond_ms, time_ms, kDrogueHoldMs)) {
        flight_state = kStateDescent;
        main_cond_ms = 0;
        landing_cond_ms = 0;
      }
      break;
    }
    case kStateDescent: {
      bool main_cond = ((alt_valid && vert_speed_valid &&
                         alt_rel_m < kMainAltMaxM && vert_speed_ms < kMainVSpeedMaxMs) ||
                        (flight_time_s > kMainTimeS));
      if (!main_deployed &&
          condition_hold(main_cond, main_cond_ms, time_ms, kMainHoldMs)) {
        main_deployed = true;
      }
      bool landing_alt_cond = alt_valid && alt_rel_m < kLandingAltMaxM;
      bool landing_speed_cond = !vert_speed_valid || fabsf(vert_speed_ms) < kLandingVSpeedMaxMs;
      bool landing_cond = ((water_detected == 1) ||
                           (landing_alt_cond && landing_speed_cond));
      if (condition_hold(landing_cond, landing_cond_ms, time_ms, kLandingHoldMs)) {
        flight_state = kStateLanded;
      }
      break;
    }
    case kStateLanded:
    default:
      break;
  }

  int32_t lat_raw = gps_lat_raw;
  int32_t lon_raw = gps_lon_raw;
  int16_t gps_alt_dm_local = gps_alt_dm;
  int16_t baro_alt_dm = baro_valid ? clamp_i16(lroundf(baro_alt_m * 10.0f)) : kInvalidBaroAltDm;
  int16_t gps_speed_dms_local = gps_speed_dms;
  uint8_t sat_count = gps_sat_count;
  int16_t roll_cdeg = clamp_i16(lroundf(roll_deg * 100.0f));
  int16_t pitch_cdeg = clamp_i16(lroundf(pitch_deg * 100.0f));
  int16_t gyro_ddeg_s = clamp_i16(lroundf(gyro_z_dps * 10.0f));
  int16_t accx_cg = clamp_i16(lroundf(accx_g * 100.0f));
  int16_t accy_cg = clamp_i16(lroundf(accy_g * 100.0f));
  int16_t accz_cg = clamp_i16(lroundf(accz_g * 100.0f));
  int16_t gyro_x_ddeg_s = clamp_i16(lroundf(gyro_x_dps * 10.0f));
  int16_t gyro_y_ddeg_s = clamp_i16(lroundf(gyro_y_dps * 10.0f));
  static uint8_t last_state = 0xFF;
  if (flight_state != last_state) {
    set_status_led(flight_state);
    last_state = flight_state;
  }
  uint32_t rtc_unix = 0;
  if (rtc_ready) {
    rtc_unix = rtc.now().unixtime();
  }
  uint16_t battery_mv = read_battery_mv();
  uint16_t servo_power_mv = 0; // TODO: read servo power rail
  int16_t servo_angle_ddeg = 0; // TODO: read servo angle (0.1 deg)
  uint32_t pressure_u32 = (pressure_pa < 0.0f) ? 0 : static_cast<uint32_t>(lroundf(pressure_pa));
  uint8_t error_code = 0;
  if (!lora_ready) {
    error_code |= kErrLoRaInit;
  }
  if (!sd_ready) {
    error_code |= kErrSdInit;
  }
  if (!rtc_ready) {
    error_code |= kErrRtcInit;
  }
  if (kEnableBmp390 && !status_bmp) {
    error_code |= kErrBaroInit;
  }
  if (!status_imu) {
    error_code |= kErrImuInit;
  }
  if (!status_adxl) {
    error_code |= kErrAdxlInit;
  }
  if (flight_state >= kStatePreflight && !gps_fix_valid) {
    error_code |= kErrGpsNoFix;
  }
  if (battery_mv == 0 || (kBatteryLowMv > 0 && battery_mv < kBatteryLowMv)) {
    error_code |= kErrBattery;
  }

  uint8_t frame[kFrameLen];
  frame[0] = kFrameHeader0;
  frame[1] = kFrameHeader1;
  uint16_t time_0p1s = static_cast<uint16_t>((time_ms / 100) & 0xFFFF);
  write_u16_le(frame, 2, time_0p1s);
  write_i32_le(frame, 4, lat_raw);
  write_i32_le(frame, 8, lon_raw);
  write_i16_le(frame, 12, gps_alt_dm_local);
  write_i16_le(frame, 14, gps_speed_dms_local);
  frame[16] = sat_count;
  write_i16_le(frame, 17, roll_cdeg);
  write_i16_le(frame, 19, pitch_cdeg);
  write_u16_le(frame, 21, heading_ddeg);
  write_i16_le(frame, 23, gyro_x_ddeg_s);
  write_i16_le(frame, 25, gyro_y_ddeg_s);
  write_i16_le(frame, 27, gyro_ddeg_s);
  write_i16_le(frame, 29, accx_cg);
  write_i16_le(frame, 31, accy_cg);
  write_i16_le(frame, 33, accz_cg);
  write_i16_le(frame, 35, baro_alt_dm);
  write_u16_le(frame, 37, battery_mv);
  write_u16_le(frame, 39, servo_power_mv);
  write_i16_le(frame, 41, servo_angle_ddeg);
  frame[43] = flight_state;
  frame[44] = error_code;
  frame[45] = water_detected;

  uint8_t crc = 0;
  for (int i = 0; i < kFrameLen - 1; i++) {
    crc ^= frame[i];
  }
  frame[kFrameLen - 1] = crc;

  if (lora_ready) {
    if (lora_send_frame(frame, kFrameLen)) {
      lora_tx_fail_count = 0;
    } else {
      if (lora_tx_fail_count < 255) {
        lora_tx_fail_count++;
      }
      if (lora_tx_fail_count >= kLoraTxFailReinitThreshold) {
        lora_ready = false;
        last_lora_retry_ms = time_ms;
        emit_setup_diag("LORA_TX_FAIL");
      }
    }
  }

  if (sd_ready && sd_log) {
    float lat_deg = gps_lat_raw / 1e7f;
    float lon_deg = gps_lon_raw / 1e7f;
    float gps_alt_m = gps_alt_dm_local / 10.0f;
    float baro_alt_log_m = baro_valid ? baro_alt_m : NAN;
    float speed_ms = gps_speed_dms_local / 10.0f;
    float heading_deg = heading_ddeg / 10.0f;
    write_sd_csv_line(
        time_ms,
        rtc_unix,
        lat_deg,
        lon_deg,
        gps_alt_m,
        baro_alt_log_m,
        speed_ms,
        heading_deg,
        sat_count,
        roll_deg,
        pitch_deg,
        accx_g,
        accy_g,
        accz_g,
        gyro_x_dps,
        gyro_y_dps,
        gyro_z_dps,
        battery_mv / 1000.0f,
        static_cast<float>(servo_power_mv),
        servo_angle_ddeg / 10.0f,
        temp_c,
        hum,
        baro_valid ? pressure_pa : NAN,
        flight_state,
        error_code,
        water_detected);
    if (time_ms % 1000 < 100) {
      sd_log.flush();
    }
  }

  ground_station_serial.write(frame, kFrameLen);

  // 簡易延遲，約 10Hz (實際飛行時不能用 delay)
  delay(100); 
}
