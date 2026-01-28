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

// Set to 1 for Serial debug logs. Keep 0 when Serial is used for binary packets.
#define DEBUG_SERIAL 0
#if DEBUG_SERIAL
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#define DBG_PRINTLN(msg) Serial.println(msg)
#else
#define DBG_PRINTF(...) do {} while (0)
#define DBG_PRINTLN(msg) do {} while (0)
#endif

// --- 腳位定義 (對應硬體表) ---
#define I2C_A_SDA 20
#define I2C_A_SCL 19
#define I2C_B_SDA 9
#define I2C_B_SCL 10

#define UART_GPS_TX 45
#define UART_GPS_RX 35
#define UART_GPS_PPS 47
#define UART_GPS_BAUD 230400

#define LORA_SCK 42
#define LORA_MISO 41
#define LORA_MOSI 40
#define LORA_NSS 39
#define LORA_RST 38
#define LORA_DIO0 37
#define LORA_DIO1 36
#define LORA_FREQ 433E6
#define LORA_SYNC_WORD 0x12
#define LORA_TX_POWER 17

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
#define CHUTE_A_PIN 5
#define CHUTE_B_PIN 6

#define NOTE_DO 262
#define NOTE_RE 294
#define NOTE_MI 330
#define NOTE_SOL 392
#define NOTE_HDO 523

// --- 建立感測器物件 ---
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_BMP3XX bmp;
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345); // ID 隨意
ICM42688 imu_primary(Wire, 0x68); // IMU42688 地址通常是 0x68 或 0x69
ICM42688 imu_alt(Wire, 0x69);
ICM42688 *imu = &imu_primary;
uint8_t imu_addr = 0x68;
RTC_DS3231 rtc;
Adafruit_NeoPixel status_led(WS2812_COUNT, WS2812_PIN, NEO_GRB + NEO_KHZ800);
HardwareSerial GPSSerial(1);
SPIClass loraSPI(HSPI);
SPIClass sdSPI(VSPI);
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

// --- 固定長度封包格式（44 bytes） ---
static const uint8_t kFrameLen = 44;
static const uint8_t kFrameHeader0 = 0x55;
static const uint8_t kFrameHeader1 = 0xAA;
static const uint8_t kMsgTypeTelemetry = 0x01;
static const float kG = 9.80665f;
static const float kBatteryRTop = 200000.0f;  // ohms (to battery)
static const float kBatteryRBottom = 100000.0f; // ohms (to GND)
static const float kBatteryDivider = kBatteryRBottom / (kBatteryRTop + kBatteryRBottom);

static int32_t gps_lat_raw = 0;
static int32_t gps_lon_raw = 0;
static int16_t gps_alt_dm = 0;
static int16_t gps_speed_dms = 0;
static uint8_t gps_sat_count = 0;
static uint32_t gps_last_fix_ms = 0;
static const uint32_t kGpsTimeoutMs = 2000;
static bool gps_fix_valid = false;

static void write_sd_csv_header() {
  if (!sd_log) {
    return;
  }
  sd_log.println("time_ms,rtc,lat,lon,gps_alt,baro_alt,speed,heading,sat,roll,pitch,accx,accy,accz,gyro_x,gyro_y,gyro_z,battery,temp,hum,pressure_pa,status,error,water");
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
                   "%lu,%lu,%.7f,%.7f,%.1f,%.1f,%.1f,%.1f,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.3f,%.2f,%.1f,%.0f,%u,%u,%u\n",
                   static_cast<unsigned long>(time_ms),
                   static_cast<unsigned long>(rtc_unix),
                   lat_deg, lon_deg, gps_alt_m, baro_alt_m, speed_ms, heading_deg, sat,
                   roll_deg, pitch_deg, accx_g, accy_g, accz_g, gyro_x_dps, gyro_y_dps, gyro_z_dps,
                   battery_v, temp_c, hum, pressure_pa,
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
  if (!star || star - line < 1) {
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
    (void)time_f;
    if (status_f && status_f[0] == 'A') {
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
        gps_last_fix_ms = millis();
        gps_fix_valid = true;
      }
      if (speed_f && *speed_f) {
        float speed_kn = static_cast<float>(atof(speed_f));
        float speed_ms = speed_kn * 0.514444f;
        gps_speed_dms = clamp_i16(lroundf(speed_ms * 10.0f));
        gps_last_fix_ms = millis();
        gps_fix_valid = true;
      }
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
    if (fix_f && fix_f[0] > '0') {
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
        gps_last_fix_ms = millis();
        gps_fix_valid = true;
      }
      if (sat_f && *sat_f) {
        gps_sat_count = static_cast<uint8_t>(atoi(sat_f));
        gps_last_fix_ms = millis();
        gps_fix_valid = true;
      }
      if (alt_f && *alt_f) {
        float alt_m = static_cast<float>(atof(alt_f));
        gps_alt_dm = clamp_i16(lroundf(alt_m * 10.0f));
        gps_last_fix_ms = millis();
        gps_fix_valid = true;
      }
    }
  }
}

static void process_gps_stream() {
  static char line[128];
  static uint8_t idx = 0;
  while (GPSSerial.available()) {
    char c = static_cast<char>(GPSSerial.read());
    if (c == '\n') {
      line[idx] = '\0';
      if (idx > 0) {
        handle_gps_line(line);
      }
      idx = 0;
      continue;
    }
    if (c == '\r') {
      continue;
    }
    if (idx < sizeof(line) - 1) {
      line[idx++] = c;
    } else {
      idx = 0;
    }
  }
}

static void handle_serial_rtc_sync() {
  static char line[64];
  static uint8_t idx = 0;
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
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
      if (idx < sizeof(line) - 1) {
        line[idx++] = c;
      }
    }
  }
}

static inline float inv_sqrt(float x) {
  return 1.0f / sqrtf(x);
}

static bool i2c_probe(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return bus.endTransmission() == 0;
}

static void i2c_scan_print(TwoWire &bus, const char *label) {
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
  Serial.begin(115200);
  while (!Serial); // 等待串口連接
  delay(1000);
  
  // 初始化 I2C
  Wire.begin(I2C_A_SDA, I2C_A_SCL);
  // 可選：提高 I2C 速度到 400kHz (Fast Mode)
  Wire.setClock(400000);
  Wire1.begin(I2C_B_SDA, I2C_B_SCL);
  Wire1.setClock(400000);
  i2c_scan_print(Wire, "I2C_A");
  i2c_scan_print(Wire1, "I2C_B");

  // LoRa init (SPI bus + radio)
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
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    DBG_PRINTLN("LoRa OK");
  } else {
    DBG_PRINTLN("LoRa init failed");
  }

  // SD init (dedicated SPI bus)
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

  status_led.begin();
  status_led.setBrightness(32);
  set_status_led(kStateIdle);

  pinMode(WATER_DETECT_PIN, INPUT);
  pinMode(DS3231_SQW_PIN, INPUT_PULLUP);
  rtc_ready = rtc.begin(&Wire1);
  if (rtc_ready) {
    if (rtc.lostPower()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
  }

  // 1. 初始化 SHT31 (溫濕度)
  if (!sht31.begin(0x44)) {   // 預設地址 0x44
  } else {
    status_sht = true;
  }

  // 2. 初始化 BMP390 (氣壓/高度)
  if (!bmp.begin_I2C()) {   // 自動偵測地址 0x77 或 0x76
  } else {
    status_bmp = true;
    // 設定 BMP390 採樣參數 (適合火箭的高速採樣)
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  // 3. 初始化 ADXL375 (高 G 值加速度計)
  if(!accel.begin()) {
  } else {
    status_adxl = true;
  }

  // 4. 初始化 IMU42688 (陀螺儀/加速度計)
  bool imu_present = false;
  if (i2c_probe(Wire, 0x68)) {
    imu = &imu_primary;
    imu_addr = 0x68;
    imu_present = true;
  } else if (i2c_probe(Wire, 0x69)) {
    imu = &imu_alt;
    imu_addr = 0x69;
    imu_present = true;
  }

  if (imu_present) {
    int imu_status = imu->begin();
    if (imu_status < 0) {
      DBG_PRINTF("ICM42688 init failed on 0x%02X\n", imu_addr);
    } else {
      status_imu = true;
      DBG_PRINTF("ICM42688 OK on 0x%02X\n", imu_addr);
      // 設定 IMU 範圍 (火箭通常需要最大範圍)
      imu->setAccelFS(ICM42688::gpm16); // ±16g
      imu->setGyroFS(ICM42688::dps2000); // ±2000 dps
    }
  } else {
    DBG_PRINTLN("ICM42688 not detected on 0x68/0x69");
  }

  ledcSetup(BUZZER_CH, 2000, BUZZER_RES);
  ledcAttachPin(BUZZER_PIN, BUZZER_CH);

  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

  GPSSerial.begin(UART_GPS_BAUD, SERIAL_8N1, UART_GPS_RX, UART_GPS_TX);
  pinMode(UART_GPS_PPS, INPUT);
  
  delay(1000);
  // DJI-style startup tone
  buzzerTone(NOTE_DO, 120);
  delay(20);
  buzzerTone(NOTE_MI, 120);
  delay(20);
  buzzerTone(NOTE_SOL, 120);
  delay(20);
  buzzerTone(NOTE_HDO, 120);
}

void loop() {
  uint32_t time_ms = millis();
  handle_serial_rtc_sync();
  process_gps_stream();
  if (gps_last_fix_ms > 0 && time_ms - gps_last_fix_ms > kGpsTimeoutMs) {
    gps_fix_valid = false;
    gps_sat_count = 0;
  }

  float temp_c = 0.0f;
  float hum = 0.0f;
  if (status_sht) {
    temp_c = sht31.readTemperature();
    hum = sht31.readHumidity();
  }

  float pressure_pa = 0.0f;
  float baro_alt_m = 0.0f;
  if (status_bmp && bmp.performReading()) {
    pressure_pa = bmp.pressure;
    baro_alt_m = bmp.readAltitude(1013.25);
  }

  float accx_g = 0.0f;
  float accy_g = 0.0f;
  float accz_g = 0.0f;
  if (status_adxl) {
    sensors_event_t event;
    accel.getEvent(&event);
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
  uint16_t gps_heading_ddeg = 0;
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
    gps_heading_ddeg = clamp_u16(lroundf(yaw_hold * 10.0f));
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

  int32_t lat_raw = gps_lat_raw;
  int32_t lon_raw = gps_lon_raw;
  int16_t gps_alt_dm_local = gps_alt_dm;
  int16_t baro_alt_dm = clamp_i16(lroundf(baro_alt_m * 10.0f));
  int16_t gps_speed_dms_local = gps_speed_dms;
  uint8_t sat_count = gps_fix_valid ? gps_sat_count : 0;
  int16_t roll_cdeg = clamp_i16(lroundf(roll_deg * 100.0f));
  int16_t pitch_cdeg = clamp_i16(lroundf(pitch_deg * 100.0f));
  int16_t gyro_ddeg_s = clamp_i16(lroundf(gyro_z_dps * 10.0f));
  int16_t accx_cg = clamp_i16(lroundf(accx_g * 100.0f));
  int16_t accy_cg = clamp_i16(lroundf(accy_g * 100.0f));
  int16_t accz_cg = clamp_i16(lroundf(accz_g * 100.0f));
  int16_t gyro_x_ddeg_s = clamp_i16(lroundf(gyro_x_dps * 10.0f));
  int16_t gyro_y_ddeg_s = clamp_i16(lroundf(gyro_y_dps * 10.0f));
  uint8_t flight_state = kStateIdle;
  static uint8_t last_state = 0xFF;
  if (flight_state != last_state) {
    set_status_led(flight_state);
    last_state = flight_state;
  }
  uint8_t error_code = 0;
  uint8_t water_detected = digitalRead(WATER_DETECT_PIN) ? 1 : 0;
  uint32_t rtc_unix = 0;
  if (rtc_ready) {
    rtc_unix = rtc.now().unixtime();
  }
  uint16_t battery_mv = read_battery_mv();
  uint32_t pressure_u32 = (pressure_pa < 0.0f) ? 0 : static_cast<uint32_t>(lroundf(pressure_pa));

  uint8_t frame[kFrameLen];
  frame[0] = kFrameHeader0;
  frame[1] = kFrameHeader1;
  frame[2] = kMsgTypeTelemetry;
  uint16_t time_0p1s = static_cast<uint16_t>((time_ms / 100) & 0xFFFF);
  write_u16_le(frame, 3, time_0p1s);
  write_i32_le(frame, 5, lat_raw);
  write_i32_le(frame, 9, lon_raw);
  write_i16_le(frame, 13, gps_alt_dm_local);
  write_i16_le(frame, 15, gps_speed_dms_local);
  frame[17] = sat_count;
  write_i16_le(frame, 18, roll_cdeg);
  write_i16_le(frame, 20, pitch_cdeg);
  write_u16_le(frame, 22, gps_heading_ddeg);
  write_i16_le(frame, 24, gyro_x_ddeg_s);
  write_i16_le(frame, 26, gyro_y_ddeg_s);
  write_i16_le(frame, 28, gyro_ddeg_s);
  write_i16_le(frame, 30, accx_cg);
  write_i16_le(frame, 32, accy_cg);
  write_i16_le(frame, 34, accz_cg);
  write_i16_le(frame, 36, baro_alt_dm);
  write_u16_le(frame, 38, battery_mv);
  frame[40] = flight_state;
  frame[41] = error_code;
  frame[42] = water_detected;

  uint8_t crc = 0;
  for (int i = 0; i < kFrameLen - 1; i++) {
    crc ^= frame[i];
  }
  frame[kFrameLen - 1] = crc;

  if (lora_ready) {
    LoRa.beginPacket();
    LoRa.write(frame, kFrameLen);
    LoRa.endPacket();
  }

  if (sd_ready && sd_log) {
    float lat_deg = gps_lat_raw / 1e7f;
    float lon_deg = gps_lon_raw / 1e7f;
    float gps_alt_m = gps_alt_dm_local / 10.0f;
    float speed_ms = gps_speed_dms_local / 10.0f;
    float heading_deg = gps_heading_ddeg / 10.0f;
    write_sd_csv_line(
        time_ms,
        rtc_unix,
        lat_deg,
        lon_deg,
        gps_alt_m,
        baro_alt_m,
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
        temp_c,
        hum,
        pressure_pa,
        flight_state,
        error_code,
        water_detected);
    if (time_ms % 1000 < 100) {
      sd_log.flush();
    }
  }

  Serial.write(frame, kFrameLen);

  // 簡易延遲，約 10Hz (實際飛行時不能用 delay)
  delay(100); 
}
