#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

// 引入感測器函式庫
#include "Adafruit_SHT31.h"
#include "Adafruit_BMP3XX.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_ADXL375.h"
#include "ICM42688.h"

// --- 定義 I2C 腳位 (請依您的 ESP32 接腳修改) ---
// ESP32 一般預設: SDA=21, SCL=22
// ESP32-S3 一般預設: SDA=8, SCL=9 或其他，請查閱您的板子圖
#define I2C_SDA 8
#define I2C_SCL 9

// --- 建立感測器物件 ---
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_BMP3XX bmp;
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345); // ID 隨意
ICM42688 imu(Wire, 0x68); // IMU42688 地址通常是 0x68 或 0x69

// --- 狀態旗標 (檢查誰活著) ---
bool status_sht = false;
bool status_bmp = false;
bool status_adxl = false;
bool status_imu = false;

// --- 舊協議 41-byte 封包格式 ---
static const uint8_t kFrameLen = 53;
static const uint8_t kFrameHeader0 = 0x55;
static const uint8_t kFrameHeader1 = 0xAA;
static const uint8_t kMsgTypeTelemetry = 0x01;
static const float kG = 9.80665f;

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

static inline float inv_sqrt(float x) {
  return 1.0f / sqrtf(x);
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
  Wire.begin(I2C_SDA, I2C_SCL);
  // 可選：提高 I2C 速度到 400kHz (Fast Mode)
  Wire.setClock(400000); 

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
  int imu_status = imu.begin();
  if (imu_status < 0) {
  } else {
    status_imu = true;
    // 設定 IMU 範圍 (火箭通常需要最大範圍)
    imu.setAccelFS(ICM42688::gpm16); // ±16g
    imu.setGyroFS(ICM42688::dps2000); // ±2000 dps
  }
  
  delay(1000);
}

void loop() {
  uint32_t time_ms = millis();

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
  if (status_imu) {
    imu.getAGT();
    float gx_raw = imu.gyrX();
    float gy_raw = imu.gyrY();
    float gz_raw = imu.gyrZ();
    gyro_x_dps = gx_raw;
    gyro_y_dps = gy_raw;
    gyro_z_dps = gz_raw;

    if (!fusion_init) {
      // Map to fusion axes so roll=Z, pitch=X, yaw=Y.
      float ax_body = imu.accZ();
      float ay_body = imu.accX();
      float az_body = imu.accY();
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
        float ax_body = imu.accZ();
        float ay_body = imu.accX();
        float az_body = imu.accY();
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

  int32_t lat_raw = 0;
  int32_t lon_raw = 0;
  int16_t gps_alt_dm = clamp_i16(lroundf(baro_alt_m * 10.0f));
  int16_t baro_alt_dm = clamp_i16(lroundf(baro_alt_m * 10.0f));
  int16_t gps_speed_dms = 0;
  uint8_t sat_count = 0;
  int16_t roll_cdeg = clamp_i16(lroundf(roll_deg * 100.0f));
  int16_t pitch_cdeg = clamp_i16(lroundf(pitch_deg * 100.0f));
  int16_t gyro_ddeg_s = clamp_i16(lroundf(gyro_z_dps * 10.0f));
  int16_t accx_cg = clamp_i16(lroundf(accx_g * 100.0f));
  int16_t accy_cg = clamp_i16(lroundf(accy_g * 100.0f));
  int16_t accz_cg = clamp_i16(lroundf(accz_g * 100.0f));
  int16_t gyro_x_ddeg_s = clamp_i16(lroundf(gyro_x_dps * 10.0f));
  int16_t gyro_y_ddeg_s = clamp_i16(lroundf(gyro_y_dps * 10.0f));
  uint8_t flight_state = 1; // IDLE
  uint8_t error_code = 0;
  uint16_t battery_mv = 0;
  int16_t temp_c_centi = clamp_i16(lroundf(temp_c * 100.0f));
  uint16_t hum_deci = clamp_u16(lroundf(hum * 10.0f));
  uint32_t pressure_u32 = (pressure_pa < 0.0f) ? 0 : static_cast<uint32_t>(lroundf(pressure_pa));

  uint8_t frame[kFrameLen];
  frame[0] = kFrameHeader0;
  frame[1] = kFrameHeader1;
  frame[2] = kMsgTypeTelemetry;
  write_u32_le(frame, 3, time_ms);
  write_i32_le(frame, 7, lat_raw);
  write_i32_le(frame, 11, lon_raw);
  write_i16_le(frame, 15, gps_alt_dm);
  write_i16_le(frame, 17, gps_speed_dms);
  write_u16_le(frame, 19, gps_heading_ddeg);
  frame[21] = sat_count;
  write_i16_le(frame, 22, roll_cdeg);
  write_i16_le(frame, 24, pitch_cdeg);
  write_i16_le(frame, 26, gyro_ddeg_s);
  frame[28] = flight_state;
  frame[29] = error_code;
  write_u16_le(frame, 30, battery_mv);
  write_i16_le(frame, 32, temp_c_centi);
  write_u16_le(frame, 34, hum_deci);
  write_u32_le(frame, 36, pressure_u32);
  write_i16_le(frame, 40, baro_alt_dm);
  write_i16_le(frame, 42, accx_cg);
  write_i16_le(frame, 44, accy_cg);
  write_i16_le(frame, 46, accz_cg);
  write_i16_le(frame, 48, gyro_x_ddeg_s);
  write_i16_le(frame, 50, gyro_y_ddeg_s);

  uint8_t crc = 0;
  for (int i = 0; i < kFrameLen - 1; i++) {
    crc ^= frame[i];
  }
  frame[kFrameLen - 1] = crc;

  Serial.write(frame, kFrameLen);

  // 簡易延遲，約 10Hz (實際飛行時不能用 delay)
  delay(100); 
}
