#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <math.h>

// ESP32-S3 N16R8 BMP388 barometric altitude test.
// Native USB CDC is disabled in platformio.ini. Console output uses CH343/UART0.

static constexpr uint32_t kSerialBaud = 115200;
static constexpr int kConsoleRx = 44;
static constexpr int kConsoleTx = 43;

static constexpr int kI2cSda = 6;
static constexpr int kI2cScl = 7;
static constexpr uint32_t kI2cInitClockHz = 100000;
static constexpr uint32_t kI2cClockHz = 400000;
static constexpr uint32_t kI2cScanClocksHz[] = {10000, 50000, 100000, 400000};

static constexpr uint8_t kBmpAddr76 = 0x76;
static constexpr uint8_t kBmpAddr77 = 0x77;
static constexpr uint8_t kIcmAddr68 = 0x68;
static constexpr uint8_t kIcmAddr69 = 0x69;
static constexpr uint8_t kIcmWhoAmIReg = 0x75;
static constexpr uint8_t kIcm42688WhoAmI = 0x47;
static constexpr uint8_t kIcmTempData1Reg = 0x1D;
static constexpr uint8_t kIcmPwrMgmt0Reg = 0x4E;
static constexpr uint8_t kIcmGyroConfig0Reg = 0x4F;
static constexpr uint8_t kIcmAccelConfig0Reg = 0x50;
static constexpr uint16_t kBaselineSamples = 100;
static constexpr uint32_t kSampleIntervalUs = 20000;  // 50 Hz
static constexpr uint32_t kImuSampleIntervalUs = 20000;  // 50 Hz
static constexpr float kPressureFilterAlpha = 0.12f;
static constexpr uint32_t kStatusIntervalMs = 1000;
static constexpr uint32_t kBmpRetryIntervalMs = 5000;

static Adafruit_BMP3XX bmp;

static bool bmp_ready = false;
static bool baseline_ready = false;
static bool bmp_addr_seen = false;
static bool bmp_begin_failed = false;
static bool icm_ready = false;
static bool icm_addr_seen = false;
static bool seen_addr_69 = false;
static bool seen_reserved_addr = false;
static bool sda_low_at_boot = false;
static bool scl_low_at_boot = false;

static uint8_t active_addr = 0;
static uint8_t icm_addr = 0;
static uint8_t icm_whoami = 0;
static uint8_t i2c_device_count = 0;
static uint16_t baseline_count = 0;

static uint32_t last_sample_us = 0;
static uint32_t last_imu_sample_us = 0;
static uint32_t last_status_ms = 0;
static uint32_t last_bmp_retry_ms = 0;
static uint32_t last_read_attempt_ms = 0;
static uint32_t last_successful_read_ms = 0;
static uint32_t sample_count = 0;
static uint32_t failed_read_count = 0;
static uint32_t imu_sample_count = 0;
static uint32_t imu_failed_read_count = 0;
static uint32_t last_velocity_us = 0;

static double baseline_pressure_sum_pa = 0.0;
static float baseline_pressure_pa = NAN;
static float temperature_c = NAN;
static float pressure_raw_pa = NAN;
static float pressure_pa = NAN;
static float raw_altitude_m = NAN;
static float filtered_altitude_m = NAN;
static float raw_vertical_speed_ms = NAN;
static float vertical_speed_ms = NAN;
static float min_altitude_m = NAN;
static float max_altitude_m = NAN;
static float last_raw_altitude_m = NAN;
static float last_filtered_altitude_m = NAN;
static float imu_temp_c = NAN;
static float imu_accel_x_g = NAN;
static float imu_accel_y_g = NAN;
static float imu_accel_z_g = NAN;
static float imu_gyro_x_dps = NAN;
static float imu_gyro_y_dps = NAN;
static float imu_gyro_z_dps = NAN;

static char command_line[32];
static uint8_t command_idx = 0;

static void print_float_or_nan(float value, uint8_t decimals);

static bool i2c_probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool i2c_read_u8(uint8_t addr, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(static_cast<int>(addr), 1) != 1) {
    return false;
  }
  value = Wire.read();
  return true;
}

static bool i2c_write_u8(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool i2c_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buffer, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(static_cast<int>(addr), static_cast<int>(len)) != static_cast<int>(len)) {
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

static void print_hex_u8(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

static void probe_common_id_registers(uint8_t addr) {
  const uint8_t regs[] = {0x00, 0xD0, 0x75};

  Serial.print("# probe addr=0x");
  print_hex_u8(addr);
  for (uint8_t i = 0; i < sizeof(regs); i++) {
    uint8_t value = 0;
    Serial.print(" reg0x");
    print_hex_u8(regs[i]);
    Serial.print('=');
    if (i2c_read_u8(addr, regs[i], value)) {
      Serial.print("0x");
      print_hex_u8(value);
    } else {
      Serial.print("ERR");
    }
  }
  Serial.println();
}

static void print_line_levels(const char *tag) {
  Serial.print("# ");
  Serial.print(tag);
  Serial.print(" sda=");
  Serial.print(digitalRead(kI2cSda) == HIGH ? "HIGH" : "LOW");
  Serial.print(" scl=");
  Serial.println(digitalRead(kI2cScl) == HIGH ? "HIGH" : "LOW");
}

static void recover_i2c_bus() {
  Wire.end();
  pinMode(kI2cSda, INPUT_PULLUP);
  pinMode(kI2cScl, INPUT_PULLUP);
  delay(10);

  print_line_levels("before_recovery");
  sda_low_at_boot = digitalRead(kI2cSda) == LOW;
  scl_low_at_boot = digitalRead(kI2cScl) == LOW;

  if (sda_low_at_boot) {
    pinMode(kI2cScl, OUTPUT_OPEN_DRAIN);
    pinMode(kI2cSda, INPUT_PULLUP);
    for (uint8_t i = 0; i < 18 && digitalRead(kI2cSda) == LOW; i++) {
      digitalWrite(kI2cScl, LOW);
      delayMicroseconds(10);
      digitalWrite(kI2cScl, HIGH);
      delayMicroseconds(10);
    }
  }

  pinMode(kI2cScl, INPUT_PULLUP);
  pinMode(kI2cSda, INPUT_PULLUP);
  delay(10);
  print_line_levels("after_recovery");
  Wire.begin(kI2cSda, kI2cScl, kI2cInitClockHz);
  Wire.setTimeOut(50);
}

static uint8_t scan_i2c_at_clock(uint32_t clock_hz) {
  uint8_t count = 0;
  Wire.setClock(clock_hz);

  Serial.print("# scan_i2c clock_hz=");
  Serial.println(clock_hz);

  for (uint8_t addr = 1; addr < 127; addr++) {
    if (i2c_probe(addr)) {
      count++;
      Serial.print("# i2c_addr=0x");
      print_hex_u8(addr);
      Serial.println();

      if (addr == kBmpAddr76 || addr == kBmpAddr77) {
        bmp_addr_seen = true;
      }
      if (addr == kIcmAddr68 || addr == kIcmAddr69) {
        icm_addr_seen = true;
      }
      if (addr == kIcmAddr69) {
        seen_addr_69 = true;
      }
      if (addr >= 0x78) {
        seen_reserved_addr = true;
      }
      if (addr == kIcmAddr68 || addr == kIcmAddr69 || addr == kBmpAddr76 || addr == kBmpAddr77 || addr >= 0x78) {
        probe_common_id_registers(addr);
      }
    }
    delay(1);
  }

  Serial.print("# i2c_count=");
  Serial.println(count);
  return count;
}

static uint8_t scan_i2c_all_speeds() {
  uint8_t best_count = 0;
  bmp_addr_seen = false;
  icm_addr_seen = false;
  seen_addr_69 = false;
  seen_reserved_addr = false;

  for (uint8_t i = 0; i < sizeof(kI2cScanClocksHz) / sizeof(kI2cScanClocksHz[0]); i++) {
    const uint8_t count = scan_i2c_at_clock(kI2cScanClocksHz[i]);
    if (count > best_count) {
      best_count = count;
    }
  }

  Wire.setClock(kI2cInitClockHz);
  return best_count;
}

static void run_manual_i2c_scan() {
  Serial.println("# manual_i2c_scan_start");
  recover_i2c_bus();
  i2c_device_count = scan_i2c_all_speeds();
  Serial.println("# manual_direct_bmp_addr_probe");
  probe_common_id_registers(kBmpAddr76);
  probe_common_id_registers(kBmpAddr77);
  Serial.println("# manual_direct_icm42688_addr_probe");
  probe_common_id_registers(kIcmAddr68);
  probe_common_id_registers(kIcmAddr69);
  Serial.print("# manual_i2c_scan_done i2c_count=");
  Serial.print(i2c_device_count);
  Serial.print(" bmp_ready=");
  Serial.print(bmp_ready ? "yes" : "no");
  Serial.print(" icm_ready=");
  Serial.println(icm_ready ? "yes" : "no");
}

static bool validate_icm42688_addr(uint8_t addr) {
  uint8_t value = 0;
  if (!i2c_read_u8(addr, kIcmWhoAmIReg, value)) {
    return false;
  }

  icm_addr_seen = true;
  if (value != kIcm42688WhoAmI) {
    return false;
  }

  icm_ready = true;
  icm_addr = addr;
  icm_whoami = value;
  return true;
}

static bool configure_icm42688() {
  if (!icm_ready) {
    return false;
  }

  // Enable accel and gyro in low-noise mode, then set both ODRs to 50 Hz.
  if (!i2c_write_u8(icm_addr, kIcmPwrMgmt0Reg, 0x0F)) {
    return false;
  }
  delay(100);
  if (!i2c_write_u8(icm_addr, kIcmGyroConfig0Reg, 0x09)) {
    return false;
  }
  if (!i2c_write_u8(icm_addr, kIcmAccelConfig0Reg, 0x09)) {
    return false;
  }
  delay(10);
  return true;
}

static bool validate_icm42688() {
  icm_ready = false;
  icm_addr = 0;
  icm_whoami = 0;

  if (i2c_probe(kIcmAddr68) && validate_icm42688_addr(kIcmAddr68)) {
    return configure_icm42688();
  }
  if (i2c_probe(kIcmAddr69) && validate_icm42688_addr(kIcmAddr69)) {
    return configure_icm42688();
  }
  return false;
}

static bool try_begin_bmp(uint8_t addr) {
  if (!i2c_probe(addr)) {
    return false;
  }

  bmp_addr_seen = true;
  Serial.print("# bmp_begin_try addr=0x");
  print_hex_u8(addr);
  Serial.println();
  if (!bmp.begin_I2C(addr, &Wire)) {
    bmp_begin_failed = true;
    Serial.print("# bmp_begin_failed addr=0x");
    print_hex_u8(addr);
    Serial.println();
    return false;
  }

  active_addr = addr;
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Wire.setClock(kI2cClockHz);
  Serial.print("# bmp_begin_ready addr=0x");
  print_hex_u8(addr);
  Serial.print(" chip_id=0x");
  print_hex_u8(bmp.chipID());
  Serial.println();
  return true;
}

static bool init_bmp() {
  bmp_ready = false;
  active_addr = 0;
  bmp_begin_failed = false;
  icm_ready = false;
  icm_addr = 0;
  icm_whoami = 0;
  baseline_ready = false;
  baseline_count = 0;
  baseline_pressure_sum_pa = 0.0;
  baseline_pressure_pa = NAN;
  pressure_raw_pa = NAN;
  pressure_pa = NAN;
  sample_count = 0;
  failed_read_count = 0;
  raw_altitude_m = NAN;
  filtered_altitude_m = NAN;
  raw_vertical_speed_ms = NAN;
  last_filtered_altitude_m = NAN;
  last_raw_altitude_m = NAN;
  vertical_speed_ms = NAN;
  min_altitude_m = NAN;
  max_altitude_m = NAN;
  last_velocity_us = 0;

  recover_i2c_bus();
  i2c_device_count = scan_i2c_all_speeds();

  Serial.println("# direct_bmp_addr_probe");
  probe_common_id_registers(kBmpAddr76);
  probe_common_id_registers(kBmpAddr77);

  if (try_begin_bmp(kBmpAddr76)) {
    Serial.println("# direct_icm42688_addr_probe");
    probe_common_id_registers(kIcmAddr68);
    probe_common_id_registers(kIcmAddr69);
    validate_icm42688();
    return true;
  }
  if (try_begin_bmp(kBmpAddr77)) {
    Serial.println("# direct_icm42688_addr_probe");
    probe_common_id_registers(kIcmAddr68);
    probe_common_id_registers(kIcmAddr69);
    validate_icm42688();
    return true;
  }
  Serial.println("# direct_icm42688_addr_probe");
  probe_common_id_registers(kIcmAddr68);
  probe_common_id_registers(kIcmAddr69);
  validate_icm42688();
  return false;
}

static void retry_bmp_if_needed(uint32_t now_ms) {
  if (bmp_ready || now_ms - last_bmp_retry_ms < kBmpRetryIntervalMs) {
    return;
  }

  last_bmp_retry_ms = now_ms;
  Serial.println("# retry_bmp_init");
  bmp_ready = init_bmp();
  Serial.print("# bmp_retry_result=");
  Serial.print(bmp_ready ? "ready" : "not_found");
  Serial.print(" addr=0x");
  print_hex_u8(active_addr);
  Serial.println();
}

static float altitude_from_pressure(float pressure, float reference_pressure) {
  return 44330.0f * (1.0f - powf(pressure / reference_pressure, 0.19029495f));
}

static void update_filtered_pressure(float compensated_pressure_pa) {
  if (!isfinite(pressure_pa) || pressure_pa <= 0.0f) {
    pressure_pa = compensated_pressure_pa;
    return;
  }
  pressure_pa = pressure_pa * (1.0f - kPressureFilterAlpha) + compensated_pressure_pa * kPressureFilterAlpha;
}

static void reset_altitude_tracking() {
  raw_altitude_m = 0.0f;
  filtered_altitude_m = 0.0f;
  raw_vertical_speed_ms = NAN;
  vertical_speed_ms = NAN;
  min_altitude_m = 0.0f;
  max_altitude_m = 0.0f;
  last_raw_altitude_m = 0.0f;
  last_filtered_altitude_m = 0.0f;
  last_velocity_us = micros();
}

static void start_baro_calibration() {
  baseline_ready = false;
  baseline_count = 0;
  baseline_pressure_sum_pa = 0.0;
  baseline_pressure_pa = NAN;
  pressure_raw_pa = NAN;
  pressure_pa = NAN;
  raw_altitude_m = NAN;
  filtered_altitude_m = NAN;
  raw_vertical_speed_ms = NAN;
  vertical_speed_ms = NAN;
  min_altitude_m = NAN;
  max_altitude_m = NAN;
  last_raw_altitude_m = NAN;
  last_filtered_altitude_m = NAN;
  last_velocity_us = 0;

  Serial.print("# baro_calibration_started samples=");
  Serial.println(kBaselineSamples);
}

static bool zero_baro_to_current_pressure() {
  if (!isfinite(pressure_pa) || pressure_pa <= 0.0f) {
    if (!bmp_ready || !bmp.performReading()) {
      Serial.println("# baro_zero_failed reason=NO_VALID_PRESSURE");
      return false;
    }
    temperature_c = static_cast<float>(bmp.temperature);
    pressure_raw_pa = static_cast<float>(bmp.pressure);
    pressure_pa = pressure_raw_pa;
  }

  if (!isfinite(pressure_pa) || pressure_pa <= 0.0f) {
    Serial.println("# baro_zero_failed reason=INVALID_PRESSURE");
    return false;
  }

  baseline_pressure_pa = pressure_pa;
  baseline_pressure_sum_pa = static_cast<double>(pressure_pa) * kBaselineSamples;
  baseline_count = kBaselineSamples;
  baseline_ready = true;
  reset_altitude_tracking();

  Serial.print("# baro_zeroed pressure_pa=");
  print_float_or_nan(baseline_pressure_pa, 2);
  Serial.println();
  return true;
}

static bool read_baro_sample() {
  last_read_attempt_ms = millis();
  if (!bmp.performReading()) {
    failed_read_count++;
    return false;
  }

  temperature_c = static_cast<float>(bmp.temperature);
  pressure_raw_pa = static_cast<float>(bmp.pressure);

  if (!isfinite(temperature_c) || !isfinite(pressure_raw_pa) || pressure_raw_pa <= 0.0f) {
    failed_read_count++;
    return false;
  }

  // BMP388 reports temperature-compensated pressure. Apply an IIR filter before
  // baseline averaging and altitude calculation so short pressure spikes do not
  // dominate vertical speed.
  update_filtered_pressure(pressure_raw_pa);

  last_successful_read_ms = last_read_attempt_ms;
  sample_count++;

  if (!baseline_ready) {
    baseline_pressure_sum_pa += pressure_pa;
    baseline_count++;
    if (baseline_count >= kBaselineSamples) {
      baseline_pressure_pa = static_cast<float>(baseline_pressure_sum_pa / baseline_count);
      baseline_ready = true;
      reset_altitude_tracking();
      Serial.print("# baseline_locked pressure_pa=");
      print_float_or_nan(baseline_pressure_pa, 2);
      Serial.println();
    }
    return true;
  }

  raw_altitude_m = altitude_from_pressure(pressure_raw_pa, baseline_pressure_pa);
  filtered_altitude_m = altitude_from_pressure(pressure_pa, baseline_pressure_pa);

  const uint32_t now_us = micros();
  if (last_velocity_us != 0 && now_us != last_velocity_us &&
      isfinite(last_raw_altitude_m) && isfinite(last_filtered_altitude_m)) {
    const float dt = static_cast<float>(now_us - last_velocity_us) / 1000000.0f;
    raw_vertical_speed_ms = (raw_altitude_m - last_raw_altitude_m) / dt;
    const float vz = (filtered_altitude_m - last_filtered_altitude_m) / dt;
    if (!isfinite(vertical_speed_ms)) {
      vertical_speed_ms = vz;
    } else {
      vertical_speed_ms = vertical_speed_ms * 0.8f + vz * 0.2f;
    }
  }

  last_velocity_us = now_us;
  last_raw_altitude_m = raw_altitude_m;
  last_filtered_altitude_m = filtered_altitude_m;

  if (!isfinite(min_altitude_m) || filtered_altitude_m < min_altitude_m) {
    min_altitude_m = filtered_altitude_m;
  }
  if (!isfinite(max_altitude_m) || filtered_altitude_m > max_altitude_m) {
    max_altitude_m = filtered_altitude_m;
  }

  return true;
}

static int16_t be_i16(const uint8_t *data) {
  return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

static bool read_imu_sample() {
  if (!icm_ready) {
    return false;
  }

  uint8_t raw[14] = {};
  if (!i2c_read_bytes(icm_addr, kIcmTempData1Reg, raw, sizeof(raw))) {
    imu_failed_read_count++;
    return false;
  }

  const int16_t temp_raw = be_i16(&raw[0]);
  const int16_t ax_raw = be_i16(&raw[2]);
  const int16_t ay_raw = be_i16(&raw[4]);
  const int16_t az_raw = be_i16(&raw[6]);
  const int16_t gx_raw = be_i16(&raw[8]);
  const int16_t gy_raw = be_i16(&raw[10]);
  const int16_t gz_raw = be_i16(&raw[12]);

  imu_temp_c = static_cast<float>(temp_raw) / 132.48f + 25.0f;
  imu_accel_x_g = static_cast<float>(ax_raw) / 2048.0f;
  imu_accel_y_g = static_cast<float>(ay_raw) / 2048.0f;
  imu_accel_z_g = static_cast<float>(az_raw) / 2048.0f;
  imu_gyro_x_dps = static_cast<float>(gx_raw) / 16.4f;
  imu_gyro_y_dps = static_cast<float>(gy_raw) / 16.4f;
  imu_gyro_z_dps = static_cast<float>(gz_raw) / 16.4f;
  imu_sample_count++;
  return true;
}

static void print_float_or_nan(float value, uint8_t decimals) {
  if (isfinite(value)) {
    Serial.print(value, decimals);
  } else {
    Serial.print("nan");
  }
}

static void print_csv_header() {
  Serial.println("time_us,sample,baseline_ready,temp_c,pressure_raw_pa,pressure_filt_pa,alt_raw_m,alt_filt_m,vz_raw_mps,vz_filt_mps,baseline_pressure_pa,failed_reads");
}

static void print_imu_csv_header() {
  Serial.println("imu_time_us,imu_sample,imu_temp_c,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,imu_failed_reads");
}

static void print_csv_sample() {
  Serial.print(micros());
  Serial.print(',');
  Serial.print(sample_count);
  Serial.print(',');
  Serial.print(baseline_ready ? 1 : 0);
  Serial.print(',');
  print_float_or_nan(temperature_c, 2);
  Serial.print(',');
  print_float_or_nan(pressure_raw_pa, 2);
  Serial.print(',');
  print_float_or_nan(pressure_pa, 2);
  Serial.print(',');
  print_float_or_nan(raw_altitude_m, 3);
  Serial.print(',');
  print_float_or_nan(filtered_altitude_m, 3);
  Serial.print(',');
  print_float_or_nan(raw_vertical_speed_ms, 3);
  Serial.print(',');
  print_float_or_nan(vertical_speed_ms, 3);
  Serial.print(',');
  print_float_or_nan(baseline_pressure_pa, 2);
  Serial.print(',');
  Serial.println(failed_read_count);
}

static void print_imu_csv_sample() {
  Serial.print(micros());
  Serial.print(',');
  Serial.print(imu_sample_count);
  Serial.print(',');
  print_float_or_nan(imu_temp_c, 3);
  Serial.print(',');
  print_float_or_nan(imu_accel_x_g, 5);
  Serial.print(',');
  print_float_or_nan(imu_accel_y_g, 5);
  Serial.print(',');
  print_float_or_nan(imu_accel_z_g, 5);
  Serial.print(',');
  print_float_or_nan(imu_gyro_x_dps, 3);
  Serial.print(',');
  print_float_or_nan(imu_gyro_y_dps, 3);
  Serial.print(',');
  print_float_or_nan(imu_gyro_z_dps, 3);
  Serial.print(',');
  Serial.println(imu_failed_read_count);
}

static const char *bmp_not_found_reason() {
  if (sda_low_at_boot || scl_low_at_boot) {
    return "I2C_LINE_STUCK_LOW_CHECK_SHORTS_POWER_PULLUPS";
  }
  if (i2c_device_count == 0) {
    return "NO_I2C_DEVICE_CHECK_WIRING_POWER_SDA6_SCL7";
  }
  if (seen_addr_69 && seen_reserved_addr) {
    return "FOUND_0x69_AND_RESERVED_ADDR_BMP388_MISSING_CHECK_MODULE_WIRING_PULLUPS_OR_SENSOR_TYPE";
  }
  if (seen_addr_69) {
    return "FOUND_0x69_LIKELY_ICM42688_BMP388_SHOULD_BE_0x76_OR_0x77";
  }
  if (seen_reserved_addr) {
    return "FOUND_RESERVED_I2C_ADDR_CHECK_PULLUPS_WIRING_OR_BAD_CONTACT";
  }
  if (!bmp_addr_seen) {
    return "I2C_DEVICE_FOUND_BUT_NOT_0x76_OR_0x77";
  }
  if (bmp_begin_failed) {
    return "BMP_ADDR_ACK_BUT_LIBRARY_BEGIN_FAILED_CHECK_BMP388_SENSOR_OR_CS_PIN";
  }
  return "UNKNOWN";
}

static void print_status(uint32_t now_ms) {
  Serial.print("# status=");
  if (!bmp_ready) {
    Serial.print("BMP_NOT_FOUND reason=");
    Serial.print(bmp_not_found_reason());
  } else if (last_read_attempt_ms == 0) {
    Serial.print("BMP_READY_WAIT_SAMPLE addr=0x");
    print_hex_u8(active_addr);
  } else if (last_successful_read_ms == 0 || last_successful_read_ms < last_read_attempt_ms) {
    Serial.print("BMP_READ_FAIL addr=0x");
    print_hex_u8(active_addr);
  } else if (!baseline_ready) {
    Serial.print("CALIBRATING addr=0x");
    print_hex_u8(active_addr);
  } else {
    Serial.print("TRACK addr=0x");
    print_hex_u8(active_addr);
  }

  Serial.print(" i2c_count=");
  Serial.print(i2c_device_count);
  Serial.print(" samples=");
  Serial.print(sample_count);
  Serial.print(" baseline=");
  Serial.print(baseline_count);
  Serial.print('/');
  Serial.print(kBaselineSamples);
  Serial.print(" failed_reads=");
  Serial.print(failed_read_count);
  Serial.print(" pressure_raw_pa=");
  print_float_or_nan(pressure_raw_pa, 2);
  Serial.print(" pressure_filter_alpha=");
  Serial.print(kPressureFilterAlpha, 3);
  Serial.print(" icm42688=");
  Serial.print(icm_ready ? "ready" : "not_ready");
  Serial.print(" icm_addr=0x");
  print_hex_u8(icm_addr);
  Serial.print(" icm_whoami=0x");
  print_hex_u8(icm_whoami);
  Serial.print(" imu_samples=");
  Serial.print(imu_sample_count);
  Serial.print(" imu_failed_reads=");
  Serial.print(imu_failed_read_count);
  Serial.print(" last_read_attempt_ms=");
  Serial.print(last_read_attempt_ms);
  Serial.print(" last_successful_read_ms=");
  Serial.print(last_successful_read_ms);
  Serial.print(" uptime_ms=");
  Serial.println(now_ms);
}

static void handle_command(const char *cmd) {
  if (strcmp(cmd, "BMP_RETRY") == 0) {
    bmp_ready = init_bmp();
    Serial.print("# bmp_manual_retry=");
    Serial.println(bmp_ready ? "ready" : "not_found");
    return;
  }
  if (strcmp(cmd, "BARO_ZERO") == 0 || strcmp(cmd, "ZERO") == 0) {
    zero_baro_to_current_pressure();
    return;
  }
  if (strcmp(cmd, "BARO_CALIBRATE") == 0 || strcmp(cmd, "CALIBRATE") == 0) {
    start_baro_calibration();
    return;
  }
  if (strcmp(cmd, "ICM_CHECK") == 0) {
    validate_icm42688();
    Serial.print("# icm42688_verify=");
    if (icm_ready) {
      Serial.print("ready addr=0x");
      print_hex_u8(icm_addr);
      Serial.print(" whoami=0x");
      print_hex_u8(icm_whoami);
      Serial.println();
    } else {
      Serial.println(icm_addr_seen ? "found_wrong_whoami" : "not_found");
    }
    return;
  }
  if (strcmp(cmd, "CSV_HEADER") == 0) {
    print_csv_header();
    print_imu_csv_header();
    return;
  }
  Serial.print("# unknown_command=");
  Serial.println(cmd);
}

static void handle_serial_commands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      command_line[command_idx] = '\0';
      if (command_idx > 0) {
        handle_command(command_line);
      }
      command_idx = 0;
      continue;
    }
    if (command_idx < sizeof(command_line) - 1) {
      command_line[command_idx++] = c;
    }
  }
}

void setup() {
  Serial.begin(kSerialBaud, SERIAL_8N1, kConsoleRx, kConsoleTx);
  delay(200);

  for (uint8_t i = 0; i < 5; i++) {
    Serial.print("# boot_marker=");
    Serial.print(i);
    Serial.print(" uart_rx=");
    Serial.print(kConsoleRx);
    Serial.print(" uart_tx=");
    Serial.println(kConsoleTx);
    Serial.flush();
    delay(50);
  }

  Serial.println("# ESP32-S3-DevKitC-1 BMP388 barometric altitude test");
  Serial.print("# serial_baud=");
  Serial.println(kSerialBaud);
  Serial.print("# i2c_sda=");
  Serial.print(kI2cSda);
  Serial.print(" i2c_scl=");
  Serial.println(kI2cScl);

  bmp_ready = init_bmp();
  Serial.print("# bmp_init=");
  if (bmp_ready) {
    Serial.print("ready addr=0x");
    print_hex_u8(active_addr);
    Serial.println();
  } else {
    Serial.print("failed reason=");
    Serial.println(bmp_not_found_reason());
  }

  print_csv_header();
  print_imu_csv_header();
  print_status(millis());
}

void loop() {
  handle_serial_commands();

  const uint32_t now_us = micros();
  const uint32_t now_ms = millis();

  if (bmp_ready && now_us - last_sample_us >= kSampleIntervalUs) {
    last_sample_us = now_us;
    if (read_baro_sample()) {
      print_csv_sample();
    }
  }

  if (icm_ready && now_us - last_imu_sample_us >= kImuSampleIntervalUs) {
    last_imu_sample_us = now_us;
    if (read_imu_sample()) {
      print_imu_csv_sample();
    }
  }

  retry_bmp_if_needed(now_ms);

  if (now_ms - last_status_ms >= kStatusIntervalMs) {
    last_status_ms = now_ms;
    print_status(now_ms);
  }

  delay(1);
}
