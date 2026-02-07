#pragma once

#include <stdint.h>

// Telemetry error bitmask (error_code).
// Multiple errors can be combined with bitwise OR.
enum ErrorBits : uint8_t {
  kErrNone = 0,
  kErrLoRaInit = 1 << 0,   // LoRa radio init failed (lora_ready == false)
  kErrSdInit = 1 << 1,     // SD init or log open failed (sd_ready == false)
  kErrRtcInit = 1 << 2,    // RTC init failed (rtc_ready == false)
  kErrBaroInit = 1 << 3,   // BMP390 init failed (status_bmp == false)
  kErrImuInit = 1 << 4,    // ICM42688 init failed (status_imu == false)
  kErrAdxlInit = 1 << 5,   // ADXL375 init failed (status_adxl == false)
  kErrGpsNoFix = 1 << 6,   // GPS no fix/timeout during flight/preflight
  kErrBattery = 1 << 7,    // Battery low or ADC invalid
};
