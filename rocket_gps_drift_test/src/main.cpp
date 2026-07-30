#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <math.h>

// Standalone GPS fixed-point drift test firmware.
// Keep the board stationary with clear sky view, then watch Serial Monitor.

static constexpr uint32_t kSerialBaud = 115200;

// Rocket GPS wiring.
static constexpr int kGpsTxPin = 35;   // ESP32 TX -> GPS RX
static constexpr int kGpsRxPin = 45;   // ESP32 RX <- GPS TX
static constexpr int kGpsPpsPin = 47;
struct GpsPinConfig {
  int rx;
  int tx;
  const char *name;
};

static constexpr GpsPinConfig kGpsPinConfigs[] = {
  {kGpsRxPin, kGpsTxPin, "normal_rx45_tx35"},
  {kGpsTxPin, kGpsRxPin, "swapped_rx35_tx45"}
};

static constexpr uint32_t kGpsBaudCandidates[] = {
  230400,
  115200,
  57600,
  38400,
  9600
};

static constexpr uint32_t kReportIntervalMs = 100;  // 10 Hz GUI/log output
static constexpr uint32_t kNoDataWarnMs = 3000;
static constexpr uint32_t kBaudProbeIntervalMs = 5000;
static constexpr uint16_t kReferenceFixCount = 100;
static constexpr double kEarthRadiusM = 6371008.8;

static HardwareSerial gps_serial(1);
static TinyGPSPlus gps;

static uint32_t last_report_ms = 0;
static uint32_t last_gps_byte_ms = 0;
static uint32_t last_fix_report_ms = 0;
static uint32_t last_baud_probe_ms = 0;
static uint32_t baud_probe_start_chars = 0;
static uint32_t baud_probe_start_passed_checksum = 0;
static uint32_t gps_byte_count = 0;
static uint32_t nmea_start_count = 0;
static uint32_t ubx_sync_count = 0;
static uint8_t raw_sample[16];
static uint8_t raw_sample_count = 0;
static uint8_t prev_gps_byte = 0;
static uint8_t gps_baud_idx = 0;
static uint8_t gps_pin_idx = 0;
static bool gps_link_locked = false;
static bool gps_config_sent = false;
static char command_line[48];
static uint8_t command_idx = 0;

static bool reference_ready = false;
static uint16_t reference_samples = 0;
static double reference_lat_sum = 0.0;
static double reference_lon_sum = 0.0;
static double reference_lat = 0.0;
static double reference_lon = 0.0;

static uint32_t drift_samples = 0;
static double drift_sum_m = 0.0;
static double drift_sum_sq_m2 = 0.0;
static double drift_max_m = 0.0;
static double drift_last_m = 0.0;
static double drift_last_east_m = 0.0;
static double drift_last_north_m = 0.0;

static void reset_drift_state() {
  reference_ready = false;
  reference_samples = 0;
  reference_lat_sum = 0.0;
  reference_lon_sum = 0.0;
  reference_lat = 0.0;
  reference_lon = 0.0;
  drift_samples = 0;
  drift_sum_m = 0.0;
  drift_sum_sq_m2 = 0.0;
  drift_max_m = 0.0;
  drift_last_m = 0.0;
  drift_last_east_m = 0.0;
  drift_last_north_m = 0.0;
  last_fix_report_ms = 0;
}

static void send_ubx(const uint8_t cls, const uint8_t id, const uint8_t *payload, const uint16_t len) {
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;

  auto checksum_byte = [&](uint8_t b) {
    ck_a = ck_a + b;
    ck_b = ck_b + ck_a;
  };

  gps_serial.write(0xB5);
  gps_serial.write(0x62);
  gps_serial.write(cls);
  gps_serial.write(id);
  gps_serial.write(static_cast<uint8_t>(len & 0xFF));
  gps_serial.write(static_cast<uint8_t>((len >> 8) & 0xFF));

  checksum_byte(cls);
  checksum_byte(id);
  checksum_byte(static_cast<uint8_t>(len & 0xFF));
  checksum_byte(static_cast<uint8_t>((len >> 8) & 0xFF));

  for (uint16_t i = 0; i < len; ++i) {
    gps_serial.write(payload[i]);
    checksum_byte(payload[i]);
  }

  gps_serial.write(ck_a);
  gps_serial.write(ck_b);
  gps_serial.flush();
}

static void send_ublox_cold_start() {
  // UBX-CFG-RST: navBbrMask=0xFFFF, resetMode=0x01 (controlled GNSS stop/start).
  // This clears receiver navigation data so each test starts from a cold search.
  const uint8_t payload[] = {0xFF, 0xFF, 0x01, 0x00};
  send_ubx(0x06, 0x04, payload, sizeof(payload));
  Serial.println("GPS_COLD_START_SENT");
}

static void send_ublox_high_rate_config() {
  // UBX-CFG-RATE: measRate=100ms, navRate=1, timeRef=UTC => 10 Hz navigation.
  const uint8_t rate_payload[] = {0x64, 0x00, 0x01, 0x00, 0x00, 0x00};
  send_ubx(0x06, 0x08, rate_payload, sizeof(rate_payload));
  Serial.println("GPS_RATE_10HZ_SENT");
}

static void set_ublox_nmea_msg(uint8_t msg_id, uint8_t uart1_rate) {
  // UBX-CFG-MSG, NMEA class 0xF0. Payload rates: I2C, UART1, UART2, USB, SPI, reserved.
  const uint8_t payload[] = {0xF0, msg_id, 0x00, uart1_rate, 0x00, 0x00, 0x00, 0x00};
  send_ubx(0x06, 0x01, payload, sizeof(payload));
  delay(10);
}

static void send_ublox_nmea_config() {
  set_ublox_nmea_msg(0x00, 1);  // GGA
  set_ublox_nmea_msg(0x02, 1);  // GSA
  set_ublox_nmea_msg(0x03, 1);  // GSV
  set_ublox_nmea_msg(0x04, 1);  // RMC
  set_ublox_nmea_msg(0x05, 1);  // VTG
  Serial.println("GPS_NMEA_UART1_SENT");
}

static uint32_t current_gps_baud() {
  return kGpsBaudCandidates[gps_baud_idx];
}

static const GpsPinConfig &current_gps_pins() {
  return kGpsPinConfigs[gps_pin_idx];
}

static void send_gps_runtime_config() {
  send_ublox_nmea_config();
  delay(50);
  send_ublox_high_rate_config();
  gps_config_sent = true;
}

static void begin_gps_serial(bool cold_start, bool send_config) {
  const uint32_t baud = current_gps_baud();
  const GpsPinConfig &pins = current_gps_pins();
  gps_serial.end();
  delay(20);
  gps_serial.begin(baud, SERIAL_8N1, pins.rx, pins.tx);
  delay(100);
  if (cold_start) {
    send_ublox_cold_start();
    delay(100);
  }
  gps_config_sent = false;
  if (send_config) {
    send_gps_runtime_config();
  }
  last_gps_byte_ms = 0;
  last_baud_probe_ms = millis();
  baud_probe_start_chars = gps.charsProcessed();
  baud_probe_start_passed_checksum = gps.passedChecksum();
  gps_byte_count = 0;
  nmea_start_count = 0;
  ubx_sync_count = 0;
  raw_sample_count = 0;
  prev_gps_byte = 0;
  Serial.print("GPS_UART_BAUD=");
  Serial.print(baud);
  Serial.print(" gps_rx_pin=");
  Serial.print(pins.rx);
  Serial.print(" gps_tx_pin=");
  Serial.print(pins.tx);
  Serial.print(" pin_mode=");
  Serial.print(pins.name);
  Serial.print(" cold_start=");
  Serial.print(cold_start ? "yes" : "no");
  Serial.print(" config_sent=");
  Serial.println(send_config ? "yes" : "no");
}

static void lock_gps_link() {
  gps_link_locked = true;
  Serial.print("GPS_LINK_LOCKED baud=");
  Serial.print(current_gps_baud());
  Serial.print(" gps_rx_pin=");
  Serial.print(current_gps_pins().rx);
  Serial.print(" gps_tx_pin=");
  Serial.print(current_gps_pins().tx);
  Serial.print(" pin_mode=");
  Serial.println(current_gps_pins().name);
}

static void try_next_gps_baud(uint32_t now_ms) {
  gps_baud_idx = (gps_baud_idx + 1) % (sizeof(kGpsBaudCandidates) / sizeof(kGpsBaudCandidates[0]));
  if (gps_baud_idx == 0) {
    gps_pin_idx = (gps_pin_idx + 1) % (sizeof(kGpsPinConfigs) / sizeof(kGpsPinConfigs[0]));
  }
  Serial.print("GPS_BAUD_PROBE timeout_ms=");
  Serial.print(now_ms - last_baud_probe_ms);
  Serial.print(" next=");
  Serial.println(current_gps_baud());
  begin_gps_serial(false, false);
}

static void handle_command_line(const char *line) {
  if (!line || line[0] == '\0') {
    return;
  }
  if (strncmp(line, "GPS_BAUD:", 9) == 0) {
    uint32_t baud = strtoul(line + 9, nullptr, 10);
    Serial.print("GPS_BAUD_CMD ignored auto_scan requested=");
    Serial.println(baud);
  } else if (strcmp(line, "GPS_RESET") == 0) {
    Serial.println("GPS_RESET_CMD accepted");
    gps_link_locked = false;
    reset_drift_state();
    begin_gps_serial(true, true);
  }
}

static void handle_host_commands() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (command_idx > 0) {
        command_line[command_idx] = '\0';
        handle_command_line(command_line);
        command_idx = 0;
      }
      continue;
    }
    if (command_idx < sizeof(command_line) - 1) {
      command_line[command_idx++] = c;
    }
  }
}

static double deg_to_rad(double deg) {
  return deg * M_PI / 180.0;
}

static double distance_m(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
  const double lat1 = deg_to_rad(lat1_deg);
  const double lat2 = deg_to_rad(lat2_deg);
  const double dlat = deg_to_rad(lat2_deg - lat1_deg);
  const double dlon = deg_to_rad(lon2_deg - lon1_deg);
  const double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
                   cos(lat1) * cos(lat2) * sin(dlon / 2.0) * sin(dlon / 2.0);
  const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return kEarthRadiusM * c;
}

static void local_offset_m(double ref_lat_deg,
                           double ref_lon_deg,
                           double lat_deg,
                           double lon_deg,
                           double &east_m,
                           double &north_m) {
  const double mean_lat_rad = deg_to_rad((ref_lat_deg + lat_deg) * 0.5);
  north_m = deg_to_rad(lat_deg - ref_lat_deg) * kEarthRadiusM;
  east_m = deg_to_rad(lon_deg - ref_lon_deg) * kEarthRadiusM * cos(mean_lat_rad);
}

static bool has_good_fix() {
  if (!gps.location.isValid() || !gps.location.isUpdated()) {
    return false;
  }
  if (gps.satellites.isValid() && gps.satellites.value() < 4) {
    return false;
  }
  if (gps.hdop.isValid() && gps.hdop.hdop() > 5.0) {
    return false;
  }
  return true;
}

static void update_reference_or_drift() {
  if (!has_good_fix()) {
    return;
  }

  const double lat = gps.location.lat();
  const double lon = gps.location.lng();
  last_fix_report_ms = millis();

  if (!reference_ready) {
    reference_lat_sum += lat;
    reference_lon_sum += lon;
    reference_samples++;

    if (reference_samples >= kReferenceFixCount) {
      reference_lat = reference_lat_sum / static_cast<double>(reference_samples);
      reference_lon = reference_lon_sum / static_cast<double>(reference_samples);
      reference_ready = true;
      Serial.println();
      Serial.println("Reference locked");
      Serial.print("ref_lat=");
      Serial.print(reference_lat, 8);
      Serial.print(" ref_lon=");
      Serial.println(reference_lon, 8);
    }
    return;
  }

  double east_m = 0.0;
  double north_m = 0.0;
  local_offset_m(reference_lat, reference_lon, lat, lon, east_m, north_m);

  const double drift_m = distance_m(reference_lat, reference_lon, lat, lon);
  drift_last_m = drift_m;
  drift_last_east_m = east_m;
  drift_last_north_m = north_m;
  drift_sum_m += drift_m;
  drift_sum_sq_m2 += drift_m * drift_m;
  drift_samples++;
  if (drift_m > drift_max_m) {
    drift_max_m = drift_m;
  }
}

static void print_hex_byte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

static void print_waiting_report(uint32_t now_ms) {
  const uint32_t age_ms = last_gps_byte_ms == 0 ? now_ms : now_ms - last_gps_byte_ms;
  const bool no_recent_bytes = last_gps_byte_ms == 0 || age_ms > kNoDataWarnMs;
  const bool has_valid_nmea = gps.passedChecksum() > baud_probe_start_passed_checksum;

  Serial.print("status=");
  if (no_recent_bytes) {
    Serial.print("NO_GPS_DATA");
  } else if (!has_valid_nmea && ubx_sync_count > 0 && nmea_start_count == 0) {
    Serial.print("GPS_UBX_ONLY");
  } else if (!has_valid_nmea) {
    Serial.print("GPS_BYTES_NO_NMEA");
  } else {
    Serial.print("WAIT_FIX");
  }
  Serial.print(" ref_samples=");
  Serial.print(reference_samples);
  Serial.print("/");
  Serial.print(kReferenceFixCount);
  Serial.print(" chars=");
  Serial.print(gps.charsProcessed());
  Serial.print(" sentences=");
  Serial.print(gps.sentencesWithFix());
  Serial.print(" failed_checksum=");
  Serial.print(gps.failedChecksum());
  Serial.print(" passed_checksum=");
  Serial.print(gps.passedChecksum());
  Serial.print(" gps_bytes=");
  Serial.print(gps_byte_count);
  Serial.print(" nmea_start=");
  Serial.print(nmea_start_count);
  Serial.print(" ubx_sync=");
  Serial.print(ubx_sync_count);
  Serial.print(" link_locked=");
  Serial.print(gps_link_locked ? "yes" : "no");
  Serial.print(" config_sent=");
  Serial.print(gps_config_sent ? "yes" : "no");
  Serial.print(" raw_hex=");
  if (raw_sample_count == 0) {
    Serial.print("none");
  } else {
    for (uint8_t i = 0; i < raw_sample_count; ++i) {
      if (i > 0) {
        Serial.print('_');
      }
      print_hex_byte(raw_sample[i]);
    }
  }
  Serial.print(" gps_baud=");
  Serial.print(current_gps_baud());
  Serial.print(" gps_rx_pin=");
  Serial.print(current_gps_pins().rx);
  Serial.print(" gps_tx_pin=");
  Serial.print(current_gps_pins().tx);
  Serial.print(" pin_mode=");
  Serial.print(current_gps_pins().name);
  Serial.print(" sat=");
  Serial.print(gps.satellites.isValid() ? static_cast<int>(gps.satellites.value()) : -1);
  Serial.print(" hdop=");
  if (gps.hdop.isValid()) {
    Serial.print(gps.hdop.hdop(), 2);
  } else {
    Serial.print("nan");
  }
  Serial.print(" alt=");
  if (gps.altitude.isValid()) {
    Serial.print(gps.altitude.meters(), 2);
    Serial.print("m");
  } else {
    Serial.print("nan");
  }
  Serial.print(" last_byte_age_ms=");
  Serial.println(age_ms);
}

static void print_drift_report() {
  const double avg_m = drift_samples == 0 ? 0.0 : drift_sum_m / static_cast<double>(drift_samples);
  const double rms_m = drift_samples == 0 ? 0.0 : sqrt(drift_sum_sq_m2 / static_cast<double>(drift_samples));
  const uint32_t fix_age_ms = last_fix_report_ms == 0 ? 0 : millis() - last_fix_report_ms;

  Serial.print("status=TRACK");
  Serial.print(" samples=");
  Serial.print(drift_samples);
  Serial.print(" drift=");
  Serial.print(drift_last_m, 3);
  Serial.print("m east=");
  Serial.print(drift_last_east_m, 3);
  Serial.print("m north=");
  Serial.print(drift_last_north_m, 3);
  Serial.print("m avg=");
  Serial.print(avg_m, 3);
  Serial.print("m rms=");
  Serial.print(rms_m, 3);
  Serial.print("m max=");
  Serial.print(drift_max_m, 3);
  Serial.print("m lat=");
  Serial.print(gps.location.lat(), 9);
  Serial.print(" lon=");
  Serial.print(gps.location.lng(), 9);
  Serial.print(" sat=");
  Serial.print(gps.satellites.isValid() ? static_cast<int>(gps.satellites.value()) : -1);
  Serial.print(" hdop=");
  if (gps.hdop.isValid()) {
    Serial.print(gps.hdop.hdop(), 2);
  } else {
    Serial.print("nan");
  }
  Serial.print(" alt=");
  if (gps.altitude.isValid()) {
    Serial.print(gps.altitude.meters(), 2);
    Serial.print("m");
  } else {
    Serial.print("nan");
  }
  Serial.print(" gps_baud=");
  Serial.print(current_gps_baud());
  Serial.print(" gps_rx_pin=");
  Serial.print(current_gps_pins().rx);
  Serial.print(" gps_tx_pin=");
  Serial.print(current_gps_pins().tx);
  Serial.print(" pin_mode=");
  Serial.print(current_gps_pins().name);
  Serial.print(" fix_age_ms=");
  Serial.println(fix_age_ms);
}

static void print_report(uint32_t now_ms) {
  if (!reference_ready) {
    print_waiting_report(now_ms);
  } else {
    print_drift_report();
  }
}

void setup() {
  pinMode(kGpsPpsPin, INPUT);

  Serial.begin(kSerialBaud);
  uint32_t start_ms = millis();
  while (!Serial && millis() - start_ms < 1500) {
    delay(10);
  }

  begin_gps_serial(true, true);

  Serial.println();
  Serial.println("GPS fixed-point drift test");
  Serial.print("GPS module=ublox_MAX-M10S baud_auto_scan_start=");
  Serial.print(current_gps_baud());
  Serial.print(" rx_pin=");
  Serial.print(current_gps_pins().rx);
  Serial.print(" tx_pin=");
  Serial.print(current_gps_pins().tx);
  Serial.print(" pin_mode=");
  Serial.println(current_gps_pins().name);
  Serial.print("Reference fixes=");
  Serial.println(kReferenceFixCount);
}

void loop() {
  handle_host_commands();

  while (gps_serial.available()) {
    const uint8_t b = static_cast<uint8_t>(gps_serial.read());
    const char c = static_cast<char>(b);
    last_gps_byte_ms = millis();
    gps_byte_count++;
    if (raw_sample_count < sizeof(raw_sample)) {
      raw_sample[raw_sample_count++] = b;
    }
    if (c == '$') {
      nmea_start_count++;
    }
    if (prev_gps_byte == 0xB5 && b == 0x62) {
      ubx_sync_count++;
    }
    prev_gps_byte = b;
    gps.encode(c);
  }

  update_reference_or_drift();

  const uint32_t now_ms = millis();
  if (!gps_link_locked && gps.passedChecksum() > baud_probe_start_passed_checksum) {
    lock_gps_link();
  }
  if (gps_link_locked && !gps_config_sent) {
    Serial.println("GPS_VALID_NMEA_DETECTED_SEND_CONFIG");
    send_gps_runtime_config();
  }

  if (!gps_link_locked && !reference_ready && now_ms - last_baud_probe_ms > kBaudProbeIntervalMs) {
    const bool no_bytes = (last_gps_byte_ms == 0 || now_ms - last_gps_byte_ms > kNoDataWarnMs);
    const bool bytes_but_no_valid_nmea =
        (gps.charsProcessed() > baud_probe_start_chars + 100) &&
        (gps.passedChecksum() == baud_probe_start_passed_checksum);
    if (no_bytes || bytes_but_no_valid_nmea) {
      try_next_gps_baud(now_ms);
    }
  }

  if (now_ms - last_report_ms >= kReportIntervalMs) {
    last_report_ms = now_ms;
    print_report(now_ms);
  }

  delay(1);
}
