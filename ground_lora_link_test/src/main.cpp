#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LoRa.h>
#include <SPI.h>

// Ground-side standalone LoRa link stability receiver.
// Receives fixed packets from rocket_lora_link_test and streams CSV reports to PC GUI.

static constexpr uint32_t kSerialBaud = 115200;

// Competition ground LoRa pins.
static constexpr int kLoraSck = 9;
static constexpr int kLoraMiso = 10;
static constexpr int kLoraMosi = 11;
static constexpr int kLoraNss = 12;
static constexpr int kLoraDio0 = 6;
static constexpr int kLoraDio1 = 5;
static constexpr int kLoraRst = 7;

static constexpr int kRgbPin = 48;
static constexpr int kRgbCount = 1;

// LoRa parameters from the agreed test table.
static constexpr long kLoraFreq = 433000000;
static constexpr long kLoraBw = 125E3;
static constexpr int kLoraSf = 9;
static constexpr int kLoraCr = 5;  // 4/5
static constexpr int kLoraPreambleLen = 8;
static constexpr byte kLoraSyncWord = 0x12;

static constexpr uint8_t kDefaultPacketLen = 16;
static constexpr uint8_t kGpsPacketLen = 40;
static constexpr uint8_t kMagic0 = 'L';
static constexpr uint8_t kMagic1 = 'Q';
static constexpr uint8_t kDefaultProtocolVersion = 3;
static constexpr uint8_t kGpsProtocolVersion = 4;
static constexpr uint8_t kPacketTypeDefault = 0xA5;
static constexpr uint8_t kPacketTypeGps = 0x47;
static constexpr uint32_t kLoraRetryMs = 3000;
static constexpr uint32_t kReportIntervalMs = 250;
static constexpr uint32_t kLinkTimeoutMs = 3000;
static constexpr uint32_t kEstimatedAirtimeMs = 247;
static constexpr float kEstimatedMaxPayloadHz = 1000.0f / static_cast<float>(kEstimatedAirtimeMs);

static SPIClass lora_spi(FSPI);
static Adafruit_NeoPixel rgb(kRgbCount, kRgbPin, NEO_GRB + NEO_KHZ800);

static bool lora_ready = false;
static bool have_seq = false;
static uint32_t last_lora_retry_ms = 0;
static uint32_t last_report_ms = 0;
static uint32_t last_rx_ms = 0;
static uint32_t first_rx_ms = 0;
static uint32_t last_seq = 0;
static uint32_t rx_valid = 0;
static uint32_t rx_bad = 0;
static uint32_t missing = 0;
static uint32_t duplicate = 0;
static uint32_t out_of_order = 0;
static uint32_t last_tx_ms = 0;
static uint32_t last_interval_ms = 0;
static uint32_t last_packet_gap_ms = 0;
static uint32_t min_packet_gap_ms = 0xFFFFFFFFUL;
static uint32_t max_packet_gap_ms = 0;
static char last_packet_type[8] = "none";
static char gps_status[16] = "---";
static bool gps_valid = false;
static double gps_lat = 0.0;
static double gps_lon = 0.0;
static double gps_alt_m = 0.0;
static uint8_t gps_sat = 0;
static float gps_hdop = 0.0f;
static uint16_t gps_fix_age_ms = 0;
static uint8_t gps_baud_code = 0;
static int last_rssi = 0;
static int min_rssi = 127;
static int max_rssi = -127;
static float avg_rssi = 0.0f;
static float last_snr = 0.0f;
static float min_snr = 99.0f;
static float max_snr = -99.0f;
static float avg_snr = 0.0f;
static char command_line[32];
static uint8_t command_idx = 0;
static uint32_t led_until_ms = 0;
static uint32_t led_color = 0;

static void set_led(uint8_t r, uint8_t g, uint8_t b, uint32_t hold_ms = 0) {
  led_color = rgb.Color(r, g, b);
  rgb.setPixelColor(0, led_color);
  rgb.show();
  led_until_ms = hold_ms == 0 ? 0 : millis() + hold_ms;
}

static void update_idle_led(uint32_t now_ms) {
  if (led_until_ms != 0 && static_cast<int32_t>(now_ms - led_until_ms) < 0) {
    return;
  }
  led_until_ms = 0;
  if (!lora_ready) {
    set_led(80, 0, 0);
    return;
  }
  if (last_rx_ms == 0 || now_ms - last_rx_ms > kLinkTimeoutMs) {
    set_led(40, 24, 0);
    return;
  }
  set_led(0, 0, 32);
}

static uint16_t read_u16_le(const uint8_t *buf, size_t idx) {
  return static_cast<uint16_t>(buf[idx]) |
         (static_cast<uint16_t>(buf[idx + 1]) << 8);
}

static uint32_t read_u32_le(const uint8_t *buf, size_t idx) {
  return static_cast<uint32_t>(buf[idx]) |
         (static_cast<uint32_t>(buf[idx + 1]) << 8) |
         (static_cast<uint32_t>(buf[idx + 2]) << 16) |
         (static_cast<uint32_t>(buf[idx + 3]) << 24);
}

static int32_t read_i32_le(const uint8_t *buf, size_t idx) {
  return static_cast<int32_t>(read_u32_le(buf, idx));
}

static uint8_t xor_crc(const uint8_t *buf, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= buf[i];
  }
  return crc;
}

static bool is_valid_default_packet(const uint8_t *buf, size_t len) {
  if (!buf || len != kDefaultPacketLen) {
    return false;
  }
  if (buf[0] != kMagic0 || buf[1] != kMagic1) {
    return false;
  }
  if (buf[2] != kDefaultProtocolVersion || buf[3] != kDefaultPacketLen) {
    return false;
  }
  if (buf[14] != kPacketTypeDefault) {
    return false;
  }
  return xor_crc(buf, kDefaultPacketLen - 1) == buf[kDefaultPacketLen - 1];
}

static bool is_valid_gps_packet(const uint8_t *buf, size_t len) {
  if (!buf || len != kGpsPacketLen) {
    return false;
  }
  if (buf[0] != kMagic0 || buf[1] != kMagic1) {
    return false;
  }
  if (buf[2] != kGpsProtocolVersion || buf[3] != kGpsPacketLen) {
    return false;
  }
  if (buf[14] != kPacketTypeGps || buf[35] != 0xA5) {
    return false;
  }
  return xor_crc(buf, kGpsPacketLen - 1) == buf[kGpsPacketLen - 1];
}

static void reset_stats() {
  have_seq = false;
  first_rx_ms = 0;
  last_rx_ms = 0;
  last_seq = 0;
  rx_valid = 0;
  rx_bad = 0;
  missing = 0;
  duplicate = 0;
  out_of_order = 0;
  last_tx_ms = 0;
  last_interval_ms = 0;
  last_packet_gap_ms = 0;
  min_packet_gap_ms = 0xFFFFFFFFUL;
  max_packet_gap_ms = 0;
  strncpy(last_packet_type, "none", sizeof(last_packet_type));
  strncpy(gps_status, "---", sizeof(gps_status));
  gps_valid = false;
  gps_lat = 0.0;
  gps_lon = 0.0;
  gps_alt_m = 0.0;
  gps_sat = 0;
  gps_hdop = 0.0f;
  gps_fix_age_ms = 0;
  gps_baud_code = 0;
  last_rssi = 0;
  min_rssi = 127;
  max_rssi = -127;
  avg_rssi = 0.0f;
  last_snr = 0.0f;
  min_snr = 99.0f;
  max_snr = -99.0f;
  avg_snr = 0.0f;
}

static bool init_lora() {
  pinMode(kLoraNss, OUTPUT);
  digitalWrite(kLoraNss, HIGH);
  pinMode(kLoraRst, OUTPUT);
  digitalWrite(kLoraRst, HIGH);
  pinMode(kLoraDio0, INPUT);
  pinMode(kLoraDio1, INPUT);

  lora_spi.begin(kLoraSck, kLoraMiso, kLoraMosi, kLoraNss);
  LoRa.setSPI(lora_spi);
  LoRa.setPins(kLoraNss, kLoraRst, kLoraDio0);

  digitalWrite(kLoraRst, LOW);
  delay(10);
  digitalWrite(kLoraRst, HIGH);
  delay(20);

  if (!LoRa.begin(kLoraFreq)) {
    return false;
  }

  LoRa.setSyncWord(kLoraSyncWord);
  LoRa.setSpreadingFactor(kLoraSf);
  LoRa.setSignalBandwidth(kLoraBw);
  LoRa.setCodingRate4(kLoraCr);
  LoRa.setPreambleLength(kLoraPreambleLen);
  LoRa.enableCrc();
  LoRa.disableInvertIQ();
  LoRa.receive();
  return true;
}

static void maintain_lora(uint32_t now_ms) {
  if (lora_ready || now_ms - last_lora_retry_ms < kLoraRetryMs) {
    return;
  }
  last_lora_retry_ms = now_ms;
  lora_ready = init_lora();
  Serial.println(lora_ready ? "# lora_rx_ready" : "# lora_rx_init_failed");
}

static void update_seq_stats(uint32_t seq) {
  if (!have_seq) {
    have_seq = true;
    last_seq = seq;
    return;
  }

  const uint32_t delta = seq - last_seq;
  if (delta == 0) {
    duplicate++;
    return;
  }
  if (delta > 0x80000000UL) {
    out_of_order++;
    return;
  }

  if (delta > 1) {
    missing += delta - 1;
  }
  last_seq = seq;
}

static void update_signal_stats(int rssi, float snr) {
  if (rx_valid == 1) {
    avg_rssi = rssi;
    avg_snr = snr;
  } else {
    avg_rssi += (static_cast<float>(rssi) - avg_rssi) / static_cast<float>(rx_valid);
    avg_snr += (snr - avg_snr) / static_cast<float>(rx_valid);
  }

  if (rssi < min_rssi) {
    min_rssi = rssi;
  }
  if (rssi > max_rssi) {
    max_rssi = rssi;
  }
  if (snr < min_snr) {
    min_snr = snr;
  }
  if (snr > max_snr) {
    max_snr = snr;
  }
}

static void handle_lora_rx() {
  const int packet_size = LoRa.parsePacket();
  if (packet_size <= 0) {
    return;
  }

  uint8_t packet[64];
  size_t len = 0;
  while (LoRa.available() && len < sizeof(packet)) {
    packet[len++] = static_cast<uint8_t>(LoRa.read());
  }
  while (LoRa.available()) {
    LoRa.read();
  }

  last_rssi = LoRa.packetRssi();
  last_snr = LoRa.packetSnr();

  const bool is_default_packet = is_valid_default_packet(packet, len);
  const bool is_gps_packet = is_valid_gps_packet(packet, len);
  if (!is_default_packet && !is_gps_packet) {
    rx_bad++;
    set_led(80, 0, 0, 80);
    LoRa.receive();
    return;
  }

  const uint32_t seq = read_u32_le(packet, 4);
  last_tx_ms = read_u32_le(packet, 8);
  last_interval_ms = read_u16_le(packet, 12);
  if (is_gps_packet) {
    strncpy(last_packet_type, "gps", sizeof(last_packet_type));
    const uint8_t flags = packet[15];
    gps_valid = (flags & 0x01) != 0;
    gps_lat = static_cast<double>(read_i32_le(packet, 16)) / 10000000.0;
    gps_lon = static_cast<double>(read_i32_le(packet, 20)) / 10000000.0;
    gps_alt_m = static_cast<double>(read_i32_le(packet, 24)) / 100.0;
    gps_sat = packet[28];
    gps_hdop = static_cast<float>(read_u16_le(packet, 29)) / 100.0f;
    gps_fix_age_ms = read_u16_le(packet, 31);
    gps_baud_code = packet[33];
    switch (packet[34]) {
      case 2:
        strncpy(gps_status, "FIX", sizeof(gps_status));
        break;
      case 1:
        strncpy(gps_status, "WAIT_FIX", sizeof(gps_status));
        break;
      default:
        strncpy(gps_status, "NO_DATA", sizeof(gps_status));
        break;
    }
  } else {
    strncpy(last_packet_type, "default", sizeof(last_packet_type));
  }

  const uint32_t now_ms = millis();
  if (rx_valid == 0) {
    first_rx_ms = now_ms;
  } else {
    last_packet_gap_ms = now_ms - last_rx_ms;
    if (last_packet_gap_ms < min_packet_gap_ms) {
      min_packet_gap_ms = last_packet_gap_ms;
    }
    if (last_packet_gap_ms > max_packet_gap_ms) {
      max_packet_gap_ms = last_packet_gap_ms;
    }
  }

  rx_valid++;
  last_rx_ms = now_ms;
  update_seq_stats(seq);
  update_signal_stats(last_rssi, last_snr);
  set_led(0, 80, 0, 12);

  LoRa.receive();
}

static void print_header() {
  Serial.println("report_ms,link,rx_valid,expected,missing,loss_pct,rx_rate_hz,bad,dup,old,last_seq,rssi,snr,avg_rssi,avg_snr,min_rssi,min_snr,max_rssi,max_snr,last_gap_ms,min_gap_ms,max_gap_ms,last_rx_age_ms,lora_ready,packet_type,gps_status,gps_valid,lat,lon,alt_m,sat,hdop,gps_fix_age_ms,gps_baud_code");
}

static void print_report(uint32_t now_ms) {
  const uint32_t expected = rx_valid + missing;
  const float loss_pct = expected == 0 ? 0.0f : 100.0f * static_cast<float>(missing) / static_cast<float>(expected);
  const float elapsed_s = first_rx_ms == 0 ? 0.0f : static_cast<float>(now_ms - first_rx_ms) / 1000.0f;
  const float rx_hz = elapsed_s <= 0.0f ? 0.0f : static_cast<float>(rx_valid) / elapsed_s;
  const uint32_t age_ms = last_rx_ms == 0 ? 0xFFFFFFFFUL : now_ms - last_rx_ms;
  const bool link_ok = lora_ready && last_rx_ms != 0 && age_ms <= kLinkTimeoutMs;

  Serial.print(now_ms);
  Serial.print(',');
  Serial.print(link_ok ? "OK" : (lora_ready ? "WAIT" : "LORA_FAIL"));
  Serial.print(',');
  Serial.print(rx_valid);
  Serial.print(',');
  Serial.print(expected);
  Serial.print(',');
  Serial.print(missing);
  Serial.print(',');
  Serial.print(loss_pct, 3);
  Serial.print(',');
  Serial.print(rx_hz, 3);
  Serial.print(',');
  Serial.print(rx_bad);
  Serial.print(',');
  Serial.print(duplicate);
  Serial.print(',');
  Serial.print(out_of_order);
  Serial.print(',');
  Serial.print(have_seq ? last_seq : 0);
  Serial.print(',');
  Serial.print(last_rssi);
  Serial.print(',');
  Serial.print(last_snr, 2);
  Serial.print(',');
  Serial.print(avg_rssi, 2);
  Serial.print(',');
  Serial.print(avg_snr, 2);
  Serial.print(',');
  Serial.print(min_rssi == 127 ? 0 : min_rssi);
  Serial.print(',');
  Serial.print(min_snr == 99.0f ? 0.0f : min_snr, 2);
  Serial.print(',');
  Serial.print(max_rssi == -127 ? 0 : max_rssi);
  Serial.print(',');
  Serial.print(max_snr == -99.0f ? 0.0f : max_snr, 2);
  Serial.print(',');
  Serial.print(last_packet_gap_ms);
  Serial.print(',');
  Serial.print(min_packet_gap_ms == 0xFFFFFFFFUL ? 0 : min_packet_gap_ms);
  Serial.print(',');
  Serial.print(max_packet_gap_ms);
  Serial.print(',');
  Serial.print(age_ms);
  Serial.print(',');
  Serial.print(lora_ready ? 1 : 0);
  Serial.print(',');
  Serial.print(last_packet_type);
  Serial.print(',');
  Serial.print(gps_status);
  Serial.print(',');
  Serial.print(gps_valid ? 1 : 0);
  Serial.print(',');
  Serial.print(gps_valid ? gps_lat : 0.0, 7);
  Serial.print(',');
  Serial.print(gps_valid ? gps_lon : 0.0, 7);
  Serial.print(',');
  Serial.print(gps_alt_m, 2);
  Serial.print(',');
  Serial.print(gps_sat);
  Serial.print(',');
  Serial.print(gps_hdop, 2);
  Serial.print(',');
  Serial.print(gps_fix_age_ms);
  Serial.print(',');
  Serial.println(gps_baud_code);
}

static void handle_command(const char *cmd) {
  if (strcmp(cmd, "RESET_STATS") == 0 || strcmp(cmd, "RESET") == 0) {
    reset_stats();
    Serial.println("# stats_reset");
    print_header();
    return;
  }
  if (strcmp(cmd, "CSV_HEADER") == 0) {
    print_header();
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
  Serial.begin(kSerialBaud);
  rgb.begin();
  rgb.setBrightness(32);
  set_led(24, 24, 0);

  uint32_t start_ms = millis();
  while (!Serial && millis() - start_ms < 1500) {
    delay(10);
  }

  reset_stats();
  Serial.println("# LoRa link stability RX");
  Serial.println("# params=freq433000000,bw125k,sf9,cr4/5,preamble8,sync0x12,crc_on,iq_off");
  Serial.print("# estimated_airtime_ms=");
  Serial.print(kEstimatedAirtimeMs);
  Serial.print(" estimated_max_payload_hz=");
  Serial.print(kEstimatedMaxPayloadHz, 2);
  Serial.println(" note=gps_payload_sf9_bw125_cr45");
  Serial.println("# packet_protocol=v3_default_len16_or_v4_gps_len40");
  Serial.println("# pins=sck=9,miso=10,mosi=11,nss=12,dio0=6,dio1=5,rst=7,rgb=48");
  print_header();

  lora_ready = init_lora();
  last_lora_retry_ms = millis();
  Serial.println(lora_ready ? "# lora_rx_ready" : "# lora_rx_init_failed");
}

void loop() {
  const uint32_t now_ms = millis();
  handle_serial_commands();
  maintain_lora(now_ms);

  if (lora_ready) {
    handle_lora_rx();
  }

  if (now_ms - last_report_ms >= kReportIntervalMs) {
    last_report_ms = now_ms;
    print_report(now_ms);
  }

  update_idle_led(now_ms);
  delay(1);
}
