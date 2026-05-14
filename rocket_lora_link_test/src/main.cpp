#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LoRa.h>
#include <SPI.h>
#include <TinyGPSPlus.h>

// Rocket-side standalone LoRa link stability transmitter.
// Sends one fixed-format packet at a fixed rate. This does not modify flight firmware.

static constexpr uint32_t kSerialBaud = 115200;
static constexpr uint32_t kTxRateHz = 10;
static constexpr uint32_t kTxIntervalMs = 1000 / kTxRateHz;

// Competition rocket LoRa pins.
static constexpr int kLoraSck = 42;
static constexpr int kLoraMiso = 41;
static constexpr int kLoraMosi = 40;
static constexpr int kLoraNss = 39;
static constexpr int kLoraRst = 38;
static constexpr int kLoraDio0 = 37;
static constexpr int kLoraDio1 = 36;

// Rocket GPS wiring, same as rocket_gps_drift_test.
static constexpr int kGpsTxPin = 35;  // ESP32 TX -> GPS RX
static constexpr int kGpsRxPin = 45;  // ESP32 RX <- GPS TX
static constexpr uint32_t kGpsBaud = 38400;

static constexpr int kRgbPin = 48;
static constexpr int kRgbCount = 1;

// LoRa parameters from the agreed test table.
static constexpr long kLoraFreq = 433000000;
static constexpr long kLoraBw = 125E3;
static constexpr int kLoraSf = 9;
static constexpr int kLoraCr = 5;  // 4/5
static constexpr int kLoraPreambleLen = 8;
static constexpr byte kLoraSyncWord = 0x12;
static constexpr int kLoraTxPower = 18;

static constexpr uint8_t kDefaultPacketLen = 16;
static constexpr uint8_t kGpsPacketLen = 40;
static constexpr uint8_t kPacketLen = kGpsPacketLen;
static constexpr uint8_t kMagic0 = 'L';
static constexpr uint8_t kMagic1 = 'Q';
static constexpr uint8_t kProtocolVersion = 4;
static constexpr uint8_t kPacketTypeGps = 0x47;  // 'G'
static constexpr uint32_t kLoraRetryMs = 3000;
static constexpr uint32_t kStatusIntervalMs = 1000;
static constexpr uint32_t kEstimatedAirtimeMs = 247;
static constexpr float kEstimatedMaxPayloadHz = 1000.0f / static_cast<float>(kEstimatedAirtimeMs);

static SPIClass lora_spi(FSPI);
static HardwareSerial gps_serial(1);
static TinyGPSPlus gps;
static Adafruit_NeoPixel rgb(kRgbCount, kRgbPin, NEO_GRB + NEO_KHZ800);

static bool lora_ready = false;
static uint32_t seq = 0;
static uint32_t sent_ok = 0;
static uint32_t sent_fail = 0;
static uint32_t tx_overrun = 0;
static uint32_t next_tx_ms = 0;
static uint32_t last_tx_duration_ms = 0;
static uint32_t max_tx_duration_ms = 0;
static uint32_t last_status_ms = 0;
static uint32_t last_lora_retry_ms = 0;
static uint32_t last_gps_byte_ms = 0;
static uint32_t gps_bytes = 0;
static uint32_t gps_sent_valid = 0;
static uint32_t gps_sent_invalid = 0;
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
  } else {
    set_led(0, 0, 24);
  }
}

static void write_u16_le(uint8_t *buf, size_t idx, uint16_t value) {
  buf[idx] = static_cast<uint8_t>(value & 0xFF);
  buf[idx + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

static void write_u32_le(uint8_t *buf, size_t idx, uint32_t value) {
  buf[idx] = static_cast<uint8_t>(value & 0xFF);
  buf[idx + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  buf[idx + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  buf[idx + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

static void write_i32_le(uint8_t *buf, size_t idx, int32_t value) {
  write_u32_le(buf, idx, static_cast<uint32_t>(value));
}

static uint8_t xor_crc(const uint8_t *buf, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= buf[i];
  }
  return crc;
}

static void build_packet(uint8_t *packet, uint32_t packet_seq, uint32_t now_ms) {
  packet[0] = kMagic0;
  packet[1] = kMagic1;
  packet[2] = kProtocolVersion;
  packet[3] = kPacketLen;
  write_u32_le(packet, 4, packet_seq);
  write_u32_le(packet, 8, now_ms);
  write_u16_le(packet, 12, static_cast<uint16_t>(kTxIntervalMs));
  packet[14] = kPacketTypeGps;
  uint8_t flags = 0;
  const bool loc_valid = gps.location.isValid();
  const bool alt_valid = gps.altitude.isValid();
  const bool sat_valid = gps.satellites.isValid();
  const bool hdop_valid = gps.hdop.isValid();
  if (loc_valid) {
    flags |= 0x01;
  }
  if (alt_valid) {
    flags |= 0x02;
  }
  if (sat_valid) {
    flags |= 0x04;
  }
  if (hdop_valid) {
    flags |= 0x08;
  }
  packet[15] = flags;

  const int32_t lat_e7 = loc_valid ? static_cast<int32_t>(gps.location.lat() * 10000000.0) : 0;
  const int32_t lon_e7 = loc_valid ? static_cast<int32_t>(gps.location.lng() * 10000000.0) : 0;
  const int32_t alt_cm = alt_valid ? static_cast<int32_t>(gps.altitude.meters() * 100.0) : 0;
  write_i32_le(packet, 16, lat_e7);
  write_i32_le(packet, 20, lon_e7);
  write_i32_le(packet, 24, alt_cm);
  packet[28] = sat_valid ? static_cast<uint8_t>(min<uint32_t>(gps.satellites.value(), 255)) : 0;
  const uint16_t hdop_x100 =
      hdop_valid ? static_cast<uint16_t>(min<uint32_t>(gps.hdop.value(), 65535)) : 0;
  write_u16_le(packet, 29, hdop_x100);
  const uint32_t fix_age = loc_valid ? gps.location.age() : 0xFFFFUL;
  write_u16_le(packet, 31, static_cast<uint16_t>(min<uint32_t>(fix_age, 65535)));
  packet[33] = 0;  // GPS baud code: 0 = fixed 38400.
  packet[34] = loc_valid ? 2 : (last_gps_byte_ms == 0 ? 0 : 1);
  packet[35] = 0xA5;
  packet[36] = static_cast<uint8_t>(gps.passedChecksum() & 0xFF);
  packet[37] = static_cast<uint8_t>(gps.failedChecksum() & 0xFF);
  packet[38] = static_cast<uint8_t>(gps_bytes & 0xFF);
  packet[kPacketLen - 1] = xor_crc(packet, kPacketLen - 1);
}

static void handle_gps_rx() {
  while (gps_serial.available()) {
    const char c = static_cast<char>(gps_serial.read());
    gps.encode(c);
    gps_bytes++;
    last_gps_byte_ms = millis();
  }
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
  LoRa.setTxPower(kLoraTxPower, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSpreadingFactor(kLoraSf);
  LoRa.setSignalBandwidth(kLoraBw);
  LoRa.setCodingRate4(kLoraCr);
  LoRa.setPreambleLength(kLoraPreambleLen);
  LoRa.enableCrc();
  LoRa.disableInvertIQ();
  LoRa.idle();
  return true;
}

static void maintain_lora(uint32_t now_ms) {
  if (lora_ready || now_ms - last_lora_retry_ms < kLoraRetryMs) {
    return;
  }
  last_lora_retry_ms = now_ms;
  lora_ready = init_lora();
  Serial.println(lora_ready ? "# lora_tx_ready" : "# lora_tx_init_failed");
}

static bool send_packet(uint32_t now_ms) {
  if (!lora_ready) {
    return false;
  }

  uint8_t packet[kPacketLen];
  build_packet(packet, seq, now_ms);

  if (LoRa.beginPacket() == 0) {
    return false;
  }
  const size_t written = LoRa.write(packet, sizeof(packet));
  const int end_ok = LoRa.endPacket();
  return written == sizeof(packet) && end_ok == 1;
}

static void print_status(uint32_t now_ms) {
  Serial.print("tx_ms=");
  Serial.print(now_ms);
  Serial.print(",seq=");
  Serial.print(seq);
  Serial.print(",sent_ok=");
  Serial.print(sent_ok);
  Serial.print(",sent_fail=");
  Serial.print(sent_fail);
  Serial.print(",overrun=");
  Serial.print(tx_overrun);
  Serial.print(",interval_ms=");
  Serial.print(kTxIntervalMs);
  Serial.print(",target_hz=");
  Serial.print(kTxRateHz);
  Serial.print(",estimated_airtime_ms=");
  Serial.print(kEstimatedAirtimeMs);
  Serial.print(",estimated_max_hz=");
  Serial.print(kEstimatedMaxPayloadHz, 2);
  Serial.print(",last_tx_duration_ms=");
  Serial.print(last_tx_duration_ms);
  Serial.print(",max_tx_duration_ms=");
  Serial.print(max_tx_duration_ms);
  Serial.print(",gps_valid=");
  Serial.print(gps.location.isValid() ? 1 : 0);
  Serial.print(",gps_sent_valid=");
  Serial.print(gps_sent_valid);
  Serial.print(",gps_sent_invalid=");
  Serial.print(gps_sent_invalid);
  Serial.print(",sat=");
  Serial.print(gps.satellites.isValid() ? static_cast<int>(gps.satellites.value()) : -1);
  Serial.print(",hdop=");
  Serial.print(gps.hdop.isValid() ? gps.hdop.hdop() : -1.0, 2);
  Serial.print(",lora=");
  Serial.println(lora_ready ? "OK" : "FAIL");
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

  Serial.println("# LoRa link stability TX");
  Serial.println("# params=freq433000000,bw125k,sf9,cr4/5,preamble8,sync0x12,crc_on,iq_off,tx18dbm");
  Serial.println("# gps=ublox_MAX-M10S,baud38400,rx45,tx35");
  Serial.println("# tx_rate_hz=10 interval_ms=100");
  Serial.println("# estimated_airtime_ms=247 estimated_max_payload_hz=4.05 note=gps_payload_sf9_bw125_cr45");
  Serial.println("# packet_protocol=v4,len40,type_gps,seq32,tx_ms32,gps_fixed_point,crc8");
  Serial.println("# pins=sck=42,miso=41,mosi=40,nss=39,dio0=37,dio1=36,rst=38,rgb=48");

  gps_serial.begin(kGpsBaud, SERIAL_8N1, kGpsRxPin, kGpsTxPin);

  lora_ready = init_lora();
  last_lora_retry_ms = millis();
  next_tx_ms = millis() + kTxIntervalMs;
  Serial.println(lora_ready ? "# lora_tx_ready" : "# lora_tx_init_failed");
  set_led(lora_ready ? 0 : 80, lora_ready ? 0 : 0, lora_ready ? 32 : 0);
}

void loop() {
  const uint32_t now_ms = millis();
  handle_gps_rx();
  maintain_lora(now_ms);

  if (static_cast<int32_t>(now_ms - next_tx_ms) >= 0) {
    if (static_cast<uint32_t>(now_ms - next_tx_ms) >= kTxIntervalMs) {
      tx_overrun++;
      next_tx_ms = now_ms;
    }

    const uint32_t tx_start_ms = millis();
    if (send_packet(tx_start_ms)) {
      sent_ok++;
      if (gps.location.isValid()) {
        gps_sent_valid++;
      } else {
        gps_sent_invalid++;
      }
      set_led(0, 80, 0, 12);
    } else {
      sent_fail++;
      set_led(80, 0, 0, 120);
      if (sent_fail % 10 == 0) {
        lora_ready = false;
      }
    }
    last_tx_duration_ms = millis() - tx_start_ms;
    if (last_tx_duration_ms > max_tx_duration_ms) {
      max_tx_duration_ms = last_tx_duration_ms;
    }
    seq++;
    next_tx_ms += kTxIntervalMs;
  }

  if (now_ms - last_status_ms >= kStatusIntervalMs) {
    last_status_ms = now_ms;
    print_status(now_ms);
  }

  update_idle_led(now_ms);
  delay(1);
}
