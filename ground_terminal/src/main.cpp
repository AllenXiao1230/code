#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>

// LoRa packet-loss test receiver.
// Open PlatformIO Serial Monitor at 115200 baud to see loss statistics.

static constexpr uint32_t kSerialBaud = 115200;

// Ground LoRa module pins.
static constexpr int kLoraSck = 9;
static constexpr int kLoraMiso = 10;
static constexpr int kLoraMosi = 11;
static constexpr int kLoraNss = 12;
static constexpr int kLoraDio0 = 6;
static constexpr int kLoraDio1 = 5;
static constexpr int kLoraRst = 7;

// LoRa radio parameters.
static constexpr long kLoraFreq = 433000000;
static constexpr byte kLoraSyncWord = 0x12;
static constexpr long kLoraBw = 125E3;
static constexpr int kLoraSf = 9;
static constexpr int kLoraCr = 6;  // 4/6
static constexpr int kLoraPreambleLen = 8;

static constexpr uint8_t kPacketLen = 16;
static constexpr uint8_t kMagic0 = 'R';
static constexpr uint8_t kMagic1 = 'L';
static constexpr uint8_t kProtocolVersion = 1;
static constexpr uint32_t kLoraRetryMs = 3000;
static constexpr uint32_t kReportIntervalMs = 1000;
static constexpr uint32_t kLinkTimeoutMs = 3000;

static SPIClass lora_spi(FSPI);
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
static int last_rssi = 0;
static float last_snr = 0.0f;

static uint8_t read_u8(const uint8_t *buf, size_t idx) {
  return buf[idx];
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

static uint8_t xor_crc(const uint8_t *buf, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= buf[i];
  }
  return crc;
}

static bool is_valid_test_packet(const uint8_t *buf, size_t len) {
  if (!buf || len != kPacketLen) {
    return false;
  }
  if (buf[0] != kMagic0 || buf[1] != kMagic1) {
    return false;
  }
  if (buf[2] != kProtocolVersion || buf[3] != kPacketLen) {
    return false;
  }
  return xor_crc(buf, kPacketLen - 1) == buf[kPacketLen - 1];
}

static bool init_lora() {
  lora_spi.begin(kLoraSck, kLoraMiso, kLoraMosi, kLoraNss);
  LoRa.setSPI(lora_spi);
  LoRa.setPins(kLoraNss, kLoraRst, kLoraDio0);

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
  if (lora_ready || (now_ms - last_lora_retry_ms) < kLoraRetryMs) {
    return;
  }
  last_lora_retry_ms = now_ms;
  lora_ready = init_lora();
  Serial.println(lora_ready ? "LoRa RX ready" : "LoRa RX init failed");
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

static void handle_lora_rx() {
  int packet_size = LoRa.parsePacket();
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

  if (!is_valid_test_packet(packet, len)) {
    rx_bad++;
    LoRa.receive();
    return;
  }

  const uint32_t seq = read_u32_le(packet, 4);
  const uint32_t tx_ms = read_u32_le(packet, 8);
  const uint16_t interval_ms = read_u16_le(packet, 12);
  const uint8_t flags = read_u8(packet, 14);
  (void)tx_ms;
  (void)interval_ms;
  (void)flags;

  const uint32_t now_ms = millis();
  if (rx_valid == 0) {
    first_rx_ms = now_ms;
  }
  rx_valid++;
  last_rx_ms = now_ms;
  update_seq_stats(seq);

  LoRa.receive();
}

static void print_report(uint32_t now_ms) {
  const uint32_t expected = rx_valid + missing;
  const float loss_rate = expected == 0 ? 0.0f : (100.0f * static_cast<float>(missing) / expected);
  const float elapsed_s = first_rx_ms == 0 ? 0.0f : static_cast<float>(now_ms - first_rx_ms) / 1000.0f;
  const float rx_hz = elapsed_s <= 0.0f ? 0.0f : static_cast<float>(rx_valid) / elapsed_s;
  const bool link_ok = (last_rx_ms != 0) && ((now_ms - last_rx_ms) <= kLinkTimeoutMs);

  Serial.print("link=");
  Serial.print(link_ok ? "OK" : "WAIT");
  Serial.print(" rx=");
  Serial.print(rx_valid);
  Serial.print(" missing=");
  Serial.print(missing);
  Serial.print(" loss=");
  Serial.print(loss_rate, 2);
  Serial.print("% rx_rate=");
  Serial.print(rx_hz, 2);
  Serial.print("Hz bad=");
  Serial.print(rx_bad);
  Serial.print(" dup=");
  Serial.print(duplicate);
  Serial.print(" old=");
  Serial.print(out_of_order);
  Serial.print(" last_seq=");
  Serial.print(have_seq ? last_seq : 0);
  Serial.print(" rssi=");
  Serial.print(last_rssi);
  Serial.print(" snr=");
  Serial.println(last_snr, 1);
}

void setup() {
  pinMode(kLoraDio1, INPUT);

  Serial.begin(kSerialBaud);
  uint32_t start_ms = millis();
  while (!Serial && (millis() - start_ms) < 1500) {
    delay(10);
  }

  Serial.println();
  Serial.println("LoRa packet-loss test RX");
  Serial.println("Expected TX: rocket_main packet-loss test firmware");

  lora_ready = init_lora();
  last_lora_retry_ms = millis();
  Serial.println(lora_ready ? "LoRa RX ready" : "LoRa RX init failed");
}

void loop() {
  const uint32_t now_ms = millis();
  maintain_lora(now_ms);

  if (lora_ready) {
    handle_lora_rx();
  }

  if (now_ms - last_report_ms >= kReportIntervalMs) {
    last_report_ms = now_ms;
    print_report(now_ms);
  }

  delay(1);
}
