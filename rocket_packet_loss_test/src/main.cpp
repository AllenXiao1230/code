#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>

// Standalone rocket-side LoRa packet-loss test transmitter.
// This project is intentionally separate from rocket_main so the flight
// firmware can stay unchanged.

static constexpr uint32_t kSerialBaud = 115200;
static constexpr uint32_t kTxIntervalMs = 100;  // 10 Hz

// Rocket LoRa module pins.
static constexpr int kLoraSck = 42;
static constexpr int kLoraMiso = 41;
static constexpr int kLoraMosi = 40;
static constexpr int kLoraNss = 39;
static constexpr int kLoraRst = 38;
static constexpr int kLoraDio0 = 37;
static constexpr int kLoraDio1 = 36;

// LoRa radio parameters. Must match ground_terminal/src/main.cpp.
static constexpr long kLoraFreq = 433000000;
static constexpr byte kLoraSyncWord = 0x12;
static constexpr int kLoraTxPower = 22;
static constexpr long kLoraBw = 125E3;
static constexpr int kLoraSf = 9;
static constexpr int kLoraCr = 6;  // 4/6
static constexpr int kLoraPreambleLen = 8;

static constexpr uint8_t kPacketLen = 16;
static constexpr uint8_t kMagic0 = 'R';
static constexpr uint8_t kMagic1 = 'L';
static constexpr uint8_t kProtocolVersion = 1;
static constexpr uint32_t kLoraRetryMs = 3000;
static constexpr uint32_t kStatusIntervalMs = 1000;

static SPIClass lora_spi(FSPI);
static bool lora_ready = false;
static uint32_t seq = 0;
static uint32_t sent_ok = 0;
static uint32_t sent_fail = 0;
static uint32_t last_tx_ms = 0;
static uint32_t last_status_ms = 0;
static uint32_t last_lora_retry_ms = 0;

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

static uint8_t xor_crc(const uint8_t *buf, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
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
  packet[14] = 0;
  packet[15] = xor_crc(packet, kPacketLen - 1);
}

static bool init_lora() {
  lora_spi.begin(kLoraSck, kLoraMiso, kLoraMosi, kLoraNss);
  LoRa.setSPI(lora_spi);
  LoRa.setPins(kLoraNss, kLoraRst, kLoraDio0);

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
  return true;
}

static void maintain_lora(uint32_t now_ms) {
  if (lora_ready || (now_ms - last_lora_retry_ms) < kLoraRetryMs) {
    return;
  }
  last_lora_retry_ms = now_ms;
  lora_ready = init_lora();
  Serial.println(lora_ready ? "LoRa TX ready" : "LoRa TX init failed");
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
  Serial.print("seq=");
  Serial.print(seq);
  Serial.print(" sent_ok=");
  Serial.print(sent_ok);
  Serial.print(" sent_fail=");
  Serial.print(sent_fail);
  Serial.print(" lora=");
  Serial.print(lora_ready ? "OK" : "FAIL");
  Serial.print(" interval_ms=");
  Serial.print(kTxIntervalMs);
  Serial.print(" uptime_ms=");
  Serial.println(now_ms);
}

void setup() {
  pinMode(kLoraDio1, INPUT);

  Serial.begin(kSerialBaud);
  uint32_t start_ms = millis();
  while (!Serial && (millis() - start_ms) < 1500) {
    delay(10);
  }

  Serial.println();
  Serial.println("LoRa packet-loss test TX");

  lora_ready = init_lora();
  last_lora_retry_ms = millis();
  Serial.println(lora_ready ? "LoRa TX ready" : "LoRa TX init failed");
}

void loop() {
  const uint32_t now_ms = millis();
  maintain_lora(now_ms);

  if (now_ms - last_tx_ms >= kTxIntervalMs) {
    last_tx_ms = now_ms;

    if (send_packet(now_ms)) {
      sent_ok++;
    } else {
      sent_fail++;
      if (sent_fail % 10 == 0) {
        lora_ready = false;
      }
    }
    seq++;
  }

  if (now_ms - last_status_ms >= kStatusIntervalMs) {
    last_status_ms = now_ms;
    print_status(now_ms);
  }

  delay(1);
}
