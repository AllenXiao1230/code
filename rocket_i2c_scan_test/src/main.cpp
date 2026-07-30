#include <Arduino.h>
#include <Wire.h>

// ESP32-S3 N16R8 standalone I2C scanner.
// Native USB CDC is disabled in platformio.ini. Console output uses CH343/UART0.

static constexpr uint32_t kSerialBaud = 115200;
static constexpr int kConsoleRx = 44;
static constexpr int kConsoleTx = 43;

static constexpr int kI2cSda = 6;
static constexpr int kI2cScl = 7;
static constexpr uint32_t kI2cInitClockHz = 100000;
static constexpr uint32_t kI2cScanClocksHz[] = {10000, 50000, 100000, 400000};
static constexpr uint32_t kAutoScanIntervalMs = 2000;

static uint32_t last_scan_ms = 0;
static char command_line[32];
static uint8_t command_idx = 0;

static void print_hex_u8(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

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

  if (digitalRead(kI2cSda) == LOW) {
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

      if (addr == 0x69 || addr == 0x76 || addr == 0x77 || addr >= 0x78) {
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

  for (uint8_t i = 0; i < sizeof(kI2cScanClocksHz) / sizeof(kI2cScanClocksHz[0]); i++) {
    const uint8_t count = scan_i2c_at_clock(kI2cScanClocksHz[i]);
    if (count > best_count) {
      best_count = count;
    }
  }

  Wire.setClock(kI2cInitClockHz);
  return best_count;
}

static void run_i2c_scan(const char *source) {
  Serial.print("# i2c_scan_start source=");
  Serial.println(source);
  recover_i2c_bus();
  const uint8_t count = scan_i2c_all_speeds();

  Serial.println("# direct_common_sensor_probe");
  probe_common_id_registers(0x68);
  probe_common_id_registers(0x69);
  probe_common_id_registers(0x76);
  probe_common_id_registers(0x77);

  Serial.print("# i2c_scan_done source=");
  Serial.print(source);
  Serial.print(" best_count=");
  Serial.print(count);
  Serial.print(" uptime_ms=");
  Serial.println(millis());
}

static void print_help() {
  Serial.println("# commands: I2C_SCAN, SCAN, HELP");
}

static void handle_command(const char *cmd) {
  if (strcmp(cmd, "I2C_SCAN") == 0 || strcmp(cmd, "SCAN") == 0) {
    run_i2c_scan("manual");
    return;
  }
  if (strcmp(cmd, "HELP") == 0) {
    print_help();
    return;
  }

  Serial.print("# unknown_command=");
  Serial.println(cmd);
  print_help();
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

  Serial.println("# ESP32-S3-DevKitC-1 standalone I2C scanner");
  Serial.print("# serial_baud=");
  Serial.println(kSerialBaud);
  Serial.print("# i2c_sda=");
  Serial.print(kI2cSda);
  Serial.print(" i2c_scl=");
  Serial.println(kI2cScl);
  print_help();

  run_i2c_scan("boot");
  last_scan_ms = millis();
}

void loop() {
  handle_serial_commands();

  const uint32_t now_ms = millis();
  if (now_ms - last_scan_ms >= kAutoScanIntervalMs) {
    last_scan_ms = now_ms;
    run_i2c_scan("auto");
  }

  delay(1);
}
