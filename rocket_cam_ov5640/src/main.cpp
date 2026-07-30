#include "esp_camera.h"
#include "camera_pins.h"

#include <Arduino.h>

// UART protocol, host -> camera:
//   STATUS
//   CAPTURE
//   BURST START <interval_ms>
//   BURST STOP
//
// UART protocol, camera -> host:
//   CAM_STATUS {...json...}
//   CAM_FRAME <jpeg_len> <frame_count> <capture_ms>
//   <raw jpeg bytes>
//   CAM_FRAME_END
//   CAM_ERROR <message>

static constexpr uint32_t UART_BAUD = 921600;

static bool burstEnabled = false;
static uint32_t burstIntervalMs = 1000;
static uint32_t lastBurstMs = 0;
static uint32_t framesCaptured = 0;
static uint32_t captureFailures = 0;
static uint32_t lastCaptureMs = 0;
static uint32_t lastFrameBytes = 0;
static String commandBuffer;

static String jsonStatus() {
  String json = "{";
  json += "\"burst\":";
  json += burstEnabled ? "true" : "false";
  json += ",\"interval_ms\":";
  json += burstIntervalMs;
  json += ",\"frames\":";
  json += framesCaptured;
  json += ",\"failures\":";
  json += captureFailures;
  json += ",\"last_capture_ms\":";
  json += lastCaptureMs;
  json += ",\"last_frame_bytes\":";
  json += lastFrameBytes;
  json += ",\"uptime_ms\":";
  json += millis();
  json += ",\"free_heap\":";
  json += ESP.getFreeHeap();
  json += ",\"psram\":";
  json += psramFound() ? "true" : "false";
  json += "}";
  return json;
}

static void sendStatus() {
  Serial.print("CAM_STATUS ");
  Serial.println(jsonStatus());
}

static bool captureAndSendFrame() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    captureFailures++;
    Serial.println("CAM_ERROR capture_failed");
    sendStatus();
    return false;
  }

  lastFrameBytes = fb->len;
  lastCaptureMs = millis();
  framesCaptured++;

  Serial.printf("CAM_FRAME %u %u %u\n",
                static_cast<unsigned>(fb->len),
                static_cast<unsigned>(framesCaptured),
                static_cast<unsigned>(lastCaptureMs));
  Serial.write(fb->buf, fb->len);
  Serial.print("\nCAM_FRAME_END\n");
  Serial.flush();

  esp_camera_fb_return(fb);
  sendStatus();
  return true;
}

static void handleCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command.length() == 0) {
    return;
  }

  if (command == "STATUS") {
    sendStatus();
    return;
  }

  if (command == "CAPTURE") {
    captureAndSendFrame();
    return;
  }

  if (command == "BURST STOP") {
    burstEnabled = false;
    sendStatus();
    return;
  }

  if (command.startsWith("BURST START")) {
    int lastSpace = command.lastIndexOf(' ');
    if (lastSpace > 0) {
      uint32_t requested = command.substring(lastSpace + 1).toInt();
      if (requested > 0) {
        burstIntervalMs = constrain(requested, 500UL, 60000UL);
      }
    }
    burstEnabled = true;
    lastBurstMs = 0;
    sendStatus();
    return;
  }

  Serial.print("CAM_ERROR unknown_command ");
  Serial.println(command);
}

static void pollSerialCommands() {
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\n' || ch == '\r') {
      if (commandBuffer.length() > 0) {
        handleCommand(commandBuffer);
        commandBuffer = "";
      }
      continue;
    }

    if (commandBuffer.length() < 96) {
      commandBuffer += ch;
    } else {
      commandBuffer = "";
      Serial.println("CAM_ERROR command_too_long");
    }
  }
}

static void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 14;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("CAM_ERROR camera_init_failed 0x%x\n", err);
    delay(2000);
    ESP.restart();
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != nullptr && sensor->id.PID == OV5640_PID) {
    sensor->set_vflip(sensor, 1);
    sensor->set_brightness(sensor, 0);
    sensor->set_saturation(sensor, 0);
  }
}

void setup() {
  Serial.begin(UART_BAUD);
  Serial.setDebugOutput(false);
  delay(300);

  commandBuffer.reserve(96);
  Serial.println("CAM_BOOT Rocket ESP32-CAM OV5640 UART");
  setupCamera();
  sendStatus();
}

void loop() {
  pollSerialCommands();

  if (burstEnabled) {
    uint32_t now = millis();
    if (lastBurstMs == 0 || now - lastBurstMs >= burstIntervalMs) {
      lastBurstMs = now;
      captureAndSendFrame();
    }
  }
}
