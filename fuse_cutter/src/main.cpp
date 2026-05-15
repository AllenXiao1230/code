#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <ICM42688.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define CUT_PIN 13

#define FIRE_TIME_MS 1500
#define SEND_PERIOD_MS 100

Adafruit_BMP280 bmp;
ICM42688 imu(Wire, 0x68);  // If the IMU is not found, try 0x69.

bool bmpOK = false;
bool imuOK = false;
bool armed = false;
bool firing = false;

unsigned long fireStart = 0;
unsigned long lastSend = 0;
unsigned long bootTime = 0;

float basePressure = 1013.25;

void handleSerialCommand();
void handleFiring();
void sendTelemetry();
void printFloatOrNull(float v);
void sendStatus(const char *message);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  delay(1000);

  pinMode(CUT_PIN, OUTPUT);
  digitalWrite(CUT_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  bmpOK = bmp.begin(0x76);  // BMP280 common address. Try 0x77 if not detected.
  if (bmpOK) {
    bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_1
    );
    basePressure = bmp.readPressure() / 100.0;
  }

  imuOK = (imu.begin() == 0);
  if (imuOK) {
    imu.setAccelFS(ICM42688::gpm16);
    imu.setGyroFS(ICM42688::dps2000);
    imu.setAccelODR(ICM42688::odr1k);
    imu.setGyroODR(ICM42688::odr1k);
  }

  bootTime = millis();
  sendStatus("ESP32-S3 fuse cutter ready");
}

void loop() {
  handleSerialCommand();
  handleFiring();
  sendTelemetry();
}

void handleSerialCommand() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "ARM") {
    armed = true;
    Serial.println("{\"type\":\"ack\",\"cmd\":\"ARM\",\"armed\":true}");
  } else if (cmd == "DISARM") {
    armed = false;
    firing = false;
    digitalWrite(CUT_PIN, LOW);
    Serial.println("{\"type\":\"ack\",\"cmd\":\"DISARM\",\"armed\":false}");
  } else if (cmd == "FIRE") {
    if (armed && !firing) {
      firing = true;
      fireStart = millis();
      digitalWrite(CUT_PIN, HIGH);
      Serial.println("{\"type\":\"ack\",\"cmd\":\"FIRE\",\"firing\":true}");
    } else {
      Serial.println("{\"type\":\"error\",\"msg\":\"not armed or already firing\"}");
    }
  } else if (cmd == "PING") {
    Serial.println("{\"type\":\"ack\",\"cmd\":\"PING\"}");
  } else if (cmd.length() > 0) {
    Serial.print("{\"type\":\"error\",\"msg\":\"unknown command\",\"cmd\":\"");
    Serial.print(cmd);
    Serial.println("\"}");
  }
}

void handleFiring() {
  if (firing && millis() - fireStart >= FIRE_TIME_MS) {
    firing = false;
    armed = false;
    digitalWrite(CUT_PIN, LOW);
    Serial.println("{\"type\":\"ack\",\"cmd\":\"FIRE_DONE\",\"armed\":false}");
  }
}

void sendTelemetry() {
  if (millis() - lastSend < SEND_PERIOD_MS) return;
  lastSend = millis();

  float temp = NAN;
  float pressure = NAN;
  float altitude = NAN;

  if (bmpOK) {
    temp = bmp.readTemperature();
    pressure = bmp.readPressure() / 100.0;
    altitude = bmp.readAltitude(basePressure);
  }

  float ax = NAN, ay = NAN, az = NAN;
  float gx = NAN, gy = NAN, gz = NAN;

  if (imuOK && imu.getAGT() == 0) {
    ax = imu.accX();
    ay = imu.accY();
    az = imu.accZ();
    gx = imu.gyrX();
    gy = imu.gyrY();
    gz = imu.gyrZ();
  }

  Serial.print("{");
  Serial.print("\"type\":\"telemetry\",");
  Serial.print("\"time_ms\":"); Serial.print(millis() - bootTime); Serial.print(",");
  Serial.print("\"armed\":"); Serial.print(armed ? "true" : "false"); Serial.print(",");
  Serial.print("\"firing\":"); Serial.print(firing ? "true" : "false"); Serial.print(",");
  Serial.print("\"bmp_ok\":"); Serial.print(bmpOK ? "true" : "false"); Serial.print(",");
  Serial.print("\"imu_ok\":"); Serial.print(imuOK ? "true" : "false"); Serial.print(",");
  Serial.print("\"cut_pin\":"); Serial.print(CUT_PIN); Serial.print(",");
  Serial.print("\"sda_pin\":"); Serial.print(SDA_PIN); Serial.print(",");
  Serial.print("\"scl_pin\":"); Serial.print(SCL_PIN); Serial.print(",");
  Serial.print("\"temp_c\":"); printFloatOrNull(temp); Serial.print(",");
  Serial.print("\"pressure_hpa\":"); printFloatOrNull(pressure); Serial.print(",");
  Serial.print("\"alt_m\":"); printFloatOrNull(altitude); Serial.print(",");
  Serial.print("\"ax\":"); printFloatOrNull(ax); Serial.print(",");
  Serial.print("\"ay\":"); printFloatOrNull(ay); Serial.print(",");
  Serial.print("\"az\":"); printFloatOrNull(az); Serial.print(",");
  Serial.print("\"gx\":"); printFloatOrNull(gx); Serial.print(",");
  Serial.print("\"gy\":"); printFloatOrNull(gy); Serial.print(",");
  Serial.print("\"gz\":"); printFloatOrNull(gz);
  Serial.println("}");
}

void printFloatOrNull(float v) {
  if (isnan(v)) Serial.print("null");
  else Serial.print(v, 3);
}

void sendStatus(const char *message) {
  Serial.print("{\"type\":\"status\",\"msg\":\"");
  Serial.print(message);
  Serial.print("\",\"cut_pin\":");
  Serial.print(CUT_PIN);
  Serial.print(",\"sda_pin\":");
  Serial.print(SDA_PIN);
  Serial.print(",\"scl_pin\":");
  Serial.print(SCL_PIN);
  Serial.println("}");
}
