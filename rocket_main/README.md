Rocket Main Firmware
====================

Build / upload (main firmware):
```bash
pio run -e esp32-s3-devkitc-1-n16r8
pio run -e esp32-s3-devkitc-1-n16r8 -t upload
pio device monitor -b 115200
```

GPS parsing is integrated in `src/main.cpp` (no separate GPS test firmware file).

Board Pin Map
-------------
| Pin | Function | Signal | Notes |
| --- | -------- | ------ | ----- |
| 42 | LoRa SCK | SPI |  |
| 41 | LoRa MISO | SPI |  |
| 40 | LoRa MOSI | SPI |  |
| 39 | LoRa NSS / CS | SPI |  |
| 38 | LoRa RESET | - |  |
| 37 | LoRa DIO0 | Interrupt |  |
| 36 | LoRa DIO1 | Interrupt |  |
| 35 | GPS TX | UART | Connect to GPS RX |
| 45 | GPS RX | UART | Connect to GPS TX |
| 47 | GPS PPS | - |  |
| 48 | WS2812 DIN | - | Onboard RGB |
| 21 | Safety Pin | SIG | Pull-up |
| 20 | I2C #1 SDA | I2C | IMU / ADXL / BMP |
| 19 | I2C #1 SCL | I2C | IMU / ADXL / BMP |
| 4 | Battery ADC | VBAT -> ADC | Divider into ADC |
| 5 | Camera A Trigger | SIG | Pulse trigger Camera A |
| 6 | Camera B Trigger | SIG | Pulse trigger Camera B |
| 15 | Water Detect | SIG | High = water |
| 16 | Separation Servo | PWM | Compartment separation servo drive |
| 17 | UART TX | UART | Connect to sub-board RX |
| 18 | UART RX | UART | Connect to sub-board TX |
| 8 | Buzzer | SIG | Active buzzer |
| 3 | DS3231 SQW / INT | - | Pull-up |
| 9 | I2C #2 SDA | I2C | DS3231 / SHT31 |
| 10 | I2C #2 SCL | I2C | DS3231 / SHT31 |
| 11 | SD Card CS | SPI |  |
| 12 | SD Card MOSI | SPI |  |
| 13 | SD Card SCK | SPI |  |
| 14 | SD Card MISO | SPI |  |

Notes
-----
- GPS baud rate: 230400
- Battery divider: 200k (top to VBAT) / 100k (bottom to GND)
- DEBUG_SERIAL should remain 0 when Serial is used for binary packets.

GPS Notes
---------
- GPS UART pin mapping: `RX=45`, `TX=35`, `PPS=47`
- Supported auto-scan baud list: `230400`, `115200`, `57600`, `38400`, `9600`
- If no NMEA is parsed, check GPS TX/RX crossing and common GND first.
