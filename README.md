Rocket Project
==============

This repository contains two main components:
- `rocket_main`: Firmware running on the flight computer (ESP32-S3).
- `ground_station`: PyQt5 GUI for telemetry display and control.

Quick Start
-----------
- Firmware: open `rocket_main` in PlatformIO and flash.
- Ground station: `cd ground_station` and run `python main.py`.

Pin Map (rocket_main)
---------------------
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
| 5 | Cam 1 | SIG | Pulse trigger Cam 1 |
| 6 | Cam 2 | SIG | Pulse trigger Cam 2 |
| 15 | Water Detect | SIG | High = water |
| 16 | PWM | PWM | Servo |
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
- Keep `DEBUG_SERIAL = 0` when Serial is used for binary packets.

References
----------
- Firmware details: `rocket_main/README.md`
- Ground station details: `ground_station/README.md`
