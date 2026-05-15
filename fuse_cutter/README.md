# Fuse Cutter

Standalone ESP32-S3 Super Mini fuse cutter project. It is independent from the ground station and other rocket-side projects.

## Hardware Pins

- Fuse cutter output: GPIO 13
- I2C SDA: GPIO 8
- I2C SCL: GPIO 9
- Serial baud: 115200
- Board: ESP32-S3 Super Mini, configured through PlatformIO as `esp32-s3-devkitc-1` with 4MB flash and USB CDC enabled

## Firmware

Firmware path:

```bash
cd /Users/xuan/Documents/rocket/code/fuse_cutter
pio run
pio run -t upload
pio device monitor
```

The firmware streams newline-delimited JSON telemetry every 100 ms and accepts these newline-terminated serial commands:

- `ARM`
- `DISARM`
- `FIRE`
- `PING`

`FIRE` only works while armed. The cutter pin is driven high for 1500 ms, then forced low and disarmed.

## GUI

GUI path:

```bash
cd /Users/xuan/Documents/rocket/code/fuse_cutter/gui
python3 -m pip install -r requirements.txt
python3 main.py
```

The GUI displays BMP280 and ICM-42688 telemetry from the ESP32-S3 and provides buttons for `ARM`, `DISARM`, `FIRE`, and `PING`.
