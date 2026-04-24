import serial

from protocol import TelemetryStreamParser, decode_frame


PORT = "/dev/tty.usbserial-56BC0010791"
BAUD = 115200


def main():
    parser = TelemetryStreamParser()
    ser = serial.Serial(PORT, BAUD, timeout=0.2)

    while True:
        raw = ser.read(ser.in_waiting or 1)
        for frame in parser.feed(raw):
            packet = decode_frame(frame)
            baro_text = "---" if packet.baro_alt_m is None else f"{packet.baro_alt_m:7.1f}m"
            print(
                f"T+{packet.time_s:6.1f}s "
                f"state={packet.flight_state:3d} "
                f"baro={baro_text} "
                f"gps={packet.gps_alt_m:7.1f}m "
                f"vbat={packet.battery_v:5.2f}V "
                f"err=0x{packet.error_code:02X}"
            )


if __name__ == "__main__":
    main()
