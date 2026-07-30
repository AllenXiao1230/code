import argparse
import sys
import time
from pathlib import Path


DEFAULT_BAUD = 115200
DEFAULT_ENCODING = "utf-8"


def configure_stdout() -> None:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def list_ports() -> int:
    try:
        import serial.tools.list_ports
    except ImportError:
        print("pyserial is not installed. Install it with: python3 -m pip install pyserial")
        return 2

    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    if not ports:
        print("No serial ports detected.")
        return 0

    for p in ports:
        print(f"{p.device}\t{p.description or ''}\t{p.hwid or ''}")
    return 0


def safe_print(text: str) -> None:
    try:
        print(text)
    except UnicodeEncodeError:
        encoded = text.encode(sys.stdout.encoding or "utf-8", errors="replace")
        print(encoded.decode(sys.stdout.encoding or "utf-8", errors="replace"))


def decode_serial_line(raw: bytes, encoding: str) -> str:
    return raw.decode(encoding, errors="replace").rstrip("\r\n")


def log_serial(port: str, baud: int, output: Path, encoding: str) -> int:
    try:
        import serial
    except ImportError:
        print("pyserial is not installed. Install it with: python3 -m pip install pyserial")
        return 2

    output.parent.mkdir(parents=True, exist_ok=True)
    print(f"Opening {port} @ {baud}, writing to {output}, decode={encoding}")
    print("Press Ctrl+C to stop.")

    try:
        with serial.Serial(port, baud, timeout=0.5) as ser, output.open(
            "a", encoding="utf-8", errors="replace"
        ) as f:
            f.write(f"# host_log_start unix={int(time.time())} port={port} baud={baud} decode={encoding}\n")
            f.flush()

            while True:
                raw = ser.readline()
                if not raw:
                    continue

                text = decode_serial_line(raw, encoding)
                safe_print(text)
                f.write(text + "\n")
                f.flush()

    except KeyboardInterrupt:
        print("\nStopped.")
        return 0
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        return 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Log barometric altitude serial output to a text file.")
    parser.add_argument("port", nargs="?", help="Serial port, for example /dev/cu.usbserial-xxxx or COM3")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate. Default: {DEFAULT_BAUD}")
    parser.add_argument(
        "--encoding",
        default=DEFAULT_ENCODING,
        help=f"Serial text decoding. Invalid bytes are replaced. Default: {DEFAULT_ENCODING}",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("baro_altitude_log.txt"),
        help="Output text file. Default: baro_altitude_log.txt",
    )
    parser.add_argument("--list-ports", action="store_true", help="List serial ports and exit.")
    return parser.parse_args()


def main() -> int:
    configure_stdout()
    args = parse_args()
    if args.list_ports:
        return list_ports()
    if not args.port:
        print("Missing serial port. Use --list-ports to find one.")
        return 2
    return log_serial(args.port, args.baud, args.output, args.encoding)


if __name__ == "__main__":
    raise SystemExit(main())
