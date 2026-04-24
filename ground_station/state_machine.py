# state_machine.py
from enum import IntEnum
from PyQt5.QtCore import QObject, pyqtSignal
from protocol import FrameDecodeError, decode_frame


class FlightState(IntEnum):
    TEST = 0
    IDLE = 1
    PREFLIGHT = 2
    ASCENT = 3
    APOGEE = 4
    DESCENT = 5
    LANDED = 6
    ABORT = 99


class ErrorBit(IntEnum):
    NONE = 0
    LORA_INIT = 1 << 0
    SD_INIT = 1 << 1
    RTC_INIT = 1 << 2
    BARO_INIT = 1 << 3
    IMU_INIT = 1 << 4
    ADXL_INIT = 1 << 5
    GPS_NO_FIX = 1 << 6
    BATTERY = 1 << 7


class StateMachine(QObject):
    """
    Telemetry signal bridge for decoded 47-byte frames.
    """

    # ---- Signals for UI / Logger / Plot ----
    state_changed = pyqtSignal(FlightState)
    attitude_updated = pyqtSignal(float, float, float)   # roll, pitch, yaw
    battery_updated = pyqtSignal(float)                  # voltage
    altitude_updated = pyqtSignal(float)                 # meters
    gps_updated = pyqtSignal(float, float, float)        # lat, lon, alt
    event_received = pyqtSignal(str)
    timestamp_updated = pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self.state = FlightState.IDLE
        self._last_error = None

    # ---------- Public API ----------

    def handle_packet(self, frame: bytes):
        """
        Entry point for 47-byte telemetry frames.
        """
        if not frame:
            return

        try:
            packet = decode_frame(frame)
        except FrameDecodeError:
            return

        try:
            new_state = FlightState(packet.flight_state)
        except Exception:
            new_state = None

        if new_state is not None and new_state != self.state:
            self.state = new_state
            self.state_changed.emit(self.state)

        altitude_m = packet.baro_alt_m if packet.has_baro_alt else packet.gps_alt_m
        self.attitude_updated.emit(packet.roll_deg, packet.pitch_deg, packet.heading_deg)
        self.battery_updated.emit(packet.battery_v)
        self.altitude_updated.emit(altitude_m)
        self.gps_updated.emit(packet.lat_deg, packet.lon_deg, packet.gps_alt_m)

        if packet.error_code != self._last_error:
            self._last_error = packet.error_code
            if packet.error_code:
                self.event_received.emit(f"Error: {decode_error(packet.error_code)}")

        self.timestamp_updated.emit(packet.time_s)


def decode_state(status: int) -> str:
    """
    Decode flight state into a readable string.
    """
    if status is None:
        return "UNKNOWN"

    try:
        state = FlightState(status)
    except Exception:
        return "UNKNOWN"

    names = {
        FlightState.TEST: "測試",
        FlightState.IDLE: "待命",
        FlightState.PREFLIGHT: "發射前",
        FlightState.ASCENT: "上升",
        FlightState.APOGEE: "過頂點",
        FlightState.DESCENT: "下降",
        FlightState.LANDED: "著陸",
        FlightState.ABORT: "中止",
    }
    return names.get(state, state.name)


def decode_error(err: int) -> str:
    if err is None:
        return "OK"
    if err == 0:
        return "OK"

    codes = [
        (ErrorBit.LORA_INIT, "LORA"),
        (ErrorBit.SD_INIT, "SD"),
        (ErrorBit.RTC_INIT, "RTC"),
        (ErrorBit.BARO_INIT, "BARO"),
        (ErrorBit.IMU_INIT, "IMU"),
        (ErrorBit.ADXL_INIT, "ADXL"),
        (ErrorBit.GPS_NO_FIX, "GPS"),
        (ErrorBit.BATTERY, "BAT"),
    ]
    labels = [label for bit, label in codes if err & int(bit)]
    if not labels:
        return f"ERR:0x{err:02X}"
    return " | ".join(labels)
