# state_machine.py
from enum import IntEnum, auto
from PyQt5.QtCore import QObject, pyqtSignal
import struct
import time


class FlightState(IntEnum):
    TEST = 0
    IDLE = 1
    PREFLIGHT = 2
    ASCENT = 3
    APOGEE = 4
    DESCENT = 5
    LANDED = 6
    ABORT = 99


class ErrorCode(IntEnum):
    ERR_NONE = 0
    ERR_LORA_LOST = 1
    ERR_GPS_LOST = 2
    ERR_IMU_FAIL = 3
    ERR_BARO_FAIL = 4
    ERR_BATTERY_LOW = 5
    ERR_SENSOR_TIMEOUT = 6
    ERR_UNKNOWN = 255


class StateMachine(QObject):
    """
    Central flight state machine.
    All decoded packets MUST go through here.
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
        self.boot_time = time.time()

    # ---------- Public API ----------

    def handle_packet(self, payload: bytes):
        """
        Entry point for all telemetry packets.
        """
        if not payload:
            return

        packet_type = payload[0]

        if packet_type == 0x01:
            self._handle_attitude(payload)
        elif packet_type == 0x02:
            self._handle_battery(payload)
        elif packet_type == 0x03:
            self._handle_altitude(payload)
        elif packet_type == 0x04:
            self._handle_event(payload)
        elif packet_type == 0x05:
            self._handle_state_change(payload)
        elif packet_type == 0x06:
            self._handle_gps(payload)

        self._update_time()

    # ---------- Packet Handlers ----------

    def _handle_attitude(self, data: bytes):
        # [0x01][roll][pitch][yaw] (float32 x3)
        try:
            roll, pitch, yaw = struct.unpack("<fff", data[1:13])
            self.attitude_updated.emit(roll, pitch, yaw)
        except struct.error:
            pass

    def _handle_battery(self, data: bytes):
        # [0x02][voltage] (float32)
        try:
            (voltage,) = struct.unpack("<f", data[1:5])
            self.battery_updated.emit(voltage)
        except struct.error:
            pass

    def _handle_altitude(self, data: bytes):
        # [0x03][altitude] (float32)
        try:
            (altitude,) = struct.unpack("<f", data[1:5])
            self.altitude_updated.emit(altitude)
        except struct.error:
            pass

    def _handle_event(self, data: bytes):
        # [0x04][ascii message...]
        try:
            msg = data[1:].decode(errors="ignore")
            self.event_received.emit(msg)
        except Exception:
            pass

    def _handle_state_change(self, data: bytes):
        # [0x05][state_id]
        try:
            new_state = FlightState(data[1])
            if new_state != self.state:
                self.state = new_state
                self.state_changed.emit(self.state)
        except Exception:
            pass

    def _handle_gps(self, data: bytes):
        # [0x06][lat][lon][alt] (float32 x3)
        try:
            lat, lon, alt = struct.unpack("<fff", data[1:13])
            self.gps_updated.emit(lat, lon, alt)
        except struct.error:
            pass

    # ---------- Time ----------

    def _update_time(self):
        self.timestamp_updated.emit(time.time() - self.boot_time)


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
    try:
        code = ErrorCode(err)
    except Exception:
        return f"ERR:{err}"

    names = {
        ErrorCode.ERR_NONE: "OK",
        ErrorCode.ERR_LORA_LOST: "LoRa 斷線",
        ErrorCode.ERR_GPS_LOST: "GPS 斷訊",
        ErrorCode.ERR_IMU_FAIL: "IMU 故障",
        ErrorCode.ERR_BARO_FAIL: "氣壓計故障",
        ErrorCode.ERR_BATTERY_LOW: "電池電量低",
        ErrorCode.ERR_SENSOR_TIMEOUT: "感測逾時",
        ErrorCode.ERR_UNKNOWN: "未知錯誤",
    }
    return names.get(code, code.name)
