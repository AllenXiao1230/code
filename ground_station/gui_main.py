import struct
import time
# gui_main.py
from PyQt5.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QGroupBox,
    QSplitter,
)
from PyQt5.QtCore import Qt

from serial_comm import SerialWorker
from state_machine import FlightState
from config import FRAME_START, PACKET_LEN
from logger import CSVLogger

from rocket_attitude import RocketAttitudeView
from battery_panel import BatteryPanel
from plot_panel import PlotPanel
from event_indicator_panel import EventIndicatorPanel
from state_control_panel import StateControlPanel
from serial_panel import SerialPanel
from mission_time import MissionTimePanel
from map_view import MapView
from gui_widgets import TelemetryPanel, SpeedAltitudePanel, LogControlPanel



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Rocket Ground Station")
        self.resize(1200, 800)

        # ---- Core logic ----
        self._legacy_buffer = bytearray()
        self._legacy_link_logged = False
        self._mission_start_uptime = None
        self._last_uptime = 0.0
        self._boot_time_seconds = 0.0
        self._current_flight_state = None
        self._logger = None
        self.telemetry = {
            "time": 0.0,
            "lat": 0.0,
            "lon": 0.0,
            "alt": 0.0,
            "gps_alt": 0.0,
            "baro_alt": 0.0,
            "speed": 0.0,
            "gps_speed": 0.0,
            "baro_speed": 0.0,
            "heading": 0.0,
            "sat": 0,
            "accx": 0.0,
            "accy": 0.0,
            "accz": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "gyro_x": 0.0,
            "gyro_y": 0.0,
            "gyro_z": 0.0,
            "battery": 0.0,
            "status": 0,
            "water": 0,
        }

        # ---- Serial worker (created on demand) ----
        self.serial_worker = None

        # ---- UI ----
        self._init_ui()
        # Fixed-length telemetry decoding (44-byte frames).

    # ---------------- UI ----------------

    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        top_layout = QHBoxLayout()
        top_layout.setSpacing(10)

        self.attitude_view = RocketAttitudeView()
        self.battery_panel = BatteryPanel()
        self.plot_panel = PlotPanel()
        self.event_panel = EventIndicatorPanel()
        self.state_panel = StateControlPanel()
        self.serial_panel = SerialPanel()
        self.time_panel = MissionTimePanel()
        self.map_view = MapView()
        self.telemetry_panel = TelemetryPanel()
        self.speed_alt_panel = SpeedAltitudePanel()
        self.log_panel = LogControlPanel()

        def wrap(title: str, widget: QWidget) -> QGroupBox:
            box = QGroupBox(title)
            box_layout = QVBoxLayout(box)
            box_layout.setContentsMargins(6, 6, 6, 6)
            box_layout.addWidget(widget)
            return box

        # ---- Column 1: Serial (with status) + Map ----
        col1 = QVBoxLayout()
        col1.setSpacing(8)
        col1.addWidget(wrap("序列埠", self.serial_panel))
        col1.addWidget(wrap("地圖", self.map_view))
        col1.setStretch(0, 0)
        col1.setStretch(1, 1)

        # ---- Column 2: Flight state, events, speed+alt, attitude ----
        col2 = QVBoxLayout()
        col2.setSpacing(8)
        fs_row = QHBoxLayout()
        fs_row.setSpacing(6)
        fs_row.addWidget(wrap("飛行狀態控制", self.state_panel))
        fs_row.addWidget(wrap("記錄控制", self.log_panel))
        fs_row_widget = QWidget()
        fs_row_widget.setLayout(fs_row)

        col2.addWidget(fs_row_widget)
        col2.addWidget(wrap("火箭事件指示器", self.event_panel))
        col2.addWidget(wrap("火箭姿態", self.attitude_view))
        col2.setStretch(0, 0)
        col2.setStretch(1, 1)
        col2.setStretch(2, 2)

        # ---- Column 3: Time+Battery inline, other sensor data, speed/alt ----
        col3 = QVBoxLayout()
        col3.setSpacing(8)
        time_batt = QHBoxLayout()
        time_batt.setSpacing(6)
        time_batt.addWidget(self.time_panel)
        time_batt.addWidget(self.battery_panel)
        time_batt_widget = QWidget()
        time_batt_widget.setLayout(time_batt)
        col3.addWidget(wrap("時間 / 電壓", time_batt_widget))
        col3.addWidget(wrap("速度 / 高度", self.speed_alt_panel))
        col3.addWidget(wrap("感測器數據", self.telemetry_panel))
        col3.setStretch(0, 0)
        col3.setStretch(1, 0)
        col3.setStretch(2, 1)

        top_layout.addLayout(col1)
        top_layout.addLayout(col2)
        top_layout.addLayout(col3)

        # Even width for three columns
        top_layout.setStretch(0, 1)
        top_layout.setStretch(1, 1)
        top_layout.setStretch(2, 1)

        top_widget = QWidget()
        top_widget.setLayout(top_layout)

        splitter = QSplitter()
        splitter.setOrientation(Qt.Vertical)
        splitter.addWidget(top_widget)
        splitter.addWidget(wrap("高度曲線", self.plot_panel))
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        main_layout.addWidget(splitter)

        # Serial panel callbacks
        self.serial_panel.connect_requested.connect(self._start_serial)
        self.serial_panel.disconnect_requested.connect(self._stop_serial)
        self.state_panel.upload_requested.connect(self._on_state_upload)
        self.state_panel.rtc_sync_requested.connect(self._on_rtc_sync)
        self.log_panel.logging_toggled.connect(self._on_logging_toggled)

    # ---------------- Serial ----------------

    def _start_serial(self, port: str, baudrate: int):
        if self.serial_worker:
            return

        self.serial_worker = SerialWorker(port, baudrate)
        self.serial_worker.data_received.connect(self._on_serial_data)
        self.serial_worker.error_occurred.connect(self._on_serial_status)
        self.serial_worker.status_changed.connect(self._on_serial_status)
        self.serial_worker.start()

    def _stop_serial(self):
        if self.serial_worker:
            self.serial_worker.stop()
            self.serial_worker = None
        self.serial_panel.set_status("Status: DISCONNECTED", color="red")

    def _on_serial_data(self, raw: bytes):
        self._process_legacy_stream(raw)

    def closeEvent(self, event):
        if self.serial_worker:
            self.serial_worker.stop()
        event.accept()

    # ---------------- Handlers ----------------

    def _on_attitude(self, roll: float, pitch: float, yaw: float):
        self.telemetry["roll"] = roll
        self.telemetry["pitch"] = pitch
        self.telemetry["heading"] = yaw
        self._refresh_telemetry()

    def _on_battery(self, voltage: float):
        self.battery_panel.update_voltage(voltage)
        self.telemetry["battery"] = voltage
        self._refresh_telemetry()

    def _on_altitude(self, altitude: float):
        self.plot_panel.update_altitude(altitude)
        # Treat incoming altitude as barometric if separate GPS alt arrives later
        self.telemetry["alt"] = altitude
        self.telemetry["baro_alt"] = altitude
        self._refresh_telemetry()

    def _on_event(self, message: str):
        self.event_panel.add_event(message)

    def _on_state_changed(self, state):
        self.state_panel.update_state(state)
        # 記錄任務開始時間：狀態切換為 ASCENT 時鎖定
        if state == FlightState.ASCENT and self._mission_start_uptime is None:
            self._mission_start_uptime = self._last_uptime
        if state == FlightState.IDLE:
            self._mission_start_uptime = None

    def _on_time(self, seconds: float):
        self._last_uptime = seconds
        self._boot_time_seconds = seconds
        mission_elapsed = None
        if self._mission_start_uptime is not None:
            mission_elapsed = max(0.0, seconds - self._mission_start_uptime)

        self.time_panel.update_time(mission_elapsed or 0.0)
        self.telemetry["time"] = self._boot_time_seconds
        self._refresh_telemetry()

    def _on_gps(self, lat: float, lon: float, alt: float):
        self.map_view.update_gps(lat, lon, alt)
        self.telemetry["lat"] = lat
        self.telemetry["lon"] = lon
        self.telemetry["gps_alt"] = alt
        self._refresh_telemetry()

    def _refresh_telemetry(self):
        self.telemetry_panel.update_data(self.telemetry)
        self.speed_alt_panel.update_data(
            self.telemetry.get("gps_speed", self.telemetry.get("speed", 0.0)),
            self.telemetry.get("baro_speed", self.telemetry.get("speed", 0.0)),
            self.telemetry.get("gps_alt", self.telemetry.get("alt", 0.0)),
            self.telemetry.get("baro_alt", self.telemetry.get("alt", 0.0)),
        )
        self.attitude_view.update_attitude(
            self.telemetry.get("roll", 0.0),
            self.telemetry.get("pitch", 0.0),
            self.telemetry.get("heading", 0.0),
        )

    def _on_serial_status(self, message: str):
        self.event_panel.add_event(message)
        msg_lower = message.lower()
        if "disconnect" in msg_lower or "failed" in msg_lower:
            self.serial_panel.set_status(message, color="red")
        elif "connected" in msg_lower:
            self.serial_panel.set_status(message, color="green")
        else:
            self.serial_panel.set_status(message)

    def _on_state_upload(self, state: FlightState):
        # Placeholder: hook actual upload to rocket when protocol is defined.
        self.event_panel.add_event(f"Upload state requested: {state.name}")

    def _on_rtc_sync(self):
        if not self.serial_worker:
            self.event_panel.add_event("RTC sync failed: serial not connected")
            return
        epoch = int(time.time())
        cmd = f"RTC_SYNC:{epoch}\n".encode("ascii")
        self.serial_worker.write(cmd)
        self.event_panel.add_event(f"RTC sync sent: {epoch}")

    # ---------------- Fixed-length frames (44 bytes) ----------------

    def _process_legacy_stream(self, data: bytes):
        """
        Fixed 44-byte frames starting with 0x55 0xAA.
        Parse them here so the UI updates from fixed-length telemetry frames.
        """
        if not data:
            return

        self._legacy_buffer.extend(data)
        frame_len = PACKET_LEN

        while True:
            if len(self._legacy_buffer) < frame_len:
                return

            try:
                # Find sync (0x55 0xAA)
                start_idx = -1
                for i in range(len(self._legacy_buffer) - 1):
                    if self._legacy_buffer[i] == 0x55 and self._legacy_buffer[i + 1] == FRAME_START:
                        start_idx = i
                        break
                if start_idx == -1:
                    self._legacy_buffer.clear()
                    return
            except ValueError:
                self._legacy_buffer.clear()
                return

            if start_idx > 0:
                del self._legacy_buffer[:start_idx]
                if len(self._legacy_buffer) < frame_len:
                    return

            frame = bytes(self._legacy_buffer[:frame_len])
            del self._legacy_buffer[:frame_len]
            self._handle_legacy_frame(frame)

    def _handle_legacy_frame(self, frame: bytes):
        # Expected layout (44 bytes):
        # 0-1: 0x55 0xAA
        # 2: MsgType (0x01 telemetry)
        # 3-4: TimeTag uint16 (0.1 s)
        # 5-8: Latitude int32 (deg * 1e7)
        # 9-12: Longitude int32 (deg * 1e7)
        # 13-14: GPS Alt int16 (0.1 m)
        # 15-16: GPS Speed int16 (0.1 m/s)
        # 17: GPS Sat count uint8
        # 18-19: Roll int16 (0.01 deg) - IMU fusion output
        # 20-21: Pitch int16 (0.01 deg)
        # 22-23: Yaw uint16 (0.1 deg) - IMU fusion output
        # 24-25: GyroX int16 (0.1 deg/s)
        # 26-27: GyroY int16 (0.1 deg/s)
        # 28-29: GyroZ int16 (0.1 deg/s)
        # 30-31: AccX int16 (0.01 g)
        # 32-33: AccY int16 (0.01 g)
        # 34-35: AccZ int16 (0.01 g)
        # 36-37: Baro Altitude int16 (0.1 m)
        # 38-39: Battery uint16 (mV)
        # 40: FlightState uint8
        # 41: ErrorCode uint8
        # 42: WaterDetected uint8 (0/1)
        # 43: CRC8 XOR(0..42)
        if len(frame) != PACKET_LEN or frame[0] != 0x55 or frame[1] != FRAME_START:
            return

        try:
            pkt_type = frame[2]
            ts_ds = struct.unpack("<H", frame[3:5])[0]
            lat_raw = struct.unpack("<i", frame[5:9])[0]
            lon_raw = struct.unpack("<i", frame[9:13])[0]
            gps_alt_dm = struct.unpack("<h", frame[13:15])[0]
            gps_speed_dms = struct.unpack("<h", frame[15:17])[0]
            sat_count = frame[17]
            roll_cdeg = struct.unpack("<h", frame[18:20])[0]
            pitch_cdeg = struct.unpack("<h", frame[20:22])[0]
            yaw_ddeg = struct.unpack("<H", frame[22:24])[0]
            gyro_x_ddeg_s = struct.unpack("<h", frame[24:26])[0]
            gyro_y_ddeg_s = struct.unpack("<h", frame[26:28])[0]
            gyro_ddeg_s = struct.unpack("<h", frame[28:30])[0]
            accx_cg = struct.unpack("<h", frame[30:32])[0]
            accy_cg = struct.unpack("<h", frame[32:34])[0]
            accz_cg = struct.unpack("<h", frame[34:36])[0]
            baro_alt_dm = struct.unpack("<h", frame[36:38])[0]
            battery_mv = struct.unpack("<H", frame[38:40])[0]
            flight_state_raw = frame[40]
            error_code = frame[41]
            water_detected = frame[42]
        except Exception:
            return

        if pkt_type != 0x01:
            return

        crc_calc = 0
        for b in frame[:PACKET_LEN - 1]:
            crc_calc ^= b
        if crc_calc != frame[PACKET_LEN - 1]:
            return

        gps_alt_m = gps_alt_dm / 10.0
        gps_speed_ms = gps_speed_dms / 10.0
        baro_speed_ms = gps_speed_ms  # no separate baro speed provided
        heading_deg = (yaw_ddeg / 10.0) % 360
        roll_deg = roll_cdeg / 100.0
        pitch_deg = pitch_cdeg / 100.0
        gyro_z_dps = gyro_ddeg_s / 10.0
        gyro_x_dps = gyro_x_ddeg_s / 10.0
        gyro_y_dps = gyro_y_ddeg_s / 10.0
        battery_v = battery_mv / 1000.0
        baro_alt_m = baro_alt_dm / 10.0
        accx_g = accx_cg / 100.0
        accy_g = accy_cg / 100.0
        accz_g = accz_cg / 100.0

        if not self._legacy_link_logged:
            self.event_panel.add_event("Detected 44-byte telemetry frames.")
            self._legacy_link_logged = True

        t_sec = ts_ds / 10.0
        self._last_uptime = t_sec
        self._boot_time_seconds = t_sec

        # Flight state handling / mission start tracking
        try:
            from state_machine import FlightState
            fs = FlightState(flight_state_raw)
        except Exception:
            fs = None
        if fs != self._current_flight_state and fs is not None:
            self._current_flight_state = fs
            self._on_state_changed(fs)

        # Update UI components
        mission_elapsed = None
        if self._mission_start_uptime is not None:
            mission_elapsed = max(0.0, t_sec - self._mission_start_uptime)

        self.time_panel.update_time(mission_elapsed or 0.0)
        self.plot_panel.update_altitude(baro_alt_m)
        self.map_view.update_gps(lat_raw / 1e7, lon_raw / 1e7, gps_alt_m)
        self.battery_panel.update_voltage(battery_v)

        # Update telemetry model and refresh derived panels
        self.telemetry.update({
            "time": self._boot_time_seconds,
            "lat": lat_raw / 1e7,
            "lon": lon_raw / 1e7,
            "alt": gps_alt_m,
            "gps_alt": gps_alt_m,
            "baro_alt": baro_alt_m,
            "speed": gps_speed_ms,
            "gps_speed": gps_speed_ms,
            "baro_speed": baro_speed_ms,
            "heading": heading_deg,
            "accx": accx_g,
            "accy": accy_g,
            "accz": accz_g,
            "roll": roll_deg,
            "pitch": pitch_deg,
            "gyro_z": gyro_z_dps,
            "gyro_x": gyro_x_dps,
            "gyro_y": gyro_y_dps,
            "sat": sat_count,
            "battery": battery_v,
            "status": fs.value if fs is not None else flight_state_raw,
            "error": error_code,
            "water": water_detected,
        })

        mission_elapsed = None
        if self._mission_start_uptime is not None:
            mission_elapsed = max(0.0, t_sec - self._mission_start_uptime)
        self._refresh_telemetry()

        if self._logger:
            try:
                self._logger.log(self.telemetry)
            except Exception:
                pass

    # ---------------- Logging ----------------

    def _on_logging_toggled(self, enable: bool):
        if enable:
            try:
                self._logger = CSVLogger()
                self.event_panel.add_event("開始記錄 CSV")
                self.log_panel.set_active(True)
            except Exception as e:
                self.event_panel.add_event(f"記錄啟動失敗: {e}")
                self.log_panel.set_active(False)
        else:
            if self._logger:
                try:
                    self._logger.close()
                except Exception:
                    pass
            self._logger = None
            self.log_panel.set_active(False)
            self.event_panel.add_event("停止記錄 CSV")
