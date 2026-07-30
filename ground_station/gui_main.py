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
from PyQt5.QtCore import Qt, QTimer

from serial_comm import SerialWorker
from state_machine import FlightState
from logger import CSVLogger
from protocol import FRAME_SYNC, TelemetryStreamParser, decode_frame

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
        self._mission_start_uptime = None
        self._last_uptime = 0.0
        self._boot_time_seconds = 0.0
        self._current_flight_state = None
        self._logger = None
        self._reset_frame_state()
        self.telemetry = {
            "time": 0.0,
            "lat": 0.0,
            "lon": 0.0,
            "alt": 0.0,
            "gps_alt": 0.0,
            "baro_alt": None,
            "speed": 0.0,
            "gps_speed": 0.0,
            "baro_speed": None,
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
            "servo_power": 0.0,
            "servo_angle": 0.0,
            "battery": 0.0,
            "status": 0,
            "water": 0,
        }

        # ---- Serial worker (created on demand) ----
        self.serial_worker = None

        # ---- UI ----
        self._init_ui()
        self._diag_timer = QTimer(self)
        self._diag_timer.setInterval(250)
        self._diag_timer.timeout.connect(self._update_stream_diagnostics)
        self._diag_timer.start()
        self._update_stream_diagnostics()
        # Fixed-length telemetry decoding (47-byte frames).

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

        self._reset_frame_state()
        self.serial_worker = SerialWorker(port, baudrate)
        self.serial_worker.data_received.connect(self._on_serial_data)
        self.serial_worker.error_occurred.connect(self._on_serial_status)
        self.serial_worker.status_changed.connect(self._on_serial_status)
        self.serial_worker.start()

    def _stop_serial(self):
        if self.serial_worker:
            self.serial_worker.stop()
            self.serial_worker = None
        self._reset_frame_state()
        self.serial_panel.set_status("Status: DISCONNECTED", color="red")

    def _on_serial_data(self, raw: bytes):
        if raw:
            if self._first_rx_monotonic is None:
                self._first_rx_monotonic = time.monotonic()
            self._rx_bytes += len(raw)
        self._maybe_log_non_telemetry_serial(raw)
        self._process_legacy_stream(raw)
        self._update_stream_diagnostics()

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
            self.serial_panel.set_connected(False)
            self.serial_panel.set_status(message, color="red")
        elif "connected" in msg_lower:
            self.serial_panel.set_connected(True)
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

    # ---------------- Fixed-length frames (47 bytes) ----------------

    def _reset_frame_state(self):
        self._frame_parser = TelemetryStreamParser()
        self._legacy_link_logged = False
        self._saw_hb_text = False
        self._boot_markers_seen = set()
        self._text_serial_logged = False
        self._rx_bytes = 0
        self._valid_frames = 0
        self._crc_fail = 0
        self._resync_count = 0
        self._first_rx_monotonic = None
        self._last_frame_monotonic = None
        self._non_telemetry_warning_logged = False
        if hasattr(self, "serial_panel"):
            self.serial_panel.reset_diagnostics()

    def _maybe_log_non_telemetry_serial(self, raw: bytes):
        if not raw or self._legacy_link_logged:
            return

        text = raw.decode("ascii", errors="ignore")
        if text:
            for line in text.splitlines():
                marker = line.strip()
                if marker.startswith("BOOT:") and marker not in self._boot_markers_seen:
                    self.event_panel.add_event(f"[FW] {marker}")
                    self._boot_markers_seen.add(marker)

        if b"HB\n" in raw and FRAME_SYNC not in raw:
            self._saw_hb_text = True

        if self._text_serial_logged:
            return

        printable = sum(1 for b in raw if 32 <= b <= 126 or b in (9, 10, 13))
        if (
            printable == len(raw)
            and FRAME_SYNC not in raw
            and "BOOT:" not in text
            and "HB" not in text
        ):
            self.event_panel.add_event(
                "收到的是文字序列資料，不是 47-byte 遙測 frame。"
            )
            self._text_serial_logged = True

    def _process_legacy_stream(self, data: bytes):
        frames = self._frame_parser.feed(data)
        self._crc_fail = self._frame_parser.total_crc_fail
        self._resync_count = self._frame_parser.total_resync_count
        if frames:
            self._valid_frames += len(frames)
            self._last_frame_monotonic = time.monotonic()
        for frame in frames:
            self._handle_legacy_frame(decode_frame(frame))

    def _update_stream_diagnostics(self):
        now = time.monotonic()
        last_frame_age_ms = None
        if self._last_frame_monotonic is not None:
            last_frame_age_ms = int((now - self._last_frame_monotonic) * 1000.0)

        self.serial_panel.set_diagnostics(
            rx_bytes=self._rx_bytes,
            valid_frames=self._valid_frames,
            crc_fail=self._crc_fail,
            resync_count=self._resync_count,
            last_frame_age_ms=last_frame_age_ms,
        )

        if (
            self._rx_bytes > 0
            and self._valid_frames == 0
            and self._first_rx_monotonic is not None
            and (now - self._first_rx_monotonic) >= 2.0
        ):
            if self._saw_hb_text:
                msg = "收到 HB/BOOT 回應，但尚未收到 47-byte 遙測（尚未進入 loop 或 port/baud 不符）"
            else:
                msg = "收到資料但非 47-byte 遙測（port/baud 不符）"
            self.serial_panel.set_diagnostic_notice(msg, color="#a35f00")
            if not self._non_telemetry_warning_logged:
                self.event_panel.add_event(msg)
                self._non_telemetry_warning_logged = True
            return

        if self._valid_frames > 0:
            self.serial_panel.set_diagnostic_notice("47-byte 遙測正常接收中。", color="green")
        elif self._rx_bytes > 0:
            self.serial_panel.set_diagnostic_notice("已收到資料，正在等待有效 47-byte frame...", color="#666")
        else:
            self.serial_panel.set_diagnostic_notice("")

    def _handle_legacy_frame(self, packet):
        gps_speed_ms = packet.gps_speed_ms
        baro_alt_m = packet.baro_alt_m
        baro_speed_ms = gps_speed_ms if baro_alt_m is not None else None
        battery_v = packet.battery_v
        primary_alt_m = baro_alt_m if baro_alt_m is not None else packet.gps_alt_m

        if not self._legacy_link_logged:
            self.event_panel.add_event("偵測到 47-byte 遙測 frame。")
            self._legacy_link_logged = True

        t_sec = packet.time_s
        self._last_uptime = t_sec
        self._boot_time_seconds = t_sec

        # Flight state handling / mission start tracking
        try:
            fs = FlightState(packet.flight_state)
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
        self.plot_panel.update_altitude(primary_alt_m)
        self.map_view.update_gps(packet.lat_deg, packet.lon_deg, packet.gps_alt_m)
        self.battery_panel.update_voltage(battery_v)

        # Update telemetry model and refresh derived panels
        self.telemetry.update({
            "time": self._boot_time_seconds,
            "lat": packet.lat_deg,
            "lon": packet.lon_deg,
            "alt": primary_alt_m,
            "gps_alt": packet.gps_alt_m,
            "baro_alt": baro_alt_m,
            "speed": gps_speed_ms,
            "gps_speed": gps_speed_ms,
            "baro_speed": baro_speed_ms,
            "heading": packet.heading_deg,
            "accx": packet.accx_g,
            "accy": packet.accy_g,
            "accz": packet.accz_g,
            "roll": packet.roll_deg,
            "pitch": packet.pitch_deg,
            "gyro_z": packet.gyro_z_dps,
            "gyro_x": packet.gyro_x_dps,
            "gyro_y": packet.gyro_y_dps,
            "servo_power": packet.servo_power_v,
            "servo_angle": packet.servo_angle_deg,
            "sat": packet.sat_count,
            "battery": battery_v,
            "status": fs.value if fs is not None else packet.flight_state,
            "error": packet.error_code,
            "water": packet.water_detected,
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
