import re
import time
from dataclasses import dataclass
from pathlib import Path

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QComboBox,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from map_view import MapView
from serial_comm import SerialWorker
from serial_panel import SerialPanel


KEY_VALUE_RE = re.compile(r"([A-Za-z_]+)=([^ ]+)")
RECORD_DURATION_S = 5 * 60


@dataclass
class GpsDriftState:
    status: str = "---"
    ref_samples: str = "0/30"
    samples: int = 0
    drift_m: float | None = None
    east_m: float | None = None
    north_m: float | None = None
    avg_m: float | None = None
    rms_m: float | None = None
    max_m: float | None = None
    lat: float | None = None
    lon: float | None = None
    alt_m: float | None = None
    sat: int | None = None
    hdop: float | None = None
    chars: int | None = None
    sentences: int | None = None
    failed_checksum: int | None = None
    fix_age_ms: int | None = None
    last_byte_age_ms: int | None = None
    last_host_rx_s: float | None = None
    raw_line: str = ""


def _parse_float(value: str) -> float | None:
    value = value.rstrip("m")
    if value.lower() == "nan":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _parse_int(value: str) -> int | None:
    try:
        return int(value.rstrip("m"))
    except ValueError:
        return None


class MetricBox(QWidget):
    def __init__(self, title: str, unit: str = "", parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(2)

        self.title = QLabel(title)
        self.title.setStyleSheet("color: #666; font-size: 12px;")
        self.value = QLabel("--")
        self.value.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.value.setStyleSheet("font-size: 24px; font-weight: 700;")
        self.unit = QLabel(unit)
        self.unit.setStyleSheet("color: #777; font-size: 12px;")

        layout.addWidget(self.title)
        layout.addWidget(self.value)
        if unit:
            layout.addWidget(self.unit)

        self.setStyleSheet(
            "MetricBox { border: 1px solid #d8d8d8; border-radius: 6px; background: #fff; }"
        )

    def set_value(self, value: str):
        self.value.setText(value)


class GpsDriftWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GPS Drift Monitor")
        self.resize(1280, 820)

        self.serial_worker: SerialWorker | None = None
        self.state = GpsDriftState()
        self._line_buffer = bytearray()
        self._rx_bytes = 0
        self._parsed_lines = 0
        self._recording = False
        self._record_started_s: float | None = None
        self._record_file = None
        self._record_path: Path | None = None
        self._record_rows = 0

        self.serial_panel = SerialPanel()
        self.serial_panel.baud_box.setCurrentText("115200")
        self.serial_panel.baud_box.setEnabled(False)
        self.serial_panel.baud_box.setToolTip("固定電腦到 ESP 序列埠鮑率 / Fixed host-to-ESP baud rate")
        self.serial_panel.connect_requested.connect(self._start_serial)
        self.serial_panel.disconnect_requested.connect(self._stop_serial)

        self.gps_baud_box = QComboBox()
        self.gps_baud_box.addItems(["Auto scan baud + RX/TX pins"])
        self.gps_baud_box.setEnabled(False)
        self.gps_baud_box.setToolTip("韌體自動掃描 GPS 模組鮑率 / Firmware auto-scans GPS module baud rate")
        self.gps_baud_btn = QPushButton("套用 GPS Baud / Apply")
        self.gps_baud_btn.setEnabled(False)
        self.gps_baud_btn.setToolTip("GPS baud 由韌體自動掃描 / GPS baud is auto-scanned by firmware")
        self.gps_baud_btn.clicked.connect(self._apply_gps_baud)

        self.map_view = MapView()

        self.status = MetricBox("狀態 Status")
        self.drift = MetricBox("即時偏移 Drift", "m")
        self.avg = MetricBox("平均 Average", "m")
        self.rms = MetricBox("RMS 均方根", "m")
        self.max_drift = MetricBox("最大偏移 Max", "m")
        self.east = MetricBox("東向 East", "m")
        self.north = MetricBox("北向 North", "m")
        self.sat = MetricBox("衛星 Satellites")
        self.hdop = MetricBox("HDOP")
        self.lat = MetricBox("緯度 Latitude")
        self.lon = MetricBox("經度 Longitude")
        self.alt = MetricBox("海拔 Altitude", "m")
        self.ref = MetricBox("基準 Reference")
        self.fix_age = MetricBox("定位年齡 Fix Age", "ms")
        self.rx = MetricBox("序列接收 Serial RX", "bytes / lines")
        self.record_metric = MetricBox("紀錄 Recording", "rows / time")

        self.record_btn = QPushButton("開始 5 分鐘紀錄 / Start 5-min Record")
        self.record_btn.setEnabled(False)
        self.record_btn.clicked.connect(self._toggle_recording)
        self.reset_btn = QPushButton("重置 / Reset")
        self.reset_btn.clicked.connect(self._reset_view)
        self.gps_reset_btn = QPushButton("強制重設 GPS / Force GPS Reset")
        self.gps_reset_btn.clicked.connect(self._force_gps_reset)
        self.record_status = QLabel("等待定位 / Waiting for fix")
        self.record_status.setWordWrap(True)
        self.record_status.setStyleSheet("color: #666; font-weight: 600;")

        self.raw_log = QPlainTextEdit()
        self.raw_log.setReadOnly(True)
        self.raw_log.setMaximumBlockCount(300)

        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)
        root.addWidget(self.serial_panel)
        gps_baud_layout = QHBoxLayout()
        gps_baud_layout.addWidget(QLabel("GPS Baud / GPS 模組鮑率:"))
        gps_baud_layout.addWidget(self.gps_baud_box)
        gps_baud_layout.addWidget(self.gps_baud_btn)
        gps_baud_layout.addStretch(1)
        root.addLayout(gps_baud_layout)

        body = QHBoxLayout()
        body.setSpacing(10)
        root.addLayout(body, stretch=1)

        left = QVBoxLayout()
        body.addLayout(left, stretch=3)
        left.addWidget(self.map_view, stretch=1)

        right = QVBoxLayout()
        body.addLayout(right, stretch=2)

        grid = QGridLayout()
        grid.setSpacing(8)
        right.addLayout(grid)
        boxes = [
            self.status,
            self.drift,
            self.avg,
            self.rms,
            self.max_drift,
            self.east,
            self.north,
            self.sat,
            self.hdop,
            self.lat,
            self.lon,
            self.alt,
            self.ref,
            self.fix_age,
            self.rx,
            self.record_metric,
        ]
        for i, box in enumerate(boxes):
            grid.addWidget(box, i // 2, i % 2)

        controls = QHBoxLayout()
        controls.addWidget(self.record_btn)
        controls.addWidget(self.reset_btn)
        controls.addWidget(self.gps_reset_btn)
        right.addLayout(controls)
        right.addWidget(self.record_status)
        right.addWidget(QLabel("原始序列資料 / Raw Serial"))
        right.addWidget(self.raw_log, stretch=1)

        self.setCentralWidget(central)

        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._refresh_ui)
        self._ui_timer.start(250)
        self._refresh_ui()

    def _start_serial(self, port: str, baudrate: int):
        if self.serial_worker:
            return
        self._line_buffer.clear()
        self._rx_bytes = 0
        self._parsed_lines = 0
        self.state = GpsDriftState()
        self._stop_recording("serial reset")
        self.serial_worker = SerialWorker(port, baudrate)
        self.serial_worker.data_received.connect(self._on_serial_data)
        self.serial_worker.error_occurred.connect(self._on_serial_status)
        self.serial_worker.status_changed.connect(self._on_serial_status)
        self.serial_worker.start()

    def _stop_serial(self):
        if self.serial_worker:
            self.serial_worker.stop()
            self.serial_worker = None
        self.serial_panel.set_connected(False)
        self.serial_panel.set_status("Status: DISCONNECTED", color="red")
        self._stop_recording("serial disconnected")

    def _on_serial_status(self, message: str):
        msg_lower = message.lower()
        if "failed" in msg_lower or "error" in msg_lower or "disconnect" in msg_lower:
            self.serial_panel.set_connected(False)
            self.serial_panel.set_status(message, color="red")
        elif "connected" in msg_lower:
            self.serial_panel.set_connected(True)
            self.serial_panel.set_status(message, color="green")
        else:
            self.serial_panel.set_status(message)

    def _apply_gps_baud(self):
        self.record_status.setText("GPS baud 由韌體自動掃描 / GPS baud is auto-scanned by firmware")

    def _force_gps_reset(self):
        if not self.serial_worker:
            self.record_status.setText("請先連線 ESP32 / Connect ESP32 first")
            return
        self._stop_recording("gps reset")
        self.serial_worker.write(b"GPS_RESET\n")
        self._clear_measurement_state()
        self.record_status.setText("已送出 GPS 強制重設 / Sent GPS force reset")

    def _on_serial_data(self, raw: bytes):
        self._rx_bytes += len(raw)
        self._line_buffer.extend(raw)

        while b"\n" in self._line_buffer:
            line, _, rest = self._line_buffer.partition(b"\n")
            self._line_buffer = bytearray(rest)
            text = line.decode("utf-8", errors="replace").strip()
            if text:
                self._handle_line(text)

    def _handle_line(self, line: str):
        self.raw_log.appendPlainText(line)
        if line.startswith("#") or line.startswith("GPS ") or line.startswith("Reference "):
            return

        pairs = dict(KEY_VALUE_RE.findall(line))
        if not pairs:
            return

        self._parsed_lines += 1
        self.state.last_host_rx_s = time.monotonic()
        self.state.raw_line = line

        if "status" in pairs:
            self.state.status = pairs["status"]
        if "ref_samples" in pairs:
            self.state.ref_samples = pairs["ref_samples"]
        if "samples" in pairs:
            self.state.samples = _parse_int(pairs["samples"]) or self.state.samples
        if "drift" in pairs:
            self.state.drift_m = _parse_float(pairs["drift"])
        if "east" in pairs:
            self.state.east_m = _parse_float(pairs["east"])
        if "north" in pairs:
            self.state.north_m = _parse_float(pairs["north"])
        if "avg" in pairs:
            self.state.avg_m = _parse_float(pairs["avg"])
        if "rms" in pairs:
            self.state.rms_m = _parse_float(pairs["rms"])
        if "max" in pairs:
            self.state.max_m = _parse_float(pairs["max"])
        if "lat" in pairs:
            self.state.lat = _parse_float(pairs["lat"])
        if "lon" in pairs:
            self.state.lon = _parse_float(pairs["lon"])
        if "alt" in pairs:
            self.state.alt_m = _parse_float(pairs["alt"])
        if "sat" in pairs:
            self.state.sat = _parse_int(pairs["sat"])
        if "hdop" in pairs:
            self.state.hdop = _parse_float(pairs["hdop"])
        if "chars" in pairs:
            self.state.chars = _parse_int(pairs["chars"])
        if "sentences" in pairs:
            self.state.sentences = _parse_int(pairs["sentences"])
        if "failed_checksum" in pairs:
            self.state.failed_checksum = _parse_int(pairs["failed_checksum"])
        if "fix_age_ms" in pairs:
            self.state.fix_age_ms = _parse_int(pairs["fix_age_ms"])
        if "last_byte_age_ms" in pairs:
            self.state.last_byte_age_ms = _parse_int(pairs["last_byte_age_ms"])

        if self.state.lat is not None and self.state.lon is not None:
            self.map_view.update_gps(self.state.lat, self.state.lon, 0.0, zoom=19)

        self._record_state_if_needed()

    @staticmethod
    def _fmt_float(value: float | None, digits: int = 2) -> str:
        if value is None:
            return "--"
        return f"{value:.{digits}f}"

    @staticmethod
    def _fmt_int(value: int | None) -> str:
        if value is None:
            return "--"
        return str(value)

    def _refresh_ui(self):
        self.status.set_value(self.state.status)
        self.drift.set_value(self._fmt_float(self.state.drift_m, 3))
        self.avg.set_value(self._fmt_float(self.state.avg_m, 3))
        self.rms.set_value(self._fmt_float(self.state.rms_m, 3))
        self.max_drift.set_value(self._fmt_float(self.state.max_m, 3))
        self.east.set_value(self._fmt_float(self.state.east_m, 3))
        self.north.set_value(self._fmt_float(self.state.north_m, 3))
        self.sat.set_value(self._fmt_int(self.state.sat))
        self.hdop.set_value(self._fmt_float(self.state.hdop, 2))
        self.lat.set_value(self._fmt_float(self.state.lat, 9))
        self.lon.set_value(self._fmt_float(self.state.lon, 9))
        self.alt.set_value(self._fmt_float(self.state.alt_m, 2))
        self.ref.set_value(self.state.ref_samples)
        age = self.state.fix_age_ms if self.state.fix_age_ms is not None else self.state.last_byte_age_ms
        self.fix_age.set_value(self._fmt_int(age))
        self.rx.set_value(f"{self._rx_bytes} / {self._parsed_lines}")
        self.record_metric.set_value(self._record_metric_text())

        can_record = self.state.lat is not None and self.state.lon is not None
        self.record_btn.setEnabled(can_record or self._recording)
        if not self._recording and not can_record:
            self.record_status.setText("等待有效定位後才能開始紀錄 / Waiting for valid GPS fix")
        elif not self._recording:
            self.record_status.setText("可開始紀錄 5 分鐘偏移量 / Ready to record 5 minutes of drift")

        last_age_ms = None
        if self.state.last_host_rx_s is not None:
            last_age_ms = int((time.monotonic() - self.state.last_host_rx_s) * 1000)
        self.serial_panel.set_diagnostics(
            rx_bytes=self._rx_bytes,
            valid_frames=self._parsed_lines,
            crc_fail=0,
            resync_count=0,
            last_frame_age_ms=last_age_ms,
        )

    def closeEvent(self, event):
        self._stop_recording("window closing")
        if self.serial_worker:
            self.serial_worker.stop()
        event.accept()

    def _reset_view(self):
        self._stop_recording("reset")
        self._clear_measurement_state()
        self.record_status.setText("已重置，等待定位 / Reset, waiting for fix")
        self._refresh_ui()

    def _clear_measurement_state(self):
        self.state = GpsDriftState()
        self._line_buffer.clear()
        self._rx_bytes = 0
        self._parsed_lines = 0
        self._record_rows = 0
        self.raw_log.clear()
        self.map_view.reset_path()
        self.serial_panel.reset_diagnostics()
        self.record_btn.setText("開始 5 分鐘紀錄 / Start 5-min Record")
        self.record_btn.setEnabled(False)

    def _record_metric_text(self) -> str:
        if not self._recording or self._record_started_s is None:
            return f"{self._record_rows} / --"
        elapsed_s = max(0, int(time.monotonic() - self._record_started_s))
        remaining_s = max(0, RECORD_DURATION_S - elapsed_s)
        return f"{self._record_rows} / {remaining_s // 60:02d}:{remaining_s % 60:02d}"

    def _toggle_recording(self):
        if self._recording:
            self._stop_recording("user stopped")
        else:
            self._start_recording()

    def _start_recording(self):
        if self.state.lat is None or self.state.lon is None:
            self.record_status.setText("尚未定位，不能開始 / No valid fix yet")
            return

        default_name = f"gps_drift_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        path, _ = QFileDialog.getSaveFileName(
            self,
            "儲存 GPS 偏移紀錄 / Save GPS Drift Log",
            default_name,
            "Text Files (*.txt);;All Files (*)",
        )
        if not path:
            return

        self._clear_measurement_state()
        self._record_path = Path(path)
        self._record_file = self._record_path.open("w", encoding="utf-8")
        self._record_started_s = time.monotonic()
        self._record_rows = 0
        self._recording = True
        self.record_btn.setText("停止紀錄 / Stop Recording")
        self.record_status.setText(f"紀錄中 / Recording: {self._record_path}")
        self._record_file.write("# GPS drift 5-minute recording\n")
        self._record_file.write(f"# start_unix={int(time.time())}\n")
        self._record_file.write(
            "host_elapsed_s,status,samples,lat,lon,alt_m,sat,hdop,drift_m,east_m,north_m,"
            "avg_m,rms_m,max_m,fix_age_ms,raw\n"
        )
        self._record_file.flush()
        self._record_state_if_needed()

    def _stop_recording(self, reason: str):
        if not self._recording:
            return
        if self._record_file:
            self._record_file.write(f"# stop_unix={int(time.time())} reason={reason}\n")
            self._record_file.close()
        self._record_file = None
        self._recording = False
        self.record_btn.setText("開始 5 分鐘紀錄 / Start 5-min Record")
        if self._record_path:
            self.record_status.setText(f"紀錄完成 / Saved: {self._record_path}")

    def _record_state_if_needed(self):
        if not self._recording or self._record_file is None or self._record_started_s is None:
            return

        elapsed_s = time.monotonic() - self._record_started_s
        if elapsed_s >= RECORD_DURATION_S:
            self._stop_recording("5 minutes completed")
            return

        if self.state.lat is None or self.state.lon is None:
            return

        def f(value, digits=3):
            if value is None:
                return ""
            if isinstance(value, float):
                return f"{value:.{digits}f}"
            return str(value)

        raw = self.state.raw_line.replace("\t", " ").replace("\n", " ")
        self._record_file.write(
            f"{elapsed_s:.3f},{self.state.status},{self.state.samples},"
            f"{f(self.state.lat, 9)},{f(self.state.lon, 9)},"
            f"{f(self.state.alt_m, 2)},"
            f"{f(self.state.sat)},{f(self.state.hdop, 2)},"
            f"{f(self.state.drift_m)},{f(self.state.east_m)},{f(self.state.north_m)},"
            f"{f(self.state.avg_m)},{f(self.state.rms_m)},{f(self.state.max_m)},"
            f"{f(self.state.fix_age_ms)},\"{raw}\"\n"
        )
        self._record_file.flush()
        self._record_rows += 1
