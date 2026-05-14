import csv
import math
import re
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
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

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from serial_comm import SerialWorker
from serial_panel import SerialPanel


KEY_VALUE_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^ ]+)")
CSV_FIELDS = [
    "time_us",
    "sample",
    "baseline_ready",
    "temp_c",
    "pressure_raw_pa",
    "pressure_filt_pa",
    "alt_raw_m",
    "alt_filt_m",
    "vz_raw_mps",
    "vz_filt_mps",
    "baseline_pressure_pa",
    "failed_reads",
]
LEGACY_CSV_FIELDS = [
    "time_us",
    "sample",
    "baseline_ready",
    "temp_c",
    "pressure_pa",
    "alt_raw_m",
    "alt_filt_m",
    "vz_mps",
    "min_m",
    "max_m",
    "span_m",
    "baseline_pressure_pa",
    "failed_reads",
]
IMU_CSV_FIELDS = [
    "imu_time_us",
    "imu_sample",
    "imu_temp_c",
    "accel_x_g",
    "accel_y_g",
    "accel_z_g",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "imu_failed_reads",
]


@dataclass
class BaroState:
    status: str = "---"
    reason: str = ""
    time_us: int | None = None
    sample: int = 0
    baseline_ready: bool = False
    baseline_text: str = "0/100"
    temp_c: float | None = None
    pressure_raw_pa: float | None = None
    pressure_filt_pa: float | None = None
    pressure_pa: float | None = None
    alt_raw_m: float | None = None
    alt_filt_m: float | None = None
    vz_raw_mps: float | None = None
    vz_filt_mps: float | None = None
    vz_mps: float | None = None
    min_m: float | None = None
    max_m: float | None = None
    span_m: float | None = None
    baseline_pressure_pa: float | None = None
    failed_reads: int = 0
    i2c_count: int | None = None
    icm42688: str = "---"
    icm_addr: str = "--"
    icm_whoami: str = "--"
    imu_time_us: int | None = None
    imu_sample: int = 0
    imu_temp_c: float | None = None
    accel_x_g: float | None = None
    accel_y_g: float | None = None
    accel_z_g: float | None = None
    imu_accel_vertical_mps2: float | None = None
    imu_vz_mps: float | None = None
    gyro_x_dps: float | None = None
    gyro_y_dps: float | None = None
    gyro_z_dps: float | None = None
    imu_failed_reads: int = 0
    uptime_ms: int | None = None
    last_host_rx_s: float | None = None
    parse_note: str = ""
    raw_line: str = ""


def _parse_float(value: str) -> float | None:
    if value == "" or value.lower() == "nan":
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def _parse_int(value: str) -> int | None:
    try:
        return int(value)
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
        self.value.setStyleSheet("font-size: 22px; font-weight: 700;")
        self.unit = QLabel(unit)
        self.unit.setStyleSheet("color: #777; font-size: 12px;")

        layout.addWidget(self.title)
        layout.addWidget(self.value)
        if unit:
            layout.addWidget(self.unit)

        self.setStyleSheet(
            "MetricBox { border: 1px solid #d7d7d7; border-radius: 6px; background: #fafafa; }"
        )

    def set_value(self, text: str):
        self.value.setText(text)


class BaroPlot(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(7, 4), tight_layout=True)
        super().__init__(self.fig)
        self.setParent(parent)
        self.ax_alt = self.fig.add_subplot(211)
        self.ax_vz = self.fig.add_subplot(212, sharex=self.ax_alt)
        self.t = deque(maxlen=1500)
        self.alt = deque(maxlen=1500)
        self.raw_alt = deque(maxlen=1500)
        self.vz = deque(maxlen=1500)
        self._style_axes()

    def _style_axes(self):
        self.ax_alt.set_ylabel("Altitude / 高度 (m)")
        self.ax_vz.set_ylabel("Vz (m/s)")
        self.ax_vz.set_xlabel("Time / 時間 (s)")
        for ax in (self.ax_alt, self.ax_vz):
            ax.grid(True, color="#dddddd", linewidth=0.8)

    def reset(self):
        self.t.clear()
        self.alt.clear()
        self.raw_alt.clear()
        self.vz.clear()
        self.redraw()

    def append(self, time_s: float, alt_filt: float | None, alt_raw: float | None, vz: float | None):
        if alt_filt is None and alt_raw is None and vz is None:
            return
        self.t.append(time_s)
        self.alt.append(float("nan") if alt_filt is None else alt_filt)
        self.raw_alt.append(float("nan") if alt_raw is None else alt_raw)
        self.vz.append(float("nan") if vz is None else vz)

    def redraw(self):
        self.ax_alt.clear()
        self.ax_vz.clear()
        self._style_axes()
        if self.t:
            t0 = self.t[0]
            xs = [x - t0 for x in self.t]
            self.ax_alt.plot(xs, list(self.alt), color="#1565c0", linewidth=1.6, label="Filtered")
            self.ax_alt.plot(xs, list(self.raw_alt), color="#9e9e9e", linewidth=0.9, alpha=0.75, label="Raw")
            self.ax_alt.legend(loc="upper right")
            self.ax_vz.plot(xs, list(self.vz), color="#c62828", linewidth=1.2)
        self.draw_idle()


class BaroAltitudeWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Barometric Altitude Test")
        self.resize(1220, 780)

        self.serial_worker: SerialWorker | None = None
        self.state = BaroState()
        self._line_buffer = bytearray()
        self._rx_bytes = 0
        self._parsed_samples = 0
        self._parsed_imu_samples = 0
        self._parsed_status = 0
        self._recording = False
        self._record_file = None
        self._record_path: Path | None = None
        self._record_rows = 0
        self._record_start_s: float | None = None
        self._last_sample_count = 0
        self._last_imu_sample_count = 0
        self._last_imu_time_us: int | None = None
        self._last_rate_time_s = time.monotonic()
        self._sample_rate_hz = 0.0
        self._imu_rate_hz = 0.0
        self._imu_vz_mps = 0.0

        self.serial_panel = SerialPanel()
        self.serial_panel.baud_box.setCurrentText("115200")
        self.serial_panel.connect_requested.connect(self._start_serial)
        self.serial_panel.disconnect_requested.connect(self._stop_serial)

        self.status = MetricBox("狀態 Status")
        self.reason = MetricBox("診斷 Reason")
        self.sample = MetricBox("樣本 Samples")
        self.rate = MetricBox("採樣率 Sample Rate", "Hz")
        self.temp = MetricBox("溫度 Temperature", "deg C")
        self.pressure = MetricBox("氣壓 Pressure", "Pa")
        self.alt_raw = MetricBox("原始高度 Raw Alt", "m")
        self.alt_filt = MetricBox("濾波高度 Filtered Alt", "m")
        self.vz = MetricBox("垂直速度 Vz", "m/s")
        self.span = MetricBox("飄移範圍 Span", "m")
        self.minmax = MetricBox("最小/最大 Min/Max", "m")
        self.baseline = MetricBox("基準 Baseline")
        self.failed = MetricBox("讀取失敗 Failed Reads")
        self.imu_status = MetricBox("IMU 狀態 IMU Status")
        self.imu_addr = MetricBox("IMU 位址 ICM Addr")
        self.imu_whoami = MetricBox("IMU WHO_AM_I")
        self.imu_sample = MetricBox("IMU 樣本 IMU Samples")
        self.imu_rate = MetricBox("IMU 採樣率 IMU Rate", "Hz")
        self.imu_temp = MetricBox("IMU 溫度 IMU Temp", "deg C")
        self.imu_accel = MetricBox("加速度 Accel XYZ", "g")
        self.imu_gyro = MetricBox("角速度 Gyro XYZ", "deg/s")
        self.imu_failed = MetricBox("IMU 讀取失敗 IMU Failed")
        self.rx = MetricBox("序列接收 Serial RX", "bytes / lines")
        self.record_metric = MetricBox("紀錄 Recording", "rows")

        self.plot = BaroPlot()
        self.raw_log = QPlainTextEdit()
        self.raw_log.setReadOnly(True)
        self.raw_log.setMaximumBlockCount(400)

        self.record_btn = QPushButton("開始紀錄 / Start Log")
        self.record_btn.clicked.connect(self._toggle_recording)
        self.reset_btn = QPushButton("重置圖表 / Reset View")
        self.reset_btn.clicked.connect(self._reset_view)
        self.zero_btn = QPushButton("歸零 / Zero")
        self.zero_btn.clicked.connect(self._request_zero)
        self.calibrate_btn = QPushButton("校正 / Calibrate")
        self.calibrate_btn.clicked.connect(self._request_calibrate)
        self.imu_check_btn = QPushButton("IMU 檢查 / IMU Check")
        self.imu_check_btn.clicked.connect(self._request_imu_check)
        self.status_label = QLabel("等待連線 / Waiting for connection")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("color: #666; font-weight: 600;")

        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)
        root.addWidget(self.serial_panel)

        boxes = [
            self.status,
            self.reason,
            self.sample,
            self.rate,
            self.temp,
            self.pressure,
            self.alt_raw,
            self.alt_filt,
            self.vz,
            self.span,
            self.minmax,
            self.baseline,
            self.failed,
            self.imu_status,
            self.imu_addr,
            self.imu_whoami,
            self.imu_sample,
            self.imu_rate,
            self.imu_temp,
            self.imu_accel,
            self.imu_gyro,
            self.imu_failed,
            self.rx,
            self.record_metric,
        ]
        grid = QGridLayout()
        grid.setSpacing(8)
        for i, box in enumerate(boxes):
            row = i % 3
            col = i // 3
            grid.addWidget(box, row, col)
        root.addLayout(grid)

        body = QHBoxLayout()
        root.addLayout(body, stretch=1)
        body.addWidget(self.plot, stretch=3)

        side = QVBoxLayout()
        body.addLayout(side, stretch=1)

        controls = QHBoxLayout()
        controls.addWidget(self.record_btn)
        controls.addWidget(self.reset_btn)
        controls.addWidget(self.zero_btn)
        controls.addWidget(self.calibrate_btn)
        controls.addWidget(self.imu_check_btn)
        side.addLayout(controls)
        side.addWidget(self.status_label)
        side.addWidget(QLabel("原始序列資料 / Raw Serial"))
        side.addWidget(self.raw_log, stretch=1)

        self.setCentralWidget(central)

        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._refresh_ui)
        self._ui_timer.start(250)

        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self.plot.redraw)
        self._plot_timer.start(500)
        self._refresh_ui()

    def _start_serial(self, port: str, baudrate: int):
        if self.serial_worker:
            return
        self._line_buffer.clear()
        self._rx_bytes = 0
        self._parsed_samples = 0
        self._parsed_imu_samples = 0
        self._parsed_status = 0
        self.state = BaroState()
        self._last_imu_time_us = None
        self._imu_vz_mps = 0.0
        self.plot.reset()
        self.serial_worker = SerialWorker(port, baudrate)
        self.serial_worker.data_received.connect(self._on_serial_data)
        self.serial_worker.error_occurred.connect(self._on_serial_status)
        self.serial_worker.status_changed.connect(self._on_serial_status)
        self.serial_worker.start()
        self.status_label.setText(f"連線中 / Connecting: {port} @ {baudrate}")

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

    def _request_zero(self):
        if not self.serial_worker:
            self.status_label.setText("請先連線 ESP32 / Connect ESP32 first")
            return
        self.serial_worker.write(b"BARO_ZERO\n")
        self.plot.reset()
        self.status_label.setText("已送出歸零指令 / Sent zero command")

    def _request_calibrate(self):
        if not self.serial_worker:
            self.status_label.setText("請先連線 ESP32 / Connect ESP32 first")
            return
        self.serial_worker.write(b"BARO_CALIBRATE\n")
        self.plot.reset()
        self.state.baseline_ready = False
        self.state.baseline_text = "0/100"
        self.status_label.setText("已送出校正指令 / Sent calibration command")

    def _request_imu_check(self):
        if not self.serial_worker:
            self.status_label.setText("請先連線 ESP32 / Connect ESP32 first")
            return
        self.serial_worker.write(b"ICM_CHECK\n")
        self.status_label.setText("已送出 IMU 檢查指令 / Sent IMU check command")

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
        self.state.raw_line = line
        self.state.last_host_rx_s = time.monotonic()
        if self._recording and self._record_file:
            self._record_selected_line(line)

        if line.startswith("#"):
            self._handle_comment(line)
            return
        self._handle_csv(line)

    def _record_prefix(self) -> list[str]:
        now = datetime.now().isoformat(timespec="milliseconds")
        elapsed_s = 0.0 if self._record_start_s is None else time.monotonic() - self._record_start_s
        return [now, f"{elapsed_s:.3f}"]

    def _write_record_row(self, values: list[str]):
        if not self._record_file:
            return
        self._record_file.write(",".join(values) + "\n")
        self._record_file.flush()
        self._record_rows += 1

    def _record_selected_line(self, line: str):
        if line.startswith("#"):
            return

        try:
            parts = next(csv.reader([line]))
        except csv.Error:
            return

        if len(parts) == len(CSV_FIELDS) and parts[0] != "time_us":
            data = dict(zip(CSV_FIELDS, parts))
            self._write_record_row(
                self._record_prefix()
                + [
                    "BMP388",
                    data["time_us"],
                    data["sample"],
                    data["temp_c"],
                    data["pressure_raw_pa"],
                    data["alt_raw_m"],
                    data["vz_raw_mps"],
                    data["pressure_filt_pa"],
                    data["alt_filt_m"],
                    data["vz_filt_mps"],
                    data["baseline_pressure_pa"],
                    "",
                    "",
                ]
            )
            return
        if len(parts) == len(LEGACY_CSV_FIELDS) and parts[0] != "time_us":
            data = dict(zip(LEGACY_CSV_FIELDS, parts))
            self._write_record_row(
                self._record_prefix()
                + [
                    "BMP388",
                    data["time_us"],
                    data["sample"],
                    data["temp_c"],
                    data["pressure_pa"],
                    data["alt_raw_m"],
                    data["vz_mps"],
                    data["pressure_pa"],
                    data["alt_filt_m"],
                    data["vz_mps"],
                    data["baseline_pressure_pa"],
                    "",
                    "",
                ]
            )
            return

        if len(parts) == len(IMU_CSV_FIELDS) and parts[0] != "imu_time_us":
            data = dict(zip(IMU_CSV_FIELDS, parts))
            imu_vz_mps, imu_accel_vertical_mps2 = self._estimate_imu_vertical_motion(data)
            self._write_record_row(
                self._record_prefix()
                + [
                    "ICM42688",
                    data["imu_time_us"],
                    data["imu_sample"],
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    "",
                    self._record_float(imu_vz_mps, 5),
                    self._record_float(imu_accel_vertical_mps2, 5),
                ]
            )

    @staticmethod
    def _record_float(value: float | None, digits: int) -> str:
        if value is None or not math.isfinite(value):
            return ""
        return f"{value:.{digits}f}"

    def _estimate_imu_vertical_motion(self, data: dict[str, str]) -> tuple[float | None, float | None]:
        time_us = _parse_int(data.get("imu_time_us", ""))
        accel_z_g = _parse_float(data.get("accel_z_g", ""))
        if time_us is None or accel_z_g is None:
            return None, None

        vertical_accel_mps2 = (accel_z_g - 1.0) * 9.80665
        if self._last_imu_time_us is not None and time_us > self._last_imu_time_us:
            dt = (time_us - self._last_imu_time_us) / 1_000_000.0
            self._imu_vz_mps += vertical_accel_mps2 * dt
        self._last_imu_time_us = time_us
        return self._imu_vz_mps, vertical_accel_mps2

    def _handle_comment(self, line: str):
        pairs = dict(KEY_VALUE_RE.findall(line))
        if "status" in pairs:
            self.state.status = pairs["status"]
            self.state.reason = pairs.get("reason", self.state.reason)
            baseline_now = pairs.get("baseline")
            if baseline_now:
                self.state.baseline_text = baseline_now
            failed = _parse_int(pairs.get("failed_reads", ""))
            if failed is not None:
                self.state.failed_reads = failed
            i2c_count = _parse_int(pairs.get("i2c_count", ""))
            if i2c_count is not None:
                self.state.i2c_count = i2c_count
            if "icm42688" in pairs:
                self.state.icm42688 = pairs["icm42688"]
            if "icm_addr" in pairs:
                self.state.icm_addr = pairs["icm_addr"]
            if "icm_whoami" in pairs:
                self.state.icm_whoami = pairs["icm_whoami"]
            imu_samples = _parse_int(pairs.get("imu_samples", ""))
            if imu_samples is not None:
                self.state.imu_sample = imu_samples
            imu_failed = _parse_int(pairs.get("imu_failed_reads", ""))
            if imu_failed is not None:
                self.state.imu_failed_reads = imu_failed
            uptime = _parse_int(pairs.get("uptime_ms", ""))
            if uptime is not None:
                self.state.uptime_ms = uptime
            self._parsed_status += 1
        elif "icm42688_verify" in pairs:
            verify = pairs["icm42688_verify"]
            if verify == "ready":
                self.state.icm42688 = "ready"
                self.state.icm_addr = pairs.get("addr", self.state.icm_addr)
                self.state.icm_whoami = pairs.get("whoami", self.state.icm_whoami)
            else:
                self.state.icm42688 = verify
                self.state.icm_addr = pairs.get("addr", self.state.icm_addr)
                self.state.icm_whoami = pairs.get("whoami", self.state.icm_whoami)
            self._parsed_status += 1
        elif "baseline_locked" in line:
            pairs = dict(KEY_VALUE_RE.findall(line))
            self.state.baseline_pressure_pa = _parse_float(pairs.get("pressure_pa", ""))
            self.state.baseline_ready = True
            self.state.status = "TRACK"
        elif "baro_zeroed" in line:
            self.state.baseline_pressure_pa = _parse_float(pairs.get("pressure_pa", ""))
            self.state.baseline_ready = True
            self.state.baseline_text = "100/100"
            self.state.status = "ZEROED"
        elif "baro_calibration_started" in line:
            self.state.baseline_ready = False
            self.state.baseline_text = f"0/{pairs.get('samples', '100')}"
            self.state.status = "CALIBRATING"

    def _handle_csv(self, line: str):
        parts = next(csv.reader([line]))
        if not parts:
            return
        if parts[0] == "imu_time_us":
            return
        if parts[0] == "time_us":
            return
        if parts[0].isdigit() and len(parts) == len(CSV_FIELDS):
            data = dict(zip(CSV_FIELDS, parts))
        elif parts[0].isdigit() and len(parts) == len(LEGACY_CSV_FIELDS):
            legacy = dict(zip(LEGACY_CSV_FIELDS, parts))
            data = {
                "time_us": legacy["time_us"],
                "sample": legacy["sample"],
                "baseline_ready": legacy["baseline_ready"],
                "temp_c": legacy["temp_c"],
                "pressure_raw_pa": legacy["pressure_pa"],
                "pressure_filt_pa": legacy["pressure_pa"],
                "alt_raw_m": legacy["alt_raw_m"],
                "alt_filt_m": legacy["alt_filt_m"],
                "vz_raw_mps": legacy["vz_mps"],
                "vz_filt_mps": legacy["vz_mps"],
                "baseline_pressure_pa": legacy["baseline_pressure_pa"],
                "failed_reads": legacy["failed_reads"],
            }
        elif parts[0].isdigit() and len(parts) == len(IMU_CSV_FIELDS):
            self._handle_imu_csv(parts)
            return
        else:
            self.state.parse_note = f"ignored csv fields={len(parts)} first={parts[0][:16]}"
            return
        time_us = _parse_int(data["time_us"])
        sample = _parse_int(data["sample"])
        if time_us is None or sample is None:
            return

        self.state.time_us = time_us
        self.state.sample = sample
        self.state.baseline_ready = data["baseline_ready"] == "1"
        self.state.status = "TRACK" if self.state.baseline_ready else "CALIBRATING"
        self.state.temp_c = _parse_float(data["temp_c"])
        self.state.pressure_raw_pa = _parse_float(data["pressure_raw_pa"])
        self.state.pressure_filt_pa = _parse_float(data["pressure_filt_pa"])
        self.state.pressure_pa = self.state.pressure_filt_pa
        self.state.alt_raw_m = _parse_float(data["alt_raw_m"])
        self.state.alt_filt_m = _parse_float(data["alt_filt_m"])
        self.state.vz_raw_mps = _parse_float(data["vz_raw_mps"])
        self.state.vz_filt_mps = _parse_float(data["vz_filt_mps"])
        self.state.vz_mps = self.state.vz_filt_mps
        self.state.baseline_pressure_pa = _parse_float(data["baseline_pressure_pa"])
        self.state.failed_reads = _parse_int(data["failed_reads"]) or self.state.failed_reads
        self.state.parse_note = "BMP CSV OK"
        self._parsed_samples += 1
        self.plot.append(time_us / 1_000_000.0, self.state.alt_filt_m, self.state.alt_raw_m, self.state.vz_mps)

    def _handle_imu_csv(self, parts: list[str]):
        data = dict(zip(IMU_CSV_FIELDS, parts))
        time_us = _parse_int(data["imu_time_us"])
        sample = _parse_int(data["imu_sample"])
        if time_us is None or sample is None:
            return

        self.state.imu_time_us = time_us
        self.state.imu_sample = sample
        self.state.imu_temp_c = _parse_float(data["imu_temp_c"])
        self.state.accel_x_g = _parse_float(data["accel_x_g"])
        self.state.accel_y_g = _parse_float(data["accel_y_g"])
        self.state.accel_z_g = _parse_float(data["accel_z_g"])
        if not self._recording:
            self._estimate_imu_vertical_motion(data)
        if self.state.accel_z_g is not None:
            self.state.imu_accel_vertical_mps2 = (self.state.accel_z_g - 1.0) * 9.80665
            self.state.imu_vz_mps = self._imu_vz_mps
        self.state.gyro_x_dps = _parse_float(data["gyro_x_dps"])
        self.state.gyro_y_dps = _parse_float(data["gyro_y_dps"])
        self.state.gyro_z_dps = _parse_float(data["gyro_z_dps"])
        self.state.imu_failed_reads = _parse_int(data["imu_failed_reads"]) or self.state.imu_failed_reads
        self.state.parse_note = "IMU CSV OK"
        self._parsed_imu_samples += 1

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

    @classmethod
    def _fmt_xyz(cls, x: float | None, y: float | None, z: float | None, digits: int = 2) -> str:
        if x is None or y is None or z is None:
            return "--"
        return f"{x:.{digits}f} / {y:.{digits}f} / {z:.{digits}f}"

    def _refresh_ui(self):
        now = time.monotonic()
        if now - self._last_rate_time_s >= 1.0:
            dt = now - self._last_rate_time_s
            ds = self.state.sample - self._last_sample_count
            dis = self.state.imu_sample - self._last_imu_sample_count
            self._sample_rate_hz = ds / dt if dt > 0 else 0.0
            self._imu_rate_hz = dis / dt if dt > 0 else 0.0
            self._last_sample_count = self.state.sample
            self._last_imu_sample_count = self.state.imu_sample
            self._last_rate_time_s = now

        self.status.set_value(self.state.status)
        self.reason.set_value(self.state.reason or "--")
        self.sample.set_value(str(self.state.sample))
        self.rate.set_value(f"{self._sample_rate_hz:.1f}")
        self.temp.set_value(self._fmt_float(self.state.temp_c, 3))
        self.pressure.set_value(self._fmt_float(self.state.pressure_pa, 2))
        self.alt_raw.set_value(self._fmt_float(self.state.alt_raw_m, 3))
        self.alt_filt.set_value(self._fmt_float(self.state.alt_filt_m, 3))
        self.vz.set_value(self._fmt_float(self.state.vz_mps, 3))
        self.span.set_value(self._fmt_float(self.state.span_m, 3))
        if self.state.min_m is None or self.state.max_m is None:
            self.minmax.set_value("--")
        else:
            self.minmax.set_value(f"{self.state.min_m:.3f} / {self.state.max_m:.3f}")
        if self.state.baseline_ready:
            self.baseline.set_value(self._fmt_float(self.state.baseline_pressure_pa, 2))
        else:
            self.baseline.set_value(self.state.baseline_text)
        self.failed.set_value(str(self.state.failed_reads))
        self.imu_status.set_value(self.state.icm42688)
        self.imu_addr.set_value(self.state.icm_addr)
        self.imu_whoami.set_value(self.state.icm_whoami)
        self.imu_sample.set_value(str(self.state.imu_sample))
        self.imu_rate.set_value(f"{self._imu_rate_hz:.1f}")
        self.imu_temp.set_value(self._fmt_float(self.state.imu_temp_c, 3))
        self.imu_accel.set_value(
            self._fmt_xyz(self.state.accel_x_g, self.state.accel_y_g, self.state.accel_z_g, 3)
        )
        self.imu_gyro.set_value(
            self._fmt_xyz(self.state.gyro_x_dps, self.state.gyro_y_dps, self.state.gyro_z_dps, 2)
        )
        self.imu_failed.set_value(str(self.state.imu_failed_reads))
        self.rx.set_value(
            f"{self._rx_bytes} / B{self._parsed_samples} I{self._parsed_imu_samples} S{self._parsed_status}"
        )
        self.record_metric.set_value(str(self._record_rows))
        if self.state.parse_note:
            self.status_label.setText(
                f"{self.state.parse_note} / BMP {self._parsed_samples}, IMU {self._parsed_imu_samples}"
            )
        self.serial_panel.set_diagnostics(
            rx_bytes=self._rx_bytes,
            valid_frames=self._parsed_samples + self._parsed_imu_samples,
            crc_fail=0,
            resync_count=0,
            last_frame_age_ms=None
            if self.state.last_host_rx_s is None
            else int((time.monotonic() - self.state.last_host_rx_s) * 1000),
        )

    def _toggle_recording(self):
        if self._recording:
            self._stop_recording("manual stop")
        else:
            self._start_recording()

    def _start_recording(self):
        default_name = f"baro_altitude_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        path_text, _ = QFileDialog.getSaveFileName(
            self,
            "Save barometric altitude log",
            str(Path.cwd() / default_name),
            "Text Files (*.txt);;All Files (*)",
        )
        if not path_text:
            return
        self._record_path = Path(path_text)
        self._record_file = self._record_path.open("w", encoding="utf-8", errors="replace")
        self._record_file.write("# Barometric altitude GUI log\n")
        self._record_file.write(f"# start_unix={int(time.time())}\n")
        self._record_file.write(
            "host_time_iso,elapsed_s,source,sensor_time_us,sample,"
            "temp_c,pressure_raw_pa,alt_raw_m,vz_raw_mps,"
            "pressure_filt_pa,alt_filt_m,vz_filt_mps,baseline_pressure_pa,"
            "imu_vz_mps,imu_accel_vertical_mps2\n"
        )
        self._record_file.flush()
        self._record_rows = 0
        self._record_start_s = time.monotonic()
        self._last_imu_time_us = None
        self._imu_vz_mps = 0.0
        self._recording = True
        self.record_btn.setText("停止紀錄 / Stop Log")
        self.status_label.setText(f"紀錄中 / Recording: {self._record_path}")

    def _stop_recording(self, reason: str):
        if not self._recording:
            return
        if self._record_file:
            self._record_file.write(f"# stop_unix={int(time.time())} reason={reason}\n")
            self._record_file.close()
        self._record_file = None
        self._recording = False
        self._record_start_s = None
        self.record_btn.setText("開始紀錄 / Start Log")
        if self._record_path:
            self.status_label.setText(f"紀錄完成 / Saved: {self._record_path}")

    def _reset_view(self):
        self.plot.reset()
        self.raw_log.clear()
        self._record_rows = 0
        self.status_label.setText("已重置圖表 / View reset")

    def closeEvent(self, event):
        self._stop_recording("window closing")
        if self.serial_worker:
            self.serial_worker.stop()
        event.accept()
