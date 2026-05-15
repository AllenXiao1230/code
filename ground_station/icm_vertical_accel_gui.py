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
STANDARD_GRAVITY = 9.80665
MAX_REASONABLE_ACCEL_G = 16.5
SPIKE_MEDIAN_WINDOW = 5
SPIKE_REJECT_DELTA_G = 3.0
MAX_CONSECUTIVE_SPIKES = 3
CALIBRATION_SAMPLE_COUNT = 100


@dataclass
class IcmVerticalAccelState:
    icm42688: str = "---"
    icm_addr: str = "--"
    icm_whoami: str = "--"
    imu_time_us: int | None = None
    imu_sample: int = 0
    imu_temp_c: float | None = None
    accel_x_g: float | None = None
    accel_y_g: float | None = None
    accel_z_g: float | None = None
    vertical_accel_mps2: float | None = None
    vertical_accel_g: float | None = None
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


class IcmVerticalAccelPlot(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(7, 4), tight_layout=True)
        super().__init__(self.fig)
        self.setParent(parent)
        self.ax_accel = self.fig.add_subplot(211)
        self.ax_z = self.fig.add_subplot(212, sharex=self.ax_accel)
        self.t = deque(maxlen=2000)
        self.vertical_accel = deque(maxlen=2000)
        self.accel_z = deque(maxlen=2000)
        self._style_axes()

    def _style_axes(self):
        self.ax_accel.set_ylabel("Vertical Accel / 垂直加速度 (m/s^2)")
        self.ax_z.set_ylabel("Reversed Accel Z (g)")
        self.ax_z.set_xlabel("Time / 時間 (s)")
        for ax in (self.ax_accel, self.ax_z):
            ax.grid(True, color="#dddddd", linewidth=0.8)

    def reset(self):
        self.t.clear()
        self.vertical_accel.clear()
        self.accel_z.clear()
        self.redraw()

    def append(self, time_s: float, accel_z_g: float | None, vertical_accel_mps2: float | None):
        if accel_z_g is None and vertical_accel_mps2 is None:
            return
        self.t.append(time_s)
        self.accel_z.append(float("nan") if accel_z_g is None else accel_z_g)
        self.vertical_accel.append(float("nan") if vertical_accel_mps2 is None else vertical_accel_mps2)

    def redraw(self):
        self.ax_accel.clear()
        self.ax_z.clear()
        self._style_axes()
        if self.t:
            t0 = self.t[0]
            xs = [x - t0 for x in self.t]
            self.ax_accel.axhline(0.0, color="#777777", linewidth=0.9)
            self.ax_accel.plot(xs, list(self.vertical_accel), color="#c62828", linewidth=1.4)
            self.ax_z.axhline(1.0, color="#777777", linewidth=0.9)
            self.ax_z.plot(xs, list(self.accel_z), color="#1565c0", linewidth=1.2)
        self.draw_idle()


class IcmVerticalAccelWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ICM42688 Vertical Acceleration Test")
        self.resize(1180, 760)

        self.serial_worker: SerialWorker | None = None
        self.state = IcmVerticalAccelState()
        self._line_buffer = bytearray()
        self._rx_bytes = 0
        self._parsed_imu_samples = 0
        self._parsed_status = 0
        self._recording = False
        self._record_file = None
        self._record_path: Path | None = None
        self._record_rows = 0
        self._record_start_s: float | None = None
        self._last_imu_sample_count = 0
        self._last_rate_time_s = time.monotonic()
        self._imu_rate_hz = 0.0
        self._min_vertical_accel: float | None = None
        self._max_vertical_accel: float | None = None
        self._z_filter_samples = deque(maxlen=SPIKE_MEDIAN_WINDOW)
        self._rejected_samples = 0
        self._consecutive_z_spikes = 0
        self._accel_z_baseline_g = 1.0
        self._calibrating_recording = False
        self._calibration_z_samples: list[float] = []

        self.serial_panel = SerialPanel()
        self.serial_panel.baud_box.setCurrentText("115200")
        self.serial_panel.connect_requested.connect(self._start_serial)
        self.serial_panel.disconnect_requested.connect(self._stop_serial)

        self.imu_status = MetricBox("IMU 狀態 ICM42688")
        self.imu_addr = MetricBox("IMU 位址 ICM Addr")
        self.imu_whoami = MetricBox("IMU WHO_AM_I")
        self.imu_sample = MetricBox("IMU 樣本 IMU Samples")
        self.imu_rate = MetricBox("IMU 採樣率 IMU Rate", "Hz")
        self.imu_temp = MetricBox("IMU 溫度 IMU Temp", "deg C")
        self.accel_z = MetricBox("反轉 Z 軸加速度 Reversed Accel Z", "g")
        self.vertical_accel = MetricBox("垂直加速度 Vertical Accel", "m/s^2")
        self.vertical_accel_g = MetricBox("垂直加速度 Vertical Accel", "g above baseline")
        self.baseline_z = MetricBox("Z 軸校正基準 Z Baseline", "g")
        self.minmax = MetricBox("最小/最大 Min/Max", "m/s^2")
        self.accel_xyz = MetricBox("加速度 Accel XYZ", "g")
        self.gyro_xyz = MetricBox("角速度 Gyro XYZ", "deg/s")
        self.imu_failed = MetricBox("IMU 讀取失敗 IMU Failed")
        self.rejected = MetricBox("尖峰排除 Rejected")
        self.rx = MetricBox("序列接收 Serial RX", "bytes / lines")
        self.record_metric = MetricBox("紀錄 Recording", "rows")

        self.plot = IcmVerticalAccelPlot()
        self.raw_log = QPlainTextEdit()
        self.raw_log.setReadOnly(True)
        self.raw_log.setMaximumBlockCount(400)

        self.record_btn = QPushButton("開始紀錄 / Start Log")
        self.record_btn.clicked.connect(self._toggle_recording)
        self.reset_btn = QPushButton("重置圖表 / Reset View")
        self.reset_btn.clicked.connect(self._reset_view)
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
            self.imu_status,
            self.imu_addr,
            self.imu_whoami,
            self.imu_sample,
            self.imu_rate,
            self.imu_temp,
            self.accel_z,
            self.vertical_accel,
            self.vertical_accel_g,
            self.baseline_z,
            self.minmax,
            self.accel_xyz,
            self.gyro_xyz,
            self.imu_failed,
            self.rejected,
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
        self._parsed_imu_samples = 0
        self._parsed_status = 0
        self._record_rows = 0
        self._last_imu_sample_count = 0
        self._last_rate_time_s = time.monotonic()
        self._imu_rate_hz = 0.0
        self._min_vertical_accel = None
        self._max_vertical_accel = None
        self._z_filter_samples.clear()
        self._rejected_samples = 0
        self._consecutive_z_spikes = 0
        self._accel_z_baseline_g = 1.0
        self._calibrating_recording = False
        self._calibration_z_samples.clear()
        self.state = IcmVerticalAccelState()
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

        if line.startswith("#"):
            self._handle_comment(line)
            return
        self._handle_csv(line)

    def _handle_comment(self, line: str):
        pairs = dict(KEY_VALUE_RE.findall(line))
        if "icm42688_verify" in pairs:
            verify = pairs["icm42688_verify"]
            self.state.icm42688 = "ready" if verify == "ready" else verify
            self.state.icm_addr = pairs.get("addr", self.state.icm_addr)
            self.state.icm_whoami = pairs.get("whoami", self.state.icm_whoami)
            self._parsed_status += 1
            return

        if "status" in pairs:
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

    def _handle_csv(self, line: str):
        try:
            parts = next(csv.reader([line]))
        except csv.Error:
            self.state.parse_note = "CSV parse error"
            return

        if not parts or parts[0] == "imu_time_us":
            return
        if not parts[0].isdigit() or len(parts) != len(IMU_CSV_FIELDS):
            self.state.parse_note = f"ignored csv fields={len(parts)} first={parts[0][:16]}"
            return
        self._handle_imu_csv(parts)

    def _handle_imu_csv(self, parts: list[str]):
        data = dict(zip(IMU_CSV_FIELDS, parts))
        time_us = _parse_int(data["imu_time_us"])
        sample = _parse_int(data["imu_sample"])
        if time_us is None or sample is None:
            return

        raw_accel_z_g = _parse_float(data["accel_z_g"])
        rejected_before = self._rejected_samples
        accel_z_g = self._filter_reversed_accel_z(None if raw_accel_z_g is None else -raw_accel_z_g)

        self.state.imu_time_us = time_us
        self.state.imu_sample = sample
        self.state.imu_temp_c = _parse_float(data["imu_temp_c"])
        self.state.accel_x_g = _parse_float(data["accel_x_g"])
        self.state.accel_y_g = _parse_float(data["accel_y_g"])
        self.state.accel_z_g = accel_z_g
        self.state.gyro_x_dps = _parse_float(data["gyro_x_dps"])
        self.state.gyro_y_dps = _parse_float(data["gyro_y_dps"])
        self.state.gyro_z_dps = _parse_float(data["gyro_z_dps"])
        self.state.imu_failed_reads = _parse_int(data["imu_failed_reads"]) or self.state.imu_failed_reads
        self._parsed_imu_samples += 1

        if self._calibrating_recording:
            self._handle_calibration_sample(accel_z_g)
            return

        vertical_accel_mps2 = None
        vertical_accel_g = None
        if accel_z_g is not None:
            vertical_accel_g = accel_z_g - self._accel_z_baseline_g
            vertical_accel_mps2 = vertical_accel_g * STANDARD_GRAVITY
            self._min_vertical_accel = (
                vertical_accel_mps2
                if self._min_vertical_accel is None
                else min(self._min_vertical_accel, vertical_accel_mps2)
            )
            self._max_vertical_accel = (
                vertical_accel_mps2
                if self._max_vertical_accel is None
                else max(self._max_vertical_accel, vertical_accel_mps2)
            )

        self.state.vertical_accel_mps2 = vertical_accel_mps2
        self.state.vertical_accel_g = vertical_accel_g
        self.state.parse_note = "IMU spike rejected" if self._rejected_samples > rejected_before else "IMU CSV OK"
        self.plot.append(time_us / 1_000_000.0, accel_z_g, vertical_accel_mps2)
        if self._recording and self._record_file:
            self._record_imu_sample(data, accel_z_g, vertical_accel_g, vertical_accel_mps2)

    def _handle_calibration_sample(self, accel_z_g: float | None):
        self.state.vertical_accel_mps2 = None
        self.state.vertical_accel_g = None
        if accel_z_g is not None:
            self._calibration_z_samples.append(accel_z_g)
        current = len(self._calibration_z_samples)
        self.state.parse_note = f"Calibrating Z baseline {current}/{CALIBRATION_SAMPLE_COUNT}"
        if current >= CALIBRATION_SAMPLE_COUNT:
            self._finish_recording_calibration()

    def _finish_recording_calibration(self):
        samples = sorted(self._calibration_z_samples)
        self._accel_z_baseline_g = samples[len(samples) // 2]
        self._calibrating_recording = False
        self._calibration_z_samples.clear()
        self._min_vertical_accel = None
        self._max_vertical_accel = None
        self.plot.reset()
        self._record_rows = 0
        self._record_start_s = time.monotonic()
        if self._record_file:
            self._record_file.write(f"# calibration_baseline_accel_z_g={self._accel_z_baseline_g:.6f}\n")
            self._record_file.flush()
        self.state.parse_note = f"Calibration complete baseline={self._accel_z_baseline_g:.6f}g"
        self.status_label.setText(f"校正完成，開始紀錄 / Baseline {self._accel_z_baseline_g:.6f}g")

    def _filter_reversed_accel_z(self, accel_z_g: float | None) -> float | None:
        if accel_z_g is None:
            return None
        if abs(accel_z_g) > MAX_REASONABLE_ACCEL_G:
            self._rejected_samples += 1
            self._consecutive_z_spikes += 1
            self.state.parse_note = "IMU spike rejected"
            return None

        if len(self._z_filter_samples) >= 3:
            median_z = sorted(self._z_filter_samples)[len(self._z_filter_samples) // 2]
            if abs(accel_z_g - median_z) > SPIKE_REJECT_DELTA_G:
                self._rejected_samples += 1
                self._consecutive_z_spikes += 1
                if self._consecutive_z_spikes < MAX_CONSECUTIVE_SPIKES:
                    self.state.parse_note = "IMU spike rejected"
                    return median_z
                self._z_filter_samples.clear()

        self._consecutive_z_spikes = 0
        self._z_filter_samples.append(accel_z_g)
        return sorted(self._z_filter_samples)[len(self._z_filter_samples) // 2]

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

    def _record_imu_sample(
        self,
        data: dict[str, str],
        accel_z_g: float | None,
        vertical_accel_g: float | None,
        vertical_accel_mps2: float | None,
    ):
        self._write_record_row(
            self._record_prefix()
            + [
                data["imu_time_us"],
                data["imu_sample"],
                data["imu_temp_c"],
                data["accel_x_g"],
                data["accel_y_g"],
                self._record_float(accel_z_g, 6),
                self._record_float(vertical_accel_g, 6),
                self._record_float(vertical_accel_mps2, 6),
                data["gyro_x_dps"],
                data["gyro_y_dps"],
                data["gyro_z_dps"],
                data["imu_failed_reads"],
            ]
        )

    @staticmethod
    def _record_float(value: float | None, digits: int) -> str:
        if value is None or not math.isfinite(value):
            return ""
        return f"{value:.{digits}f}"

    @staticmethod
    def _fmt_float(value: float | None, digits: int = 2) -> str:
        if value is None:
            return "--"
        return f"{value:.{digits}f}"

    @classmethod
    def _fmt_xyz(cls, x: float | None, y: float | None, z: float | None, digits: int = 2) -> str:
        if x is None or y is None or z is None:
            return "--"
        return f"{x:.{digits}f} / {y:.{digits}f} / {z:.{digits}f}"

    def _refresh_ui(self):
        now = time.monotonic()
        if now - self._last_rate_time_s >= 1.0:
            dt = now - self._last_rate_time_s
            dis = self.state.imu_sample - self._last_imu_sample_count
            self._imu_rate_hz = dis / dt if dt > 0 else 0.0
            self._last_imu_sample_count = self.state.imu_sample
            self._last_rate_time_s = now

        self.imu_status.set_value(self.state.icm42688)
        self.imu_addr.set_value(self.state.icm_addr)
        self.imu_whoami.set_value(self.state.icm_whoami)
        self.imu_sample.set_value(str(self.state.imu_sample))
        self.imu_rate.set_value(f"{self._imu_rate_hz:.1f}")
        self.imu_temp.set_value(self._fmt_float(self.state.imu_temp_c, 3))
        self.accel_z.set_value(self._fmt_float(self.state.accel_z_g, 5))
        self.vertical_accel.set_value(self._fmt_float(self.state.vertical_accel_mps2, 5))
        self.vertical_accel_g.set_value(self._fmt_float(self.state.vertical_accel_g, 5))
        self.baseline_z.set_value(self._fmt_float(self._accel_z_baseline_g, 5))
        if self._min_vertical_accel is None or self._max_vertical_accel is None:
            self.minmax.set_value("--")
        else:
            self.minmax.set_value(f"{self._min_vertical_accel:.5f} / {self._max_vertical_accel:.5f}")
        self.accel_xyz.set_value(
            self._fmt_xyz(self.state.accel_x_g, self.state.accel_y_g, self.state.accel_z_g, 5)
        )
        self.gyro_xyz.set_value(
            self._fmt_xyz(self.state.gyro_x_dps, self.state.gyro_y_dps, self.state.gyro_z_dps, 2)
        )
        self.imu_failed.set_value(str(self.state.imu_failed_reads))
        self.rejected.set_value(str(self._rejected_samples))
        self.rx.set_value(f"{self._rx_bytes} / I{self._parsed_imu_samples} S{self._parsed_status}")
        self.record_metric.set_value(str(self._record_rows))
        if self.state.parse_note:
            self.status_label.setText(f"{self.state.parse_note} / IMU {self._parsed_imu_samples}")
        self.serial_panel.set_diagnostics(
            rx_bytes=self._rx_bytes,
            valid_frames=self._parsed_imu_samples,
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
        default_name = f"icm_vertical_accel_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        path_text, _ = QFileDialog.getSaveFileName(
            self,
            "Save ICM42688 vertical acceleration log",
            str(Path.cwd() / default_name),
            "Text Files (*.txt);;All Files (*)",
        )
        if not path_text:
            return
        self._reset_measurement_view()
        self._record_path = Path(path_text)
        self._record_file = self._record_path.open("w", encoding="utf-8", errors="replace")
        self._record_file.write("# ICM42688 vertical acceleration GUI log\n")
        self._record_file.write("# accel_z_g is reversed from the raw firmware accel_z_g field\n")
        self._record_file.write(
            f"# recording starts after {CALIBRATION_SAMPLE_COUNT} filtered static samples calibrate accel_z_g baseline\n"
        )
        self._record_file.write(
            f"# accel_z_g uses {SPIKE_MEDIAN_WINDOW}-sample median filtering; "
            f"jumps over {SPIKE_REJECT_DELTA_G:.1f}g are held at the median unless repeated "
            f"{MAX_CONSECUTIVE_SPIKES} times\n"
        )
        self._record_file.write("# vertical_accel_mps2=(reversed_accel_z_g-calibration_baseline_accel_z_g)*9.80665\n")
        self._record_file.write(f"# start_unix={int(time.time())}\n")
        self._record_file.write(
            "host_time_iso,elapsed_s,imu_time_us,imu_sample,imu_temp_c,"
            "accel_x_g,accel_y_g,accel_z_g,vertical_accel_g,vertical_accel_mps2,"
            "gyro_x_dps,gyro_y_dps,gyro_z_dps,imu_failed_reads\n"
        )
        self._record_file.flush()
        self._record_rows = 0
        self._record_start_s = None
        self._recording = True
        self._calibrating_recording = True
        self._calibration_z_samples.clear()
        self.record_btn.setText("停止紀錄 / Stop Log")
        self.status_label.setText(
            f"校正中，請保持靜止 / Calibrating 0/{CALIBRATION_SAMPLE_COUNT}: {self._record_path}"
        )

    def _stop_recording(self, reason: str):
        if not self._recording:
            return
        if self._record_file:
            self._record_file.write(f"# stop_unix={int(time.time())} reason={reason}\n")
            self._record_file.close()
        self._record_file = None
        self._recording = False
        self._calibrating_recording = False
        self._calibration_z_samples.clear()
        self._record_start_s = None
        self.record_btn.setText("開始紀錄 / Start Log")
        if self._record_path:
            self.status_label.setText(f"紀錄完成 / Saved: {self._record_path}")

    def _reset_view(self):
        self._reset_measurement_view()
        self.status_label.setText("已重置圖表 / View reset")

    def _reset_measurement_view(self):
        self.plot.reset()
        self.raw_log.clear()
        self._parsed_imu_samples = 0
        self._parsed_status = 0
        self._record_rows = 0
        self._last_imu_sample_count = self.state.imu_sample
        self._last_rate_time_s = time.monotonic()
        self._imu_rate_hz = 0.0
        self._min_vertical_accel = None
        self._max_vertical_accel = None
        self._z_filter_samples.clear()
        self._rejected_samples = 0
        self._consecutive_z_spikes = 0
        self._accel_z_baseline_g = 1.0
        self._calibrating_recording = False
        self._calibration_z_samples.clear()

    def closeEvent(self, event):
        self._stop_recording("window closing")
        if self.serial_worker:
            self.serial_worker.stop()
        event.accept()
