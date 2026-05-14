import csv
import math
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
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from map_view import MapView
from serial_comm import SerialWorker
from serial_panel import SerialPanel


BASE_CSV_FIELDS = [
    "report_ms",
    "link",
    "rx_valid",
    "expected",
    "missing",
    "loss_pct",
    "rx_rate_hz",
    "bad",
    "dup",
    "old",
    "last_seq",
    "rssi",
    "snr",
    "avg_rssi",
    "avg_snr",
    "min_rssi",
    "min_snr",
    "max_rssi",
    "max_snr",
    "last_gap_ms",
    "min_gap_ms",
    "max_gap_ms",
    "last_rx_age_ms",
    "lora_ready",
]
GPS_CSV_FIELDS = [
    "packet_type",
    "gps_status",
    "gps_valid",
    "lat",
    "lon",
    "alt_m",
    "sat",
    "hdop",
    "gps_fix_age_ms",
    "gps_baud_code",
]
CSV_FIELDS = BASE_CSV_FIELDS + GPS_CSV_FIELDS
RECORD_DURATION_S = 60


@dataclass
class LinkState:
    report_ms: int | None = None
    link: str = "---"
    rx_valid: int = 0
    expected: int = 0
    missing: int = 0
    loss_pct: float | None = None
    rx_rate_hz: float | None = None
    bad: int = 0
    dup: int = 0
    old: int = 0
    last_seq: int = 0
    rssi: int | None = None
    snr: float | None = None
    avg_rssi: float | None = None
    avg_snr: float | None = None
    min_rssi: int | None = None
    min_snr: float | None = None
    max_rssi: int | None = None
    max_snr: float | None = None
    last_gap_ms: int | None = None
    min_gap_ms: int | None = None
    max_gap_ms: int | None = None
    last_rx_age_ms: int | None = None
    lora_ready: bool = False
    packet_type: str = "---"
    gps_status: str = "---"
    gps_valid: bool = False
    lat: float | None = None
    lon: float | None = None
    alt_m: float | None = None
    sat: int | None = None
    hdop: float | None = None
    gps_fix_age_ms: int | None = None
    gps_baud_code: int | None = None
    last_host_rx_s: float | None = None
    raw_line: str = ""


def _parse_int(value: str) -> int | None:
    try:
        return int(value)
    except ValueError:
        return None


def _parse_float(value: str) -> float | None:
    try:
        number = float(value)
    except ValueError:
        return None
    return number if math.isfinite(number) else None


class MetricBox(QWidget):
    def __init__(self, title: str, unit: str = "", parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(3)
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
        self.setMinimumHeight(86)

    def set_value(self, text: str):
        self.value.setText(text)


class LinkPlot(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(6, 6), tight_layout=True)
        super().__init__(self.fig)
        self.setParent(parent)
        self.setMinimumSize(440, 440)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.ax_loss = self.fig.add_subplot(311)
        self.ax_rssi = self.fig.add_subplot(312, sharex=self.ax_loss)
        self.ax_snr = self.fig.add_subplot(313, sharex=self.ax_loss)
        self.t = deque(maxlen=1200)
        self.loss = deque(maxlen=1200)
        self.rssi = deque(maxlen=1200)
        self.snr = deque(maxlen=1200)
        self._style_axes()

    def _style_axes(self):
        self.ax_loss.set_ylabel("Loss / 掉包 (%)")
        self.ax_rssi.set_ylabel("RSSI (dBm)")
        self.ax_snr.set_ylabel("SNR (dB)")
        self.ax_snr.set_xlabel("Time / 時間 (s)")
        for ax in (self.ax_loss, self.ax_rssi, self.ax_snr):
            ax.grid(True, color="#dddddd", linewidth=0.8)

    def reset(self):
        self.t.clear()
        self.loss.clear()
        self.rssi.clear()
        self.snr.clear()
        self.redraw()

    def append(self, report_s: float, loss: float | None, rssi: int | None, snr: float | None):
        self.t.append(report_s)
        self.loss.append(float("nan") if loss is None else loss)
        self.rssi.append(float("nan") if rssi is None else rssi)
        self.snr.append(float("nan") if snr is None else snr)

    def redraw(self):
        self.ax_loss.clear()
        self.ax_rssi.clear()
        self.ax_snr.clear()
        self._style_axes()
        if self.t:
            t0 = self.t[0]
            xs = [x - t0 for x in self.t]
            self.ax_loss.plot(xs, list(self.loss), color="#c62828", linewidth=1.4)
            self.ax_rssi.plot(xs, list(self.rssi), color="#1565c0", linewidth=1.2)
            self.ax_snr.plot(xs, list(self.snr), color="#2e7d32", linewidth=1.2)
        self.draw_idle()


class LoRaLinkWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LoRa Link Stability Test")
        self.resize(1520, 920)

        self.serial_worker: SerialWorker | None = None
        self.state = LinkState()
        self._line_buffer = bytearray()
        self._rx_bytes = 0
        self._parsed_reports = 0
        self._recording = False
        self._record_mode = ""
        self._record_file = None
        self._record_path: Path | None = None
        self._record_rows = 0
        self._record_start_s: float | None = None

        self.serial_panel = SerialPanel()
        self.serial_panel.baud_box.setCurrentText("115200")
        self.serial_panel.connect_requested.connect(self._start_serial)
        self.serial_panel.disconnect_requested.connect(self._stop_serial)

        self.link = MetricBox("連線 Link")
        self.loss = MetricBox("掉包率 Loss", "%")
        self.rx = MetricBox("有效封包 RX")
        self.expected = MetricBox("預期封包 Expected")
        self.missing = MetricBox("掉包 Missing")
        self.rx_rate = MetricBox("接收率 RX Rate", "Hz")
        self.rssi = MetricBox("RSSI", "dBm")
        self.snr = MetricBox("SNR", "dB")
        self.avg_rssi = MetricBox("平均 RSSI", "dBm")
        self.avg_snr = MetricBox("平均 SNR", "dB")
        self.bad = MetricBox("錯誤封包 Bad")
        self.dup_old = MetricBox("重複/舊包 Dup/Old")
        self.packet_type = MetricBox("封包 Packet")
        self.gps_status = MetricBox("GPS 狀態 Status")
        self.gps_valid = MetricBox("GPS 有效 Valid")
        self.lat = MetricBox("緯度 Latitude")
        self.lon = MetricBox("經度 Longitude")
        self.alt = MetricBox("高度 Altitude", "m")
        self.gps_sat = MetricBox("衛星 Satellites")
        self.gps_hdop = MetricBox("GPS HDOP")
        self.gps_fix_age = MetricBox("GPS 年齡 Fix Age", "ms")
        self.serial_rx = MetricBox("序列接收 Serial RX", "bytes / reports")
        self.record_metric = MetricBox("紀錄 Recording", "rows")

        self.plot = LinkPlot()
        self.map_view = MapView()
        self.map_view.setMinimumSize(460, 760)
        self.map_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.raw_log = QPlainTextEdit()
        self.raw_log.setReadOnly(True)
        self.raw_log.setMaximumBlockCount(180)
        self.raw_log.setMaximumHeight(190)

        self.reset_btn = QPushButton("重置資料並清除畫面 / Reset & Clear")
        self.reset_btn.clicked.connect(self._reset_and_clear)
        self.link_record_btn = QPushButton("紀錄1分鐘通訊 / Log 1-min Link")
        self.link_record_btn.clicked.connect(lambda: self._start_recording("link"))
        self.gps_record_btn = QPushButton("開始GPS紀錄 / Start GPS Log")
        self.gps_record_btn.clicked.connect(self._toggle_gps_recording)
        self.status_label = QLabel("等待連線 / Waiting for connection")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("color: #666; font-weight: 600;")

        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)
        root.addWidget(self.serial_panel)

        body = QHBoxLayout()
        body.setSpacing(10)
        root.addLayout(body, stretch=1)

        left_col = QVBoxLayout()
        left_col.setSpacing(8)
        body.addLayout(left_col, stretch=2)
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(12)
        boxes = [
            self.link,
            self.loss,
            self.rx,
            self.expected,
            self.missing,
            self.rx_rate,
            self.rssi,
            self.snr,
            self.avg_rssi,
            self.avg_snr,
            self.bad,
            self.dup_old,
            self.packet_type,
            self.gps_status,
            self.gps_valid,
            self.lat,
            self.lon,
            self.alt,
            self.gps_sat,
            self.gps_hdop,
            self.gps_fix_age,
            self.serial_rx,
            self.record_metric,
        ]
        for i, box in enumerate(boxes):
            grid.addWidget(box, i // 3, i % 3)
        left_col.addLayout(grid, stretch=1)

        middle_col = QVBoxLayout()
        middle_col.setSpacing(8)
        body.addLayout(middle_col, stretch=3)
        controls = QHBoxLayout()
        controls.addWidget(self.reset_btn)
        controls.addWidget(self.link_record_btn)
        controls.addWidget(self.gps_record_btn)
        middle_col.addLayout(controls)
        middle_col.addWidget(self.status_label)
        middle_col.addWidget(self.plot, stretch=1)

        right_col = QVBoxLayout()
        right_col.setSpacing(8)
        body.addLayout(right_col, stretch=2)
        right_col.addWidget(self.map_view, stretch=1)
        right_col.addWidget(QLabel("原始序列資料 / Raw Serial"))
        right_col.addWidget(self.raw_log, stretch=0)

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
        self._parsed_reports = 0
        self.state = LinkState()
        self.plot.reset()
        self.map_view.reset_path()
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

    def _reset_and_clear(self):
        self.raw_log.clear()
        self.plot.reset()
        self.map_view.reset_path()
        self.state = LinkState()
        self._parsed_reports = 0
        if self.serial_worker:
            self.serial_worker.write(b"RESET_STATS\n")
            self.status_label.setText("已重置資料並清除畫面 / Reset stats and cleared view")
        else:
            self.status_label.setText("已清除畫面 / Cleared view")

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
            return
        self._handle_csv(line)

    def _handle_csv(self, line: str):
        try:
            parts = next(csv.reader([line]))
        except csv.Error:
            return
        if not parts or parts[0] == "report_ms":
            return
        if len(parts) == len(BASE_CSV_FIELDS):
            parts = parts + ["", "", "0", "0", "0", "0", "0", "0", "0", "0"]
        if len(parts) != len(CSV_FIELDS):
            self.status_label.setText(f"忽略資料 / Ignored fields={len(parts)}")
            return

        data = dict(zip(CSV_FIELDS, parts))
        report_ms = _parse_int(data["report_ms"])
        if report_ms is None:
            return

        self.state.report_ms = report_ms
        self.state.link = data["link"]
        self.state.rx_valid = _parse_int(data["rx_valid"]) or 0
        self.state.expected = _parse_int(data["expected"]) or 0
        self.state.missing = _parse_int(data["missing"]) or 0
        self.state.loss_pct = _parse_float(data["loss_pct"])
        self.state.rx_rate_hz = _parse_float(data["rx_rate_hz"])
        self.state.bad = _parse_int(data["bad"]) or 0
        self.state.dup = _parse_int(data["dup"]) or 0
        self.state.old = _parse_int(data["old"]) or 0
        self.state.last_seq = _parse_int(data["last_seq"]) or 0
        self.state.rssi = _parse_int(data["rssi"])
        self.state.snr = _parse_float(data["snr"])
        self.state.avg_rssi = _parse_float(data["avg_rssi"])
        self.state.avg_snr = _parse_float(data["avg_snr"])
        self.state.min_rssi = _parse_int(data["min_rssi"])
        self.state.min_snr = _parse_float(data["min_snr"])
        self.state.max_rssi = _parse_int(data["max_rssi"])
        self.state.max_snr = _parse_float(data["max_snr"])
        self.state.last_gap_ms = _parse_int(data["last_gap_ms"])
        self.state.min_gap_ms = _parse_int(data["min_gap_ms"])
        self.state.max_gap_ms = _parse_int(data["max_gap_ms"])
        self.state.last_rx_age_ms = _parse_int(data["last_rx_age_ms"])
        self.state.lora_ready = data["lora_ready"] == "1"
        self.state.packet_type = data["packet_type"] or "---"
        self.state.gps_status = data["gps_status"] or "---"
        self.state.gps_valid = data["gps_valid"] == "1"
        self.state.lat = _parse_float(data["lat"]) if self.state.gps_valid else None
        self.state.lon = _parse_float(data["lon"]) if self.state.gps_valid else None
        self.state.alt_m = _parse_float(data["alt_m"])
        self.state.sat = _parse_int(data["sat"])
        self.state.hdop = _parse_float(data["hdop"])
        self.state.gps_fix_age_ms = _parse_int(data["gps_fix_age_ms"])
        self.state.gps_baud_code = _parse_int(data["gps_baud_code"])
        self._parsed_reports += 1
        self.plot.append(report_ms / 1000.0, self.state.loss_pct, self.state.rssi, self.state.snr)
        if self.state.gps_valid and self.state.lat is not None and self.state.lon is not None:
            self.map_view.update_gps(self.state.lat, self.state.lon, self.state.alt_m or 0.0, zoom=18)

        if self._recording and self._record_file:
            now = datetime.now().isoformat(timespec="milliseconds")
            elapsed_s = 0.0 if self._record_start_s is None else time.monotonic() - self._record_start_s
            if self._record_mode == "link" and elapsed_s >= RECORD_DURATION_S:
                self._stop_recording("duration complete")
                return
            if self._record_mode == "link":
                values = [data[field] for field in BASE_CSV_FIELDS]
            else:
                values = [data[field] for field in CSV_FIELDS]
            self._record_file.write(f"{now},{elapsed_s:.3f}," + ",".join(values) + "\n")
            self._record_file.flush()
            self._record_rows += 1

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
        self.link.set_value(self.state.link)
        self.loss.set_value(self._fmt_float(self.state.loss_pct, 3))
        self.rx.set_value(str(self.state.rx_valid))
        self.expected.set_value(str(self.state.expected))
        self.missing.set_value(str(self.state.missing))
        self.rx_rate.set_value(self._fmt_float(self.state.rx_rate_hz, 2))
        self.rssi.set_value(self._fmt_int(self.state.rssi))
        self.snr.set_value(self._fmt_float(self.state.snr, 2))
        self.avg_rssi.set_value(self._fmt_float(self.state.avg_rssi, 2))
        self.avg_snr.set_value(self._fmt_float(self.state.avg_snr, 2))
        self.bad.set_value(str(self.state.bad))
        self.dup_old.set_value(f"{self.state.dup} / {self.state.old}")
        self.packet_type.set_value(self.state.packet_type)
        self.gps_status.set_value(self.state.gps_status)
        self.gps_valid.set_value("YES" if self.state.gps_valid else "NO")
        self.lat.set_value(self._fmt_float(self.state.lat, 7))
        self.lon.set_value(self._fmt_float(self.state.lon, 7))
        self.alt.set_value(self._fmt_float(self.state.alt_m, 2))
        self.gps_sat.set_value(self._fmt_int(self.state.sat))
        self.gps_hdop.set_value(self._fmt_float(self.state.hdop, 2))
        self.gps_fix_age.set_value(self._fmt_int(self.state.gps_fix_age_ms))
        self.serial_rx.set_value(f"{self._rx_bytes} / {self._parsed_reports}")
        if self._recording and self._record_start_s is not None:
            elapsed_s = time.monotonic() - self._record_start_s
            if self._record_mode == "link":
                remain = max(0.0, RECORD_DURATION_S - elapsed_s)
                self.record_metric.set_value(f"{self._record_rows} / {remain:.0f}s")
            else:
                self.record_metric.set_value(f"{self._record_rows} / {elapsed_s:.0f}s")
            if self._record_mode == "link" and elapsed_s >= RECORD_DURATION_S:
                self._stop_recording("duration complete")
        else:
            self.record_metric.set_value(str(self._record_rows))
        self.serial_panel.set_diagnostics(
            rx_bytes=self._rx_bytes,
            valid_frames=self._parsed_reports,
            crc_fail=self.state.bad,
            resync_count=0,
            last_frame_age_ms=None
            if self.state.last_host_rx_s is None
            else int((time.monotonic() - self.state.last_host_rx_s) * 1000),
        )

    def _toggle_gps_recording(self):
        if self._recording and self._record_mode == "gps":
            self._stop_recording("manual stop")
            return
        self._start_recording("gps")

    def _start_recording(self, mode: str):
        if self._recording:
            self._stop_recording("manual restart")
        default_name = f"lora_{mode}_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        path_text, _ = QFileDialog.getSaveFileName(
            self,
            "Save LoRa log",
            str(Path.cwd() / default_name),
            "Text Files (*.txt);;All Files (*)",
        )
        if not path_text:
            return
        self._record_path = Path(path_text)
        self._record_file = self._record_path.open("w", encoding="utf-8", errors="replace")
        self._record_mode = mode
        self._record_file.write("# LoRa link stability GUI log\n")
        duration_text = str(RECORD_DURATION_S) if mode == "link" else "manual"
        self._record_file.write(f"# mode={mode} duration_s={duration_text}\n")
        self._record_file.write(f"# start_unix={int(time.time())}\n")
        fields = BASE_CSV_FIELDS if mode == "link" else CSV_FIELDS
        self._record_file.write("host_time_iso,elapsed_s," + ",".join(fields) + "\n")
        self._record_file.flush()
        self._record_rows = 0
        self._record_start_s = time.monotonic()
        self._recording = True
        self.link_record_btn.setEnabled(False)
        if mode == "gps":
            self.gps_record_btn.setEnabled(True)
            self.gps_record_btn.setText("停止GPS紀錄 / Stop GPS Log")
        else:
            self.gps_record_btn.setEnabled(False)
        self.status_label.setText(f"紀錄中 / Recording {mode}: {self._record_path}")

    def _stop_recording(self, reason: str):
        if not self._recording:
            return
        if self._record_file:
            self._record_file.write(f"# stop_unix={int(time.time())} reason={reason}\n")
            self._record_file.close()
        self._record_file = None
        self._recording = False
        self._record_mode = ""
        self._record_start_s = None
        self.link_record_btn.setEnabled(True)
        self.gps_record_btn.setEnabled(True)
        self.gps_record_btn.setText("開始GPS紀錄 / Start GPS Log")
        if self._record_path:
            self.status_label.setText(f"紀錄完成 / Saved: {self._record_path}")

    def closeEvent(self, event):
        self._stop_recording("window closing")
        if self.serial_worker:
            self.serial_worker.stop()
        event.accept()
