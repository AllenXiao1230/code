import json
import time
from dataclasses import dataclass

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QCheckBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from serial_comm import SerialWorker
from serial_panel import SerialPanel


FRAME_END = b"\nCAM_FRAME_END\n"


@dataclass
class CameraStatus:
    burst: bool = False
    interval_ms: int = 0
    frames: int = 0
    failures: int = 0
    last_capture_ms: int = 0
    last_frame_bytes: int = 0
    uptime_ms: int = 0
    free_heap: int = 0
    psram: bool = False


class MetricBox(QWidget):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(2)
        title_label = QLabel(title)
        title_label.setStyleSheet("color: #666; font-size: 12px;")
        self.value = QLabel("--")
        self.value.setStyleSheet("font-size: 20px; font-weight: 700;")
        layout.addWidget(title_label)
        layout.addWidget(self.value)
        self.setStyleSheet(
            "MetricBox { border: 1px solid #d7d7d7; border-radius: 6px; background: #fafafa; }"
        )

    def set_value(self, text: str):
        self.value.setText(text)


class CameraUartParser:
    def __init__(self):
        self.buffer = bytearray()
        self.expected_frame_len: int | None = None
        self.last_frame_meta: tuple[int, int] | None = None

    def feed(self, data: bytes):
        self.buffer.extend(data)
        events = []

        while True:
            if self.expected_frame_len is not None:
                needed = self.expected_frame_len + len(FRAME_END)
                if len(self.buffer) < needed:
                    break

                jpeg = bytes(self.buffer[: self.expected_frame_len])
                del self.buffer[: self.expected_frame_len]
                if self.buffer.startswith(FRAME_END):
                    del self.buffer[: len(FRAME_END)]
                else:
                    marker_index = self.buffer.find(FRAME_END)
                    if marker_index >= 0:
                        del self.buffer[: marker_index + len(FRAME_END)]

                frame_count, capture_ms = self.last_frame_meta or (0, 0)
                events.append(("frame", (jpeg, frame_count, capture_ms)))
                self.expected_frame_len = None
                self.last_frame_meta = None
                continue

            newline = self.buffer.find(b"\n")
            if newline < 0:
                break

            raw_line = bytes(self.buffer[:newline]).strip()
            del self.buffer[: newline + 1]
            if not raw_line:
                continue

            line = raw_line.decode("utf-8", errors="replace")
            if line.startswith("CAM_FRAME "):
                parts = line.split()
                if len(parts) >= 4:
                    try:
                        self.expected_frame_len = int(parts[1])
                        self.last_frame_meta = (int(parts[2]), int(parts[3]))
                    except ValueError:
                        events.append(("log", f"Bad frame header: {line}"))
                continue

            if line.startswith("CAM_STATUS "):
                try:
                    events.append(("status", json.loads(line[len("CAM_STATUS ") :])))
                except json.JSONDecodeError as exc:
                    events.append(("log", f"Bad status JSON: {exc}"))
                continue

            events.append(("log", line))

        return events


class CameraUartMonitorWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32-CAM OV5640 UART Monitor")
        self.resize(1180, 760)

        self.serial_worker: SerialWorker | None = None
        self.parser = CameraUartParser()
        self.status = CameraStatus()
        self._rx_bytes = 0
        self._last_frame_host_s: float | None = None

        self._init_ui()

        self.status_timer = QTimer(self)
        self.status_timer.setInterval(1000)
        self.status_timer.timeout.connect(self._request_status)
        self.status_timer.start()

    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        self.serial_panel = SerialPanel()
        self.serial_panel.baud_box.setCurrentText("921600")
        serial_group = QGroupBox("UART")
        serial_layout = QVBoxLayout(serial_group)
        serial_layout.addWidget(self.serial_panel)

        controls = QGroupBox("相機控制")
        controls_layout = QHBoxLayout(controls)
        self.interval_spin = QSpinBox()
        self.interval_spin.setRange(500, 60000)
        self.interval_spin.setValue(1000)
        self.interval_spin.setSuffix(" ms")
        self.start_button = QPushButton("開始連拍")
        self.stop_button = QPushButton("停止連拍")
        self.capture_button = QPushButton("拍一張")
        self.preview_check = QCheckBox("顯示影像")
        self.preview_check.setChecked(True)
        controls_layout.addWidget(QLabel("連拍間隔"))
        controls_layout.addWidget(self.interval_spin)
        controls_layout.addWidget(self.start_button)
        controls_layout.addWidget(self.stop_button)
        controls_layout.addWidget(self.capture_button)
        controls_layout.addWidget(self.preview_check)
        controls_layout.addStretch(1)

        metrics = QGroupBox("狀態")
        metric_grid = QGridLayout(metrics)
        self.burst_box = MetricBox("連拍")
        self.frames_box = MetricBox("已拍張數")
        self.failures_box = MetricBox("失敗")
        self.size_box = MetricBox("最新影像")
        self.uptime_box = MetricBox("Uptime")
        self.heap_box = MetricBox("Free Heap")
        self.psram_box = MetricBox("PSRAM")
        self.rx_box = MetricBox("UART RX")
        boxes = [
            self.burst_box,
            self.frames_box,
            self.failures_box,
            self.size_box,
            self.uptime_box,
            self.heap_box,
            self.psram_box,
            self.rx_box,
        ]
        for index, box in enumerate(boxes):
            metric_grid.addWidget(box, index // 4, index % 4)

        self.image_label = QLabel("尚未取得影像")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(720, 480)
        self.image_label.setStyleSheet("border: 1px solid #cfcfcf; background: #111; color: #ddd;")

        self.status_label = QLabel("Status: disconnected")
        self.status_label.setStyleSheet("color: #555;")

        root.addWidget(serial_group)
        root.addWidget(controls)
        root.addWidget(metrics)
        root.addWidget(self.image_label, 1)
        root.addWidget(self.status_label)

        self.serial_panel.connect_requested.connect(self._start_serial)
        self.serial_panel.disconnect_requested.connect(self._stop_serial)
        self.start_button.clicked.connect(self._start_burst)
        self.stop_button.clicked.connect(self._stop_burst)
        self.capture_button.clicked.connect(self._capture_once)
        self._set_controls_enabled(False)

    def _set_controls_enabled(self, enabled: bool):
        self.start_button.setEnabled(enabled)
        self.stop_button.setEnabled(enabled)
        self.capture_button.setEnabled(enabled)
        self.interval_spin.setEnabled(enabled)

    def _start_serial(self, port: str, baudrate: int):
        if self.serial_worker:
            return
        self.parser = CameraUartParser()
        self._rx_bytes = 0
        self.serial_worker = SerialWorker(port, baudrate)
        self.serial_worker.data_received.connect(self._on_serial_data)
        self.serial_worker.status_changed.connect(self._on_serial_status)
        self.serial_worker.error_occurred.connect(self._on_serial_error)
        self.serial_worker.start()
        self._set_controls_enabled(True)

    def _stop_serial(self):
        if self.serial_worker:
            self.serial_worker.stop()
            self.serial_worker = None
        self._set_controls_enabled(False)
        self.serial_panel.set_status("Status: DISCONNECTED", color="red")
        self.status_label.setText("Status: disconnected")

    def _write_command(self, command: str):
        if not self.serial_worker:
            return
        self.serial_worker.write((command.strip() + "\n").encode("ascii"))

    def _request_status(self):
        if self.serial_worker:
            self._write_command("STATUS")

    def _start_burst(self):
        self._write_command(f"BURST START {self.interval_spin.value()}")

    def _stop_burst(self):
        self._write_command("BURST STOP")

    def _capture_once(self):
        self._write_command("CAPTURE")

    def _on_serial_status(self, message: str):
        self.serial_panel.set_status(f"Status: {message}", color="green")
        self.status_label.setText(f"Status: {message}")
        self._request_status()

    def _on_serial_error(self, message: str):
        self.serial_panel.set_status(f"Status: {message}", color="red")
        self.status_label.setText(f"Status: {message}")

    def _on_serial_data(self, data: bytes):
        self._rx_bytes += len(data)
        for event_type, payload in self.parser.feed(data):
            if event_type == "status":
                self._apply_status(payload)
            elif event_type == "frame":
                jpeg, frame_count, capture_ms = payload
                self._apply_frame(jpeg, frame_count, capture_ms)
            elif event_type == "log":
                self.status_label.setText(f"Status: {payload}")
        self._update_metrics()

    def _apply_status(self, data: dict):
        self.status = CameraStatus(
            burst=bool(data.get("burst", False)),
            interval_ms=int(data.get("interval_ms", 0) or 0),
            frames=int(data.get("frames", 0) or 0),
            failures=int(data.get("failures", 0) or 0),
            last_capture_ms=int(data.get("last_capture_ms", 0) or 0),
            last_frame_bytes=int(data.get("last_frame_bytes", 0) or 0),
            uptime_ms=int(data.get("uptime_ms", 0) or 0),
            free_heap=int(data.get("free_heap", 0) or 0),
            psram=bool(data.get("psram", False)),
        )
        self._update_metrics()

    def _apply_frame(self, jpeg: bytes, frame_count: int, capture_ms: int):
        self._last_frame_host_s = time.time()
        if not self.preview_check.isChecked():
            return

        pixmap = QPixmap()
        if not pixmap.loadFromData(jpeg):
            self.status_label.setText(f"Status: invalid JPEG frame {frame_count}")
            return

        scaled = pixmap.scaled(
            self.image_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.image_label.setPixmap(scaled)
        self.status_label.setText(
            f"Status: frame={frame_count} camera_t={capture_ms} ms size={len(jpeg)} bytes"
        )

    def _update_metrics(self):
        self.burst_box.set_value("ON" if self.status.burst else "OFF")
        self.frames_box.set_value(str(self.status.frames))
        self.failures_box.set_value(str(self.status.failures))
        self.size_box.set_value(f"{self.status.last_frame_bytes / 1024:.1f} KB")
        self.uptime_box.set_value(f"{self.status.uptime_ms / 1000:.1f} s")
        self.heap_box.set_value(f"{self.status.free_heap / 1024:.0f} KB")
        self.psram_box.set_value("YES" if self.status.psram else "NO")
        self.rx_box.set_value(f"{self._rx_bytes / 1024:.1f} KB")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.serial_worker:
            self._request_status()

    def closeEvent(self, event):
        if self.serial_worker:
            self.serial_worker.stop()
        event.accept()
