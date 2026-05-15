import json
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass

from PyQt5.QtCore import QObject, QRunnable, QThreadPool, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import (
    QCheckBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


DEFAULT_CAMERA_HOST = "http://192.168.4.1"


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
    ip: str = ""
    ap_ip: str = ""


def _base_url(text: str) -> str:
    url = text.strip()
    if not url:
        return DEFAULT_CAMERA_HOST
    if not url.startswith(("http://", "https://")):
        url = f"http://{url}"
    return url.rstrip("/")


def _get_json(url: str, timeout: float = 2.0) -> dict:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        payload = response.read()
    return json.loads(payload.decode("utf-8"))


def _get_bytes(url: str, timeout: float = 3.0) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.read()


class WorkerSignals(QObject):
    ok = pyqtSignal(str, object)
    error = pyqtSignal(str, str)


class HttpTask(QRunnable):
    def __init__(self, name: str, fn, *args):
        super().__init__()
        self.name = name
        self.fn = fn
        self.args = args
        self.signals = WorkerSignals()

    def run(self):
        try:
            result = self.fn(*self.args)
        except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            self.signals.error.emit(self.name, str(exc))
            return
        self.signals.ok.emit(self.name, result)


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

    def set_value(self, value: str):
        self.value.setText(value)


class CameraMonitorWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32-CAM OV5640 Monitor")
        self.resize(1180, 760)

        self.pool = QThreadPool.globalInstance()
        self.status = CameraStatus()
        self._status_busy = False
        self._image_busy = False
        self._last_status_s: float | None = None

        self._init_ui()

        self.status_timer = QTimer(self)
        self.status_timer.setInterval(1000)
        self.status_timer.timeout.connect(self.refresh_status)
        self.status_timer.start()

        self.image_timer = QTimer(self)
        self.image_timer.setInterval(1500)
        self.image_timer.timeout.connect(self.refresh_image)

        self.refresh_status()

    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        controls = QGroupBox("相機控制")
        form = QFormLayout(controls)
        form.setContentsMargins(10, 10, 10, 10)

        self.host_edit = QLineEdit(DEFAULT_CAMERA_HOST)
        self.interval_spin = QSpinBox()
        self.interval_spin.setRange(200, 60000)
        self.interval_spin.setValue(1000)
        self.interval_spin.setSuffix(" ms")

        self.start_button = QPushButton("開始連拍")
        self.stop_button = QPushButton("停止連拍")
        self.capture_button = QPushButton("拍一張")
        self.preview_check = QCheckBox("自動更新預覽")
        self.preview_check.setChecked(True)

        button_row = QHBoxLayout()
        button_row.addWidget(self.start_button)
        button_row.addWidget(self.stop_button)
        button_row.addWidget(self.capture_button)
        button_row.addWidget(self.preview_check)
        button_row.addStretch(1)

        form.addRow("Camera URL / IP", self.host_edit)
        form.addRow("連拍間隔", self.interval_spin)
        form.addRow(button_row)

        metrics = QGroupBox("狀態")
        metric_grid = QGridLayout(metrics)
        self.burst_box = MetricBox("連拍")
        self.frames_box = MetricBox("已拍張數")
        self.failures_box = MetricBox("失敗")
        self.size_box = MetricBox("最新影像")
        self.uptime_box = MetricBox("Uptime")
        self.heap_box = MetricBox("Free Heap")
        self.psram_box = MetricBox("PSRAM")
        boxes = [
            self.burst_box,
            self.frames_box,
            self.failures_box,
            self.size_box,
            self.uptime_box,
            self.heap_box,
            self.psram_box,
        ]
        for index, box in enumerate(boxes):
            metric_grid.addWidget(box, index // 4, index % 4)

        self.image_label = QLabel("尚未取得影像")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(720, 480)
        self.image_label.setStyleSheet("border: 1px solid #cfcfcf; background: #111; color: #ddd;")

        self.status_label = QLabel("Status: --")
        self.status_label.setStyleSheet("color: #555;")

        root.addWidget(controls)
        root.addWidget(metrics)
        root.addWidget(self.image_label, 1)
        root.addWidget(self.status_label)

        self.start_button.clicked.connect(self.start_burst)
        self.stop_button.clicked.connect(self.stop_burst)
        self.capture_button.clicked.connect(self.capture_once)
        self.preview_check.toggled.connect(self._on_preview_toggled)

    def _camera_base_url(self) -> str:
        return _base_url(self.host_edit.text())

    def _submit(self, name: str, fn, *args):
        task = HttpTask(name, fn, *args)
        task.signals.ok.connect(self._on_task_ok)
        task.signals.error.connect(self._on_task_error)
        self.pool.start(task)

    def refresh_status(self):
        if self._status_busy:
            return
        self._status_busy = True
        self._submit("status", _get_json, f"{self._camera_base_url()}/status")

    def refresh_image(self):
        if self._image_busy or not self.preview_check.isChecked():
            return
        self._image_busy = True
        self._submit("image", _get_bytes, f"{self._camera_base_url()}/latest.jpg")

    def start_burst(self):
        query = urllib.parse.urlencode({"interval_ms": self.interval_spin.value()})
        self._submit("start", _get_json, f"{self._camera_base_url()}/burst/start?{query}")

    def stop_burst(self):
        self._submit("stop", _get_json, f"{self._camera_base_url()}/burst/stop")

    def capture_once(self):
        self._submit("capture", _get_json, f"{self._camera_base_url()}/capture")

    def _on_preview_toggled(self, enabled: bool):
        if enabled:
            self.image_timer.start()
            self.refresh_image()
        else:
            self.image_timer.stop()

    def _on_task_ok(self, name: str, result: object):
        if name == "status":
            self._status_busy = False
            self._apply_status(result)
            if not self.image_timer.isActive() and self.preview_check.isChecked():
                self.image_timer.start()
                self.refresh_image()
            return

        if name == "image":
            self._image_busy = False
            self._apply_image(result)
            return

        if name in {"start", "stop", "capture"}:
            self._apply_status(result)
            self.refresh_status()
            self.refresh_image()

    def _on_task_error(self, name: str, message: str):
        if name == "status":
            self._status_busy = False
        elif name == "image":
            self._image_busy = False
        self.status_label.setText(f"Status: {name} failed: {message}")

    def _apply_status(self, data: object):
        if not isinstance(data, dict):
            return
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
            ip=str(data.get("ip", "")),
            ap_ip=str(data.get("ap_ip", "")),
        )
        self._last_status_s = time.time()
        self._update_metrics()

    def _apply_image(self, data: object):
        if not isinstance(data, bytes):
            return
        pixmap = QPixmap()
        if not pixmap.loadFromData(data):
            self.status_label.setText("Status: latest.jpg is not a valid image")
            return
        scaled = pixmap.scaled(
            self.image_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.image_label.setPixmap(scaled)

    def _update_metrics(self):
        self.burst_box.set_value("ON" if self.status.burst else "OFF")
        self.frames_box.set_value(str(self.status.frames))
        self.failures_box.set_value(str(self.status.failures))
        self.size_box.set_value(f"{self.status.last_frame_bytes / 1024:.1f} KB")
        self.uptime_box.set_value(f"{self.status.uptime_ms / 1000:.1f} s")
        self.heap_box.set_value(f"{self.status.free_heap / 1024:.0f} KB")
        self.psram_box.set_value("YES" if self.status.psram else "NO")
        display_ip = self.status.ip or self.status.ap_ip or "--"
        self.status_label.setText(
            f"Status: connected {display_ip} | interval={self.status.interval_ms} ms"
        )

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.refresh_image()
