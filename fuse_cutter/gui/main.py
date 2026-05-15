import json
import math
import sys
import time
from dataclasses import dataclass, field


import serial
import serial.tools.list_ports
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


BAUD_RATES = ["115200", "230400", "460800", "921600"]


@dataclass
class FuseState:
    connected: bool = False
    armed: bool = False
    firing: bool = False
    bmp_ok: bool = False
    imu_ok: bool = False
    time_ms: int | None = None
    temp_c: float | None = None
    pressure_hpa: float | None = None
    alt_m: float | None = None
    ax: float | None = None
    ay: float | None = None
    az: float | None = None
    gx: float | None = None
    gy: float | None = None
    gz: float | None = None
    last_rx_s: float | None = None
    last_message: str = ""
    raw: dict = field(default_factory=dict)


class SerialReader(QThread):
    line_received = pyqtSignal(str)
    status_changed = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, port: str, baud: int, parent=None):
        super().__init__(parent)
        self.port = port
        self.baud = baud
        self._serial: serial.Serial | None = None
        self._running = True

    def run(self):
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=0.5)
            self.status_changed.emit(f"Connected: {self.port} @ {self.baud}")
        except serial.SerialException as exc:
            self.error_occurred.emit(f"Serial open failed: {exc}")
            return

        buffer = bytearray()
        while self._running:
            try:
                chunk = self._serial.read(256)
            except serial.SerialException as exc:
                self.error_occurred.emit(f"Serial read failed: {exc}")
                break
            if not chunk:
                continue
            buffer.extend(chunk)
            while b"\n" in buffer:
                line, _, buffer = buffer.partition(b"\n")
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    self.line_received.emit(text)

        if self._serial and self._serial.is_open:
            self._serial.close()
        self.status_changed.emit("Disconnected")

    def stop(self):
        self._running = False
        self.wait(1000)

    def send_command(self, command: str):
        if not self._serial or not self._serial.is_open:
            self.error_occurred.emit("Serial is not connected")
            return
        try:
            self._serial.write((command.strip() + "\n").encode("ascii"))
            self._serial.flush()
        except serial.SerialException as exc:
            self.error_occurred.emit(f"Serial write failed: {exc}")


class MetricBox(QWidget):
    def __init__(self, title: str, unit: str = "", parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(2)
        self.title_label = QLabel(title)
        self.title_label.setStyleSheet("color: #5f6670; font-size: 12px;")
        self.value_label = QLabel("--")
        self.value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.value_label.setStyleSheet("font-size: 22px; font-weight: 700; color: #17202a;")
        self.unit_label = QLabel(unit)
        self.unit_label.setStyleSheet("color: #737b86; font-size: 12px;")
        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)
        if unit:
            layout.addWidget(self.unit_label)
        self.setMinimumHeight(78)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setStyleSheet(
            "MetricBox { background: #fbfcfd; border: 1px solid #d9dde3; border-radius: 6px; }"
        )

    def set_value(self, text: str):
        self.value_label.setText(text)


class FuseCutterWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Fuse Cutter Control")
        self.resize(1180, 760)
        self.state = FuseState()
        self.reader: SerialReader | None = None

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(12)

        root.addWidget(self._build_connection_bar())
        root.addWidget(self._build_status_bar())

        body = QHBoxLayout()
        body.setSpacing(12)
        body.addWidget(self._build_metrics_panel(), 3)
        body.addWidget(self._build_control_panel(), 2)
        root.addLayout(body, 1)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(600)
        self.log.setPlaceholderText("Serial log")
        root.addWidget(self.log, 1)

        self.refresh_ports()
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.refresh_ui)
        self.ui_timer.start(100)
        self.refresh_ui()

    def _build_connection_bar(self) -> QWidget:
        box = QGroupBox("Serial")
        layout = QHBoxLayout(box)

        self.port_box = QComboBox()
        self.baud_box = QComboBox()
        self.baud_box.addItems(BAUD_RATES)
        self.baud_box.setCurrentText("115200")
        self.refresh_button = QPushButton("Refresh")
        self.connect_button = QPushButton("Connect")
        self.disconnect_button = QPushButton("Disconnect")

        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.connect_serial)
        self.disconnect_button.clicked.connect(self.disconnect_serial)

        layout.addWidget(QLabel("Port"))
        layout.addWidget(self.port_box, 2)
        layout.addWidget(QLabel("Baud"))
        layout.addWidget(self.baud_box)
        layout.addWidget(self.refresh_button)
        layout.addWidget(self.connect_button)
        layout.addWidget(self.disconnect_button)
        return box

    def _build_status_bar(self) -> QWidget:
        box = QGroupBox("State")
        layout = QGridLayout(box)
        self.connection_label = QLabel("DISCONNECTED")
        self.arm_label = QLabel("DISARMED")
        self.fire_label = QLabel("IDLE")
        self.bmp_label = QLabel("BMP: --")
        self.imu_label = QLabel("IMU: --")
        self.age_label = QLabel("Last RX: --")
        for label in (self.connection_label, self.arm_label, self.fire_label):
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumHeight(34)
            label.setStyleSheet("font-size: 16px; font-weight: 700;")
        layout.addWidget(self.connection_label, 0, 0)
        layout.addWidget(self.arm_label, 0, 1)
        layout.addWidget(self.fire_label, 0, 2)
        layout.addWidget(self.bmp_label, 1, 0)
        layout.addWidget(self.imu_label, 1, 1)
        layout.addWidget(self.age_label, 1, 2)
        return box

    def _build_metrics_panel(self) -> QWidget:
        box = QGroupBox("Telemetry")
        grid = QGridLayout(box)
        self.metrics = {
            "time_ms": MetricBox("Time", "ms"),
            "temp_c": MetricBox("Temperature", "deg C"),
            "pressure_hpa": MetricBox("Pressure", "hPa"),
            "alt_m": MetricBox("Altitude", "m"),
            "ax": MetricBox("Accel X", ""),
            "ay": MetricBox("Accel Y", ""),
            "az": MetricBox("Accel Z", ""),
            "gx": MetricBox("Gyro X", "dps"),
            "gy": MetricBox("Gyro Y", "dps"),
            "gz": MetricBox("Gyro Z", "dps"),
        }
        positions = [
            ("time_ms", 0, 0), ("temp_c", 0, 1), ("pressure_hpa", 0, 2),
            ("alt_m", 1, 0), ("ax", 1, 1), ("ay", 1, 2),
            ("az", 2, 0), ("gx", 2, 1), ("gy", 2, 2), ("gz", 3, 0),
        ]
        for name, row, col in positions:
            grid.addWidget(self.metrics[name], row, col)
        return box

    def _build_control_panel(self) -> QWidget:
        box = QGroupBox("Fuse Cutter")
        layout = QVBoxLayout(box)

        self.arm_button = QPushButton("ARM")
        self.disarm_button = QPushButton("DISARM")
        self.fire_button = QPushButton("FIRE")
        self.ping_button = QPushButton("PING")

        self.arm_button.clicked.connect(lambda: self.send_command("ARM"))
        self.disarm_button.clicked.connect(lambda: self.send_command("DISARM"))
        self.ping_button.clicked.connect(lambda: self.send_command("PING"))
        self.fire_button.clicked.connect(self.confirm_fire)

        self.arm_button.setMinimumHeight(44)
        self.disarm_button.setMinimumHeight(44)
        self.fire_button.setMinimumHeight(64)
        self.ping_button.setMinimumHeight(38)
        self.fire_button.setStyleSheet(
            "QPushButton { background: #b42318; color: white; font-size: 22px; font-weight: 800; }"
            "QPushButton:disabled { background: #d0d5dd; color: #667085; }"
        )

        layout.addWidget(self.arm_button)
        layout.addWidget(self.disarm_button)
        layout.addWidget(self.fire_button)
        layout.addSpacing(10)
        layout.addWidget(self.ping_button)
        layout.addStretch(1)

        pin_text = "Pins: CUT GPIO13, SDA GPIO8, SCL GPIO9"
        self.pin_label = QLabel(pin_text)
        self.pin_label.setWordWrap(True)
        self.pin_label.setStyleSheet("color: #4d5761;")
        layout.addWidget(self.pin_label)
        return box

    def refresh_ports(self):
        selected = self.port_box.currentData()
        self.port_box.clear()
        ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
        for port in ports:
            desc = port.description if port.description and port.description != "n/a" else ""
            label = f"{port.device} ({desc})" if desc else port.device
            self.port_box.addItem(label, port.device)
        if selected:
            index = self.port_box.findData(selected)
            if index >= 0:
                self.port_box.setCurrentIndex(index)

    def connect_serial(self):
        if self.reader:
            return
        port = self.port_box.currentData()
        if not port:
            QMessageBox.warning(self, "Serial", "No serial port selected.")
            return
        baud = int(self.baud_box.currentText())
        self.reader = SerialReader(port, baud, self)
        self.reader.line_received.connect(self.handle_line)
        self.reader.status_changed.connect(self.handle_status)
        self.reader.error_occurred.connect(self.handle_error)
        self.reader.start()
        self.state.connected = True
        self.append_log(f"> connecting {port} @ {baud}")
        self.refresh_ui()

    def disconnect_serial(self):
        if self.reader:
            self.reader.stop()
            self.reader = None
        self.state = FuseState()
        self.refresh_ui()

    def closeEvent(self, event):
        self.disconnect_serial()
        super().closeEvent(event)

    def send_command(self, command: str):
        if not self.reader:
            return
        self.reader.send_command(command)
        self.append_log(f"> {command}")

    def confirm_fire(self):
        if not self.state.armed:
            QMessageBox.warning(self, "FIRE blocked", "Device must be armed before firing.")
            return
        answer = QMessageBox.question(
            self,
            "Confirm FIRE",
            "Send FIRE command to GPIO13 for 1500 ms?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.send_command("FIRE")

    def handle_line(self, line: str):
        self.append_log(line)
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            return

        msg_type = payload.get("type", "")
        self.state.raw = payload
        self.state.last_rx_s = time.monotonic()
        if msg_type == "telemetry":
            self.apply_telemetry(payload)
        elif msg_type == "ack":
            self.state.last_message = f"ACK: {payload.get('cmd', '')}"
            if "armed" in payload:
                self.state.armed = bool(payload["armed"])
            if "firing" in payload:
                self.state.firing = bool(payload["firing"])
        elif msg_type == "status":
            self.state.last_message = str(payload.get("msg", "status"))
        elif msg_type == "error":
            self.state.last_message = f"ERROR: {payload.get('msg', '')}"
        self.refresh_ui()

    def apply_telemetry(self, payload: dict):
        self.state.time_ms = payload.get("time_ms")
        self.state.armed = bool(payload.get("armed", False))
        self.state.firing = bool(payload.get("firing", False))
        self.state.bmp_ok = bool(payload.get("bmp_ok", False))
        self.state.imu_ok = bool(payload.get("imu_ok", False))
        for name in ("temp_c", "pressure_hpa", "alt_m", "ax", "ay", "az", "gx", "gy", "gz"):
            setattr(self.state, name, self._number_or_none(payload.get(name)))

    def handle_status(self, message: str):
        self.state.connected = message.startswith("Connected:")
        self.state.last_message = message
        self.append_log(f"# {message}")
        if not self.state.connected and self.reader:
            self.reader = None
        self.refresh_ui()

    def handle_error(self, message: str):
        self.state.last_message = message
        self.append_log(f"! {message}")
        self.refresh_ui()

    def refresh_ui(self):
        connected = self.reader is not None and self.state.connected
        self.connect_button.setEnabled(not connected)
        self.disconnect_button.setEnabled(connected)
        self.refresh_button.setEnabled(not connected)
        self.port_box.setEnabled(not connected)
        self.baud_box.setEnabled(not connected)
        self.arm_button.setEnabled(connected and not self.state.armed)
        self.disarm_button.setEnabled(connected and (self.state.armed or self.state.firing))
        self.fire_button.setEnabled(connected and self.state.armed and not self.state.firing)
        self.ping_button.setEnabled(connected)

        self.connection_label.setText("CONNECTED" if connected else "DISCONNECTED")
        self.connection_label.setStyleSheet(self._badge_style("#027a48" if connected else "#b42318"))
        self.arm_label.setText("ARMED" if self.state.armed else "DISARMED")
        self.arm_label.setStyleSheet(self._badge_style("#b54708" if self.state.armed else "#344054"))
        self.fire_label.setText("FIRING" if self.state.firing else "IDLE")
        self.fire_label.setStyleSheet(self._badge_style("#b42318" if self.state.firing else "#175cd3"))
        self.bmp_label.setText(f"BMP: {'OK' if self.state.bmp_ok else '--'}")
        self.imu_label.setText(f"IMU: {'OK' if self.state.imu_ok else '--'}")
        self.age_label.setText(f"Last RX: {self._age_text()}")

        self.metrics["time_ms"].set_value("--" if self.state.time_ms is None else str(self.state.time_ms))
        for name in ("temp_c", "pressure_hpa", "alt_m", "ax", "ay", "az", "gx", "gy", "gz"):
            self.metrics[name].set_value(self._format_number(getattr(self.state, name)))

    def append_log(self, text: str):
        self.log.appendPlainText(text)
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    @staticmethod
    def _number_or_none(value):
        if value is None:
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    @staticmethod
    def _format_number(value: float | None) -> str:
        return "--" if value is None else f"{value:.3f}"

    @staticmethod
    def _badge_style(color: str) -> str:
        return (
            f"background: {color}; color: white; border-radius: 5px; "
            "font-size: 16px; font-weight: 700; padding: 6px;"
        )

    def _age_text(self) -> str:
        if self.state.last_rx_s is None:
            return "--"
        age_ms = int((time.monotonic() - self.state.last_rx_s) * 1000)
        return f"{age_ms} ms"


def main() -> int:
    app = QApplication(sys.argv)
    window = FuseCutterWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
