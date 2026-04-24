# serial_panel.py
from PyQt5.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QComboBox, QLabel
)
from PyQt5.QtCore import pyqtSignal
import serial.tools.list_ports


class SerialPanel(QWidget):
    """
    Serial control panel.
    Emits connect / disconnect requests.
    """

    connect_requested = pyqtSignal(str, int)
    disconnect_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(4)

        layout = QHBoxLayout()

        layout.addWidget(QLabel("Port:"))

        self.port_box = QComboBox()
        layout.addWidget(self.port_box)

        layout.addWidget(QLabel("Baud:"))

        self.baud_box = QComboBox()
        self.baud_box.addItems(["9600", "57600", "115200", "230400"])
        self.baud_box.setCurrentText("115200")
        layout.addWidget(self.baud_box)

        self.refresh_btn = QPushButton("Refresh")
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)

        layout.addWidget(self.refresh_btn)
        layout.addWidget(self.connect_btn)
        layout.addWidget(self.disconnect_btn)

        self.refresh_btn.clicked.connect(self._refresh_ports)
        self.connect_btn.clicked.connect(self._on_connect)
        self.disconnect_btn.clicked.connect(self._on_disconnect)
        self._refresh_ports()

        outer.addLayout(layout)

        self.status_label = QLabel("Status: DISCONNECTED")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        outer.addWidget(self.status_label)

        self.diag_label = QLabel("")
        self.diag_label.setStyleSheet("color: #666;")
        outer.addWidget(self.diag_label)

        self.notice_label = QLabel("")
        self.notice_label.setWordWrap(True)
        self.notice_label.setStyleSheet("color: #a35f00; font-weight: bold;")
        outer.addWidget(self.notice_label)

        self.reset_diagnostics()

    # ---------- Internal ----------

    @staticmethod
    def _port_priority(port_info) -> tuple[int, str]:
        device = (port_info.device or "").lower()
        desc = (port_info.description or "").lower()
        hwid = (port_info.hwid or "").lower()
        text = " ".join((device, desc, hwid))

        if any(token in text for token in ("wchusbserial", "usb serial", "usbserial", "usbmodem", "slab_usbtouart", "ttyusb", "ttyacm")):
            return (0, device)
        if "usb" in text:
            return (1, device)
        if any(token in text for token in ("bluetooth", "debug-console")):
            return (3, device)
        return (2, device)

    @staticmethod
    def _port_label(port_info) -> str:
        desc = (port_info.description or "").strip()
        if not desc or desc.lower() == "n/a":
            return port_info.device
        return f"{port_info.device} ({desc})"

    def _refresh_ports(self):
        selected_device = self.port_box.currentData()
        self.port_box.clear()
        ports = sorted(serial.tools.list_ports.comports(), key=self._port_priority)
        if not ports:
            self.port_box.addItem("No serial ports detected", "")
            self.connect_btn.setEnabled(False)
            return

        for p in ports:
            self.port_box.addItem(self._port_label(p), p.device)

        restored_index = -1
        if selected_device:
            restored_index = self.port_box.findData(selected_device)
        if restored_index >= 0:
            self.port_box.setCurrentIndex(restored_index)
        else:
            self.port_box.setCurrentIndex(0)
        self.connect_btn.setEnabled(True)

    def _on_connect(self):
        self._refresh_ports()
        port = self.port_box.currentData()
        baud = int(self.baud_box.currentText())

        if port:
            self.connect_requested.emit(port, baud)
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.set_status("Status: CONNECTING...", color="orange")

    def _on_disconnect(self):
        self.disconnect_requested.emit()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.set_status("Status: DISCONNECTED", color="red")
        self.reset_diagnostics()

    # ---------- Public API ----------

    def set_connected(self, connected: bool):
        self.connect_btn.setEnabled(not connected and bool(self.port_box.currentData()))
        self.refresh_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)

    def set_status(self, text: str, color: str = None):
        self.status_label.setText(text)
        if color:
            self.status_label.setStyleSheet(f"color: {color}; font-weight: bold;")

    def set_diagnostics(
        self,
        rx_bytes: int,
        valid_frames: int,
        crc_fail: int,
        resync_count: int,
        last_frame_age_ms: int | None,
    ):
        if last_frame_age_ms is None:
            age_text = "--"
        else:
            age_text = f"{int(last_frame_age_ms)} ms"
        self.diag_label.setText(
            "Diag: "
            f"rx_bytes={rx_bytes} "
            f"valid_frames={valid_frames} "
            f"crc_fail={crc_fail} "
            f"resync={resync_count} "
            f"last_frame_age={age_text}"
        )

    def set_diagnostic_notice(self, text: str = "", color: str = "#a35f00"):
        self.notice_label.setText(text)
        self.notice_label.setStyleSheet(f"color: {color}; font-weight: bold;")

    def reset_diagnostics(self):
        self.set_diagnostics(
            rx_bytes=0,
            valid_frames=0,
            crc_fail=0,
            resync_count=0,
            last_frame_age_ms=None,
        )
        self.set_diagnostic_notice("")
