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
        self._refresh_ports()
        layout.addWidget(self.port_box)

        layout.addWidget(QLabel("Baud:"))

        self.baud_box = QComboBox()
        self.baud_box.addItems(["9600", "57600", "115200", "230400"])
        self.baud_box.setCurrentText("115200")
        layout.addWidget(self.baud_box)

        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)

        layout.addWidget(self.connect_btn)
        layout.addWidget(self.disconnect_btn)

        self.connect_btn.clicked.connect(self._on_connect)
        self.disconnect_btn.clicked.connect(self._on_disconnect)

        outer.addLayout(layout)

        self.status_label = QLabel("Status: DISCONNECTED")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        outer.addWidget(self.status_label)

    # ---------- Internal ----------

    def _refresh_ports(self):
        self.port_box.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_box.addItem(p.device)

    def _on_connect(self):
        port = self.port_box.currentText()
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

    # ---------- Public API ----------

    def set_status(self, text: str, color: str = None):
        self.status_label.setText(text)
        if color:
            self.status_label.setStyleSheet(f"color: {color}; font-weight: bold;")
