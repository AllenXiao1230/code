from PyQt5.QtWidgets import (
    QWidget,
    QLabel,
    QGridLayout,
    QGroupBox,
    QVBoxLayout,
    QPushButton,
)
from PyQt5.QtCore import pyqtSignal
from state_machine import decode_state, decode_error
from connection_state import SerialStatus, LoRaStatus


class ConnectionPanel(QWidget):
    def __init__(self):
        super().__init__()
        box = QGroupBox("Connection Status")
        g = QGridLayout(box)

        self.lbl_serial = QLabel("---")
        self.lbl_lora = QLabel("---")
        self.lbl_last = QLabel("---")

        g.addWidget(QLabel("PC ↔ ESP32"), 0, 0)
        g.addWidget(self.lbl_serial, 0, 1)
        g.addWidget(QLabel("LoRa Link"), 1, 0)
        g.addWidget(self.lbl_lora, 1, 1)
        g.addWidget(QLabel("Last Packet"), 2, 0)
        g.addWidget(self.lbl_last, 2, 1)

        layout = QGridLayout(self)
        layout.addWidget(box, 0, 0)

    def update_state(self, conn):
        if conn.serial_status == SerialStatus.CONNECTED:
            self.lbl_serial.setText("CONNECTED")
        elif conn.serial_status == SerialStatus.CONNECTING:
            self.lbl_serial.setText("CONNECTING")
        else:
            self.lbl_serial.setText("DISCONNECTED")

        if conn.lora_status == LoRaStatus.LINK_OK:
            self.lbl_lora.setText("LINK OK")
        else:
            self.lbl_lora.setText("NO LINK")

        self.lbl_last.setText(f"{conn._last_lora_time:.1f}")


class TelemetryPanel(QWidget):
    def __init__(self):
        super().__init__()
        layout = QGridLayout(self)

        # ===== Flight Status =====
        fs = QGroupBox("飛行狀態")
        f = QGridLayout(fs)
        self.lbl_state = QLabel()
        self.lbl_error = QLabel()
        self.lbl_time = QLabel()
        self.lbl_water = QLabel()

        f.addWidget(QLabel("狀態"), 0, 0)
        f.addWidget(self.lbl_state, 0, 1)
        f.addWidget(QLabel("錯誤碼"), 1, 0)
        f.addWidget(self.lbl_error, 1, 1)
        f.addWidget(QLabel("時間 (秒)"), 2, 0)
        f.addWidget(self.lbl_time, 2, 1)
        f.addWidget(QLabel("落水"), 3, 0)
        f.addWidget(self.lbl_water, 3, 1)

        # ===== GPS / Yaw =====
        gps = QGroupBox("定位 / 偏航")
        g = QGridLayout(gps)
        self.lbl_lat = QLabel()
        self.lbl_lon = QLabel()
        self.lbl_head = QLabel()
        self.lbl_sat = QLabel()

        items = [
            ("緯度 (GPS)", self.lbl_lat),
            ("經度 (GPS)", self.lbl_lon),
            ("偏航 (IMU, 度)", self.lbl_head),
            ("衛星數 (GPS)", self.lbl_sat),
        ]
        for i, (n, l) in enumerate(items):
            g.addWidget(QLabel(n), i, 0)
            g.addWidget(l, i, 1)

        # ===== IMU =====
        imu = QGroupBox("姿態 / IMU")
        i = QGridLayout(imu)
        self.lbl_roll = QLabel()
        self.lbl_pitch = QLabel()
        self.lbl_accx = QLabel()
        self.lbl_accy = QLabel()
        self.lbl_accz = QLabel()
        self.lbl_gx = QLabel()
        self.lbl_gy = QLabel()
        self.lbl_gz = QLabel()

        i.addWidget(QLabel("滾轉 (IMU, 度)"), 0, 0)
        i.addWidget(self.lbl_roll, 0, 1)
        i.addWidget(QLabel("俯仰 (IMU, 度)"), 1, 0)
        i.addWidget(self.lbl_pitch, 1, 1)
        i.addWidget(QLabel("加速度X (ADXL, g)"), 2, 0)
        i.addWidget(self.lbl_accx, 2, 1)
        i.addWidget(QLabel("加速度Y (ADXL, g)"), 3, 0)
        i.addWidget(self.lbl_accy, 3, 1)
        i.addWidget(QLabel("加速度Z (ADXL, g)"), 4, 0)
        i.addWidget(self.lbl_accz, 4, 1)
        i.addWidget(QLabel("陀螺X (ICM, 度/秒)"), 5, 0)
        i.addWidget(self.lbl_gx, 5, 1)
        i.addWidget(QLabel("陀螺Y (ICM, 度/秒)"), 6, 0)
        i.addWidget(self.lbl_gy, 6, 1)
        i.addWidget(QLabel("陀螺Z (ICM, 度/秒)"), 7, 0)
        i.addWidget(self.lbl_gz, 7, 1)

        # ===== Power =====
        pwr = QGroupBox("環境感測")
        p = QGridLayout(pwr)
        self.lbl_temp = QLabel()
        self.lbl_hum = QLabel()
        self.lbl_press = QLabel()

        p.addWidget(QLabel("溫度 (°C)"), 0, 0)
        p.addWidget(self.lbl_temp, 0, 1)
        p.addWidget(QLabel("濕度 (%)"), 1, 0)
        p.addWidget(self.lbl_hum, 1, 1)
        p.addWidget(QLabel("氣壓 (kPa)"), 2, 0)
        p.addWidget(self.lbl_press, 2, 1)

        layout.addWidget(fs, 0, 0)
        layout.addWidget(gps, 1, 0)
        layout.addWidget(imu, 2, 0)
        layout.addWidget(pwr, 3, 0)

    def update_data(self, d):
        data = d or {}
        self.lbl_state.setText(decode_state(data.get("status")))
        self.lbl_error.setText(decode_error(data.get("error")))
        self.lbl_time.setText(self._fmt(data.get("time"), "{:.1f}"))
        self.lbl_water.setText("是" if data.get("water") else "否")

        self.lbl_lat.setText(self._fmt(data.get("lat"), "{:.7f}"))
        self.lbl_lon.setText(self._fmt(data.get("lon"), "{:.7f}"))
        self.lbl_head.setText(self._fmt(data.get("heading"), "{:.1f}"))
        self.lbl_sat.setText(self._fmt(data.get("sat"), "{}", fallback="0"))

        self.lbl_roll.setText(self._fmt(data.get("roll"), "{:.2f}"))
        self.lbl_pitch.setText(self._fmt(data.get("pitch"), "{:.2f}"))
        self.lbl_accx.setText(self._fmt(data.get("accx"), "{:.2f}"))
        self.lbl_accy.setText(self._fmt(data.get("accy"), "{:.2f}"))
        self.lbl_accz.setText(self._fmt(data.get("accz"), "{:.2f}"))
        self.lbl_gx.setText(self._fmt(data.get("gyro_x"), "{:.1f}"))
        self.lbl_gy.setText(self._fmt(data.get("gyro_y"), "{:.1f}"))
        self.lbl_gz.setText(self._fmt(data.get("gyro"), "{:.1f}"))

        self.lbl_temp.setText(self._fmt(data.get("temp"), "{:.2f}"))
        self.lbl_hum.setText(self._fmt(data.get("hum"), "{:.1f}"))
        self.lbl_press.setText(self._fmt(data.get("pressure"), "{:.3f}", fallback="---"))

    @staticmethod
    def _fmt(val, fmt, fallback="---"):
        try:
            if val is None:
                return fallback
            return fmt.format(val)
        except Exception:
            return fallback


class SpeedAltitudePanel(QWidget):
    """
    Speed and altitude display with GPS / Baro channels.
    """

    def __init__(self):
        super().__init__()
        layout = QGridLayout(self)
        layout.setHorizontalSpacing(16)
        layout.setVerticalSpacing(6)

        # Column titles
        speed_title = QLabel("速度")
        alt_title = QLabel("高度")
        speed_title.setStyleSheet("font-weight: bold; border-bottom: 1px solid #ccc;")
        alt_title.setStyleSheet("font-weight: bold; border-bottom: 1px solid #ccc;")
        layout.addWidget(speed_title, 0, 0, 1, 2)
        layout.addWidget(alt_title, 0, 2, 1, 2)

        # Speed labels
        layout.addWidget(QLabel("GPS 速度 (m/s)"), 1, 0)
        layout.addWidget(QLabel("氣壓 速度 (m/s)"), 2, 0)
        self.lbl_speed_gps = QLabel("0.00")
        self.lbl_speed_baro = QLabel("0.00")
        layout.addWidget(self.lbl_speed_gps, 1, 1)
        layout.addWidget(self.lbl_speed_baro, 2, 1)

        # Altitude labels
        layout.addWidget(QLabel("GPS 高度 (m)"), 1, 2)
        layout.addWidget(QLabel("氣壓 高度 (m)"), 2, 2)
        self.lbl_alt_gps = QLabel("0.0")
        self.lbl_alt_baro = QLabel("0.0")
        layout.addWidget(self.lbl_alt_gps, 1, 3)
        layout.addWidget(self.lbl_alt_baro, 2, 3)

        # Visual separator between speed and altitude columns
        for row in range(0, 3):
            sep = QLabel()
            sep.setStyleSheet("border-left: 1px solid #bbb;")
            layout.addWidget(sep, row, 2 - 1)
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 0)
        layout.setColumnStretch(3, 1)

    def update_data(self, gps_speed, baro_speed, gps_alt, baro_alt):
        self._set(self.lbl_speed_gps, gps_speed, "{:.2f}", "0.00")
        self._set(self.lbl_speed_baro, baro_speed, "{:.2f}", "0.00")
        self._set(self.lbl_alt_gps, gps_alt, "{:.1f}", "0.0")
        self._set(self.lbl_alt_baro, baro_alt, "{:.1f}", "0.0")

    @staticmethod
    def _set(label: QLabel, val, fmt, fallback):
        try:
            if val is None:
                label.setText(fallback)
            else:
                label.setText(fmt.format(val))
        except Exception:
            label.setText(fallback)


class LogControlPanel(QWidget):
    """
    Simple log control: start/stop CSV logging.
    """

    logging_toggled = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        self.status = QLabel("狀態: 未記錄")
        self.btn = QPushButton("開始記錄")
        self.btn.clicked.connect(self._on_click)

        layout.addWidget(self.status)
        layout.addWidget(self.btn)

        self._active = False

    def _on_click(self):
        self._active = not self._active
        self.btn.setText("停止記錄" if self._active else "開始記錄")
        self.status.setText("狀態: 記錄中" if self._active else "狀態: 未記錄")
        self.logging_toggled.emit(self._active)

    def set_active(self, active: bool):
        self._active = active
        self.btn.setText("停止記錄" if active else "開始記錄")
        self.status.setText("狀態: 記錄中" if active else "狀態: 未記錄")
