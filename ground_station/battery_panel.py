# battery_panel.py
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt


class BatteryPanel(QWidget):
    """
    Display battery voltage.
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)

        self.label = QLabel("Battery: --.- V")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: white;"
        )

        layout.addWidget(self.label)

    # ---------- Public API ----------

    def update_voltage(self, voltage: float):
        """
        Called by StateMachine.battery_updated signal.
        """
        try:
            self.label.setText(f"Battery: {voltage:.2f} V")
        except Exception:
            pass
