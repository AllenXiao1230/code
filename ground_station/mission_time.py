# mission_time.py
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import Qt


class MissionTimePanel(QWidget):
    """
    Display mission elapsed time (seconds).
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)

        self.label = QLabel("T + 0.0 s")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet(
            "font-size: 20px; font-weight: bold; color: white;"
        )

        layout.addWidget(self.label)

    # ---------- Public API ----------

    def update_time(self, seconds: float):
        """
        Called by StateMachine.timestamp_updated signal.
        """
        try:
            self.label.setText(f"T + {seconds:.1f} s")
        except Exception:
            pass
