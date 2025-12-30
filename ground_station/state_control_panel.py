# state_control_panel.py
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QComboBox, QPushButton, QHBoxLayout
from PyQt5.QtCore import Qt, pyqtSignal
from state_machine import FlightState


class StateControlPanel(QWidget):
    """
    Flight state panel with dropdown and upload button.
    """

    upload_requested = pyqtSignal(FlightState)

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        self.label = QLabel("State: IDLE")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 18px; font-weight: bold;")

        self.combo = QComboBox()
        for s in FlightState:
            self.combo.addItem(s.name, s)

        self.upload_btn = QPushButton("Upload")
        self.upload_btn.clicked.connect(self._emit_upload)

        row = QHBoxLayout()
        row.addWidget(self.combo)
        row.addWidget(self.upload_btn)

        layout.addWidget(self.label)
        layout.addLayout(row)

    # ---------- Public API ----------

    def update_state(self, state: FlightState):
        """
        Called by StateMachine.state_changed signal.
        """
        if not isinstance(state, FlightState):
            return

        self.label.setText(f"State: {state.name}")
        idx = self.combo.findText(state.name)
        if idx >= 0:
            self.combo.setCurrentIndex(idx)

    # ---------- Internal ----------

    def _emit_upload(self):
        state = self.combo.currentData()
        if isinstance(state, FlightState):
            self.upload_requested.emit(state)
