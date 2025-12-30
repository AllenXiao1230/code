from PyQt5.QtWidgets import QWidget, QLabel, QGridLayout, QGroupBox


class TimePanel(QWidget):
    def __init__(self):
        super().__init__()

        box = QGroupBox("Time")
        g = QGridLayout(box)

        self.lbl_time = QLabel("0.0 s")
        self.lbl_tplus = QLabel("T+---.- s")

        self.lbl_time.setStyleSheet("font-size: 18px;")

        g.addWidget(self.lbl_time, 0, 0)
        g.addWidget(self.lbl_tplus, 1, 0)

        layout = QGridLayout(self)
        layout.addWidget(box, 0, 0)

    def update_time(self, time_s, t_plus):
        self.lbl_time.setText(f"{time_s:.1f} s")
        self.lbl_tplus.setText(
            "T+---.- s" if t_plus is None else f"T+{t_plus:.1f} s"
        )
