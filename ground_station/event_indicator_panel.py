# event_indicator_panel.py
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QListWidget


class EventIndicatorPanel(QWidget):
    """
    Simple event log panel.
    Receives string messages and displays them.
    """

    MAX_EVENTS = 200

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        self.list_widget = QListWidget()

        layout.addWidget(self.list_widget)

    # ---------- Public API ----------

    def add_event(self, message: str):
        if not message:
            return

        self.list_widget.addItem(message)

        # Limit event count
        if self.list_widget.count() > self.MAX_EVENTS:
            self.list_widget.takeItem(0)

        # Auto scroll
        self.list_widget.scrollToBottom()
