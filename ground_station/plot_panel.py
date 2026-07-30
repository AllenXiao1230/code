# plot_panel.py
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from collections import deque
import time


class PlotPanel(QWidget):
    """
    Altitude plot panel with rate-limited redraw.
    """

    WINDOW_SEC = 30         # sliding window in seconds
    REFRESH_HZ = 5          # redraw frequency

    def __init__(self, parent=None):
        super().__init__(parent)

        self._times = deque()
        self._alts = deque()
        self._t_start = None
        self._last_elapsed = 0.0

        self._init_ui()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._redraw)
        self._timer.start(int(1000 / self.REFRESH_HZ))

    def _init_ui(self):
        layout = QVBoxLayout(self)

        self.figure = Figure(figsize=(5, 3))
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.ax = self.figure.add_subplot(111)

        self.ax.set_title("Altitude")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Altitude (m)")
        self.ax.grid(True)

        self._line, = self.ax.plot([], [], color="cyan")

        layout.addWidget(self.canvas)

    # ---------- Public API ----------

    def update_altitude(self, altitude: float):
        """
        Called by StateMachine signal.
        """
        now = time.time()
        if self._t_start is None:
            self._t_start = now
        self._times.append(now)
        self._alts.append(altitude)
        self._last_elapsed = now - self._t_start

    # ---------- Internal ----------

    def _redraw(self):
        if not self._times:
            return

        # Keep only data within the sliding window
        t_last = self._times[-1]
        cutoff = t_last - self.WINDOW_SEC
        while self._times and self._times[0] < cutoff:
            self._times.popleft()
            self._alts.popleft()

        t0 = self._t_start if self._t_start is not None else self._times[0]
        times = [t - t0 for t in self._times]

        self._line.set_data(times, self._alts)
        x_max = self._last_elapsed
        x_min = max(0, x_max - self.WINDOW_SEC)
        self.ax.set_xlim(x_min, x_max)
        self.ax.relim()
        self.ax.autoscale_view()

        self.canvas.draw_idle()
