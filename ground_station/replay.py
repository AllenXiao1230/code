# replay.py
from PyQt5.QtCore import QThread, pyqtSignal
import time


class ReplayWorker(QThread):
    """
    Replay telemetry data as if it comes from serial.
    Emits raw bytes to be fed into the serial data handler.
    """

    data_emitted = pyqtSignal(bytes)
    status_changed = pyqtSignal(str)

    def __init__(self, data: list, speed: float = 1.0, parent=None):
        """
        data: list of (timestamp, raw_bytes)
        speed: replay speed multiplier
        """
        super().__init__(parent)
        self.data = data
        self.speed = speed
        self._running = False
        self._paused = False

    def run(self):
        if not self.data:
            return

        self._running = True
        self.status_changed.emit("Replay started")

        t0 = self.data[0][0]
        start = time.time()

        for ts, raw in self.data:
            if not self._running:
                break

            while self._paused and self._running:
                time.sleep(0.1)

            dt = (ts - t0) / self.speed
            while time.time() - start < dt:
                time.sleep(0.001)

            self.data_emitted.emit(raw)

        self.status_changed.emit("Replay finished")

    def stop(self):
        self._running = False
        self.wait()

    def pause(self):
        self._paused = True
        self.status_changed.emit("Replay paused")

    def resume(self):
        self._paused = False
        self.status_changed.emit("Replay resumed")
