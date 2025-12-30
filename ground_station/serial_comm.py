# serial_comm.py
from PyQt5.QtCore import QThread, pyqtSignal
import serial
import time


class SerialWorker(QThread):
    """
    Serial I/O worker running in its own thread.
    UI must NEVER directly touch serial.
    """

    data_received = pyqtSignal(bytes)
    status_changed = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, port: str, baudrate: int = 115200, parent=None):
        super().__init__(parent)
        self.port = port
        self.baudrate = baudrate
        self._running = False
        self._ser = None

    def run(self):
        try:
            self._ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1  # NON-blocking
            )
            self._running = True
            self.status_changed.emit(f"Connected to {self.port}")

            while self._running:
                if self._ser.in_waiting:
                    try:
                        data = self._ser.read(self._ser.in_waiting)
                        if data:
                            self.data_received.emit(data)
                    except Exception as e:
                        self.error_occurred.emit(str(e))
                time.sleep(0.01)

        except Exception as e:
            self.error_occurred.emit(f"Serial open failed: {e}")

        finally:
            self._cleanup()

    def write(self, data: bytes):
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(data)
            except Exception as e:
                self.error_occurred.emit(f"Write failed: {e}")

    def stop(self):
        self._running = False
        self.wait()

    def _cleanup(self):
        if self._ser:
            try:
                self._ser.close()
                self.status_changed.emit("Serial disconnected")
            except Exception:
                pass
            self._ser = None
