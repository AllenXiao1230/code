import time
from enum import Enum, auto
from config import LORA_TIMEOUT_SEC


class SerialStatus(Enum):
    DISCONNECTED = auto()
    CONNECTING = auto()
    CONNECTED = auto()


class LoRaStatus(Enum):
    NO_LINK = auto()
    LINK_OK = auto()


class ConnectionState:
    def __init__(self):
        self.serial_status = SerialStatus.DISCONNECTED
        self.lora_status = LoRaStatus.NO_LINK
        self._last_lora_time = 0.0

    def set_serial_connecting(self):
        self.serial_status = SerialStatus.CONNECTING

    def set_serial_connected(self):
        self.serial_status = SerialStatus.CONNECTED

    def set_serial_disconnected(self):
        self.serial_status = SerialStatus.DISCONNECTED

    def notify_lora_packet(self):
        self._last_lora_time = time.time()
        self.lora_status = LoRaStatus.LINK_OK

    def update(self):
        if time.time() - self._last_lora_time > LORA_TIMEOUT_SEC:
            self.lora_status = LoRaStatus.NO_LINK
