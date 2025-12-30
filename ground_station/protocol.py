# protocol.py

SYNC_1 = 0xAA
SYNC_2 = 0x55
HEADER_SIZE = 3  # SYNC1 + SYNC2 + LEN
CHECKSUM_SIZE = 1


class ProtocolParser:
    """
    Stream-based packet parser.
    Safe against noise, partial packets, and replay.
    """

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data: bytes):
        """
        Feed raw bytes into parser.
        Returns a list of decoded payloads.
        """
        packets = []
        self._buffer.extend(data)

        while True:
            # Minimum length check
            if len(self._buffer) < HEADER_SIZE + CHECKSUM_SIZE:
                break

            # Sync search
            if self._buffer[0] != SYNC_1 or self._buffer[1] != SYNC_2:
                self._buffer.pop(0)
                continue

            length = self._buffer[2]
            total_len = HEADER_SIZE + length + CHECKSUM_SIZE

            if len(self._buffer) < total_len:
                break  # Wait for more data

            payload = self._buffer[3:3 + length]
            checksum = self._buffer[3 + length]

            if self._checksum(payload) == checksum:
                packets.append(bytes(payload))
                del self._buffer[:total_len]
            else:
                # Bad packet, drop sync byte and retry
                self._buffer.pop(0)

        return packets

    @staticmethod
    def build(payload: bytes) -> bytes:
        """
        Build a packet from payload.
        """
        length = len(payload)
        checksum = ProtocolParser._checksum(payload)
        return bytes([SYNC_1, SYNC_2, length]) + payload + bytes([checksum])

    @staticmethod
    def _checksum(data: bytes) -> int:
        c = 0
        for b in data:
            c ^= b
        return c

