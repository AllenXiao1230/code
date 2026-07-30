from dataclasses import dataclass
import struct


FRAME_HEADER_0 = 0x55
FRAME_HEADER_1 = 0xAA
FRAME_SYNC = bytes((FRAME_HEADER_0, FRAME_HEADER_1))
PACKET_LEN = 47
INVALID_I16 = -32768
INVALID_BARO_ALT_DM = INVALID_I16


class FrameDecodeError(ValueError):
    pass


@dataclass(frozen=True)
class TelemetryFrame:
    time_ds: int
    lat_raw: int
    lon_raw: int
    gps_alt_dm: int
    gps_speed_dms: int
    sat_count: int
    roll_cdeg: int
    pitch_cdeg: int
    yaw_ddeg: int
    gyro_x_ddeg_s: int
    gyro_y_ddeg_s: int
    gyro_z_ddeg_s: int
    accx_cg: int
    accy_cg: int
    accz_cg: int
    baro_alt_dm: int
    battery_mv: int
    servo_power_mv: int
    servo_angle_ddeg: int
    flight_state: int
    error_code: int
    water_detected: int

    @property
    def time_s(self) -> float:
        return self.time_ds / 10.0

    @property
    def lat_deg(self) -> float:
        return self.lat_raw / 1e7

    @property
    def lon_deg(self) -> float:
        return self.lon_raw / 1e7

    @property
    def gps_alt_m(self) -> float:
        return self.gps_alt_dm / 10.0

    @property
    def gps_speed_ms(self) -> float:
        return self.gps_speed_dms / 10.0

    @property
    def heading_deg(self) -> float:
        return (self.yaw_ddeg / 10.0) % 360.0

    @property
    def roll_deg(self) -> float:
        return self.roll_cdeg / 100.0

    @property
    def pitch_deg(self) -> float:
        return self.pitch_cdeg / 100.0

    @property
    def gyro_x_dps(self) -> float:
        return self.gyro_x_ddeg_s / 10.0

    @property
    def gyro_y_dps(self) -> float:
        return self.gyro_y_ddeg_s / 10.0

    @property
    def gyro_z_dps(self) -> float:
        return self.gyro_z_ddeg_s / 10.0

    @property
    def accx_g(self) -> float:
        return self.accx_cg / 100.0

    @property
    def accy_g(self) -> float:
        return self.accy_cg / 100.0

    @property
    def accz_g(self) -> float:
        return self.accz_cg / 100.0

    @property
    def has_baro_alt(self) -> bool:
        return self.baro_alt_dm != INVALID_BARO_ALT_DM

    @property
    def baro_alt_m(self) -> float | None:
        if not self.has_baro_alt:
            return None
        return self.baro_alt_dm / 10.0

    @property
    def battery_v(self) -> float:
        return self.battery_mv / 1000.0

    @property
    def servo_power_v(self) -> float:
        return self.servo_power_mv / 1000.0

    @property
    def servo_angle_deg(self) -> float:
        return self.servo_angle_ddeg / 10.0


def _xor_crc(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc


def _pack_into(fmt: str, buf: bytearray, offset: int, value: int) -> None:
    struct.pack_into(fmt, buf, offset, value)


def build_frame(
    *,
    time_ds: int = 0,
    lat_raw: int = 0,
    lon_raw: int = 0,
    gps_alt_dm: int = 0,
    gps_speed_dms: int = 0,
    sat_count: int = 0,
    roll_cdeg: int = 0,
    pitch_cdeg: int = 0,
    yaw_ddeg: int = 0,
    gyro_x_ddeg_s: int = 0,
    gyro_y_ddeg_s: int = 0,
    gyro_z_ddeg_s: int = 0,
    accx_cg: int = 0,
    accy_cg: int = 0,
    accz_cg: int = 0,
    baro_alt_dm: int = 0,
    battery_mv: int = 0,
    servo_power_mv: int = 0,
    servo_angle_ddeg: int = 0,
    flight_state: int = 0,
    error_code: int = 0,
    water_detected: int = 0,
) -> bytes:
    frame = bytearray(PACKET_LEN)
    frame[0] = FRAME_HEADER_0
    frame[1] = FRAME_HEADER_1
    _pack_into("<H", frame, 2, time_ds & 0xFFFF)
    _pack_into("<i", frame, 4, int(lat_raw))
    _pack_into("<i", frame, 8, int(lon_raw))
    _pack_into("<h", frame, 12, int(gps_alt_dm))
    _pack_into("<h", frame, 14, int(gps_speed_dms))
    frame[16] = sat_count & 0xFF
    _pack_into("<h", frame, 17, int(roll_cdeg))
    _pack_into("<h", frame, 19, int(pitch_cdeg))
    _pack_into("<H", frame, 21, int(yaw_ddeg) & 0xFFFF)
    _pack_into("<h", frame, 23, int(gyro_x_ddeg_s))
    _pack_into("<h", frame, 25, int(gyro_y_ddeg_s))
    _pack_into("<h", frame, 27, int(gyro_z_ddeg_s))
    _pack_into("<h", frame, 29, int(accx_cg))
    _pack_into("<h", frame, 31, int(accy_cg))
    _pack_into("<h", frame, 33, int(accz_cg))
    _pack_into("<h", frame, 35, int(baro_alt_dm))
    _pack_into("<H", frame, 37, int(battery_mv) & 0xFFFF)
    _pack_into("<H", frame, 39, int(servo_power_mv) & 0xFFFF)
    _pack_into("<h", frame, 41, int(servo_angle_ddeg))
    frame[43] = flight_state & 0xFF
    frame[44] = error_code & 0xFF
    frame[45] = water_detected & 0xFF
    frame[46] = _xor_crc(frame[:-1])
    return bytes(frame)


def is_valid_frame(frame: bytes) -> bool:
    if len(frame) != PACKET_LEN:
        return False
    if frame[:2] != FRAME_SYNC:
        return False
    return _xor_crc(frame[:-1]) == frame[-1]


def decode_frame(frame: bytes) -> TelemetryFrame:
    if not is_valid_frame(frame):
        raise FrameDecodeError("invalid telemetry frame")

    return TelemetryFrame(
        time_ds=struct.unpack_from("<H", frame, 2)[0],
        lat_raw=struct.unpack_from("<i", frame, 4)[0],
        lon_raw=struct.unpack_from("<i", frame, 8)[0],
        gps_alt_dm=struct.unpack_from("<h", frame, 12)[0],
        gps_speed_dms=struct.unpack_from("<h", frame, 14)[0],
        sat_count=frame[16],
        roll_cdeg=struct.unpack_from("<h", frame, 17)[0],
        pitch_cdeg=struct.unpack_from("<h", frame, 19)[0],
        yaw_ddeg=struct.unpack_from("<H", frame, 21)[0],
        gyro_x_ddeg_s=struct.unpack_from("<h", frame, 23)[0],
        gyro_y_ddeg_s=struct.unpack_from("<h", frame, 25)[0],
        gyro_z_ddeg_s=struct.unpack_from("<h", frame, 27)[0],
        accx_cg=struct.unpack_from("<h", frame, 29)[0],
        accy_cg=struct.unpack_from("<h", frame, 31)[0],
        accz_cg=struct.unpack_from("<h", frame, 33)[0],
        baro_alt_dm=struct.unpack_from("<h", frame, 35)[0],
        battery_mv=struct.unpack_from("<H", frame, 37)[0],
        servo_power_mv=struct.unpack_from("<H", frame, 39)[0],
        servo_angle_ddeg=struct.unpack_from("<h", frame, 41)[0],
        flight_state=frame[43],
        error_code=frame[44],
        water_detected=frame[45],
    )


class TelemetryStreamParser:
    def __init__(self):
        self._buffer = bytearray()
        self.total_crc_fail = 0
        self.total_resync_count = 0

    def reset(self) -> None:
        self._buffer.clear()
        self.total_crc_fail = 0
        self.total_resync_count = 0

    def feed(self, data: bytes) -> list[bytes]:
        frames = []
        if not data:
            return frames

        self._buffer.extend(data)

        while True:
            if len(self._buffer) < len(FRAME_SYNC):
                return frames

            start_idx = self._buffer.find(FRAME_SYNC)
            if start_idx == -1:
                if self._buffer[-1] == FRAME_HEADER_0:
                    if len(self._buffer) > 1:
                        self.total_resync_count += 1
                    self._buffer[:] = self._buffer[-1:]
                else:
                    if self._buffer:
                        self.total_resync_count += 1
                    self._buffer.clear()
                return frames

            if start_idx > 0:
                self.total_resync_count += 1
                del self._buffer[:start_idx]

            if len(self._buffer) < PACKET_LEN:
                return frames

            frame = bytes(self._buffer[:PACKET_LEN])
            if is_valid_frame(frame):
                frames.append(frame)
                del self._buffer[:PACKET_LEN]
            else:
                self.total_crc_fail += 1
                self.total_resync_count += 1
                del self._buffer[0]
