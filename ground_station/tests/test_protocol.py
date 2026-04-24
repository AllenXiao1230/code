import unittest

from protocol import (
    FRAME_HEADER_0,
    FRAME_HEADER_1,
    INVALID_BARO_ALT_DM,
    TelemetryStreamParser,
    build_frame,
    decode_frame,
    is_valid_frame,
)


class ProtocolTests(unittest.TestCase):
    def test_decode_frame_scaling(self):
        frame = build_frame(
            time_ds=123,
            lat_raw=250123456,
            lon_raw=1219876543,
            gps_alt_dm=1234,
            gps_speed_dms=87,
            sat_count=9,
            roll_cdeg=-1234,
            pitch_cdeg=567,
            yaw_ddeg=1234,
            gyro_x_ddeg_s=-12,
            gyro_y_ddeg_s=34,
            gyro_z_ddeg_s=-56,
            accx_cg=321,
            accy_cg=-654,
            accz_cg=987,
            baro_alt_dm=4321,
            battery_mv=11850,
            servo_power_mv=5100,
            servo_angle_ddeg=-120,
            flight_state=5,
            error_code=0x12,
            water_detected=1,
        )

        packet = decode_frame(frame)

        self.assertTrue(is_valid_frame(frame))
        self.assertAlmostEqual(packet.time_s, 12.3)
        self.assertAlmostEqual(packet.lat_deg, 25.0123456)
        self.assertAlmostEqual(packet.lon_deg, 121.9876543)
        self.assertAlmostEqual(packet.gps_alt_m, 123.4)
        self.assertAlmostEqual(packet.gps_speed_ms, 8.7)
        self.assertAlmostEqual(packet.roll_deg, -12.34)
        self.assertAlmostEqual(packet.pitch_deg, 5.67)
        self.assertAlmostEqual(packet.heading_deg, 123.4)
        self.assertAlmostEqual(packet.gyro_x_dps, -1.2)
        self.assertAlmostEqual(packet.accy_g, -6.54)
        self.assertAlmostEqual(packet.baro_alt_m, 432.1)
        self.assertAlmostEqual(packet.battery_v, 11.85)
        self.assertEqual(packet.flight_state, 5)
        self.assertEqual(packet.error_code, 0x12)
        self.assertEqual(packet.water_detected, 1)

    def test_decode_frame_supports_empty_baro_altitude(self):
        frame = build_frame(baro_alt_dm=INVALID_BARO_ALT_DM)

        packet = decode_frame(frame)

        self.assertFalse(packet.has_baro_alt)
        self.assertIsNone(packet.baro_alt_m)

    def test_stream_parser_keeps_split_sync_byte(self):
        frame = build_frame(time_ds=10, battery_mv=12000)
        parser = TelemetryStreamParser()

        part1 = bytes([0x00, 0x13, FRAME_HEADER_0])
        part2 = bytes([FRAME_HEADER_1]) + frame[2:]

        self.assertEqual(parser.feed(part1), [])
        frames = parser.feed(part2)

        self.assertEqual(frames, [frame])

    def test_stream_parser_skips_bad_crc_and_recovers(self):
        good = build_frame(time_ds=25, baro_alt_dm=200)
        bad = bytearray(build_frame(time_ds=24, baro_alt_dm=100))
        bad[-1] ^= 0xFF

        parser = TelemetryStreamParser()
        stream = bytes(bad) + good
        frames = parser.feed(stream)

        self.assertEqual(frames, [good])


if __name__ == "__main__":
    unittest.main()
