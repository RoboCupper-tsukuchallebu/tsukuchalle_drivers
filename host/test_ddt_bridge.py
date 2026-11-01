import os
import struct
import unittest

from ddt_bridge import (
    Bridge,
    TYPE_FEEDBACK,
    TYPE_STATUS,
    crc16,
    decode_frames,
    encode_frame,
    parse_feedback,
    sweep_rpm,
)


class DisconnectTest(unittest.TestCase):
    def test_poll_raises_when_the_port_reports_end_of_file(self):
        read_end, write_end = os.pipe()
        self.addCleanup(os.close, read_end)
        os.close(write_end)
        bridge = Bridge.__new__(Bridge)
        bridge._file_descriptor = read_end
        bridge._buffer = bytearray()
        with self.assertRaises(OSError):
            bridge.poll(0.0)


class SweepTest(unittest.TestCase):
    def test_ramps_up_and_back_down_within_limits(self):
        self.assertEqual(sweep_rpm(0.0, 1, 30, 6.0), 1)
        self.assertEqual(sweep_rpm(6.0, 1, 30, 6.0), 30)
        self.assertEqual(sweep_rpm(12.0, 1, 30, 6.0), 1)
        self.assertEqual(sweep_rpm(3.0, 1, 30, 6.0), sweep_rpm(9.0, 1, 30, 6.0))
        for step in range(0, 1300):
            self.assertTrue(1 <= sweep_rpm(step / 100, 1, 30, 6.0) <= 30)


class ProtocolTest(unittest.TestCase):
    def test_crc_matches_known_vector_shared_with_firmware(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)

    def test_round_trip_survives_split_delivery(self):
        frame = encode_frame(TYPE_STATUS, b"\x02\x00")
        buffer = bytearray(frame[:3])
        self.assertEqual(decode_frames(buffer), [])
        buffer += frame[3:]
        self.assertEqual(decode_frames(buffer), [(TYPE_STATUS, b"\x02\x00")])
        self.assertEqual(buffer, b"")

    def test_keeps_header_byte_split_across_reads(self):
        frame = encode_frame(TYPE_STATUS, b"\x01\x00")
        buffer = bytearray(b"\x00" + frame[:1])
        self.assertEqual(decode_frames(buffer), [])
        buffer += frame[1:]
        self.assertEqual(decode_frames(buffer), [(TYPE_STATUS, b"\x01\x00")])

    def test_skips_garbage_and_corrupted_frame(self):
        corrupted = bytearray(encode_frame(TYPE_STATUS, b"\x02\x00"))
        corrupted[4] ^= 0xFF
        valid = encode_frame(TYPE_STATUS, b"\x01\x00")
        buffer = bytearray(b"\x01\x02" + corrupted + valid)
        self.assertEqual(decode_frames(buffer), [(TYPE_STATUS, b"\x01\x00")])

    def test_rejects_oversized_payload(self):
        with self.assertRaises(ValueError):
            encode_frame(TYPE_STATUS, bytes(17))

    def test_feedback_scaling(self):
        payload = struct.pack("<BhhHBB", 2, -100, 32767, 32767, 0x0A, 2)
        feedback = parse_feedback(payload)
        self.assertEqual(feedback.motor_id, 2)
        self.assertEqual(feedback.velocity_rpm, -100)
        self.assertAlmostEqual(feedback.current_amps, 33.0)
        self.assertAlmostEqual(feedback.angle_degrees, 360.0)
        self.assertEqual(feedback.fault, 0x0A)
        self.assertEqual(TYPE_FEEDBACK, 0x81)


if __name__ == "__main__":
    unittest.main()
