import struct
import unittest

from ddt_bridge import TYPE_FEEDBACK, TYPE_STATUS, crc16, decode_frames, encode_frame, parse_feedback


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
