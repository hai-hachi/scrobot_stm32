import unittest

from scrobot_protocol.protocol import (
    FrameParser, encode_frame, crc16_ccitt_false,
    TYPE_INFO_REQUEST,
)


class ProtocolTests(unittest.TestCase):
    def test_crc_known_ccitt_false(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_round_trip(self):
        raw = encode_frame(TYPE_INFO_REQUEST, 0x1234, b"")
        parser = FrameParser()
        frames = parser.feed(raw)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].msg_type, TYPE_INFO_REQUEST)
        self.assertEqual(frames[0].seq, 0x1234)
        self.assertEqual(frames[0].payload, b"")

    def test_resync_after_noise(self):
        raw = b"noise" + encode_frame(TYPE_INFO_REQUEST, 7, b"")
        frames = FrameParser().feed(raw)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].seq, 7)


if __name__ == "__main__":
    unittest.main()
