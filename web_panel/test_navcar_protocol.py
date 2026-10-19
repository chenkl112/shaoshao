import math
import struct
import unittest
from pathlib import Path
import tempfile
import subprocess
import sys

from navcar_protocol import ACK, STATE, Parser, Session, control, crc16_modbus, frame, stop_confirmed
from navcar_bridge import targets, load_config


def status(seq=65535, values=(0., 0., 0., 0.)):
    return frame(STATE, seq, struct.pack('<4f', *values))


class NavcarTests(unittest.TestCase):
    def test_existing_bridge_dispatch_and_missing_calibration_block(self):
        import yaml
        config = yaml.safe_load(Path(__file__).with_name('serial.navcar.example.yaml').read_text(encoding='utf-8'))
        config['serial_port'] = '/dev/serial/by-id/never-open-this-test'
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'serial.yaml';path.write_text(yaml.safe_dump(config))
            command = [sys.executable, str(Path(__file__).with_name('chassis_bridge.py')),
                       '--config', str(path), '--check-config']
            result = subprocess.run(command, capture_output=True, text=True, timeout=3)
            self.assertEqual(result.returncode, 0, result.stderr)
            config['forward_units_per_mps'] = None
            path.write_text(yaml.safe_dump(config))
            result = subprocess.run(command+['--enable-control'], capture_output=True, text=True, timeout=3)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('forward_units_per_mps', result.stderr)

    def test_official_crc_and_all_wire_vectors(self):
        self.assertEqual(crc16_modbus(b'123456789'), 0x4b37)
        vectors = [
            (0, (1, 0, 0, 0), 'A55A0101000010000000803F0000000000000000000000004B6C', 'A55A0180000001000045C0', 'A55A0181000010000000803F000000000000000000000000D4A4'),
            (1, (0, 0, 3, 0), 'A55A01010100100000000000000000000000404000000000AD0D', 'A55A018001000100007800', 'A55A0181010010000000000000000000000040400000000032C5'),
            (2, (0, 0, 0, 0), 'A55A01010200100000000000000000000000000000000000E631', 'A55A018002000100003C00', None)]
        for seq, values, command_hex, ack_hex, state_hex in vectors:
            self.assertEqual(control(seq, values), bytes.fromhex(command_hex))
            self.assertEqual(frame(ACK, seq, b'\0'), bytes.fromhex(ack_hex))
            if state_hex:self.assertEqual(status(seq, values), bytes.fromhex(state_hex))

    def test_split_concatenated_frames_and_noise(self):
        parser = Parser();raw = status(9) + frame(ACK, 9, b'\0')
        self.assertEqual(parser.feed(b'boot log\n\xa5', 0), [])
        self.assertEqual(parser.feed(raw[:8], .01), [])
        packets = parser.feed(raw[8:], .02)
        self.assertEqual([(p.kind, p.sequence) for p in packets], [(STATE, 9), (ACK, 9)])
        parser = Parser();packets = []
        for i, byte in enumerate(raw):packets += parser.feed(bytes([byte]), i*.001)
        self.assertEqual(len(packets), 2)

    def test_corrupt_crc_bad_length_timeout_version_and_nonfinite(self):
        bad = bytearray(status());bad[-1] ^= 1
        parser = Parser()
        self.assertEqual(len(parser.feed(bytes(bad)+b'\xa5\x5a\x01\x81\x00\x00\xff\xff'+status(3), 0)), 1)
        parser = Parser();parser.feed(status()[:8], 0)
        self.assertEqual(parser.feed(status()[8:], .101), [])
        self.assertEqual(len(parser.feed(status(2), .11)), 1)
        self.assertEqual(Parser().feed(status(0, (math.nan,0,0,0)), 0), [])
        wrong = bytearray(status());wrong[2] = 2;wrong[-2:] = struct.pack('<H', crc16_modbus(wrong[2:-2]))
        self.assertEqual(Parser().feed(wrong, 0), [])

    def test_values_are_finite_float32_and_sequence_is_uint16(self):
        for seq in (-1, 65536, True):
            with self.assertRaises(ValueError):control(seq, (0,0,0,0))
        for value in (math.nan, math.inf, 1e100):
            with self.assertRaises(ValueError):control(0, (value,0,0,0))

    def test_single_flight_exact_retry_ack_match_wrap_and_latest_target(self):
        s = Session();self.assertIsNone(s.poll((1,0,0,0), 0))
        s.receive(status(65534), 0)
        original = s.poll((1,0,0,0), 0)
        self.assertEqual(original, control(65535, (1,0,0,0)))
        s.receive(frame(ACK, 4, b'\0'), .01)
        self.assertIsNone(s.poll((2,0,0,0), .02))
        self.assertEqual(s.poll((2,0,0,0), .051), original)
        s.receive(frame(ACK, 65535, b'\1'), .06)
        self.assertEqual(s.poll((2,0,0,0), .061), control(0, (2,0,0,0)))

    def test_retries_are_bounded_and_nack_faults_do_not_resume_from_state(self):
        s = Session();s.receive(status(), 0);original = s.poll((1,0,0,0), 0)
        for now in (.051,.102,.153):self.assertEqual(s.poll((9,0,0,0), now), original)
        self.assertIsNone(s.poll((9,0,0,0), .204));self.assertIn('3 retries', s.error)
        s.receive(status(42), .21);self.assertIsNone(s.poll((9,0,0,0), .22))
        for code in range(2,7):
            s = Session();s.receive(status(50), 0);s.poll((1,0,0,0), 0)
            s.receive(frame(ACK, 51, bytes([code])), .01)
            self.assertIn('ACK='+str(code), s.error)
            s.receive(status(51), .02);self.assertIsNone(s.poll((1,0,0,0), .03))

    def test_repeated_status_sequence_is_valid_but_stale_status_stops(self):
        s = Session();s.receive(status(7), 0);s.receive(status(7), .1)
        self.assertEqual(s.state_time, .1)
        self.assertIsNone(s.poll((1,0,0,0), .401));self.assertIn('status timeout', s.error)

    def test_stopping_requires_new_ack_zero_and_zero_target_echo(self):
        s = Session();s.receive(status(), 0);s.poll((0,0,0,27), .01)
        self.assertFalse(stop_confirmed(s, .01))
        s.receive(frame(ACK, 0, b'\0'), .02)
        self.assertFalse(stop_confirmed(s, .01))
        s.receive(status(0, (0,0,0,27)), .03)
        self.assertTrue(stop_confirmed(s, .01))
        s.receive(status(1, (0,0,0,27)), .04)
        self.assertFalse(stop_confirmed(s, .01))

    def test_mapping_order_signs_deg_normalization_and_limits(self):
        cfg = dict(forward_units_per_mps=10, left_units_per_mps=20,
                   rot_units_per_radps=3, yaw_target_deg=450,
                   max_translation_units=20, max_rot_units=5)
        self.assertEqual(targets((.1,.2,.3), cfg), (1,-4,-.8999999999999999,90))
        self.assertEqual(targets((0,0,0), cfg), (0,0,0,90))
        with self.assertRaises(ValueError):targets((100,0,0), cfg)


if __name__ == '__main__':unittest.main()
