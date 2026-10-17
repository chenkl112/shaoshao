"""Yaw dwell/timeout and reference serial round-trip; no ROS or motors required."""
import argparse
import json
import math
import subprocess
import sys
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robot_io import (YawGuard, decode_feedback, encode_command, validate_config, wrap_angle,
                      optical_center_in_base, sample_monotonic_time, load_io_config, validate_io_config)
from types import SimpleNamespace
from yaw_tf import quaternion
from server import Panel


class YawTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.guard = YawGuard({'target_rad': .2, 'settle_time_s': .5,
            'feedback_timeout_s': .3}, clock=lambda: self.now)

    def update(self, angle=.2, rate=0., elapsed=.1):
        self.now += elapsed
        self.guard.update(angle, rate)

    def test_continuous_feedback_is_required_for_dwell(self):
        self.guard.update(.2, 0.)
        for _ in range(6): self.update()
        self.assertTrue(self.guard.status()['ready'])
        self.now += .31
        self.assertFalse(self.guard.status()['ready'])
        self.guard.update(.2, 0.)
        self.assertFalse(self.guard.status()['ready'])

    def test_feedback_gap_resets_dwell_without_gate_tick(self):
        self.guard.update(.2, 0.)
        self.update(elapsed=1.)
        self.assertFalse(self.guard.status()['ready'])

    def test_delayed_feedback_expires_from_sample_time(self):
        self.guard.update(.2, 0.)
        for _ in range(6):
            self.update()
        self.assertTrue(self.guard.status()['ready'])
        self.guard.update(.2, 0., sampled_at=self.now-.28)
        self.now += .03
        self.assertFalse(self.guard.status()['ready'])

    def test_ros_stamp_accounts_for_transport_delay_and_rejects_invalid_age(self):
        def sample(sec, nanosec=0):
            return sample_monotonic_time(SimpleNamespace(sec=sec, nanosec=nanosec),
                                         10_000_000_000, 100., 1.)
        self.assertAlmostEqual(sample(9, 200_000_000), 99.2)
        self.assertEqual(sample(10, 50_000_000), 100.)
        self.assertIsNone(sample(8))
        self.assertIsNone(sample(0))
        self.assertIsNone(sample(10, 200_000_000))
        self.assertIsNone(sample(9, 1_000_000_000))

    def test_motion_wrong_angle_and_invalid_values_block(self):
        for angle, rate in [(.4, 0), (.2, .1), (float('nan'), 0), (.2, float('inf'))]:
            self.guard.update(angle, rate)
            self.assertFalse(self.guard.status()['ready'])

    def test_wrapped_angle_and_quaternion(self):
        self.guard.update(.2+2*math.pi, 0.)
        for _ in range(6): self.update(.2+2*math.pi)
        self.assertTrue(self.guard.status()['ready'])
        q = quaternion(0, 0, math.pi/2)
        self.assertAlmostEqual(q[2], math.sqrt(.5))
        self.assertAlmostEqual(sum(v*v for v in q), 1.)

    def test_optical_center_composes_pivot_mount_and_actual_hold_angle(self):
        yaw = {'axis_xyz_m': [1., 2., .3], 'imu_xyz_m': [.211, .02329, .35588],
               'imu_rpy_rad': [0., 0., 0.], 'target_rad': math.pi/2}
        x, y, z = optical_center_in_base(yaw)
        self.assertAlmostEqual(x, 1.)
        self.assertAlmostEqual(y, 2.2)
        self.assertAlmostEqual(z, .7)


class ProtocolTests(unittest.TestCase):
    def test_command_encoder_calibration_and_feedback(self):
        hardware = {'yaw_encoder_sign': -1, 'yaw_encoder_zero_rad': 1.}
        raw = encode_command(7, (.1, -.2, .3), .2, hardware)
        self.assertTrue(raw.endswith(b'\n'))
        command = json.loads(raw)
        self.assertAlmostEqual(command['yaw_target_encoder_rad'], .8)
        packet = decode_feedback(json.dumps({'type':'state', 'seq':8,
            'yaw_encoder_rad': .8, 'yaw_rate_rad_s': 0.,
            'odom': dict(x=0., y=0., yaw=0., vx=.1, vy=-.2, wz=.3)}).encode())
        self.assertAlmostEqual(-1*(packet['yaw_encoder_rad']-1.), .2)
        integer_state = decode_feedback(b'{"type":"state","seq":0,"yaw_encoder_rad":0,"yaw_rate_rad_s":0}')
        self.assertIsInstance(integer_state['yaw_encoder_rad'], float)

    def test_nonfinite_or_missing_feedback_is_rejected(self):
        for line in [b'{}', b'{"type":"state","yaw_encoder_rad":NaN,"yaw_rate_rad_s":0}',
                     b'{"type":"state","yaw_encoder_rad":0}', b'x'*4097]:
            with self.assertRaises((ValueError, json.JSONDecodeError)):
                decode_feedback(line)

    def test_placeholder_configuration_cannot_be_used_as_real_geometry(self):
        import yaml
        config = yaml.safe_load(Path(__file__).with_name('robot.example.yaml').read_text(encoding='utf-8'))
        missing = validate_config(config)
        self.assertIn('yaw.target_rad', missing)
        self.assertIn('yaw.axis_xyz_m', missing)
        self.assertIn('hardware.serial_port', missing)
        self.assertIn('hardware.baud', missing)

    def test_serial_interface_can_be_checked_without_navigation_geometry(self):
        config = {'version':1, 'yaw':{'target_rad':0}, 'hardware':{
            'transport':'serial_jsonl_v1', 'serial_port':'/dev/serial/by-id/example',
            'baud':460800, 'yaw_encoder_sign':-1, 'yaw_encoder_zero_rad':.7}}
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'serial.json'
            path.write_text(json.dumps(config))
            loaded = load_io_config(path)
            self.assertEqual(loaded['yaw']['target_rad'], 0.)
            self.assertTrue(validate_config(loaded))
            checked = subprocess.run([sys.executable, str(Path(__file__).with_name('chassis_bridge.py')),
                '--config', str(path), '--check-config'], capture_output=True, timeout=5)
            self.assertEqual(checked.returncode, 0, checked.stderr)
        config['hardware']['command_timeout_s'] = float('nan')
        self.assertIn('hardware.command_timeout_s', validate_io_config(config))

    def test_version_and_sequence_reject_incompatible_frames(self):
        valid = {'type':'state', 'version':1, 'seq':0, 'yaw_encoder_rad':0, 'yaw_rate_rad_s':0}
        self.assertEqual(decode_feedback(json.dumps(valid))['seq'], 0)
        for version in (2, True, '1'):
            with self.subTest(version=version), self.assertRaises(ValueError):
                decode_feedback(json.dumps(dict(valid, version=version)))
        for seq in (-1, True, 1.5):
            with self.subTest(seq=seq), self.assertRaises(ValueError):
                encode_command(seq, (0., 0., 0.), 0., {})

    def test_yaw_drift_blocks_velocity_even_with_fresh_lidar_and_odom(self):
        with tempfile.TemporaryDirectory() as temp:
            panel = Panel(argparse.Namespace(maps_dir=Path(temp), demo=True,
                                             max_speed=.3, max_yaw_rate=.5))
            fake_now = 10.
            panel.yaw_guard = YawGuard({'target_rad':0., 'settle_time_s':.5}, clock=lambda: fake_now)
            with patch('server.time.monotonic', side_effect=lambda: fake_now):
                panel.mode = 'navigation'
                panel.yaw_guard.update(0., 0.)
                for _ in range(6):
                    fake_now += .1
                    panel.yaw_guard.update(0., 0.)
                panel.cloud_time = panel.odom_time = fake_now
                panel.commands['auto'] = (fake_now, (.2, 0., 0.))
                self.assertEqual(panel.tick(), (.2, 0., 0.))
                # A single actual encoder packet reports unexpected turret movement.
                feedback = decode_feedback(b'{"type":"state","seq":1,"yaw_encoder_rad":0.1,"yaw_rate_rad_s":0.1}')
                panel.yaw_guard.update(feedback['yaw_encoder_rad'], feedback['yaw_rate_rad_s'])
                self.assertEqual(panel.tick(), (0., 0., 0.))
                panel.args.demo = False
                with self.assertRaisesRegex(RuntimeError, 'yaw'):
                    panel.request_mode('mapping')
            panel.close()


if __name__ == '__main__':
    unittest.main()
