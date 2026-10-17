"""Hardware-independent checks for mode control, velocity blocking and cloud decoding."""
import argparse
import ast
from http.server import ThreadingHTTPServer
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import server


class PanelTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.panel = server.Panel(argparse.Namespace(
            maps_dir=Path(self.tmp.name), demo=True, max_speed=.3, max_yaw_rate=.5))
        self.unlock_yaw()

    def unlock_yaw(self):
        self.panel.yaw_guard.update(0., 0.)
        self.panel.yaw_guard.stable_since = time.monotonic()-1.

    def tearDown(self):
        self.panel.close()
        self.tmp.cleanup()

    def command(self, mode='navigation'):
        p = self.panel
        self.unlock_yaw()
        p.mode = mode
        p.cloud_time = p.odom_time = time.monotonic()
        p.commands['auto' if mode == 'navigation' else 'manual'] = (
            time.monotonic(), (1., 1., 2.))

    def test_gate_limits_diagonal_speed_and_yaw(self):
        self.command()
        vx, vy, wz = self.panel.tick()
        self.assertAlmostEqual(math.hypot(vx, vy), .3)
        self.assertEqual(wz, .5)

    def test_gate_blocks_idle_transition_stale_and_wrong_source(self):
        p = self.panel
        for condition in ('idle', 'transition', 'cloud', 'odom', 'command', 'source', 'nan'):
            with self.subTest(condition=condition):
                self.command()
                p.switching = False
                if condition == 'idle': p.mode = 'idle'
                if condition == 'transition': p.switching = True
                if condition == 'cloud': p.cloud_time -= 2
                if condition == 'odom': p.odom_time -= 2
                if condition == 'command': p.commands['auto'] = (0, (1., 0., 0.))
                if condition == 'source': p.mode = 'mapping'
                if condition == 'nan': p.commands['auto'] = (time.monotonic(), (float('nan'), 0., 0.))
                self.assertEqual(p.tick(), (0., 0., 0.))

    def test_mapping_uses_only_manual_commands(self):
        self.command('mapping')
        self.assertGreater(self.panel.tick()[0], 0)

    def test_missing_maps_do_not_launch_navigation(self):
        self.panel.args.demo = False
        with patch.object(server.subprocess, 'Popen') as popen:
            self.panel.request_mode('navigation')
            self.panel.worker.join(2)
            popen.assert_not_called()
        self.assertEqual(self.panel.mode, 'idle')
        self.assertIn('Missing map', self.panel.error)

    def test_save_failure_preserves_mapping_for_navigation(self):
        p = self.panel
        p.args.demo = False
        p.mode = 'mapping'
        p.bridge = SimpleNamespace(save_maps=lambda: (_ for _ in ()).throw(RuntimeError('disk full')))
        p.request_mode('navigation')
        p.worker.join(2)
        self.assertEqual(p.mode, 'mapping')
        self.assertIn('disk full', p.error)
        p.request_mode('idle')
        p.worker.join(2)
        self.assertEqual(p.mode, 'idle')
        self.assertIn('disk full', p.error)

    def test_stop_failure_keeps_velocity_blocked(self):
        self.command('mapping')
        with patch.object(self.panel, 'stop_all', side_effect=RuntimeError('stop failed')):
            self.panel.request_mode('navigation')
            self.panel.worker.join(2)
        self.assertEqual(self.panel.mode, 'idle')
        self.assertEqual(self.panel.tick(), (0., 0., 0.))

    def test_launch_exit_blocks_velocity(self):
        self.command()
        with patch.object(self.panel, 'stop_all'):
            self.panel.processes = [SimpleNamespace(poll=lambda: 1)]
            self.assertEqual(self.panel.tick(), (0., 0., 0.))
            self.panel.worker.join(2)
            self.panel.processes.clear()
        self.assertEqual(self.panel.mode, 'idle')

    def test_mapping_save_navigation_idle_workflow(self):
        p = self.panel
        p.args.demo = False
        saved = threading.Event()
        release_save = threading.Event()
        def save_maps():
            saved.set()
            if not release_save.wait(2):
                raise RuntimeError('test save did not resume')
            (p.args.maps_dir/'site.pcd').write_bytes(b'test map')
            (p.args.maps_dir/'site.yaml').write_text('image: site.png\n')
        p.bridge = SimpleNamespace(save_maps=save_maps)
        process = SimpleNamespace(poll=lambda: None)
        with patch.object(server.subprocess, 'Popen', return_value=process) as launch, \
             patch.object(server, 'stop_process') as stop:
            p.request_mode('mapping')
            p.worker.join(2)
            self.assertEqual(p.mode, 'mapping')
            self.assertIn('use_rviz:=false', launch.call_args.args[0])
            self.command('mapping')
            self.assertGreater(p.tick()[0], 0.)
            p.request_mode('navigation')
            self.assertTrue(saved.wait(1))
            self.assertEqual(p.tick(), (0., 0., 0.))
            release_save.set()
            p.worker.join(2)
            self.assertEqual(p.mode, 'navigation')
            self.assertTrue(launch.call_args.args[0][-1].endswith('navigation_web.launch.py'))
            self.assertEqual(launch.call_count, 2)
            self.command()
            self.assertGreater(p.tick()[0], 0.)
            p.request_mode('idle')
            p.worker.join(2)
            self.assertEqual(p.tick(), (0., 0., 0.))
            self.assertEqual(stop.call_count, 2)

    def test_http_auth_invalid_modes_and_valid_transition(self):
        http = ThreadingHTTPServer(('127.0.0.1', 0), server.handler_class(self.panel, 'test-token'))
        thread = threading.Thread(target=http.serve_forever, daemon=True)
        thread.start()
        base = f'http://127.0.0.1:{http.server_port}'
        try:
            with self.assertRaises(HTTPError) as error:
                urlopen(base + '/api/state', timeout=2)
            self.assertEqual(error.exception.code, 401)
            error.exception.close()
            def request(mode):
                return Request(base+'/api/mode', json.dumps({'mode': mode}).encode(),
                    {'Authorization': 'Bearer test-token', 'Content-Type': 'application/json'})
            with self.assertRaises(HTTPError) as error:
                urlopen(request('invalid'), timeout=2)
            self.assertEqual(error.exception.code, 400)
            error.exception.close()
            with urlopen(request('mapping'), timeout=2) as response:
                self.assertEqual(response.status, 202)
            self.panel.worker.join(2)
            with urlopen(Request(base+'/api/state', headers={
                    'Authorization': 'Bearer test-token'}), timeout=2) as response:
                self.assertEqual(json.load(response)['mode'], 'mapping')
        finally:
            http.shutdown()
            http.server_close()
            thread.join(2)


class CloudTests(unittest.TestCase):
    def test_big_endian_organized_cloud_with_row_padding(self):
        data = b''.join(struct.pack('>fff', *p) for p in [(1, 2, 3), (4, 5, 6)]) + b'PAD!'
        data += b''.join(struct.pack('>fff', *p) for p in [(7, 8, 9), (float('nan'), 0, 0)]) + b'PAD!'
        msg = SimpleNamespace(height=2, width=2, point_step=12, row_step=28,
            is_bigendian=True, data=data, fields=[SimpleNamespace(name=n, offset=i*4, datatype=7)
                                                 for i, n in enumerate(('x', 'y', 'z'))])
        self.assertEqual(server.decode_cloud(msg, 10), [[1, 2, 3], [4, 5, 6], [7, 8, 9]])
        self.assertEqual(server.decode_cloud(msg, 2), [[1, 2, 3], [7, 8, 9]])


class ConfigurationTests(unittest.TestCase):
    def test_configuration_paths_geometry_and_backups(self):
        import yaml
        root = Path(__file__).resolve().parent.parent
        files = ['web_panel/configure.py', 'web_panel/robot_io.py', 'sentry_bringup/launch/relocalization.launch.py',
                 'sentry_bringup/params/fast_lio_mapping_param.yaml',
                 'sentry_bringup/params/fast_lio_relocalization_param.yaml',
                 'sentry_bringup/params/nav2_params.yaml', 'livox_ros_driver2/config/MID360_config.json',
                 'autonomous_exploration_development_environment/src/terrain_analysis/launch/terrain_analysis.launch',
                 'autonomous_exploration_development_environment/src/terrain_analysis_ext/launch/terrain_analysis_ext.launch']
        with tempfile.TemporaryDirectory(prefix="scurm robot's ") as temp:
            target = Path(temp)
            for name in files:
                dest = target / name
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(root / name, dest)
            command = [sys.executable, str(target/'web_panel/configure.py'), '--length', '.6',
                       '--width', '.4', '--lidar-height', '.7', '--imu-xyz', '.1', '0', '.65',
                       '--yaw-axis-xyz', '0', '0', '.2', '--yaw-target', '0', '--yaw-reference', 'base_link',
                       '--host-ip', '192.168.1.50', '--lidar-ip', '192.168.1.196',
                       '--maps-dir', str(target/'maps')]
            subprocess.run(command, check=True, capture_output=True)
            backup = target/'web_panel/original_configs/sentry_bringup/params/nav2_params.yaml'
            before = backup.read_bytes()
            subprocess.run(command, check=True, capture_output=True)
            self.assertEqual(backup.read_bytes(), before)
            cfg = yaml.safe_load((target/'sentry_bringup/params/nav2_params.yaml').read_text(encoding='utf-8'))
            costmap = cfg['local_costmap']['local_costmap']['ros__parameters']
            self.assertEqual(ast.literal_eval(costmap['footprint'])[0], [.3, .2])
            self.assertNotIn('robot_radius', costmap)
            self.assertFalse(costmap['use_sim_time'])
            self.assertEqual(cfg['velocity_smoother']['ros__parameters']['feedback'], 'OPEN_LOOP')
            import xml.etree.ElementTree as ET
            terrain = ET.parse(target/'autonomous_exploration_development_environment/src/terrain_analysis_ext/launch/terrain_analysis_ext.launch')
            terrain_params = {item.attrib['name']: float(item.attrib['value']) for item in terrain.findall('.//param')
                              if item.attrib['name'] in ('sensorOffsetX', 'sensorOffsetY', 'lowerBoundZ', 'terrainUnderVehicle')}
            self.assertEqual(terrain_params['sensorOffsetX'], 0.)
            self.assertEqual(terrain_params['sensorOffsetY'], 0.)
            self.assertLess(terrain_params['lowerBoundZ'], terrain_params['terrainUnderVehicle'])
            source = (target/'sentry_bringup/launch/relocalization.launch.py').read_text(encoding='utf-8')
            ast.parse(source)
            self.assertIn(repr(str(target/'maps/site.pcd')), source)
            ast.parse((target/'web_panel/robot_tf.launch.py').read_text(encoding='utf-8'))


if __name__ == '__main__':
    unittest.main()
