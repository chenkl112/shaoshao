"""Hardware-independent checks for mode control, velocity blocking and cloud decoding."""
import argparse
import base64
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

    def write_saved_map(self):
        # Valid one-pixel PNG: exercise the actual authenticated image response.
        image = base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+a7x8AAAAASUVORK5CYII=')
        root = self.panel.args.maps_dir
        (root/'site.png').write_bytes(image)
        (root/'site.yaml').write_text('image: site.png\nresolution: 0.05\norigin: [1, -2, 0]\n')
        return image

    def test_saved_map_absent_valid_and_updated(self):
        info, path = self.panel.saved_map()
        self.assertFalse(info['available'])
        self.assertIsNone(path)
        self.write_saved_map()
        info, path = self.panel.saved_map()
        self.assertTrue(info['available'])
        self.assertEqual(info['resolution'], .05)
        self.assertEqual(info['origin'], [1, -2, 0])
        self.assertEqual(path, self.panel.args.maps_dir/'site.png')
        with path.open('ab') as out:
            out.write(b'changed')
        self.assertNotEqual(info['revision'], self.panel.saved_map()[0]['revision'])

    def test_saved_map_rejects_escape_and_invalid_metadata(self):
        self.write_saved_map()
        root = self.panel.args.maps_dir
        cases = [
            'image: ../outside.png\nresolution: 0.05\norigin: [0,0,0]',
            'image: site.png\nresolution: .nan\norigin: [0,0,0]',
            'image: site.png\nresolution: 0.05\norigin: [0,0]',
            'image: site.png\nresolution: true\norigin: [0,0,0]',
            'image: site.png\nresolution: 0\norigin: [0,0,0]',
            'image: missing.png\nresolution: 0.05\norigin: [0,0,0]',
            'image: [',
        ]
        for content in cases:
            with self.subTest(content=content):
                (root/'site.yaml').write_text(content)
                info, path = self.panel.saved_map()
                self.assertFalse(info['available'])
                self.assertIsNone(path)
        (root/'site.yaml').write_bytes(b'\xff')
        self.assertFalse(self.panel.saved_map()[0]['available'])

    def test_saved_map_http_auth_metadata_image_and_missing(self):
        http = ThreadingHTTPServer(('127.0.0.1', 0), server.handler_class(self.panel, 'test-token'))
        thread = threading.Thread(target=http.serve_forever, daemon=True)
        thread.start()
        base = f'http://127.0.0.1:{http.server_port}'
        def get(path, authorized=True):
            headers = {'Authorization': 'Bearer test-token'} if authorized else {}
            return urlopen(Request(base+path, headers=headers), timeout=2)
        try:
            for path in ('/api/map', '/api/map/image'):
                with self.assertRaises(HTTPError) as error:
                    get(path, authorized=False)
                self.assertEqual(error.exception.code, 401)
                error.exception.close()
            with get('/api/map') as response:
                self.assertFalse(json.load(response)['available'])
            with self.assertRaises(HTTPError) as error:
                get('/api/map/image')
            self.assertEqual(error.exception.code, 404)
            error.exception.close()
            png = self.write_saved_map()
            with get('/api/map') as response:
                info = json.load(response)
                self.assertTrue(info['available'])
            with get(info['image_url']) as response:
                self.assertEqual(response.headers['Content-Type'], 'image/png')
                self.assertEqual(response.headers['Cache-Control'], 'no-store')
                self.assertEqual(response.read(), png)
        finally:
            http.shutdown()
            http.server_close()
            thread.join(2)

    def test_default_gate_limits_diagonal_speed_and_blocks_rotation(self):
        self.command()
        vx, vy, wz = self.panel.tick()
        self.assertAlmostEqual(math.hypot(vx, vy), .3)
        self.assertEqual(wz, 0.)

    def test_manual_spin_preserves_translation_and_fixed_yaw_target(self):
        from robot_io import encode_command
        self.command('mapping')
        self.panel.set_spin(True, -.4)
        velocity = self.panel.tick()
        self.assertAlmostEqual(math.hypot(*velocity[:2]), .3)
        self.assertEqual(velocity[2], -.4)
        hardware = {'yaw_encoder_sign': -1, 'yaw_encoder_zero_rad': 1.2}
        packet = json.loads(encode_command(1, velocity, .2, hardware))
        self.assertAlmostEqual(packet['yaw_target_encoder_rad'], 1.)
        self.assertEqual(packet['wz'], -.4)

    def test_spin_in_place_discards_stale_translation(self):
        self.command('mapping')
        self.panel.set_spin(True, .5)
        self.panel.commands['manual'] = (0., (.2, .1, .4))
        self.assertEqual(self.panel.tick(), (0., 0., .5))

    def test_spin_heartbeat_loss_latches_off_and_does_not_resume(self):
        self.command('mapping')
        self.panel.set_spin(True)
        self.panel.spin_heartbeat -= 1.
        self.assertEqual(self.panel.tick()[2], 0.)
        self.assertFalse(self.panel.state()['spin']['enabled'])
        self.panel.heartbeat_spin()
        self.assertEqual(self.panel.tick()[2], 0.)

    def test_spin_heartbeat_renews_only_an_existing_enable(self):
        self.command('mapping')
        self.panel.heartbeat_spin()
        self.assertFalse(self.panel.state()['spin']['enabled'])
        self.panel.set_spin(True)
        self.panel.spin_heartbeat -= .3
        before = self.panel.spin_heartbeat
        self.panel.heartbeat_spin()
        self.assertGreater(self.panel.spin_heartbeat, before)
        self.assertEqual(self.panel.tick()[2], .5)
        self.panel.set_spin(False)
        self.assertEqual(self.panel.tick()[2], 0.)

    def test_spin_sensor_or_yaw_fault_stops_and_requires_reenable(self):
        for fault in ('cloud', 'odom', 'yaw'):
            with self.subTest(fault=fault):
                self.command('mapping')
                self.panel.set_spin(True)
                if fault == 'yaw': self.panel.yaw_guard.updated -= 1.
                else: setattr(self.panel, fault+'_time', 0.)
                self.assertEqual(self.panel.tick(), (0., 0., 0.))
                self.command('mapping')
                self.assertEqual(self.panel.tick()[2], 0.)

    def test_spin_disabled_on_mode_change_and_rejected_outside_manual(self):
        self.command('mapping')
        self.panel.set_spin(True)
        self.panel.request_mode('navigation')
        self.panel.worker.join(2)
        self.assertFalse(self.panel.state()['spin']['enabled'])
        for mode in ('idle', 'navigation'):
            self.panel.mode = mode
            with self.assertRaises(RuntimeError): self.panel.set_spin(True)

    def test_spin_rejects_invalid_rates_and_unready_real_hardware(self):
        self.command('mapping')
        for value in (0, .6, float('nan'), float('inf'), True, '0.5'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.panel.set_spin(True, value)
        with self.assertRaises(ValueError): self.panel.set_spin('true')
        self.panel.args.demo = False
        self.panel.cloud_time = 0.
        with self.assertRaises(RuntimeError): self.panel.set_spin(True)
        self.assertFalse(self.panel.state()['spin']['enabled'])

    def test_http_spin_auth_toggle_and_heartbeat_do_not_reenable(self):
        self.command('mapping')
        http = ThreadingHTTPServer(('127.0.0.1', 0), server.handler_class(self.panel, 'test-token'))
        thread = threading.Thread(target=http.serve_forever, daemon=True)
        thread.start()
        base = f'http://127.0.0.1:{http.server_port}'
        def request(path, body, authorized=True):
            headers = {'Content-Type': 'application/json'}
            if authorized: headers['Authorization'] = 'Bearer test-token'
            return urlopen(Request(base+path, json.dumps(body).encode(), headers), timeout=2)
        try:
            with self.assertRaises(HTTPError) as error:
                request('/api/spin', {'enabled': True}, authorized=False)
            self.assertEqual(error.exception.code, 401)
            error.exception.close()
            with request('/api/spin', {'enabled': True, 'rate_rad_s': -.3}) as response:
                self.assertTrue(json.load(response)['spin']['enabled'])
            with request('/api/spin', {'enabled': False}) as response:
                self.assertFalse(json.load(response)['spin']['enabled'])
            with request('/api/spin/heartbeat', {}) as response:
                self.assertFalse(json.load(response)['spin']['enabled'])
            with self.assertRaises(HTTPError) as error:
                request('/api/spin', [])
            self.assertEqual(error.exception.code, 400)
            error.exception.close()
        finally:
            http.shutdown()
            http.server_close()
            thread.join(2)

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



class NavigationTests(unittest.TestCase):
    setUp = PanelTests.setUp
    tearDown = PanelTests.tearDown
    unlock_yaw = PanelTests.unlock_yaw
    write_saved_map = PanelTests.write_saved_map

    def ready(self):
        from concurrent.futures import Future
        self.write_saved_map()
        p = self.panel
        p.args.demo = False
        p.mode = 'navigation'
        p.bridge = SimpleNamespace(send_goal=self.send)
        p.nav_server_ready = p.nav_localized = True
        p.map_pose = {'x': 1., 'y': -2., 'yaw': .7, 'frame': 'map'}
        p.map_pose_time = p.cloud_time = p.odom_time = time.monotonic()
        self.unlock_yaw()
        self.sent, self.feedback = [], None
        self.response = Future()
        self.result = Future()
        self.cancel = Future()
        self.cancel.set_result(SimpleNamespace(goals_canceling=['goal']))
        self.cancel_count = 0
        def cancel_goal():
            self.cancel_count += 1
            return self.cancel
        self.handle = SimpleNamespace(accepted=True, get_result_async=lambda: self.result,
                                      cancel_goal_async=cancel_goal)
        self.revision = p.saved_map()[0]['revision']

    def send(self, target, feedback):
        self.sent.append(dict(target))
        self.feedback = feedback
        return self.response

    def submit(self):
        self.panel.send_navigation_goal(1.025, -1.975, self.revision)

    def prepare_plan(self):
        from concurrent.futures import Future
        self.ready();self.plans=[]
        def plan(target):
            response,result=Future(),Future()
            handle=SimpleNamespace(accepted=True,get_result_async=lambda:result,cancel_goal_async=lambda:None)
            self.plans.append((response,result,handle))
            return response
        self.panel.bridge.plan_goal=plan

    def finish_plan(self,index=0,code=0,points=((1.,-2.),(1.025,-1.975))):
        response,result,handle=self.plans[index]
        response.set_result(handle)
        poses=[SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x,y=y))) for x,y in points]
        result.set_result(SimpleNamespace(status=6 if code else 4,result=SimpleNamespace(
            error_code=code,error_msg='',path=SimpleNamespace(header=SimpleNamespace(frame_id='map'),poses=poses))))

    def test_plan_preview_does_not_send_motion_and_reports_path_and_error(self):
        self.prepare_plan();p=self.panel
        p.plan_navigation_goal(1.025,-1.975,self.revision)
        self.assertEqual(p.nav_plan['status'],'planning')
        self.assertEqual(self.sent,[]);self.assertEqual(p.nav_status,'idle')
        self.finish_plan()
        self.assertEqual(p.nav_plan['status'],'ready')
        self.assertAlmostEqual(p.nav_plan['length_m'],math.hypot(.025,.025))
        self.assertEqual(p.state()['navigation']['plan']['path'],[[1.,-2.],[1.025,-1.975]])
        self.assertEqual(self.sent,[])
        p.plan_navigation_goal(1.025,-1.975,self.revision);self.finish_plan(1,208)
        self.assertEqual(p.nav_plan['status'],'failed');self.assertEqual(p.nav_plan['path'],[])
        self.assertIn('无可通行路径',p.nav_plan['message'])

    def test_new_selection_timeout_and_mode_stop_ignore_old_planning_results(self):
        self.prepare_plan();p=self.panel
        p.plan_navigation_goal(1.025,-1.975,self.revision)
        p.plan_navigation_goal(1.03,-1.97,self.revision)
        self.finish_plan(0)
        self.assertEqual(p.nav_plan['status'],'planning');self.assertEqual(p.nav_plan['goal']['x'],1.03)
        p.plan_started-=11.;p.state()
        self.finish_plan(1)
        self.assertEqual(p.nav_plan['status'],'failed');self.assertIn('超时',p.nav_plan['message'])
        p.plan_navigation_goal(1.025,-1.975,self.revision);p.clear_navigation_plan();self.finish_plan(2)
        self.assertEqual(p.nav_plan['status'],'idle')

    def test_navigation_failure_preserves_nav2_reason(self):
        self.ready();self.submit();self.response.set_result(self.handle)
        self.result.set_result(SimpleNamespace(status=6,result=SimpleNamespace(error_code=208,error_msg='')))
        self.assertIn('无可通行路径',self.panel.nav_message)
        self.assertIn('208',self.panel.nav_message)

    def test_http_planning_only_and_obstacle_overlay(self):
        self.prepare_plan();p=self.panel
        p.navigation_costmap={'available':True,'blocked':[2],'lethal':[3]}
        http=ThreadingHTTPServer(('127.0.0.1',0),server.handler_class(p,'token'))
        worker=threading.Thread(target=http.serve_forever,daemon=True);worker.start()
        base=f'http://127.0.0.1:{http.server_port}'
        headers={'Authorization':'Bearer token','Content-Type':'application/json'}
        try:
            payload=json.dumps({'x':1.025,'y':-1.975,'map_revision':self.revision}).encode()
            with urlopen(Request(base+'/api/navigation/plan',payload,headers)) as r:
                self.assertEqual(r.status,202);self.assertEqual(json.load(r)['navigation']['plan']['status'],'planning')
            self.assertEqual(self.sent,[])
            with self.assertRaises(HTTPError) as e:urlopen(base+'/api/navigation/costmap')
            self.assertEqual(e.exception.code,401);e.exception.close()
            with urlopen(Request(base+'/api/navigation/costmap',headers=headers)) as r:
                self.assertEqual(json.load(r)['blocked'],[2])
            self.finish_plan()
            with urlopen(Request(base+'/api/state',headers=headers)) as r:
                self.assertEqual(json.load(r)['navigation']['plan']['status'],'ready')
        finally:http.shutdown();http.server_close();worker.join(2)

    def test_goal_accept_feedback_complete_and_no_rotation(self):
        self.ready()
        p = self.panel
        self.submit()
        self.assertEqual(p.nav_status, 'sending')
        self.assertTrue(p.nav_hold)
        self.assertEqual(self.sent[0], {'x': 1.025, 'y': -1.975, 'yaw': .7, 'frame': 'map'})
        self.response.set_result(self.handle)
        self.assertEqual(p.nav_status, 'active')
        self.feedback(SimpleNamespace(feedback=SimpleNamespace(distance_remaining=2.3)))
        self.assertEqual(p.state()['navigation']['distance_remaining'], 2.3)
        p.commands['auto'] = (time.monotonic(), (.1, 0., 2.))
        self.assertEqual(p.tick(), (.1, 0., 0.))
        self.result.set_result(SimpleNamespace(status=4))
        self.assertEqual(p.nav_status, 'succeeded')
        p.commands['auto'] = (time.monotonic(), (.1, 0., 2.))
        self.assertEqual(p.tick(), (0., 0., 0.))

    def test_goal_invalid_coords_revision_bounds_and_rotated_origin(self):
        self.ready()
        p = self.panel
        for x, y, revision, error in [
            (True, -1.975, self.revision, ValueError),
            (math.nan, -1.975, self.revision, ValueError),
            (1.025, -1.975, 'old-map', RuntimeError),
            (0., 0., self.revision, ValueError),
        ]:
            with self.subTest(x=x, y=y, revision=revision):
                with self.assertRaises(error): p.send_navigation_goal(x, y, revision)
        self.assertEqual(self.sent, [])
        (p.args.maps_dir/'site.yaml').write_text('image: site.png\nresolution: 0.05\norigin: [10,20,1.5707963267948966]\n')
        revision = p.saved_map()[0]['revision']
        p.send_navigation_goal(9.975, 20.025, revision)
        self.assertEqual(len(self.sent), 1)

    def test_readiness_blocks_preview_missing_localization_and_stale_data(self):
        for condition in ('demo', 'mode', 'server', 'localization', 'map_pose', 'cloud', 'odom', 'yaw', 'switching'):
            with self.subTest(condition=condition):
                self.ready()
                p = self.panel
                if condition == 'demo': p.args.demo = True
                elif condition == 'mode': p.mode = 'mapping'
                elif condition == 'server': p.nav_server_ready = False
                elif condition == 'localization': p.nav_localized = False
                elif condition == 'map_pose': p.map_pose_time -= 2.
                elif condition == 'cloud': p.cloud_time -= 2.
                elif condition == 'odom': p.odom_time -= 2.
                elif condition == 'yaw': p.yaw_guard.updated = None
                else: p.switching = True
                with self.assertRaises(RuntimeError): self.submit()
                self.assertFalse(p.state()['navigation']['ready'])
                self.assertEqual(self.sent, [])
                p.switching = False

    def test_cancel_while_send_pending_cancels_late_acceptance(self):
        self.ready()
        p = self.panel
        self.submit()
        p.cancel_navigation()
        self.assertEqual(p.nav_status, 'canceling')
        p.commands['auto'] = (time.monotonic(), (.2, 0., 0.))
        self.assertEqual(p.tick(), (0., 0., 0.))
        with self.assertRaises(RuntimeError): self.submit()
        self.response.set_result(self.handle)
        self.assertEqual(self.cancel_count, 1)
        self.assertTrue(p.nav_hold)
        self.result.set_result(SimpleNamespace(status=5))
        self.assertEqual(p.nav_status, 'canceled')

    def test_rejection_failure_and_cancel_unconfirmed_hold_velocity(self):
        for outcome in ('reject', 'abort', 'cancel_unconfirmed'):
            with self.subTest(outcome=outcome):
                self.ready()
                p = self.panel
                self.submit()
                if outcome == 'reject':
                    self.response.set_result(SimpleNamespace(accepted=False))
                    self.assertEqual(p.nav_status, 'rejected')
                else:
                    self.response.set_result(self.handle)
                    if outcome == 'abort':
                        self.result.set_result(SimpleNamespace(status=6))
                        self.assertEqual(p.nav_status, 'failed')
                    else:
                        from concurrent.futures import Future
                        self.cancel = Future()
                        self.cancel.set_result(SimpleNamespace(goals_canceling=[]))
                        p.cancel_navigation()
                        self.assertEqual(p.nav_status, 'canceling')
                        self.assertIn('取消未确认', p.nav_message)
                self.assertTrue(p.nav_hold)
                p.stop_all()

    def test_switch_mode_ignores_late_action_callbacks(self):
        self.ready()
        p = self.panel
        self.submit()
        p.request_mode('idle')
        p.worker.join(2)
        self.assertFalse(p.worker.is_alive())
        self.response.set_result(self.handle)
        self.assertEqual(self.cancel_count, 1)
        self.assertEqual(p.mode, 'idle')
        self.assertEqual(p.nav_status, 'idle')
        self.assertFalse(p.nav_localized)

    def test_pending_goal_timeout_and_late_acceptance(self):
        self.ready()
        p = self.panel
        self.submit()
        p.nav_started -= 11.
        self.assertEqual(p.tick(), (0., 0., 0.))
        self.assertEqual(p.nav_status, 'canceling')
        self.response.set_result(self.handle)
        self.assertEqual(self.cancel_count, 1)

    def test_navigation_http_auth_submit_duplicate_cancel(self):
        self.ready()
        http = ThreadingHTTPServer(('127.0.0.1', 0), server.handler_class(self.panel, 'test-token'))
        thread = threading.Thread(target=http.serve_forever, daemon=True)
        thread.start()
        base = f'http://127.0.0.1:{http.server_port}'
        def post(path, body, authorized=True):
            headers = {'Content-Type': 'application/json'}
            if authorized: headers['Authorization'] = 'Bearer test-token'
            return urlopen(Request(base+path, json.dumps(body).encode(), headers), timeout=2)
        payload = {'x': 1.025, 'y': -1.975, 'map_revision': self.revision}
        try:
            for path in ('/api/navigation/goal', '/api/navigation/plan', '/api/navigation/cancel'):
                with self.assertRaises(HTTPError) as error: post(path, payload, False)
                self.assertEqual(error.exception.code, 401)
                error.exception.close()
            with self.assertRaises(HTTPError) as error: post('/api/navigation/goal', {})
            self.assertEqual(error.exception.code, 400)
            error.exception.close()
            with post('/api/navigation/goal', payload) as response:
                self.assertEqual(response.status, 202)
                self.assertEqual(json.load(response)['navigation']['status'], 'sending')
            self.response.set_result(self.handle)
            with self.assertRaises(HTTPError) as error: post('/api/navigation/goal', payload)
            self.assertEqual(error.exception.code, 409)
            error.exception.close()
            with post('/api/navigation/cancel', {}) as response:
                self.assertEqual(json.load(response)['navigation']['status'], 'canceling')
            self.assertEqual(self.cancel_count, 1)
        finally:
            http.shutdown()
            http.server_close()
            thread.join(2)



class MappingFilterTests(unittest.TestCase):
    setUp = PanelTests.setUp
    tearDown = PanelTests.tearDown
    unlock_yaw = PanelTests.unlock_yaw

    def test_default_persistence_and_mode_lock(self):
        p = self.panel
        self.assertEqual(p.mapping_filter, {'enabled': True, 'height_m': 1.5, 'dynamic_enabled': True})
        p.set_mapping_filter(True, 1.2)
        reloaded = server.Panel(p.args)
        try:
            self.assertEqual(reloaded.mapping_filter, {'enabled': True, 'height_m': 1.2, 'dynamic_enabled': True})
        finally:
            reloaded.close()
        for mode, switching in [('mapping', False), ('navigation', False), ('idle', True)]:
            p.mode, p.switching = mode, switching
            with self.assertRaises(RuntimeError): p.set_mapping_filter(False, 2.)
            self.assertTrue(p.state()['mapping_filter']['locked'])
            self.assertEqual(p.mapping_filter['height_m'], 1.2)
        p.mode, p.switching = 'idle', False
        p.set_mapping_filter(False, 1.2)
        self.assertFalse(p.mapping_filter['enabled'])

    def test_invalid_setting_preserves_previous_settings(self):
        p = self.panel
        for enabled, height in [(1, 1.5), (True, True), (True, 0), (True, -1),
                                (True, 51), (True, math.inf), (True, math.nan), (True, '1.5')]:
            with self.subTest(enabled=enabled, height=height):
                with self.assertRaises(ValueError): p.set_mapping_filter(enabled, height)
        self.assertEqual(p.mapping_filter, {'enabled': True, 'height_m': 1.5, 'dynamic_enabled': True})
        self.assertFalse((p.args.maps_dir/'mapping_filter.json').exists())

    def test_mapping_launch_uses_saved_settings(self):
        p = self.panel
        p.set_mapping_filter(True, 1.25)
        p.args.demo = False
        process = SimpleNamespace(poll=lambda: None)
        try:
            with patch.object(server.subprocess, 'Popen', return_value=process) as popen:
                p.transition('idle', 'mapping')
            command = popen.call_args.args[0]
            self.assertIn('map_dynamic_enabled:=true', command)
            self.assertIn('map_ceiling_enabled:=true', command)
            self.assertIn('map_ceiling_height:=1.25', command)
            self.assertEqual(p.mode, 'mapping')
        finally:
            p.processes.clear()

    def test_dynamic_setting_legacy_load_and_validation(self):
        p = self.panel
        path = p.args.maps_dir/'mapping_filter.json'
        path.write_text(json.dumps({'enabled': False, 'height_m': 2.}))
        reloaded = server.Panel(p.args)
        try:
            self.assertTrue(reloaded.mapping_filter['dynamic_enabled'])
        finally:
            reloaded.close()
        p.set_mapping_filter(True, 1.5, False)
        p.set_mapping_filter(True, 1.2)  # old clients preserve the dynamic setting
        self.assertFalse(p.mapping_filter['dynamic_enabled'])
        for invalid in [1, 0, 'false', {}, []]:
            with self.assertRaises(ValueError): p.set_mapping_filter(True, 1.5, invalid)
        reloaded = server.Panel(p.args)
        try:
            self.assertFalse(reloaded.mapping_filter['dynamic_enabled'])
        finally:
            reloaded.close()
        p.mode = 'mapping'
        with self.assertRaises(RuntimeError): p.set_mapping_filter(True, 1.5, True)
        p.mode = 'idle'

    def test_filter_http_auth_persist_and_reject_during_mapping(self):
        p = self.panel
        http = ThreadingHTTPServer(('127.0.0.1', 0), server.handler_class(p, 'test-token'))
        thread = threading.Thread(target=http.serve_forever, daemon=True)
        thread.start()
        base = f'http://127.0.0.1:{http.server_port}'
        def post(body, authorized=True):
            headers = {'Content-Type': 'application/json'}
            if authorized: headers['Authorization'] = 'Bearer test-token'
            return urlopen(Request(base+'/api/mapping/filter', json.dumps(body).encode(), headers), timeout=2)
        try:
            with self.assertRaises(HTTPError) as error:
                post({'enabled': True, 'height_m': 1.2, 'dynamic_enabled': True}, False)
            self.assertEqual(error.exception.code, 401)
            error.exception.close()
            with post({'enabled': True, 'height_m': 1.2, 'dynamic_enabled': True}) as response:
                self.assertEqual(json.load(response)['mapping_filter']['height_m'], 1.2)
            with post({'enabled': True, 'height_m': 1.2, 'dynamic_enabled': False}) as response:
                self.assertFalse(json.load(response)['mapping_filter']['dynamic_enabled'])
            with self.assertRaises(HTTPError) as error:
                post({'enabled': True, 'height_m': 1.2, 'dynamic_enabled': 'false'})
            self.assertEqual(error.exception.code, 400)
            error.exception.close()
            p.mode = 'mapping'
            with self.assertRaises(HTTPError) as error:
                post({'enabled': False, 'height_m': 1.2})
            self.assertEqual(error.exception.code, 409)
            error.exception.close()
            self.assertTrue(p.mapping_filter['enabled'])
        finally:
            p.mode = 'idle'
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
            controller = cfg['controller_server']['ros__parameters']
            self.assertEqual(controller['FollowPath']['max_vel_theta'], 0.)
            self.assertEqual(controller['FollowPath']['vtheta_samples'], 1)
            self.assertAlmostEqual(controller['general_goal_checker']['yaw_goal_tolerance'], math.pi)
            self.assertNotIn('RotateToGoal', controller['FollowPath']['critics'])
            self.assertNotIn('spin', cfg['behavior_server']['ros__parameters']['behavior_plugins'])
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
