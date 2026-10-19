import json
from pathlib import Path
import subprocess
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from http.server import ThreadingHTTPServer

import server
from lidar_mapping_control import MappingControl, LidarMappingPanelMixin


class MappingStopTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.maps = self.root/'maps'/'session'
        self.maps.mkdir(parents=True)
        (self.root/'mapping.pid').write_text('12345')
        (self.root/'current_mapping_dir').write_text(str(self.maps))
        self.control = MappingControl(self.root, self.maps)
        self.running = True
        self.running_patch = patch.object(self.control, 'is_running', side_effect=lambda: self.running)
        self.running_patch.start()
        self.release = threading.Event()
        self.entered = threading.Event()
        self.calls = []
        self.failed = False
        def run(command, **kwargs):
            self.calls.append((command, kwargs))
            self.entered.set()
            if not self.release.wait(3):raise RuntimeError('Test worker blocked')
            if self.failed:return SimpleNamespace(returncode=1, stderr='PCD write failed; mapping kept running', stdout='')
            self.running = False
            return SimpleNamespace(returncode=0, stderr='', stdout='saved')
        self.control.runner = run

    def tearDown(self):
        self.release.set()
        if self.control.worker:self.control.worker.join(3)
        self.running_patch.stop()
        self.tmp.cleanup()

    def test_nonblocking_stop_duplicate_requests_and_final_state(self):
        self.control.request_stop()
        self.assertTrue(self.entered.wait(1))
        self.assertTrue(self.control.status()['stopping'])
        self.assertFalse(self.control.status()['can_stop'])
        self.control.request_stop()
        self.assertEqual(len(self.calls), 1)
        command, kwargs = self.calls[0]
        self.assertIn('--expected-pid', command)
        self.assertIn(str(self.maps), command)
        self.assertEqual(kwargs['timeout'], 55)
        self.release.set();self.control.worker.join(2)
        status = self.control.status()
        self.assertTrue(status['saved'])
        self.assertFalse(status['running'])
        self.assertFalse(status['stopping'])

    def test_save_failure_keeps_running_and_allows_retry(self):
        self.failed = True;self.release.set()
        self.control.request_stop();self.control.worker.join(2)
        status = self.control.status()
        self.assertTrue(status['running'])
        self.assertTrue(status['can_stop'])
        self.assertFalse(status['saved'])
        self.assertIn('PCD write failed', status['error'])
        self.failed = False
        self.control.request_stop();self.control.worker.join(2)
        self.assertTrue(self.control.status()['saved'])
        self.assertEqual(self.control.status()['error'], '')

    def test_timeout_is_not_reported_as_saved(self):
        def timeout(*args, **kwargs):raise subprocess.TimeoutExpired('stop',55)
        self.control.runner = timeout
        self.control.request_stop();self.control.worker.join(2)
        self.assertFalse(self.control.status()['saved'])
        self.assertIn('超时',self.control.status()['error'])
        self.assertTrue(self.control.status()['can_stop'])

    def test_success_code_without_process_exit_is_error(self):
        self.control.runner = lambda *a, **k:SimpleNamespace(returncode=0)
        self.control.request_stop();self.control.worker.join(2)
        self.assertFalse(self.control.status()['saved'])
        self.assertIn('仍在运行',self.control.status()['error'])

    def test_foreign_session_cannot_be_stopped(self):
        (self.root/'mapping.pid').write_text('54321')
        with self.assertRaises(RuntimeError):self.control.request_stop()
        self.assertEqual(self.calls, [])
        (self.root/'mapping.pid').write_text('12345')
        (self.root/'current_mapping_dir').write_text(str(self.root/'other-map'))
        with self.assertRaises(RuntimeError):self.control.request_stop()

    def test_already_stopped_is_idempotent(self):
        self.running = False
        self.control.request_stop()
        self.assertEqual(self.calls, [])
        self.assertFalse(self.control.status()['saved'])

    def test_actual_http_auth_async_progress_and_saved_map_access(self):
        class Panel(LidarMappingPanelMixin, server.Panel):pass
        panel = Panel(SimpleNamespace(demo=True,maps_dir=self.maps,max_points=100,max_speed=.3,max_yaw_rate=.5))
        panel.mapping_control = self.control
        http = ThreadingHTTPServer(('127.0.0.1',0),server.handler_class(panel,'token'))
        thread = threading.Thread(target=http.serve_forever,daemon=True);thread.start()
        base = f'http://127.0.0.1:{http.server_port}'
        def post(token='token'):
            return urlopen(Request(base+'/api/mode',data=b'{"mode":"idle"}',headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'}),timeout=2)
        def get(path):
            return json.load(urlopen(Request(base+path,headers={'Authorization':'Bearer token'}),timeout=2))
        try:
            with self.assertRaises(HTTPError) as error:post('wrong')
            self.assertEqual(error.exception.code,401);error.exception.close()
            self.assertFalse(self.entered.is_set())
            with post() as response:
                self.assertEqual(response.status,202)
                self.assertTrue(json.load(response)['switching'])
            self.assertTrue(self.entered.wait(1))
            with post():pass
            self.assertEqual(len(self.calls),1)
            self.release.set();self.control.worker.join(2)
            state=get('/api/state')
            self.assertEqual(state['mode'],'idle')
            self.assertEqual(state['point_topic'],'/livox/lidar')
            self.assertFalse(state['switching'])
            self.assertTrue(state['mapping_control']['saved'])
            self.assertTrue(state['preview'])
            self.assertFalse(state['navigation']['ready'])
            self.assertIn('底盘自动控制尚未接通',state['navigation']['reason'])
            with self.assertRaises(RuntimeError) as error:panel.request_mode('navigation')
            self.assertEqual(str(error.exception),state['navigation']['reason'])
            # Serving maps continues even though the mapping worker has stopped.
            self.assertIn('available',get('/api/map'))
        finally:
            http.shutdown();http.server_close();thread.join(2);panel.close()


class MappingStartTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.old=self.root/'old';self.new=self.root/'new';self.old.mkdir();self.new.mkdir()
        (self.root/'mapping.pid').write_text('12345')
        (self.root/'current_mapping_dir').write_text(str(self.old))
        (self.old/'site.pcd').write_bytes(b'previous saved map')
        (self.old/'mapping_filter.json').write_text('{"enabled":true,"height_m":2.5,"dynamic_enabled":true}')
        self.running=False;self.release=threading.Event();self.entered=threading.Event();self.calls=[];self.adopted=[]
        self.failed=False
        def run(command,**kwargs):
            self.calls.append((command,kwargs));self.entered.set()
            if not self.release.wait(3):raise RuntimeError('test blocked')
            if self.failed:return SimpleNamespace(returncode=1,stderr='driver busy',stdout='')
            (self.root/'mapping.pid').write_text('54321')
            (self.root/'current_mapping_dir').write_text(str(self.new));self.running=True
            return SimpleNamespace(returncode=0,stderr='',stdout='RESULT_JSON '+json.dumps({'pid':54321,'maps_dir':str(self.new)}))
        self.control=MappingControl(self.root,self.old,runner=run,on_started=self.adopted.append)
        self.patch=patch.object(self.control,'is_running',side_effect=lambda:self.running);self.patch.start()

    def tearDown(self):
        self.release.set()
        if self.control.worker:self.control.worker.join(3)
        self.patch.stop();self.tmp.cleanup()

    def test_async_start_idempotence_new_directory_and_filter_preservation(self):
        self.assertTrue(self.control.status()['can_start'])
        self.control.request_start();self.assertTrue(self.entered.wait(1))
        self.assertTrue(self.control.status()['starting']);self.assertFalse(self.control.status()['can_start'])
        self.control.request_start();self.assertEqual(len(self.calls),1)
        with self.assertRaises(RuntimeError):self.control.request_stop()
        command,kwargs=self.calls[0]
        self.assertIn('--keep-web',command);self.assertIn(str(self.old/'mapping_filter.json'),command)
        self.assertIn('--expected-pid',command);self.assertEqual(kwargs['timeout'],30)
        self.release.set();self.control.worker.join(2)
        self.assertEqual(self.control.pid,54321);self.assertEqual(self.control.maps_dir,self.new)
        self.assertEqual(self.adopted,[self.new]);self.assertTrue(self.control.status()['can_stop'])
        self.assertFalse(self.control.status()['starting'])
        self.assertEqual((self.old/'site.pcd').read_bytes(),b'previous saved map')

    def test_start_failure_keeps_saved_map_and_retry_available(self):
        self.failed=True;self.release.set();self.control.request_start();self.control.worker.join(2)
        self.assertEqual(self.control.maps_dir,self.old);self.assertIn('driver busy',self.control.status()['error'])
        self.assertTrue(self.control.status()['can_start']);self.assertEqual(self.adopted,[])

    def test_foreign_session_rejected_and_navigation_not_enabled(self):
        (self.root/'mapping.pid').write_text('999')
        with self.assertRaises(RuntimeError):self.control.request_start()
        self.assertEqual(self.calls,[])

    def test_http_start_switches_served_map_without_restarting_web(self):
        class Panel(LidarMappingPanelMixin,server.Panel):pass
        panel=Panel(SimpleNamespace(demo=True,maps_dir=self.old,max_points=100,max_speed=.3,max_yaw_rate=.5))
        panel.mapping_control=self.control
        self.control.on_started=lambda maps:setattr(panel.args,'maps_dir',maps)
        http=ThreadingHTTPServer(('127.0.0.1',0),server.handler_class(panel,'token'))
        thread=threading.Thread(target=http.serve_forever,daemon=True);thread.start()
        base=f'http://127.0.0.1:{http.server_port}'
        try:
            with urlopen(Request(base+'/api/mode',data=b'{"mode":"mapping"}',headers={'Authorization':'Bearer token','Content-Type':'application/json'}),timeout=2) as response:
                self.assertEqual(response.status,202);self.assertTrue(json.load(response)['switching'])
            self.release.set();self.control.worker.join(2)
            with urlopen(Request(base+'/api/state',headers={'Authorization':'Bearer token'}),timeout=2) as response:
                status=json.load(response)
            self.assertEqual(status['mode'],'mapping');self.assertFalse(status['switching'])
            self.assertEqual(status['point_topic'],'/cloud_registered');self.assertEqual(panel.args.maps_dir,self.new)
            with self.assertRaises(RuntimeError):panel.request_mode('navigation')
        finally:http.shutdown();http.server_close();thread.join(2);panel.close()


if __name__ == '__main__':unittest.main()
