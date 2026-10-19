import json
from http.server import ThreadingHTTPServer
import struct
import threading
import time
import unittest
from urllib.request import Request,urlopen
from urllib.error import HTTPError

from navcar_protocol import STATE, ACK, frame, crc16_modbus
from navcar_move_web import Controller, handler


def config():
    return dict(forward_units_per_mps=1,left_units_per_mps=1,rot_units_per_radps=1,
                max_translation_units=.3,max_rot_units=6.,yaw_target_deg=0)


def state_packet(seq=65535,values=(0,0,0,17)):
    return frame(STATE,seq,struct.pack('<4f',*values))


class MoveTests(unittest.TestCase):
    def setUp(self):
        self.now=10.
        self.control=Controller(config(),clock=lambda:self.now)
        self.control.session.receive(state_packet(),self.now)
        self.control.yaw=17.

    def send(self,action,seq=1,client='test',strength=.1):
        self.control.command(dict(action=action,strength=strength,speed_unit='m/s',client=client,seq=seq))

    def test_four_directions_no_spin_and_preserves_yaw(self):
        for seq,(action,expected) in enumerate([('forward',(.1,0,0,17)),('backward',(-.1,0,0,17)),
                                              ('left',(0,-.1,0,17)),('right',(0,.1,0,17))],1):
            self.send(action,seq);self.assertEqual(self.control.desired(),expected)
        self.send('stop',5);self.assertEqual(self.control.desired(),(0,0,0,17))

    def test_measured_tenfold_scale_applies_once_to_manual_and_navigation(self):
        self.control.config.update(forward_units_per_mps=10.,left_units_per_mps=10.,max_translation_units=30.)
        with self.assertRaisesRegex(ValueError,'请刷新'):
            self.control.command(dict(action='forward',strength=.5,client='old-page',seq=1))
        self.assertFalse(self.control.armed)
        self.assertEqual(self.control.state()['max_strength'],3.)
        self.assertEqual(self.control.state()['speed_unit'],'m/s')
        for seq,(action,expected) in enumerate([('forward',(5.,0.,0.,17.)),('backward',(-5.,0.,0.,17.)),
                                              ('left',(0.,-5.,0.,17.)),('right',(0.,5.,0.,17.))],1):
            self.send(action,seq,strength=.5)
            self.assertEqual(self.control.desired(),expected)
        self.send('forward',5,strength=3.)
        self.assertEqual(self.control.desired()[0],30.)
        with self.assertRaises(ValueError):self.send('forward',6,strength=3.01)
        self.send('stop',7,strength=.5)
        lease=self.control.acquire_navigation()
        self.control.navigation_command(dict(lease=lease,seq=1,forward=.06,left=.08))
        self.assertEqual(self.control.desired(),(.6,-.8,0.,17.))
        self.now+=.201
        self.assertEqual(self.control.desired(),(0.,0.,0.,17.))

    def test_expiry_stale_request_and_other_page_exclusion(self):
        self.send('forward',1)
        with self.assertRaises(RuntimeError):self.send('left',1,client='other')
        self.send('stop',3);self.send('forward',2)
        self.assertEqual(self.control.desired()[:3],(0,0,0))
        self.send('stop',1,client='other');self.send('forward',2)
        self.assertEqual(self.control.desired()[:3],(0,0,0))
        self.send('forward',4)
        self.now+=.201
        self.assertEqual(self.control.desired()[:3],(0,0,0))

    def test_bad_commands_and_disconnected_controller_rejected(self):
        for strength in (0,-.1,.301,float('nan'),float('inf'),True):
            with self.assertRaises(ValueError):self.send('forward',strength=strength)
        with self.assertRaises(ValueError):self.send('spin')
        self.now+=.301
        with self.assertRaises(RuntimeError):self.send('forward')
        self.control.session.fail('ACK timeout')
        with self.assertRaises(RuntimeError):self.send('forward')

    def test_yaw_sets_target_stops_translation_and_spin_and_rejects_stale_requests(self):
        self.control.set_spin(dict(enabled=True,rate=.2,client='test',seq=1))
        self.control.command(dict(action='forward',strength=.1,client='test',seq=2,spin_enabled=True,spin_rate=.2))
        self.assertEqual(self.control.desired(),(.1,0,-.2,17))
        self.control.set_yaw(dict(angle_deg=45,client='test',seq=3))
        self.assertEqual(self.control.desired(),(0,0,0,45))
        self.assertFalse(self.control.state()['spin']['enabled'])
        self.control.set_yaw(dict(angle_deg=-90,client='test',seq=2))
        self.assertEqual(self.control.yaw,45)
        for angle in (-181,181,float('nan'),float('inf'),True,None):
            with self.assertRaises(ValueError):self.control.set_yaw(dict(angle_deg=angle,client='test',seq=4))

    def test_spin_translation_release_rate_adjustment_stop_and_timeout(self):
        self.assertFalse(self.control.state()['spin']['enabled'])
        self.control.set_spin(dict(enabled=True,rate=.2,client='test',seq=1))
        self.assertEqual(self.control.desired(),(0,0,-.2,17))
        with self.assertRaises(RuntimeError):self.control.acquire_navigation()
        self.control.command(dict(action='left',strength=.1,client='test',seq=2,spin_enabled=True,spin_rate=.2))
        self.assertEqual(self.control.desired(),(0,-.1,-.2,17))
        self.control.set_spin(dict(enabled=True,rate=-6.,client='test',seq=3))
        self.assertEqual(self.control.desired(),(0,-.1,6.,17))
        self.control.command(dict(action='stop',strength=.1,client='test',seq=4,spin_enabled=True,spin_rate=6.))
        self.assertEqual(self.control.desired(),(0,0,-6.,17))
        self.assertEqual(self.control.state()['spin']['max_rate'],6.)
        self.now+=.201
        self.assertEqual(self.control.desired(),(0,0,0,17))
        self.assertFalse(self.control.state()['spin']['enabled'])
        self.control.session.state_time=self.now
        self.control.set_spin(dict(enabled=True,rate=.2,client='test',seq=5))
        self.control.set_spin(dict(enabled=False,client='other',seq=1))
        self.assertEqual(self.control.desired(),(0,0,0,17))
        self.control.set_spin(dict(enabled=True,rate=.3,client='test',seq=4))
        self.assertEqual(self.control.desired(),(0,0,0,17))

    def test_spin_validation_and_manual_yaw_revoke_navigation(self):
        for rate in (0,6.01,-6.01,float('nan'),True):
            with self.assertRaises(ValueError):self.control.set_spin(dict(enabled=True,rate=rate,client='test',seq=1))
        with self.assertRaises(ValueError):self.control.set_spin(dict(enabled=1,rate=.1,client='test',seq=1))
        lease=self.control.acquire_navigation()
        self.control.set_yaw(dict(angle_deg=-20,client='test',seq=1))
        with self.assertRaises(RuntimeError):self.control.navigation_command(dict(lease=lease,seq=1,forward=.1,left=0))
        self.assertEqual(self.control.desired(),(0,0,0,-20))
        self.now+=.301
        with self.assertRaises(RuntimeError):self.control.set_yaw(dict(angle_deg=20,client='test',seq=2))
        with self.assertRaises(RuntimeError):self.control.set_spin(dict(enabled=True,rate=.1,client='test',seq=2))

    def test_http_auth_and_command_validation(self):
        http=ThreadingHTTPServer(('127.0.0.1',0),handler(self.control,'secret'))
        worker=threading.Thread(target=http.serve_forever,daemon=True);worker.start()
        base=f'http://127.0.0.1:{http.server_port}'
        try:
            with self.assertRaises(HTTPError) as e:urlopen(base+'/api/state')
            self.assertEqual(e.exception.code,401);e.exception.close()
            headers={'Authorization':'Bearer secret','Content-Type':'application/json'}
            packet=dict(action='forward',strength=.1,client='browser',seq=1)
            response=urlopen(Request(base+'/api/command',data=json.dumps(packet).encode(),headers=headers))
            self.assertEqual(json.load(response)['active'],'forward');response.close()
            packet['action']='stop';packet['seq']=2
            response=urlopen(Request(base+'/api/command',data=json.dumps(packet).encode(),headers=headers))
            self.assertEqual(json.load(response)['active'],'stop');response.close()
            packet=dict(angle_deg=30,client='browser',seq=3)
            with urlopen(Request(base+'/api/yaw',data=json.dumps(packet).encode(),headers=headers)) as response:
                self.assertEqual(json.load(response)['yaw_target_deg'],30)
            packet=dict(enabled=True,rate=.2,client='browser',seq=4)
            with urlopen(Request(base+'/api/spin',data=json.dumps(packet).encode(),headers=headers)) as response:
                self.assertTrue(json.load(response)['spin']['enabled'])
        finally:http.shutdown();http.server_close();worker.join(1)

    def test_serial_loop_with_simulated_mcu_and_deadman(self):
        class Port:
            def __init__(self):self.buffer=bytearray(state_packet());self.commands=[];self.closed=False
            @property
            def in_waiting(self):return len(self.buffer)
            def read(self,n):
                data=bytes(self.buffer[:n]);del self.buffer[:n];return data
            def write(self,packet):
                self.assert_packet(packet)
                seq=struct.unpack_from('<H',packet,4)[0];values=struct.unpack_from('<4f',packet,8)
                self.commands.append((time.monotonic(),values))
                self.buffer.extend(frame(ACK,seq,b'\0')+state_packet(seq,values))
                return len(packet)
            def assert_packet(self,p):
                assert len(p)==26 and p[:4]==b'\xa5\x5a\x01\x01'
                assert struct.unpack_from('<H',p,24)[0]==crc16_modbus(p[2:24])
            def close(self):self.closed=True
        c=Controller(config());port=Port();worker=threading.Thread(target=c.run,args=(port,));worker.start()
        try:
            deadline=time.monotonic()+1
            while c.yaw is None and time.monotonic()<deadline:time.sleep(.005)
            self.assertEqual(c.yaw,17)
            self.assertEqual(port.commands,[])  # Merely opening the page cannot take control.
            c.command(dict(action='forward',strength=.1,client='sim',seq=1))
            time.sleep(.08)
            self.assertTrue(any(v[0]>.09 for _,v in port.commands))
            time.sleep(.2)
            self.assertEqual(port.commands[-1][1],(0.,0.,0.,17.))
            self.assertTrue(all(v[2]==0 and v[3]==17 for _,v in port.commands))
        finally:c.closed.set();worker.join(1)
        self.assertTrue(port.closed)


if __name__=='__main__':unittest.main()
