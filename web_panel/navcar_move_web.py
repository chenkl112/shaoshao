#!/usr/bin/env python3
"""Independent, hold-to-move Navcar controller. No ROS or mapping process changes."""
import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import secrets
import signal
import struct
import threading
import time
from urllib.parse import urlparse

from navcar_bridge import load_config, targets
from navcar_protocol import Session


class Controller:
    def __init__(self, config, clock=time.monotonic):
        self.config, self.clock = config, clock
        self.lock = threading.RLock()
        self.session = Session()
        self.action, self.strength, self.updated = 'stop', .1, 0.
        self.owner, self.sequence = None, -1
        self.client_sequences = {}
        self.armed = False
        self.navigation_lease = None
        self.navigation_sequence = -1
        self.navigation_vector = (0., 0.)
        self.spin_enabled, self.spin_rate, self.spin_updated = False, .1, 0.
        self.yaw = None
        self.closed = threading.Event()

    def max_speed(self):
        return self.config['max_translation_units']/max(self.config['forward_units_per_mps'],self.config['left_units_per_mps'])

    def spin_live(self):
        return self.spin_enabled and self.clock()-self.spin_updated < .2

    def validate_spin(self, enabled, rate):
        if type(enabled) is not bool:raise ValueError('enabled必须为true或false')
        limit=self.config['max_rot_units']/self.config['rot_units_per_radps']
        if type(rate) not in (int,float) or not math.isfinite(rate) or abs(rate)>limit or (enabled and rate==0):
            raise ValueError('旋转指令须非零且绝对值不超过'+str(limit))

    def accept_manual(self, client, seq, wants_motion, takeover=False):
        if not isinstance(client,str) or not 1 <= len(client) <= 80 or type(seq) is not int or not 0 <= seq <= 2**53:
            raise ValueError('无效控制会话或序号')
        if wants_motion and (not self.state()['connected'] or self.session.error or self.yaw is None):
            raise RuntimeError('下位机未就绪')
        if seq <= self.client_sequences.get(client,-1):return False
        if client not in self.client_sequences and len(self.client_sequences)>=64:
            raise RuntimeError('控制会话已满，请重启控制服务')
        active=(self.action!='stop' and self.clock()-self.updated<.2) or self.spin_live()
        if wants_motion and not takeover and self.owner!=client and active and not self.navigation_lease:
            raise RuntimeError('另一个页面正在控制')
        self.owner,self.sequence=client,seq
        self.client_sequences[client]=seq
        self.navigation_lease=None
        return True

    def state(self):
        with self.lock:
            now = self.clock();s = self.session
            fresh = s.state_time is not None and now-s.state_time < .3
            active = self.action if now-self.updated < .2 else 'stop'
            return {'connected': fresh, 'error': s.error, 'active': active if self.armed else 'stop',
                    'control_started': self.armed, 'strength': self.strength, 'max_strength': self.max_speed(),
                    'speed_unit': 'm/s',
                    'translation_scale': {'forward': self.config['forward_units_per_mps'], 'left': self.config['left_units_per_mps']},
                    'control_source': 'navigation' if self.navigation_lease else 'manual',
                    'control_client': self.owner,
                    'spin': {'enabled': self.spin_live(), 'rate': self.spin_rate,
                             'max_rate': self.config['max_rot_units']/self.config['rot_units_per_radps'], 'unit': 'command'},
                    'yaw_target_deg': self.yaw, 'last_ack': s.last_ack,
                    'feedback_targets': s.state.values if s.state else None,
                    'feedback_age_s': round(now-s.state_time,3) if s.state_time is not None else None}

    def command(self, data):
        if not isinstance(data, dict):raise ValueError('需要命令对象')
        action, strength, client, seq = (data.get(k) for k in ('action','strength','client','seq'))
        if action not in ('forward','backward','left','right','stop'):raise ValueError('无效方向')
        if (action != 'stop' and data.get('speed_unit') != 'm/s' and
                any(self.config[k] != 1 for k in ('forward_units_per_mps','left_units_per_mps'))):
            raise ValueError('速度倍率已更新，请刷新移动网页后操作')
        if type(strength) not in (int,float) or not math.isfinite(strength) or not 0 < strength <= self.max_speed():
            raise ValueError('目标速度超出范围')
        spin_enabled,spin_rate=data.get('spin_enabled',False),data.get('spin_rate',self.spin_rate)
        self.validate_spin(spin_enabled,spin_rate)
        with self.lock:
            now=self.clock()
            if not self.accept_manual(client,seq,action!='stop' or spin_enabled):return
            self.action, self.strength, self.updated = action, float(strength), now
            self.spin_enabled,self.spin_rate,self.spin_updated=spin_enabled,float(spin_rate),now
            self.armed = True

    def set_yaw(self, data):
        if not isinstance(data,dict):raise ValueError('需要命令对象')
        angle=data.get('angle_deg')
        if type(angle) not in (int,float) or not math.isfinite(angle) or not -180 <= angle <= 180:
            raise ValueError('yaw目标角必须在-180到180度之间')
        with self.lock:
            if not self.accept_manual(data.get('client'),data.get('seq'),True,takeover=True):return
            self.action='stop';self.updated=0.;self.spin_enabled=False
            self.navigation_vector=(0.,0.)
            self.yaw=float(angle);self.armed=True

    def set_spin(self, data):
        if not isinstance(data,dict):raise ValueError('需要命令对象')
        enabled,rate=data.get('enabled'),data.get('rate',self.spin_rate)
        self.validate_spin(enabled,rate)
        with self.lock:
            action=self.action if self.owner==data.get('client') and self.clock()-self.updated<.2 and self.action!='navigation' else 'stop'
            # Turning off is also a stop request and may always take control.
            if not enabled:action='stop'
            self.command({'action':action,'strength':self.strength,'speed_unit':'m/s',
                          'spin_enabled':enabled,'spin_rate':rate,'client':data.get('client'),'seq':data.get('seq')})

    def acquire_navigation(self):
        with self.lock:
            if not self.state()['connected'] or self.session.error or self.yaw is None:
                raise RuntimeError('下位机未就绪')
            if self.navigation_lease or self.spin_live() or (self.action != 'stop' and self.clock()-self.updated < .2):
                raise RuntimeError('控制器正在使用，请先停止当前控制')
            self.navigation_lease = secrets.token_urlsafe(24)
            self.navigation_sequence = -1
            self.navigation_vector = (0., 0.)
            self.action = 'stop'
            self.spin_enabled=False
            return self.navigation_lease

    def navigation_command(self, data):
        if not isinstance(data, dict):raise ValueError('需要命令对象')
        forward,left,seq=(data.get(k) for k in ('forward','left','seq'))
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in (forward,left)):
            raise ValueError('导航速度必须为有限数值')
        if math.hypot(forward,left) > min(.1,self.max_speed())+1e-9:
            raise ValueError('导航速度上限为0.10米/秒')
        if type(seq) is not int or not 0 <= seq <= 2**53:raise ValueError('无效导航序号')
        with self.lock:
            if not self.navigation_lease or data.get('lease') != self.navigation_lease:
                raise RuntimeError('导航控制权已失效，可能已被手动接管')
            if not self.state()['connected'] or self.session.error:raise RuntimeError('下位机未就绪')
            if seq <= self.navigation_sequence:return
            self.navigation_sequence=seq
            self.spin_enabled=False
            self.navigation_vector=(float(forward),float(left))
            self.action='navigation' if forward or left else 'stop'
            self.updated=self.clock();self.armed=True

    def release_navigation(self, lease):
        with self.lock:
            if self.navigation_lease and lease == self.navigation_lease:
                self.navigation_lease=None
                self.navigation_vector=(0.,0.)
                self.action='stop';self.updated=0.

    def desired(self):
        with self.lock:
            direction=self.action if self.clock()-self.updated < .2 else 'stop'
            u=self.strength
            forward,left=self.navigation_vector if direction=='navigation' and self.navigation_lease else {'forward':(u,0.),'backward':(-u,0.),'left':(0.,u),'right':(0.,-u),'stop':(0.,0.)}.get(direction,(0.,0.))
            cfg={**self.config,'yaw_target_deg':self.yaw if self.yaw is not None else 0.}
            return targets((forward,left,self.spin_rate if self.spin_live() else 0.),cfg)

    def run(self, port):
        try:
            while not self.closed.is_set():
                now=self.clock()
                data=port.read(min(port.in_waiting,4096))
                with self.lock:
                    s=self.session;s.receive(data,now)
                    if self.yaw is None and s.state is not None:self.yaw=s.state.values[3]
                    if self.armed and not s.error:
                        values=self.desired()
                        # Never retry an unacknowledged movement after release/timeout.
                        moving_pending=s.pending is not None and any(struct.unpack_from('<4f',s.pending,8)[:3])
                        if moving_pending and not any(values[:3]) and now-s.sent_at >= .05:
                            s.fail('运动指令ACK超时，已停止发帧；请重启控制服务')
                        else:
                            packet=s.poll(values,now)
                            if packet and port.write(packet) != len(packet):raise IOError('串口发送不完整')
                self.closed.wait(.002)
        except Exception as exc:
            with self.lock:self.session.fail(str(exc))
        finally:
            # Finish a normal shutdown with a fresh zero command and retain yaw target.
            with self.lock:self.action='stop';self.updated=0.;self.spin_enabled=False
            deadline=time.monotonic()+.25
            while self.armed and not self.session.error and time.monotonic()<deadline:
                try:
                    now=self.clock()
                    with self.lock:
                        self.session.receive(port.read(min(port.in_waiting,4096)),now)
                        pending=self.session.pending
                        if pending and any(struct.unpack_from('<4f',pending,8)[:3]):
                            if now-self.session.sent_at >= .05:break
                        else:
                            packet=self.session.poll(self.desired(),now)
                            if packet and port.write(packet) != len(packet):break
                    time.sleep(.002)
                except Exception:break
            port.close()


def handler(controller, token):
    page=Path(__file__).with_name('navcar_move.html').read_bytes()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,code,value,kind='application/json; charset=utf-8'):
            body=value if isinstance(value,bytes) else json.dumps(value,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(code);self.send_header('Content-Type',kind)
            self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff');self.end_headers();self.wfile.write(body)
        def authenticated(self):
            return hmac.compare_digest(self.headers.get('Authorization',''), 'Bearer '+token)
        def do_GET(self):
            path=urlparse(self.path).path
            if path=='/':return self.reply(200,page,'text/html; charset=utf-8')
            if not self.authenticated():return self.reply(401,{'error':'访问令牌错误'})
            if path=='/api/state':return self.reply(200,controller.state())
            self.reply(404,{'error':'不存在'})
        def do_POST(self):
            if not self.authenticated():return self.reply(401,{'error':'访问令牌错误'})
            path=urlparse(self.path).path
            if path not in ('/api/command','/api/yaw','/api/spin','/api/navigation/acquire','/api/navigation/command','/api/navigation/release'):return self.reply(404,{'error':'不存在'})
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0 < length <= 1024:raise ValueError('命令长度错误')
                self.connection.settimeout(2)
                data=json.loads(self.rfile.read(length))
                if not isinstance(data,dict):raise ValueError('需要命令对象')
                if path=='/api/navigation/acquire':return self.reply(200,{'lease':controller.acquire_navigation()})
                if path=='/api/yaw':controller.set_yaw(data)
                elif path=='/api/spin':controller.set_spin(data)
                elif path=='/api/navigation/command':controller.navigation_command(data)
                elif path=='/api/navigation/release':controller.release_navigation(data.get('lease'))
                else:controller.command(data)
                self.reply(200,controller.state())
            except (ValueError,TypeError,TimeoutError) as exc:self.reply(400,{'error':str(exc)})
            except RuntimeError as exc:self.reply(409,{'error':str(exc)})
    return Handler


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=Path(__file__).with_name('serial.navcar.yaml'))
    p.add_argument('--bind',default='0.0.0.0');p.add_argument('--port',type=int,default=8081)
    args=p.parse_args();config=load_config(args.config,control=True)
    import serial
    controller=Controller(config)
    token_path=Path.home()/'ccc/navcar_move.token'
    if token_path.exists():token=token_path.read_text().strip()
    else:
        token=secrets.token_urlsafe(24)
        with token_path.open('x') as f:f.write(token+'\n')
        token_path.chmod(0o600)
    http=ThreadingHTTPServer((args.bind,args.port),handler(controller,token));http.daemon_threads=True
    port=serial.Serial(config['serial_port'],2000000,timeout=0,write_timeout=.02,exclusive=True,
                       xonxoff=False,rtscts=False,dsrdtr=False)
    port.reset_input_buffer()
    Path.home().joinpath('ccc/move_web.pid').write_text(str(os.getpid())+'\n')
    worker=threading.Thread(target=controller.run,args=(port,),daemon=True);worker.start()
    print(f'NAVCAR MOVE: http://192.168.1.194:{args.port}/?token={token}',flush=True)
    def interrupt(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    try:http.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:pass
    finally:
        controller.closed.set();http.server_close();worker.join(timeout=1)


if __name__=='__main__':main()
