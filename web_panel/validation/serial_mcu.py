"""Isolated POSIX pseudo-terminal peer for exercising the real pyserial bridge."""
import json
import math
import os
import threading
import time


class PtyMCU:
    def __init__(self, target_rad=0., sign=-1, zero=.7):
        if (os.name != 'posix' or os.environ.get('SCURM_SYNTHETIC_VALIDATION') != '1'
                or os.environ.get('ROS_LOCALHOST_ONLY') != '1'):
            raise RuntimeError('Virtual MCU requires the isolated Linux validation environment.')
        import pty
        import tty
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.port = os.ttyname(self.slave)
        self.sign, self.zero = sign, zero
        self.raw_angle = target_rad/sign+zero
        self.pose = [0., 0., 0.]
        self.command = (0., 0., 0.)
        self.command_time = 0.
        self.commands = []
        self.feedback_enabled = True
        self.repeat_sequence = False
        self.partial_frames = True
        self.inject_bad_frame = False
        self.seq = 0
        self.error = None
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, packet):
        data = (json.dumps(packet, separators=(',', ':'))+'\n').encode()
        # A pair of writes also exercises packet boundaries independent of serial reads.
        if self.partial_frames:
            split = len(data)//2
            os.write(self.master, data[:split])
            self.closed.wait(.002)
            os.write(self.master, data[split:])
        else:
            os.write(self.master, data)

    def run(self):
        buffer = bytearray()
        last = next_state = time.monotonic()
        try:
            while not self.closed.is_set():
                now = time.monotonic()
                dt, last = min(.1, now-last), now
                try:
                    buffer.extend(os.read(self.master, 8192))
                except BlockingIOError:
                    pass
                while b'\n' in buffer:
                    line, _, remaining = buffer.partition(b'\n')
                    buffer = bytearray(remaining)
                    packet = json.loads(line)
                    if packet.get('type') != 'command' or packet.get('version') != 1:
                        raise ValueError('Unexpected command format')
                    values = tuple(float(packet[k]) for k in ('vx','vy','wz'))
                    if not all(math.isfinite(v) for v in (*values, packet['yaw_target_encoder_rad'])):
                        raise ValueError('Invalid command values')
                    self.command, self.command_time = values, now
                    self.raw_angle = packet['yaw_target_encoder_rad']
                    self.commands.append((now, packet))
                # Virtual motion is used only to exercise the upper-computer interfaces.
                vx, vy, wz = self.command if now-self.command_time < .3 else (0., 0., 0.)
                x, y, heading = self.pose
                self.pose = [x+(vx*math.cos(heading)-vy*math.sin(heading))*dt,
                             y+(vx*math.sin(heading)+vy*math.cos(heading))*dt, heading+wz*dt]
                if now >= next_state:
                    next_state = now+.02
                    if not self.repeat_sequence:
                        self.seq += 1
                    if self.feedback_enabled:
                        if self.inject_bad_frame:
                            os.write(self.master, b'{invalid-json}\n')
                            self.inject_bad_frame = False
                        self.send({'type':'state', 'version':1, 'seq':self.seq,
                            'yaw_encoder_rad':self.raw_angle, 'yaw_rate_rad_s':0.,
                            'odom':dict(zip(('x','y','yaw','vx','vy','wz'), [*self.pose,vx,vy,wz]))})
                self.closed.wait(.002)
        except Exception as exc:
            self.error = repr(exc)

    def close(self):
        self.closed.set()
        self.thread.join(timeout=3)
        os.close(self.master)
        os.close(self.slave)
