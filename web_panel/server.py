#!/usr/bin/env python3
"""ROS 2 point-cloud viewer and serialized mapping/navigation/idle supervisor."""
import argparse
import hmac
import json
import math
import os
from pathlib import Path
import secrets
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
from robot_io import YawGuard, load_runtime, sample_monotonic_time


def decode_cloud(msg, max_points):
    import numpy as np
    fields = {field.name: field for field in msg.fields}
    endian = '>' if msg.is_bigendian else '<'
    formats = [endian + {7: 'f4', 8: 'f8'}[fields[n].datatype] for n in ('x', 'y', 'z')]
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': formats,
        'offsets': [fields[n].offset for n in ('x', 'y', 'z')], 'itemsize': msg.point_step})
    cloud = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.row_step, msg.point_step)).reshape(-1)
    step = max(1, math.ceil(len(cloud) / max_points))
    selected = cloud[::step]
    xyz = np.column_stack([selected[n] for n in ('x', 'y', 'z')])
    return np.round(xyz[np.isfinite(xyz).all(axis=1)], 3).tolist()


def stop_process(process):
    """Stop the launch's whole process group, including children after parent exit."""
    if process is None:
        return
    if os.name == 'nt':
        process.terminate()
        process.wait(timeout=10)
        return
    for sig, timeout in [(signal.SIGINT, 10), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
        try:
            if sig == signal.SIGINT and process.poll() is None:
                # ros2 launch forwards SIGINT to its children. Signalling the
                # group here would interrupt each ROS node twice.
                process.send_signal(sig)
            else:
                os.killpg(process.pid, sig)
        except ProcessLookupError:
            process.wait()
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()  # Reap the launch parent without losing its group identity.
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.05)
    raise RuntimeError('Launch process group did not stop; inspect its log.')


class Panel:
    def __init__(self, args):
        self.args = args
        self.mode = 'idle'
        self.switching = False
        self.error = ''
        self.processes = []
        self.log_files = []
        self.lock = threading.RLock()
        self.cloud = {'points': [], 'frame': '', 'sequence': 0}
        self.cloud_time = 0.0
        self.odom_time = 0.0
        self.pose = None
        self.commands = {}
        self.bridge = None
        self.closed = False
        self.worker = None
        self.yaw_guard = YawGuard(getattr(args, 'robot_config', {}).get('yaw', {'target_rad': 0.0}))
        self.args.maps_dir.mkdir(parents=True, exist_ok=True)
        self.logs = self.args.maps_dir / 'web_logs'
        self.logs.mkdir(exist_ok=True)

    def state(self):
        with self.lock:
            now = time.monotonic()
            return {'mode': self.mode, 'switching': self.switching, 'error': self.error,
                    'demo': self.args.demo, 'pose': self.pose,
                    'mock_hardware': getattr(self.args, 'mock_hardware', False),
                    'yaw': self.yaw_guard.status(),
                    'cloud_age': round(now - self.cloud_time, 2) if self.cloud_time else None,
                    'odom_age': round(now - self.odom_time, 2) if self.odom_time else None,
                    'log_directory': str(self.logs)}

    def request_mode(self, mode):
        if mode not in ('idle', 'mapping', 'navigation'):
            raise ValueError('Unknown mode')
        with self.lock:
            if self.closed or self.switching:
                raise RuntimeError('Mode transition is in progress')
            if mode == self.mode:
                return
            if not self.args.demo and mode != 'idle' and not self.yaw_guard.status()['ready']:
                raise RuntimeError('yaw 尚未锁定：' + self.yaw_guard.status()['reason'])
            previous = self.mode
            self.switching = True  # Velocity gate closes before any process changes.
            self.commands.clear()
            self.error = ''
            self.worker = threading.Thread(target=self.transition,
                                           args=(previous, mode), daemon=True)
            self.worker.start()

    def transition(self, previous, target):
        try:
            if not self.args.demo and previous == 'mapping':
                try:
                    self.bridge.save_maps()
                except Exception as exc:
                    if target != 'idle':
                        raise RuntimeError(f'Map save failed; mapping retained: {exc}') from exc
                    self.error = f'Map save failed; entered idle: {exc}'
            if not self.args.demo and target == 'navigation':
                for name in ('site.pcd', 'site.yaml'):
                    if not (self.args.maps_dir / name).is_file():
                        raise RuntimeError(f'Missing map: {name}. Map the site first.')
            with self.lock:
                self.mode = 'idle'
            self.stop_all()
            if not self.args.demo and target != 'idle':
                if target == 'mapping':
                    commands = [['ros2', 'launch', 'sentry_bringup',
                                 'mapping.launch.py', 'use_rviz:=false']]
                else:
                    commands = [['ros2', 'launch', str(Path(__file__).with_name(
                        'navigation_web.launch.py'))]]
                for command in commands:
                    log = (self.logs / f'{target}-{time.time_ns()}.log').open('ab')
                    self.log_files.append(log)
                    self.processes.append(subprocess.Popen(
                        command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            with self.lock:
                self.mode = target
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
        finally:
            with self.lock:
                self.commands.clear()
                self.switching = False

    def stop_all(self):
        for process in self.processes:
            stop_process(process)
        self.processes.clear()
        for log in self.log_files:
            log.close()
        self.log_files.clear()

    def tick(self):
        with self.lock:
            if not self.switching and any(p.poll() is not None for p in self.processes):
                self.mode = 'idle'
                self.error = 'ROS launch exited. Velocity blocked; inspect web_logs.'
                self.switching = True
                self.worker = threading.Thread(target=self.cleanup_failure, daemon=True)
                self.worker.start()
            now = time.monotonic()
            source = {'mapping': 'manual', 'navigation': 'auto'}.get(self.mode)
            stamp, values = self.commands.get(source, (0, (0.0, 0.0, 0.0)))
            if (self.closed or self.switching or now - stamp > 0.3 or
                    not self.yaw_guard.status()['ready'] or
                    now - self.cloud_time > 1.0 or now - self.odom_time > 1.0):
                return (0.0, 0.0, 0.0)
            vx, vy, wz = values
            if not all(math.isfinite(v) for v in values):
                return (0.0, 0.0, 0.0)
            speed = math.hypot(vx, vy)
            scale = min(1.0, self.args.max_speed / speed) if speed else 1.0
            return (vx * scale, vy * scale,
                    max(-self.args.max_yaw_rate, min(self.args.max_yaw_rate, wz)))

    def cleanup_failure(self):
        try:
            self.stop_all()
        except Exception as exc:
            self.error += f' Cleanup: {exc}'
        finally:
            self.switching = False

    def close(self):
        with self.lock:
            self.closed = True
        if self.worker:
            self.worker.join()  # Do not race with launch creation or map saving.
        self.stop_all()


def create_ros_bridge(panel):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64
    from std_srvs.srv import Trigger

    class Bridge(Node):
        def __init__(self):
            super().__init__('scurm_web_panel')
            self.last_cloud = 0.0
            self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
            self.yaw_target = self.create_publisher(Float64, '/yaw/target', 1)
            self.create_subscription(JointState, '/yaw/joint_states', self.yaw_cb, qos_profile_sensor_data)
            self.create_timer(.1, self.hold_yaw)
            self.create_subscription(PointCloud2, '/cloud_registered', self.cloud_cb,
                                     qos_profile_sensor_data)
            self.create_subscription(Odometry, '/state_estimation', self.odom_cb,
                                     qos_profile_sensor_data)
            for source in ('auto', 'manual'):
                self.create_subscription(Twist, f'/web/cmd_vel_{source}',
                                         lambda msg, s=source: self.command_cb(s, msg), 1)
            self.save_client = self.create_client(Trigger, '/map_save')
            self.create_timer(0.05, self.velocity_tick)

        def hold_yaw(self):
            msg = Float64()
            msg.data = panel.yaw_guard.config['target_rad']
            self.yaw_target.publish(msg)

        def yaw_cb(self, msg):
            try:
                index = msg.name.index(panel.yaw_guard.config.get('joint_name', 'yaw_joint'))
                sampled_at = sample_monotonic_time(msg.header.stamp,
                    self.get_clock().now().nanoseconds, time.monotonic(),
                    panel.yaw_guard.config.get('feedback_timeout_s', .3))
                if sampled_at is None:
                    return
                with panel.lock:
                    panel.yaw_guard.update(msg.position[index], msg.velocity[index], sampled_at)
            except (ValueError, IndexError):
                return

        def command_cb(self, source, msg):
            with panel.lock:
                panel.commands[source] = (time.monotonic(),
                    (msg.linear.x, msg.linear.y, msg.angular.z))

        def velocity_tick(self):
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = panel.tick()
            self.publisher.publish(msg)

        def odom_cb(self, msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            sampled_at = sample_monotonic_time(msg.header.stamp,
                self.get_clock().now().nanoseconds, time.monotonic(), 1.)
            valid_pose = (all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) and
                          abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) < .05)
            with panel.lock:
                if sampled_at is None or not valid_pose:
                    panel.odom_time = 0.
                    return
                panel.odom_time = sampled_at
                panel.pose = {'x': p.x, 'y': p.y, 'z': p.z,
                              'yaw': math.atan2(2 * (q.w*q.z + q.x*q.y),
                                                1 - 2 * (q.y*q.y + q.z*q.z)),
                              'frame': msg.header.frame_id}

        def cloud_cb(self, msg):
            now = time.monotonic()
            sampled_at = sample_monotonic_time(msg.header.stamp,
                self.get_clock().now().nanoseconds, now, 1.)
            if sampled_at is None:
                with panel.lock:
                    panel.cloud_time = 0.
                return
            if now - self.last_cloud < 0.2:
                return
            self.last_cloud = now
            try:
                xyz = decode_cloud(msg, panel.args.max_points)
                with panel.lock:
                    panel.cloud = {'points': xyz, 'frame': msg.header.frame_id,
                                   'sequence': panel.cloud['sequence'] + 1}
                    panel.cloud_time = sampled_at if xyz else 0.
            except (ValueError, KeyError, TypeError) as exc:
                with panel.lock:
                    panel.cloud_time = 0.
                self.get_logger().warning(f'Cannot decode point cloud: {exc}')

        def save_maps(self):
            pcd = panel.args.maps_dir / 'site.pcd'
            previous_write = pcd.stat().st_mtime_ns if pcd.is_file() else None
            if not self.save_client.wait_for_service(timeout_sec=3):
                raise RuntimeError('/map_save is unavailable')
            done = threading.Event()
            future = self.save_client.call_async(Trigger.Request())
            future.add_done_callback(lambda _: done.set())
            if not done.wait(30):
                raise TimeoutError('/map_save timed out')
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError(response.message if response else 'No map response')
            if not pcd.is_file():
                raise RuntimeError('PCD not at maps_dir/site.pcd; check map_file_path')
            if pcd.stat().st_size == 0 or pcd.stat().st_mtime_ns == previous_write:
                raise RuntimeError('PCD was not updated; inspect map path and write permissions')
            log_path = panel.logs / f'map-save-{time.time_ns()}.log'
            with log_path.open('wb') as log:
                process = subprocess.Popen(['ros2', 'run', 'nav2_map_server',
                    'map_saver_cli', '-t', '/projected_map', '-f',
                    str(panel.args.maps_dir / 'site'), '--fmt', 'png'],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    code = process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    stop_process(process)
                    raise TimeoutError(f'2D map save timed out: {log_path}')
            if code or not (panel.args.maps_dir / 'site.yaml').is_file():
                raise RuntimeError(f'2D map save failed: {log_path}')

    return rclpy, Bridge()


def handler_class(panel, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, status, value, content_type='application/json; charset=utf-8'):
            data = value if isinstance(value, bytes) else json.dumps(
                value, ensure_ascii=False, allow_nan=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.end_headers()
            self.wfile.write(data)

        def authorized(self):
            supplied = self.headers.get('Authorization', '')
            if not hmac.compare_digest(supplied.encode('utf-8'), ('Bearer ' + token).encode('utf-8')):
                self.reply(401, {'error': 'Invalid access token'})
                return False
            return True

        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/':
                self.reply(200, Path(__file__).with_name('index.html').read_bytes(),
                           'text/html; charset=utf-8')
            elif path in ('/api/state', '/api/cloud') and self.authorized():
                with panel.lock:
                    value = panel.state() if path == '/api/state' else dict(panel.cloud)
                self.reply(200, value)
            elif path not in ('/api/state', '/api/cloud'):
                self.reply(404, {'error': 'Not found'})

        def do_POST(self):
            if not self.authorized():
                return
            if urlparse(self.path).path != '/api/mode':
                self.reply(404, {'error': 'Not found'})
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 256:
                    raise ValueError('Invalid request size')
                data = json.loads(self.rfile.read(size))
                panel.request_mode(data['mode'])
                self.reply(202, panel.state())
            except (ValueError, KeyError, TypeError) as exc:
                self.reply(400, {'error': str(exc)})
            except RuntimeError as exc:
                self.reply(409, {'error': str(exc)})

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', action='store_true', help='Synthetic cloud, no ROS or hardware')
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--maps-dir', type=Path)
    parser.add_argument('--max-points', type=int, default=6000)
    parser.add_argument('--max-speed', type=float, default=0.3)
    parser.add_argument('--max-yaw-rate', type=float, default=0.5)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('runtime.json'))
    parser.add_argument('--mock-hardware', action='store_true', help='标记连接的是模拟底盘')
    args = parser.parse_args()
    if (args.max_points < 1 or not math.isfinite(args.max_speed) or
            not math.isfinite(args.max_yaw_rate) or args.max_speed <= 0 or args.max_yaw_rate <= 0):
        parser.error('Point count and velocity limits must be positive')
    if not args.demo:
        try:
            args.robot_config = load_runtime(args.config)
        except (ValueError, OSError) as exc:
            parser.error('先完成 robot.yaml 并运行 configure.py：' + str(exc))
    args.maps_dir = (args.maps_dir or Path(getattr(args, 'robot_config', {}).get(
        'maps_dir', Path.home() / 'scurm_maps'))).resolve()
    if not args.demo and str(args.maps_dir) != args.robot_config['maps_dir']:
        parser.error('--maps-dir 必须与 configure.py 生成的地图路径一致')
    panel = Panel(args)
    ros = None
    spin = None
    if not args.demo:
        if os.name == 'nt':
            parser.error('Real robot supervision requires Linux; use --demo on Windows')
        import rclpy
        rclpy.init()
        ros, panel.bridge = create_ros_bridge(panel)
        spin = threading.Thread(target=ros.spin, args=(panel.bridge,), daemon=True)
        spin.start()
    else:
        def demo():
            while not panel.closed:
                now = time.monotonic()
                points = [[round(r*math.cos(i*.04), 3), round(r*math.sin(i*.04), 3),
                           round(.4*math.sin(i*.08+now), 3)]
                          for r in (2, 4, 6) for i in range(158)]
                with panel.lock:
                    panel.cloud = {'points': points, 'frame': 'demo_odom',
                                   'sequence': panel.cloud['sequence'] + 1}
                    panel.cloud_time = panel.odom_time = now
                    panel.yaw_guard.update(0., 0.)
                panel.tick()
                time.sleep(.2)
        threading.Thread(target=demo, daemon=True).start()
    token = os.environ.get('SCURM_WEB_TOKEN') or secrets.token_urlsafe(24)
    server = ThreadingHTTPServer((args.bind, args.port), handler_class(panel, token))
    server.daemon_threads = True
    print(f"{'DEMO' if args.demo else 'ROS2'}: http://{args.bind}:{args.port}/?token={token}",
          flush=True)
    if args.bind == '0.0.0.0':
        print('Replace 0.0.0.0 with the vehicle LAN IP in your browser.', flush=True)
    # Keep ROS spinning until launch shutdown and any pending map save finish.
    def shutdown_signal(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, shutdown_signal)
    signal.signal(signal.SIGTERM, shutdown_signal)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        try:
            panel.close()
        finally:
            if ros:
                if ros.ok():
                    panel.bridge.publisher.publish(__import__(
                        'geometry_msgs.msg', fromlist=['Twist']).Twist())
                    ros.shutdown()
                spin.join(timeout=3)
                panel.bridge.destroy_node()


if __name__ == '__main__':
    main()
