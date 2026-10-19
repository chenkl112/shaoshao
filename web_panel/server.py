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


def navigation_failure(code, detail=''):
    reasons = {201:'规划器不可用', 202:'定位坐标变换失败', 203:'车辆位置在地图范围外',
               204:'目标位置在地图范围外', 205:'车辆当前位置落在障碍物内',
               206:'目标落在障碍物内，请重新选点', 207:'路径规划超时',
               208:'无可通行路径：实时障碍物或车体安全范围阻断通路',
               101:'控制器不可用', 102:'导航坐标变换失败',
               103:'控制路径无效', 104:'局部避障持续失败', 105:'车辆未能持续前进',
               106:'局部避障找不到可行轨迹', 107:'控制器计算超时'}
    text = reasons.get(code, '导航未完成，请检查定位和障碍物')
    return text+(f'（错误码 {code}）' if code else '')+('：'+str(detail)[:250] if detail else '')


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
        self.nav_server_ready = False
        self.nav_localized = False
        self.map_pose = None
        self.map_pose_time = 0.
        self.nav_serial = 0
        self.nav_handle = None
        self.nav_status = 'idle'
        self.nav_message = '请在地图上选择目标位置'
        self.nav_goal = None
        self.nav_remaining = None
        self.nav_hold = False
        self.nav_started = 0.
        self.plan_serial = 0
        self.nav_plan = {'status':'idle', 'message':'选点后自动规划路径', 'goal':None, 'path':[]}
        self.plan_started = 0.
        self.nav_path = []
        self.navigation_costmap = {'available':False}
        self.spin_enabled = False
        self.spin_rate = min(.5, args.max_yaw_rate)
        self.spin_heartbeat = 0.
        self.spin_timeout = .75
        self.yaw_guard = YawGuard(getattr(args, 'robot_config', {}).get('yaw', {'target_rad': 0.0}))
        self.args.maps_dir.mkdir(parents=True, exist_ok=True)
        self.logs = self.args.maps_dir / 'web_logs'
        self.logs.mkdir(exist_ok=True)
        self.mapping_filter = {'enabled': True, 'height_m': 1.5, 'dynamic_enabled': True}
        settings = self.args.maps_dir / 'mapping_filter.json'
        if settings.is_file():
            saved = json.loads(settings.read_text(encoding='utf-8'))
            self.mapping_filter = self.validate_mapping_filter(saved['enabled'], saved['height_m'], saved.get('dynamic_enabled', True))

    def state(self):
        with self.lock:
            now = time.monotonic()
            self.spin_live(now)
            return {'mode': self.mode, 'switching': self.switching, 'error': self.error,
                    'demo': self.args.demo, 'pose': self.pose,
                    'mock_hardware': getattr(self.args, 'mock_hardware', False),
                    'yaw': self.yaw_guard.status(),
                    'cloud_age': round(now - self.cloud_time, 2) if self.cloud_time else None,
                    'odom_age': round(now - self.odom_time, 2) if self.odom_time else None,
                    'spin': {'enabled': self.spin_enabled, 'rate_rad_s': self.spin_rate,
                             'max_rate_rad_s': self.args.max_yaw_rate,
                             'timeout_s': self.spin_timeout},
                    'navigation': self.navigation_state(),
                    'mapping_filter': {**self.mapping_filter,
                        'locked': self.closed or self.switching or self.mode != 'idle',
                        'reference': '开始建图时的雷达高度'},
                    'log_directory': str(self.logs)}

    def disable_spin(self):
        self.spin_enabled = False
        self.spin_heartbeat = 0.

    def spin_live(self, now):
        if self.spin_enabled and now - self.spin_heartbeat > self.spin_timeout:
            self.disable_spin()
        return self.spin_enabled

    def set_spin(self, enabled, rate=None):
        if type(enabled) is not bool:
            raise ValueError('enabled 必须为 true 或 false')
        with self.lock:
            if not enabled:
                self.disable_spin()
                return
            rate = self.spin_rate if rate is None else rate
            if (isinstance(rate, bool) or not isinstance(rate, (int, float)) or
                    not math.isfinite(rate) or not 0 < abs(rate) <= self.args.max_yaw_rate):
                raise ValueError('小陀螺角速度必须非零且不超过角速度上限')
            if self.closed or self.switching or self.mode != 'mapping':
                raise RuntimeError('小陀螺仅在手动/建图模式启用')
            now = time.monotonic()
            if not self.args.demo and (not self.yaw_guard.status()['ready'] or
                    not self.cloud_time or now-self.cloud_time > 1. or
                    not self.odom_time or now-self.odom_time > 1.):
                raise RuntimeError('yaw、点云或定位未就绪，无法启用小陀螺')
            self.spin_rate = float(rate)
            self.spin_enabled = True
            self.spin_heartbeat = now

    def heartbeat_spin(self):
        with self.lock:
            now = time.monotonic()
            if self.spin_live(now):
                self.spin_heartbeat = now

    def saved_map(self):
        """Read only the configured saved map, never the bundled test map."""
        import yaml
        root = self.args.maps_dir.resolve()
        metadata = root / 'site.yaml'
        missing = {'available': False, 'message': '暂无建图地图，完成建图并保存后会显示在这里。'}
        if not metadata.is_file():
            return missing, None
        try:
            config = yaml.safe_load(metadata.read_text(encoding='utf-8'))
            if not isinstance(config, dict) or not isinstance(config.get('image'), str):
                raise ValueError('地图配置缺少 image')
            image = (root / config['image']).resolve()
            if not image.is_relative_to(root) or image.suffix.lower() != '.png':
                raise ValueError('地图图像必须是地图目录内的 PNG 文件')
            if not image.is_file():
                raise ValueError('地图图像尚未保存')
            resolution = config.get('resolution')
            origin = config.get('origin')
            if (isinstance(resolution, bool) or not isinstance(resolution, (int, float)) or
                    not math.isfinite(resolution) or resolution <= 0 or
                    not isinstance(origin, list) or len(origin) != 3 or
                    not all(type(v) in (int, float) and math.isfinite(v) for v in origin)):
                raise ValueError('地图分辨率或原点无效')
            stat = image.stat()
            with image.open('rb') as stream:
                header = stream.read(24)
            if header[:8] != b'\x89PNG\r\n\x1a\n' or header[12:16] != b'IHDR' or len(header) != 24:
                raise ValueError('地图 PNG 文件无效')
            width, height = int.from_bytes(header[16:20], 'big'), int.from_bytes(header[20:24], 'big')
            if not width or not height:
                raise ValueError('地图尺寸无效')
            revision = f'{metadata.stat().st_mtime_ns}-{stat.st_mtime_ns}-{stat.st_size}'
            return {'available': True, 'frame': 'map', 'resolution': resolution,
                    'width': width, 'height': height,
                    'origin': origin, 'revision': revision,
                    'image_url': '/api/map/image?revision='+revision}, image
        except (OSError, ValueError, UnicodeError, OverflowError, yaml.YAMLError) as exc:
            return {'available': False, 'message': '地图暂不可用：'+str(exc)}, None

    @staticmethod
    def validate_mapping_filter(enabled, height, dynamic_enabled=True):
        if type(enabled) is not bool:
            raise ValueError('enabled 必须为 true 或 false')
        if (isinstance(height, bool) or not isinstance(height, (int, float)) or
                not math.isfinite(height) or not 0 < height <= 50):
            raise ValueError('建图去顶高度必须大于 0 且不超过 50 米')
        if type(dynamic_enabled) is not bool:
            raise ValueError('dynamic_enabled 必须为 true 或 false')
        return {'enabled': enabled, 'height_m': float(height), 'dynamic_enabled': dynamic_enabled}

    def set_mapping_filter(self, enabled, height, dynamic_enabled=None):
        with self.lock:
            if dynamic_enabled is None:
                dynamic_enabled = self.mapping_filter['dynamic_enabled']
            settings = self.validate_mapping_filter(enabled, height, dynamic_enabled)
            if self.closed or self.switching or self.mode != 'idle':
                raise RuntimeError('请先进入待机，再修改建图过滤设置；下次建图生效')
            path = self.args.maps_dir / 'mapping_filter.json'
            temporary = path.with_suffix('.tmp')
            try:
                temporary.write_text(json.dumps(settings, ensure_ascii=False)+'\n', encoding='utf-8')
                temporary.replace(path)
            except OSError as exc:
                raise RuntimeError('建图过滤设置保存失败：'+str(exc)) from exc
            self.mapping_filter = settings

    def navigation_reason(self):
        if self.args.demo or self.bridge is None:
            return '当前为预览，尚未启动实车导航'
        if self.closed or self.switching:
            return '正在切换运行模式'
        if self.mode != 'navigation':
            return '请先点击顶部“导航”'
        if not self.nav_server_ready:
            return '等待 Nav2 导航服务就绪'
        now = time.monotonic()
        if not self.nav_localized or not self.map_pose or now-self.map_pose_time > 1.:
            return '等待地图定位成功，请将车辆放回建图起点附近'
        if not self.yaw_guard.status()['ready']:
            return 'yaw 尚未锁定'
        if not self.cloud_time or now-self.cloud_time > 1. or not self.odom_time or now-self.odom_time > 1.:
            return '点云或定位数据未就绪'
        return ''

    def navigation_state(self):
        reason = self.navigation_reason()
        if self.nav_plan['status']=='planning' and time.monotonic()-self.plan_started>10.:
            self.plan_serial += 1
            self.nav_plan.update(status='failed',message='路径规划超时，请重新选点',path=[])
        return {'ready': not reason, 'reason': reason, 'status': self.nav_status,
                'localized':bool(self.nav_localized and self.map_pose and self.map_pose_time and time.monotonic()-self.map_pose_time<=1.),
                'localization':getattr(self.bridge,'localization_info',None),
                'message': self.nav_message, 'goal': self.nav_goal,
                'plan': self.nav_plan, 'path': self.nav_path,
                'distance_remaining': self.nav_remaining,
                'pose': self.map_pose if self.map_pose_time and time.monotonic()-self.map_pose_time <= 1. else None}

    def navigation_target(self, x, y, revision):
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (x, y)):
            raise ValueError('目标坐标必须是有限数值')
        info, _ = self.saved_map()
        if not info['available']:
            raise RuntimeError(info['message'])
        if not isinstance(revision, str) or revision != info['revision']:
            raise RuntimeError('地图已更新，请刷新地图后重新选点')
        ox, oy, angle = info['origin']
        dx, dy = x-ox, y-oy
        mx, my = math.cos(angle)*dx+math.sin(angle)*dy, -math.sin(angle)*dx+math.cos(angle)*dy
        if not (0 <= mx < info['width']*info['resolution'] and 0 <= my < info['height']*info['resolution']):
            raise ValueError('请选择地图范围内的目标')
        return {'x':float(x),'y':float(y),'map_revision':revision,'frame':'map'}

    def plan_navigation_goal(self, x, y, revision):
        target=self.navigation_target(x,y,revision)
        with self.lock:
            reason=self.navigation_reason()
            if reason:raise RuntimeError(reason)
            if self.nav_status in ('sending','active','canceling'):
                raise RuntimeError('已有导航任务，请先取消或等待完成')
            if not hasattr(self.bridge,'plan_goal'):raise RuntimeError('当前导航服务不支持路径预览')
            self.plan_serial+=1;serial=self.plan_serial
            self.plan_started=time.monotonic()
            self.nav_plan={'status':'planning','message':'正在计算路径…','goal':target,'path':[]}
            try:
                future=self.bridge.plan_goal(target)
                future.add_done_callback(lambda f:self.navigation_plan_response(serial,f))
            except Exception as exc:
                self.nav_plan.update(status='failed',message='路径规划请求失败：'+str(exc))
                raise RuntimeError(self.nav_plan['message']) from exc

    def navigation_plan_response(self, serial, future):
        with self.lock:
            try:
                handle=future.result()
                if serial!=self.plan_serial:
                    if handle.accepted:handle.cancel_goal_async()
                    return
                if not handle.accepted:
                    self.nav_plan.update(status='failed',message='规划请求被拒绝，请等待规划器就绪')
                    return
                handle.get_result_async().add_done_callback(lambda f:self.navigation_plan_result(serial,f))
            except Exception as exc:
                if serial==self.plan_serial:self.nav_plan.update(status='failed',message='规划通信失败：'+str(exc))

    def navigation_plan_result(self, serial, future):
        with self.lock:
            if serial!=self.plan_serial:return
            try:
                response=future.result();result=response.result
                if response.status!=4 or result.error_code:
                    self.nav_plan.update(status='failed',message=navigation_failure(result.error_code,result.error_msg),path=[])
                    return
                if result.path.header.frame_id!='map':raise ValueError('规划路径坐标系不是 map')
                points=[[float(p.pose.position.x),float(p.pose.position.y)] for p in result.path.poses]
                if not points or not all(math.isfinite(v) for p in points for v in p):raise ValueError('规划路径为空或无效')
                length=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(points,points[1:]))
                stride=max(1,math.ceil(len(points)/2000))
                path=points[::stride]
                if path[-1]!=points[-1]:path.append(points[-1])
                self.nav_plan.update(status='ready',message=f'路径已计算，长 {length:.2f} 米',path=path,length_m=length)
            except Exception as exc:
                self.nav_plan.update(status='failed',message='规划结果无效：'+str(exc),path=[])

    def clear_navigation_plan(self):
        self.plan_serial+=1
        self.nav_plan={'status':'idle','message':'选点后自动规划路径','goal':None,'path':[]}
        self.nav_path=[]

    def send_navigation_goal(self, x, y, revision):
        self.navigation_target(x,y,revision)
        with self.lock:
            reason = self.navigation_reason()
            if reason:
                raise RuntimeError(reason)
            if self.nav_status in ('sending', 'active', 'canceling'):
                raise RuntimeError('已有导航任务，请先取消或等待完成')
            self.nav_serial += 1
            serial = self.nav_serial
            self.nav_goal = {'x': float(x), 'y': float(y), 'yaw': self.map_pose['yaw'], 'frame': 'map'}
            self.plan_serial+=1
            self.nav_path=[]
            planned=self.nav_plan.get('goal')
            if self.nav_plan['status']=='ready' and planned and planned['map_revision']==revision and math.hypot(planned['x']-x,planned['y']-y)<.001:
                self.nav_path=list(self.nav_plan['path'])
            self.nav_status, self.nav_message = 'sending', '正在发送导航目标…'
            self.nav_remaining, self.nav_hold = None, True
            self.nav_started = time.monotonic()
            self.commands.pop('auto', None)
            try:
                future = self.bridge.send_goal(self.nav_goal, lambda msg: self.navigation_feedback(serial, msg))
                future.add_done_callback(lambda f: self.navigation_goal_response(serial, f))
            except Exception as exc:
                self.nav_status, self.nav_message = 'failed', '发送目标失败：'+str(exc)
                raise RuntimeError(self.nav_message) from exc

    def navigation_goal_response(self, serial, future):
        with self.lock:
            try:
                handle = future.result()
                if not handle.accepted:
                    if serial == self.nav_serial:
                        self.nav_status, self.nav_message = 'rejected', '导航目标被拒绝，请检查 Nav2 状态'
                        self.nav_hold = True
                    return
                if serial != self.nav_serial:
                    handle.cancel_goal_async()
                    return
                self.nav_handle = handle
                result = handle.get_result_async()
                result.add_done_callback(lambda f: self.navigation_result(serial, f))
                if self.nav_status == 'canceling' or self.closed or self.switching or self.mode != 'navigation':
                    self._cancel_navigation_handle(serial)
                elif self.nav_status == 'sending':
                    self.commands.pop('auto', None)
                    self.nav_status, self.nav_message, self.nav_hold = 'active', '正在前往目标', False
            except Exception as exc:
                if serial == self.nav_serial:
                    if self.nav_handle is not None:
                        self.cancel_navigation()
                        self.nav_message = '导航通信失败，已停止输出并请求取消：'+str(exc)
                    else:
                        self.nav_status, self.nav_message = 'failed', '导航通信失败：'+str(exc)
                    self.nav_hold = True

    def navigation_feedback(self, serial, message):
        with self.lock:
            if serial == self.nav_serial and self.nav_status == 'active':
                distance = message.feedback.distance_remaining
                if math.isfinite(distance) and distance >= 0:
                    self.nav_remaining = float(distance)

    def navigation_result(self, serial, future):
        with self.lock:
            if serial != self.nav_serial:
                return
            try:
                response = future.result();status = response.status
                self.nav_status, self.nav_message = {
                    4: ('succeeded', '已到达目标'), 5: ('canceled', '导航已取消'),
                }.get(status, ('failed', '导航失败，无法到达目标，请检查路径和定位'))
                if self.nav_status=='failed':
                    result=getattr(response,'result',None)
                    self.nav_message=navigation_failure(getattr(result,'error_code',0),getattr(result,'error_msg',''))
            except Exception as exc:
                self.nav_status, self.nav_message = 'failed', '导航结果读取失败：'+str(exc)
            self.nav_hold = True
            self.nav_handle, self.nav_remaining = None, None
            self.commands.pop('auto', None)

    def _cancel_navigation_handle(self, serial):
        try:
            future = self.nav_handle.cancel_goal_async()
            future.add_done_callback(lambda f: self.navigation_cancel_response(serial, f))
        except Exception as exc:
            self.nav_message = '取消未确认，已停止输出，请切换待机：'+str(exc)

    def navigation_cancel_response(self, serial, future):
        with self.lock:
            if serial != self.nav_serial or self.nav_status != 'canceling':
                return
            try:
                if not future.result().goals_canceling:
                    self.nav_message = '取消未确认，已停止输出，请切换待机'
            except Exception:
                self.nav_message = '取消通信失败，已停止输出，请切换待机'

    def cancel_navigation(self):
        with self.lock:
            if self.nav_status not in ('sending', 'active', 'canceling'):
                return
            if self.nav_status == 'canceling':
                return
            self.nav_status, self.nav_message, self.nav_hold = 'canceling', '正在取消导航…', True
            self.commands.pop('auto', None)
            if self.nav_handle is not None:
                self._cancel_navigation_handle(self.nav_serial)

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
            self.cancel_navigation()
            self.disable_spin()
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
                self.disable_spin()
            self.stop_all()
            if not self.args.demo and target != 'idle':
                if target == 'mapping':
                    commands = [['ros2', 'launch', 'sentry_bringup',
                                 'mapping.launch.py', 'use_rviz:=false',
                                 'map_ceiling_enabled:='+str(self.mapping_filter['enabled']).lower(),
                                 'map_dynamic_enabled:='+str(self.mapping_filter['dynamic_enabled']).lower(),
                                 'map_ceiling_height:='+str(self.mapping_filter['height_m'])]]
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
        self.cancel_navigation()
        for process in self.processes:
            stop_process(process)
        self.processes.clear()
        for log in self.log_files:
            log.close()
        self.log_files.clear()
        with self.lock:
            self.nav_serial += 1  # Late action callbacks cannot modify a new session.
            self.clear_navigation_plan()
            self.navigation_costmap={'available':False}
            self.nav_status, self.nav_message = 'idle', '请在地图上选择目标位置'
            self.nav_handle, self.nav_goal, self.nav_remaining = None, None, None
            self.nav_localized, self.nav_server_ready, self.nav_hold = False, False, False
            self.map_pose, self.map_pose_time = None, 0.

    def tick(self):
        with self.lock:
            if not self.switching and any(p.poll() is not None for p in self.processes):
                self.mode = 'idle'
                self.error = 'ROS launch exited. Velocity blocked; inspect web_logs.'
                self.switching = True
                self.worker = threading.Thread(target=self.cleanup_failure, daemon=True)
                self.worker.start()
            now = time.monotonic()
            if self.nav_status == 'sending' and now-self.nav_started > 10.:
                self.cancel_navigation()
                self.nav_message = '导航响应超时，已停止输出，请切换待机'
            source = {'mapping': 'manual', 'navigation': 'auto'}.get(self.mode)
            stamp, values = self.commands.get(source, (0, (0.0, 0.0, 0.0)))
            if (self.closed or self.switching or source is None or
                    not self.yaw_guard.status()['ready'] or
                    now - self.cloud_time > 1.0 or now - self.odom_time > 1.0):
                self.disable_spin()
                return (0.0, 0.0, 0.0)
            if self.mode == 'navigation' and (self.nav_hold or
                    (self.nav_goal is not None and self.navigation_reason())):
                return (0.0, 0.0, 0.0)
            spin = self.spin_live(now) and self.mode == 'mapping'
            if now - stamp > .3:
                values = (0., 0., 0.)
            vx, vy, _ = values
            if not all(math.isfinite(v) for v in values):
                self.disable_spin()
                return (0.0, 0.0, 0.0)
            speed = math.hypot(vx, vy)
            scale = min(1.0, self.args.max_speed / speed) if speed else 1.0
            return (vx * scale, vy * scale, self.spin_rate if spin else 0.)

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
            self.disable_spin()
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
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from tf2_ros import Buffer, TransformListener, TransformException

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
            self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(PoseWithCovarianceStamped, '/icp_result', self.localization_cb, 10)
            self.create_timer(.2, self.navigation_tick)
            self.create_timer(0.05, self.velocity_tick)

        def localization_cb(self, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            sampled = sample_monotonic_time(msg.header.stamp, self.get_clock().now().nanoseconds,
                                            time.monotonic(), 2.)
            if (sampled is None or msg.header.frame_id != 'map' or
                    not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
                    abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) >= .05):
                return
            with panel.lock:
                if panel.mode == 'navigation' and not panel.switching:
                    panel.nav_localized = True

        def navigation_tick(self):
            # Query the composed map->base pose; odometry coordinates alone are not map coordinates.
            with panel.lock:
                panel.nav_server_ready = self.nav_client.server_is_ready()
                if panel.mode != 'navigation' or panel.switching or not panel.nav_localized:
                    return
                try:
                    transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                    sampled = sample_monotonic_time(transform.header.stamp,
                        self.get_clock().now().nanoseconds, time.monotonic(), 1.)
                    p, q = transform.transform.translation, transform.transform.rotation
                    if (sampled is None or not all(math.isfinite(v) for v in
                            (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
                            abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) >= .05):
                        panel.map_pose_time = 0.
                        return
                    panel.map_pose = {'x': p.x, 'y': p.y, 'frame': 'map',
                        'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}
                    panel.map_pose_time = sampled
                except TransformException:
                    panel.map_pose_time = 0.

        def send_goal(self, target, feedback):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = target['x']
            goal.pose.pose.position.y = target['y']
            goal.pose.pose.orientation.z = math.sin(target['yaw']/2.)
            goal.pose.pose.orientation.w = math.cos(target['yaw']/2.)
            return self.nav_client.send_goal_async(goal, feedback_callback=feedback)

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
            elif path in ('/api/state', '/api/cloud', '/api/map', '/api/map/image', '/api/navigation/costmap'):
                if not self.authorized():
                    return
                if path=='/api/navigation/costmap':
                    with panel.lock:self.reply(200,panel.navigation_costmap)
                elif path in ('/api/map', '/api/map/image'):
                    info, image = panel.saved_map()
                    if path == '/api/map':
                        self.reply(200, info)
                    elif image is None:
                        self.reply(404, {'error': info['message']})
                    else:
                        try:
                            data = image.read_bytes()
                        except OSError:
                            self.reply(404, {'error': '地图正在更新，请刷新重试'})
                            return
                        self.reply(200, data, 'image/png')
                else:
                    with panel.lock:
                        value = panel.state() if path == '/api/state' else dict(panel.cloud)
                    self.reply(200, value)
            else:
                self.reply(404, {'error': 'Not found'})

        def do_POST(self):
            if not self.authorized():
                return
            path = urlparse(self.path).path
            if path not in ('/api/mode', '/api/spin', '/api/spin/heartbeat',
                            '/api/navigation/goal', '/api/navigation/plan', '/api/navigation/cancel', '/api/mapping/filter'):
                self.reply(404, {'error': 'Not found'})
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 256:
                    raise ValueError('Invalid request size')
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError('请求必须为 JSON 对象')
                if path == '/api/mode':
                    panel.request_mode(data['mode'])
                elif path == '/api/spin':
                    panel.set_spin(data['enabled'], data.get('rate_rad_s'))
                elif path == '/api/navigation/goal':
                    panel.send_navigation_goal(data['x'], data['y'], data['map_revision'])
                elif path == '/api/navigation/plan':
                    panel.plan_navigation_goal(data['x'], data['y'], data['map_revision'])
                elif path == '/api/navigation/cancel':
                    panel.cancel_navigation()
                elif path == '/api/mapping/filter':
                    panel.set_mapping_filter(data['enabled'], data['height_m'], data.get('dynamic_enabled'))
                else:
                    panel.heartbeat_spin()
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
