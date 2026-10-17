"""Shared, hardware-independent yaw interlock, calibration and reference wire protocol."""
import json
import math
import time


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def finite_number(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def sample_monotonic_time(stamp, ros_now_ns, monotonic_now, max_age_s):
    """Convert a valid ROS sample stamp to the local watchdog time base."""
    stamp_ns = stamp.sec * 10**9 + stamp.nanosec
    age = (ros_now_ns - stamp_ns) * 1e-9
    if stamp_ns <= 0 or not 0 <= stamp.nanosec < 10**9 or not -.1 <= age <= max_age_s:
        return None
    return monotonic_now - max(0., age)


def optical_center_in_base(yaw, lidar_to_imu=(-.011, -.02329, .04412)):
    """Compose the yaw pivot, IMU mount and internal MID360 translation."""
    roll, pitch, heading = yaw['imu_rpy_rad']
    tx, ty, tz = lidar_to_imu
    ry = math.cos(roll)*ty - math.sin(roll)*tz
    rz = math.sin(roll)*ty + math.cos(roll)*tz
    rx = math.cos(pitch)*tx + math.sin(pitch)*rz
    rz = -math.sin(pitch)*tx + math.cos(pitch)*rz
    ix, iy, iz = yaw['imu_xyz_m']
    local_x = ix + math.cos(heading)*rx - math.sin(heading)*ry
    local_y = iy + math.sin(heading)*rx + math.cos(heading)*ry
    angle = yaw['target_rad']
    ax, ay, az = yaw['axis_xyz_m']
    return (ax + math.cos(angle)*local_x - math.sin(angle)*local_y,
            ay + math.sin(angle)*local_x + math.cos(angle)*local_y, az + iz + rz)


def validate_io_config(config, serial=True):
    """Validate only the serial/ROS interface, without navigation geometry."""
    if not isinstance(config, dict):
        return ['配置必须为字典']
    missing = []
    if config.get('version') != 1:
        missing.append('version（需要 1）')
    yaw, hardware = config.get('yaw'), config.get('hardware')
    if not isinstance(yaw, dict) or not isinstance(hardware, dict):
        return missing + ['yaw / hardware（需要配置字典）']
    if not finite_number(yaw.get('target_rad')):
        missing.append('yaw.target_rad')
    if not isinstance(yaw.get('joint_name', 'yaw_joint'), str) or not yaw.get('joint_name', 'yaw_joint'):
        missing.append('yaw.joint_name')
    for key, default in [('angle_tolerance_rad', .02), ('rate_tolerance_rad_s', .03),
                         ('feedback_timeout_s', .3), ('settle_time_s', .5)]:
        if not finite_number(yaw.get(key, default)) or yaw.get(key, default) <= 0:
            missing.append('yaw.' + key)
    if hardware.get('transport') not in ('ros', 'serial_jsonl_v1'):
        missing.append('hardware.transport')
    if not finite_number(hardware.get('command_timeout_s', .3)) or hardware.get('command_timeout_s', .3) <= 0:
        missing.append('hardware.command_timeout_s')
    if serial:
        if hardware.get('transport') != 'serial_jsonl_v1':
            missing.append('hardware.transport（串口桥需要 serial_jsonl_v1）')
        if not isinstance(hardware.get('serial_port'), str) or not hardware.get('serial_port'):
            missing.append('hardware.serial_port')
        if (not isinstance(hardware.get('baud'), int) or isinstance(hardware.get('baud'), bool)
                or hardware.get('baud', 0) <= 0):
            missing.append('hardware.baud')
        if isinstance(hardware.get('yaw_encoder_sign'), bool) or hardware.get('yaw_encoder_sign') not in (-1, 1):
            missing.append('hardware.yaw_encoder_sign')
        if not finite_number(hardware.get('yaw_encoder_zero_rad')):
            missing.append('hardware.yaw_encoder_zero_rad')
    return missing


def load_io_config(path, serial=True):
    if path.suffix.lower() in ('.yaml', '.yml'):
        import yaml
        config = yaml.safe_load(path.read_text(encoding='utf-8'))
    else:
        config = json.loads(path.read_text(encoding='utf-8'))
    missing = validate_io_config(config, serial=serial)
    if missing:
        raise ValueError('接口参数仍待测：' + '、'.join(missing))
    config['yaw']['target_rad'] = float(config['yaw']['target_rad'])
    return config


def validate_config(config, serial=False):
    missing = []
    if not isinstance(config, dict):
        return ['配置必须为字典']
    if config.get('version') != 1:
        missing.append('version（需要 1）')
    for section in ('robot', 'yaw', 'network', 'hardware'):
        if not isinstance(config.get(section), dict):
            return missing + [section + '（需要配置字典）']
    for section, keys in {
        'robot': ['length_m', 'width_m', 'lidar_height_m'],
        'yaw': ['target_rad'],
    }.items():
        for key in keys:
            value = config.get(section, {}).get(key)
            if not finite_number(value) or (section == 'robot' and value <= 0):
                missing.append(f'{section}.{key}')
    yaw = config.get('yaw', {})
    if yaw.get('reference') != 'base_link':
        missing.append('yaw.reference（本版需填写 base_link；世界锁定另行适配）')
    for key in ('axis_xyz_m', 'imu_xyz_m', 'imu_rpy_rad'):
        value = yaw.get(key)
        if not isinstance(value, list) or len(value) != 3 or not all(finite_number(v) for v in value):
            missing.append('yaw.' + key)
    import ipaddress
    for key in ('host_ip', 'lidar_ip'):
        try:
            ipaddress.IPv4Address(config.get('network', {}).get(key))
        except (ValueError, TypeError):
            missing.append('network.' + key)
    hardware = config.get('hardware', {})
    return list(dict.fromkeys(missing + validate_io_config(config,
        serial=serial or hardware.get('transport') == 'serial_jsonl_v1')))


def load_runtime(path):
    config = json.loads(path.read_text(encoding='utf-8'))
    missing = validate_config(config)
    if missing:
        raise ValueError('实车参数仍待测：' + '、'.join(missing))
    # ROS message float setters require floats even when YAML/JSON contains integer 0.
    for key in ('length_m', 'width_m', 'lidar_height_m'):
        config['robot'][key] = float(config['robot'][key])
    config['yaw']['target_rad'] = float(config['yaw']['target_rad'])
    for key in ('axis_xyz_m', 'imu_xyz_m', 'imu_rpy_rad'):
        config['yaw'][key] = [float(value) for value in config['yaw'][key]]
    return config


class YawGuard:
    def __init__(self, config, clock=time.monotonic):
        self.config = config
        self.clock = clock
        self.angle = self.rate = None
        self.updated = None
        self.stable_since = None

    def update(self, angle, rate, sampled_at=None):
        now = self.clock()
        # Feedback gaps must restart the dwell interval even if no gate tick ran.
        if self.updated is None or now - self.updated > self.config.get('feedback_timeout_s', .3):
            self.stable_since = None
        self.angle, self.rate, self.updated = angle, rate, now if sampled_at is None else sampled_at
        in_position = (finite_number(angle) and finite_number(rate) and
            abs(wrap_angle(angle - self.config['target_rad'])) <= self.config.get('angle_tolerance_rad', .02) and
            abs(rate) <= self.config.get('rate_tolerance_rad_s', .03))
        if not in_position:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = now

    def status(self):
        now = self.clock()
        fresh = self.updated is not None and now - self.updated <= self.config.get('feedback_timeout_s', .3)
        if not fresh:
            self.stable_since = None
        ready = fresh and self.stable_since is not None and now - self.stable_since >= self.config.get('settle_time_s', .5)
        return {'ready': ready, 'angle_rad': self.angle if finite_number(self.angle) else None,
                'rate_rad_s': self.rate if finite_number(self.rate) else None,
                'target_rad': self.config['target_rad'],
                'age_s': round(now-self.updated, 3) if self.updated is not None else None,
                'reason': '' if ready else 'yaw 反馈超时' if not fresh else 'yaw 未到位或尚未稳定'}


def decode_feedback(line):
    if len(line) > 4096:
        raise ValueError('状态帧过长')
    packet = json.loads(line)
    if not isinstance(packet, dict) or packet.get('type') != 'state':
        raise ValueError('需要 state 状态帧')
    if 'version' in packet and (type(packet['version']) is not int or packet['version'] != 1):
        raise ValueError('状态协议 version 需要为 1')
    for key in ('yaw_encoder_rad', 'yaw_rate_rad_s'):
        if not finite_number(packet.get(key)):
            raise ValueError(f'{key} 缺失或非法')
        packet[key] = float(packet[key])
    # Wheel odometry is optional, but a provided sample must be complete and finite.
    if 'odom' in packet:
        if not isinstance(packet['odom'], dict):
            raise ValueError('底盘里程计必须为字典')
        for key in ('x', 'y', 'yaw', 'vx', 'vy', 'wz'):
            if not finite_number(packet['odom'].get(key)):
                raise ValueError('底盘里程计字段非法：' + key)
            packet['odom'][key] = float(packet['odom'][key])
    if not isinstance(packet.get('seq'), int) or isinstance(packet.get('seq'), bool) or packet['seq'] < 0:
        raise ValueError('状态 seq 必须为非负整数')
    return packet


def encode_command(seq, velocity, yaw_target, hardware):
    if type(seq) is not int or seq < 0:
        raise ValueError('命令 seq 必须为非负整数')
    vx, vy, wz = velocity
    if not all(finite_number(v) for v in (vx, vy, wz, yaw_target)):
        raise ValueError('命令中存在非有限数值')
    raw_target = yaw_target / hardware['yaw_encoder_sign'] + hardware['yaw_encoder_zero_rad']
    return (json.dumps({'type': 'command', 'version': 1, 'seq': seq,
                        'vx': vx, 'vy': vy, 'wz': wz, 'yaw_target_encoder_rad': raw_target},
                       allow_nan=False, separators=(',', ':')) + '\n').encode('utf-8')
