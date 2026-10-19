#!/usr/bin/env python3
"""UART1 Navcar V1 bridge. Default: read-only monitor, no motor commands."""
import argparse
import json
import math
from pathlib import Path
import signal
import time

from navcar_protocol import Session, stop_confirmed


def load_config(path, control=False):
    import yaml
    config = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(config, dict) or config.get('transport') != 'navcar_uart1_v1':
        raise ValueError('transport must be navcar_uart1_v1')
    if not isinstance(config.get('serial_port'), str) or not config['serial_port']:
        raise ValueError('Fill serial_port with the actual /dev/serial/by-id/...')
    if config.get('baud') != 2000000:
        raise ValueError('Navcar requires baud=2000000')
    if control:
        for key in ('forward_units_per_mps', 'left_units_per_mps',
                    'rot_units_per_radps', 'max_translation_units', 'max_rot_units'):
            v = config.get(key)
            if type(v) not in (int, float) or not math.isfinite(v) or v <= 0:
                raise ValueError('Measure and fill positive ' + key)
        angle = config.get('yaw_target_deg')
        if type(angle) not in (int, float) or not math.isfinite(angle):
            raise ValueError('Fill finite yaw_target_deg (startup heading reference)')
    return config


def targets(velocity, config):
    """ROS-style yaw-frame forward/left/CCW -> native forward/right/CW."""
    forward, left, ccw = velocity
    values = (forward * config['forward_units_per_mps'],
              -left * config['left_units_per_mps'],
              -ccw * config['rot_units_per_radps'])
    limits = (config['max_translation_units'], config['max_translation_units'], config['max_rot_units'])
    if not all(math.isfinite(v) and abs(v) <= limit for v, limit in zip(values, limits)):
        raise ValueError('Command exceeds commissioned native-unit limits')
    return (*values, math.remainder(config['yaw_target_deg'], 360.))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('serial.navcar.yaml'))
    parser.add_argument('--enable-control', action='store_true', help='启用运动发送；默认只读，不接管底盘')
    parser.add_argument('--check-config', action='store_true')
    parser.add_argument('--no-ros', action='store_true', help='只读串口监视，不依赖 ROS')
    args = parser.parse_args(argv)
    if args.no_ros and args.enable_control:
        parser.error('--no-ros only supports read-only monitoring')
    try:
        config = load_config(args.config, args.enable_control)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    if args.check_config:
        print('Navcar 配置通过；未打开串口、未发送命令。')
        return
    import serial
    port = serial.Serial(config['serial_port'], config['baud'], timeout=0, write_timeout=.02,
                         bytesize=8, parity='N', stopbits=1, xonxoff=False, rtscts=False, dsrdtr=False,
                         exclusive=True)
    port.reset_input_buffer()
    session = Session()
    node = None
    velocity, command_time = (0., 0., 0.), 0.
    stopped, armed = False, False
    def interrupt(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        if not args.no_ros:
            import rclpy
            from geometry_msgs.msg import Twist
            from std_msgs.msg import String
            rclpy.init(args=[])
            node = rclpy.create_node('navcar_uart1_bridge')
            status_pub = node.create_publisher(String, '/navcar/target_feedback', 10)
            def command(msg):
                nonlocal velocity, command_time
                vals = (msg.linear.x, msg.linear.y, msg.angular.z)
                if all(math.isfinite(v) for v in vals):
                    velocity, command_time = vals, time.monotonic()
                else:
                    velocity, command_time = (0., 0., 0.), 0.
            if args.enable_control:
                # Explicit yaw-frame topic: /cmd_vel is base-frame and is NOT interchangeable.
                subscription = node.create_subscription(Twist, '/navcar/cmd_vel_yaw', command, 1)
        print('Navcar: ' + ('控制已启用，等待状态同步；小陀螺默认0' if args.enable_control else '只读监视，不发送任何命令'), flush=True)
        last_report = 0.
        connected_at = time.monotonic()
        while not stopped:
            now = time.monotonic()
            session.receive(port.read(min(port.in_waiting, 4096)), now)
            if node:
                rclpy.spin_once(node, timeout_sec=0)
            if args.enable_control:
                # Start only with zero translation/rotation; discard commands received before synchronization.
                if not armed and session.state is not None:
                    velocity, command_time = (0., 0., 0.), 0.
                    armed = True
                active = velocity if now-command_time < .15 else (0., 0., 0.)
                try:
                    values = targets(active, config)
                except ValueError as exc:
                    print(str(exc), flush=True)
                    velocity, command_time = (0., 0., 0.), 0.
                    values = targets(velocity, config)
                packet = session.poll(values, now)
                if packet and port.write(packet) != len(packet):
                    raise IOError('Partial serial write')
                if session.error:
                    raise RuntimeError(session.error + '; stopped; restart to reconnect/synchronize')
                if session.state is None and now-connected_at > 3:
                    raise RuntimeError('No valid MCU status within 3 seconds; no commands sent')
            if now-last_report >= .2:
                last_report = now
                state = session.state
                report = {'protocol': 'navcar_uart1_v1', 'control_enabled': args.enable_control,
                          'feedback_kind': 'targets_only', 'measured_yaw_available': False,
                          'wheel_odometry_available': False, 'sequence': state.sequence if state else None,
                          'targets_vy_vx_rot_yaw_deg': state.values if state else None,
                          'status_age_s': now-session.state_time if state else None,
                          'last_ack': session.last_ack, 'error': session.error}
                encoded = json.dumps(report, allow_nan=False)
                if node:
                    msg = String();msg.data = encoded;status_pub.publish(msg)
                else:
                    print(encoded, flush=True)
            time.sleep(.002)
    finally:
        if args.enable_control and session.sequence is not None and not session.error:
            # Complete any in-flight command before a new sequence. Preserve current yaw target.
            after = time.monotonic()
            if session.pending is not None:
                # Do not retry an old moving command during shutdown; await ACK or fall back to watchdog.
                deadline = after + .05
                while session.pending is not None and time.monotonic() < deadline:
                    session.receive(port.read(min(port.in_waiting, 4096)), time.monotonic())
                    time.sleep(.002)
            if session.pending is None and not session.error:
                after = time.monotonic()
                packet = session.poll(targets((0., 0., 0.), config), max(after, session.next_at))
                if packet and port.write(packet) != len(packet):
                    session.fail('Partial stop write')
                deadline = after + .25
                while not stop_confirmed(session, after) and not session.error and time.monotonic() < deadline:
                    now = time.monotonic()
                    session.receive(port.read(min(port.in_waiting, 4096)), now)
                    if session.pending is not None:
                        packet = session.poll(targets((0., 0., 0.), config), now)
                        if packet and port.write(packet) != len(packet):
                            session.fail('Partial stop retry')
                    time.sleep(.002)
                print('零速 ACK 和状态已确认' if stop_confirmed(session, after) else '未确认零速；已停止发帧，MCU 300ms 看门狗归零', flush=True)
            else:
                print('未完成 ACK；停止发帧，MCU 300ms 看门狗归零', flush=True)
        port.close()
        if node:
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
