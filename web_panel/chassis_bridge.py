#!/usr/bin/env python3
"""ROS interfaces for legacy JSONL/mock hardware or the Navcar UART1 V1 adapter."""
import argparse
import math
from pathlib import Path
import time
from robot_io import decode_feedback, encode_command, load_io_config, finite_number


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('runtime.json'))
    p.add_argument('--mock', action='store_true', help='模拟下位机，不打开串口、不驱动电机')
    p.add_argument('--check-config', action='store_true', help='只检查串口/yaw 参数，不连接串口或 ROS')
    p.add_argument('--enable-control', action='store_true', help='Navcar：显式启用控制，默认只读')
    p.add_argument('--no-ros', action='store_true', help='Navcar：不依赖ROS的只读串口监视')
    args = p.parse_args()
    # Navcar has different units and target-only feedback; never route it through JSONL/yaw odometry.
    if args.config.exists():
        import json
        import yaml
        raw = (yaml.safe_load(args.config.read_text()) if args.config.suffix.lower() in ('.yaml', '.yml')
               else json.loads(args.config.read_text()))
        if isinstance(raw, dict) and raw.get('transport') == 'navcar_uart1_v1':
            if args.mock:p.error('Navcar does not use the legacy JSONL mock')
            from navcar_bridge import main as navcar_main
            return navcar_main(['--config',str(args.config)] +
                               [flag for flag, enabled in [('--check-config',args.check_config),
                                ('--enable-control',args.enable_control),('--no-ros',args.no_ros)] if enabled])
    if args.enable_control or args.no_ros:
        p.error('--enable-control / --no-ros require a Navcar configuration')
    try:
        config = load_io_config(args.config, serial=not args.mock)
    except (OSError, ValueError) as exc:
        p.error(str(exc))
    hw, yaw = config['hardware'], config['yaw']
    if args.check_config:
        print('接口配置通过；未连接串口或 ROS。')
        return
    serial = None
    if not args.mock:
        import serial as pyserial
        serial = pyserial.Serial(hw['serial_port'], hw['baud'], timeout=0, write_timeout=.02)
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist

    class Bridge(Node):
        def __init__(self):
            super().__init__('scurm_chassis_bridge')
            self.velocity = (0., 0., 0.)
            self.command_time = 0.
            self.target = yaw['target_rad']
            self.buffer = bytearray()
            self.seq = 0
            self.feedback_seq = -1
            self.feedback_time = 0.
            self.mock_angle = self.target
            self.mock_pose = [0., 0., 0.]
            self.last_tick = time.monotonic()
            self.error_time = 0.
            self.odom = self.create_publisher(Odometry, '/wheel/odom', 10)
            self.joint = self.create_publisher(JointState, '/yaw/joint_states', 10)
            self.create_subscription(Twist, '/cmd_vel', self.command, 1)
            self.create_subscription(Float64, '/yaw/target', self.yaw_command, 1)
            self.create_timer(.02, self.tick)
            self.get_logger().info('模拟底盘，无电机输出' if args.mock else '参考串口协议已打开；需要匹配的下位机固件')

        def command(self, msg):
            values = (msg.linear.x, msg.linear.y, msg.angular.z)
            self.velocity = values if all(finite_number(v) for v in values) else (0., 0., 0.)
            self.command_time = time.monotonic()

        def yaw_command(self, msg):
            # This bridge intentionally accepts only the commissioned hold target.
            if finite_number(msg.data) and abs(msg.data-yaw['target_rad']) < 1e-6:
                self.target = msg.data

        def publish_state(self, packet):
            sequence = packet.get('seq')
            if not isinstance(sequence, int) or sequence <= self.feedback_seq:
                raise ValueError('状态 seq 必须递增；下位机重启后重启桥接进程')
            self.feedback_seq = sequence
            self.feedback_time = time.monotonic()
            sign = 1 if args.mock else hw['yaw_encoder_sign']
            zero = 0 if args.mock else hw['yaw_encoder_zero_rad']
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [yaw.get('joint_name', 'yaw_joint')]
            msg.position = [sign * (packet['yaw_encoder_rad'] - zero)]
            msg.velocity = [sign * packet['yaw_rate_rad_s']]
            self.joint.publish(msg)
            if 'odom' in packet:
                state = packet['odom']
                odom = Odometry()
                odom.header.stamp = msg.header.stamp
                odom.header.frame_id, odom.child_frame_id = 'wheel_odom', 'base_link'
                odom.pose.pose.position.x, odom.pose.pose.position.y = state['x'], state['y']
                odom.pose.pose.orientation.z = math.sin(state['yaw']/2)
                odom.pose.pose.orientation.w = math.cos(state['yaw']/2)
                odom.twist.twist.linear.x, odom.twist.twist.linear.y = state['vx'], state['vy']
                odom.twist.twist.angular.z = state['wz']
                self.odom.publish(odom)  # No odom->base TF; FAST-LIO owns that edge.

        def tick(self):
            now = time.monotonic()
            dt = min(.1, now-self.last_tick)
            self.last_tick = now
            velocity = self.velocity if now-self.command_time < hw.get('command_timeout_s', .3) else (0., 0., 0.)
            if not args.mock and now-self.feedback_time > yaw.get('feedback_timeout_s', .3):
                velocity = (0., 0., 0.)
            self.seq += 1
            try:
                if args.mock:
                    vx, vy, wz = velocity
                    x, y, theta = self.mock_pose
                    self.mock_pose = [x+(vx*math.cos(theta)-vy*math.sin(theta))*dt,
                                      y+(vx*math.sin(theta)+vy*math.cos(theta))*dt, theta+wz*dt]
                    self.publish_state({'seq': self.seq, 'yaw_encoder_rad': self.mock_angle,
                        'yaw_rate_rad_s': 0., 'odom': dict(zip(('x','y','yaw','vx','vy','wz'), [*self.mock_pose,*velocity]))})
                else:
                    command = encode_command(self.seq, velocity, self.target, hw)
                    if serial.write(command) != len(command):
                        raise IOError('串口命令未完整发送')
                    self.buffer.extend(serial.read(min(serial.in_waiting, 4096)))
                    if len(self.buffer) > 8192:
                        self.buffer.clear()
                        raise ValueError('串口缓冲溢出，检查换行分帧或波特率')
                    while b'\n' in self.buffer:
                        line, _, rest = self.buffer.partition(b'\n')
                        self.buffer = bytearray(rest)
                        self.publish_state(decode_feedback(line))
            except Exception as exc:
                self.velocity = (0., 0., 0.)
                if now-self.error_time > 1.:
                    self.get_logger().error(str(exc))
                    self.error_time = now

    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if serial:
            try:
                serial.write(encode_command(node.seq+1, (0., 0., 0.), node.target, hw))
            finally:
                serial.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
