#!/usr/bin/env python3
"""Isolated synthetic MID360 + IMU input; never a replacement for vehicle measurements."""
import math
import os
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg, CustomPoint


class SyntheticMID360(Node):
    def __init__(self):
        super().__init__('synthetic_mid360_validation_only')
        self.pose = (0., 0., 0., 0., 0., 0.)
        self.joint_angle = 0.
        self.last_velocity = (0., 0., 0.)
        self.last_imu = self.get_clock().now().nanoseconds
        self.points = self.make_scene()
        self.imu = self.create_publisher(Imu, '/livox/imu', 100)
        self.lidar = self.create_publisher(CustomMsg, '/livox/lidar', 10)
        self.create_subscription(Odometry, '/wheel/odom', self.wheel, 10)
        self.create_subscription(JointState, '/yaw/joint_states', self.yaw, 10)
        self.imu_group = MutuallyExclusiveCallbackGroup()
        self.scan_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(.005, self.imu_tick, callback_group=self.imu_group)
        self.create_timer(.1, self.lidar_tick, callback_group=self.scan_group)

    @staticmethod
    def make_scene():
        points = []
        for x in np.arange(-6., 6.01, .16):
            for y in np.arange(-4., 4.01, .16):
                points.append((x, y, 0.))
        for z in np.arange(.1, 2.01, .15):
            for y in np.arange(-4., 4.01, .16):
                points.extend([(-6., y, z), (6., y, z)])
            for x in np.arange(-6., 6.01, .16):
                points.extend([(x, -4., z), (x, 4., z)])
        # Deterministic point order keeps time offsets increasing without raster-line aliasing.
        result = np.asarray(points, dtype=np.float64)
        return result[np.random.default_rng(17).permutation(len(result))]

    def wheel(self, msg):
        q = msg.pose.pose.orientation
        heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, heading,
                     msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)

    def yaw(self, msg):
        if 'yaw_joint' in msg.name:
            self.joint_angle = msg.position[msg.name.index('yaw_joint')]

    def imu_tick(self):
        now = self.get_clock().now()
        dt = max(.001, (now.nanoseconds-self.last_imu)*1e-9)
        self.last_imu = now.nanoseconds
        x, y, heading, vx, vy, wz = self.pose
        previous = self.last_velocity
        self.last_velocity = (vx, vy, wz)
        alpha = (wz-previous[2])/dt
        ax = (vx-previous[0])/dt-wz*vy-alpha*.02329-wz*wz*.011
        ay = (vy-previous[1])/dt+wz*vx+alpha*.011-wz*wz*.02329
        angle = self.joint_angle
        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.w = 1.
        msg.angular_velocity.z = float(wz)
        msg.linear_acceleration.x = float(ax*math.cos(angle)+ay*math.sin(angle))
        msg.linear_acceleration.y = float(-ax*math.sin(angle)+ay*math.cos(angle))
        msg.linear_acceleration.z = 9.81
        self.imu.publish(msg)

    def lidar_tick(self):
        end = self.get_clock().now().nanoseconds
        start = end-100_000_000
        x, y, heading, vx, vy, wz = self.pose
        fraction = np.linspace(0., 1., len(self.points))
        age = (1.-fraction)*.1
        px = x-(vx*math.cos(heading)-vy*math.sin(heading))*age
        py = y-(vx*math.sin(heading)+vy*math.cos(heading))*age
        angle = heading+self.joint_angle-wz*age
        dx, dy = self.points[:, 0]-px, self.points[:, 1]-py
        xyz = np.column_stack([dx*np.cos(angle)+dy*np.sin(angle),
                               -dx*np.sin(angle)+dy*np.cos(angle), self.points[:, 2]-.7])
        msg = CustomMsg()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(start, 10**9)
        msg.header.frame_id = 'livox_frame'
        msg.timebase = start
        points = []
        for i, point in enumerate(xyz):
            p = CustomPoint()
            p.x, p.y, p.z = [float(v) for v in point]
            p.offset_time = int(fraction[i]*100_000_000)
            p.line, p.tag, p.reflectivity = i % 4, 0x10, 100
            points.append(p)
        msg.points, msg.point_num = points, len(points)
        self.lidar.publish(msg)


def main():
    if os.environ.get('SCURM_SYNTHETIC_VALIDATION') != '1' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        sys.exit('Synthetic sensor input is restricted to the isolated validation environment.')
    rclpy.init()
    node = SyntheticMID360()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
