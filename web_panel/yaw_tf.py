#!/usr/bin/env python3
"""Publish the measured base_link -> yaw_link and fixed yaw_link -> imu_link chain."""
import argparse
import math
from pathlib import Path
from robot_io import load_runtime, finite_number


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('runtime.json'))
    args = p.parse_args()
    config = load_runtime(args.config)['yaw']
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

    class YawTF(Node):
        def __init__(self):
            super().__init__('scurm_yaw_tf')
            self.dynamic = TransformBroadcaster(self)
            self.static = StaticTransformBroadcaster(self)
            fixed = TransformStamped()
            fixed.header.stamp = self.get_clock().now().to_msg()
            fixed.header.frame_id, fixed.child_frame_id = 'yaw_link', 'imu_link'
            fixed.transform.translation.x, fixed.transform.translation.y, fixed.transform.translation.z = config['imu_xyz_m']
            q = quaternion(*config['imu_rpy_rad'])
            fixed.transform.rotation.x, fixed.transform.rotation.y, fixed.transform.rotation.z, fixed.transform.rotation.w = q
            self.static.sendTransform(fixed)
            self.create_subscription(JointState, '/yaw/joint_states', self.feedback, qos_profile_sensor_data)

        def feedback(self, msg):
            try:
                index = msg.name.index(config.get('joint_name', 'yaw_joint'))
                angle = msg.position[index]
                if not finite_number(angle) or msg.header.stamp.sec == 0:
                    return
            except (ValueError, IndexError):
                return
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id, tf.child_frame_id = 'base_link', 'yaw_link'
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = config['axis_xyz_m']
            tf.transform.rotation.z = math.sin(angle/2)
            tf.transform.rotation.w = math.cos(angle/2)
            self.dynamic.sendTransform(tf)

    rclpy.init()
    node = YawTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
