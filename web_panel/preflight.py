#!/usr/bin/env python3
"""只读链路检查：离线待测表，或 ROS 话题、TF、服务与导航 action。"""
import argparse
import json
from pathlib import Path
import time
from robot_io import load_runtime, validate_config, YawGuard, sample_monotonic_time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('runtime.json'))
    p.add_argument('--offline', action='store_true')
    p.add_argument('--mode', choices=['idle', 'mapping', 'navigation'], default='idle')
    p.add_argument('--timeout', type=float, default=8.)
    p.add_argument('--report', type=Path)
    args = p.parse_args()
    if args.offline:
        import yaml
        config = yaml.safe_load(args.config.read_text(encoding='utf-8'))
        missing = validate_config(config)
        report = {'模式': '离线检查', '实车必填待测项': missing,
                  '下位机待测记录': config.get('pending', {}), '可生成配置': not missing}
        result = not missing
    else:
        config = load_runtime(args.config)
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
        from sensor_msgs.msg import Imu, JointState, PointCloud2
        from nav_msgs.msg import Odometry, OccupancyGrid
        from livox_ros_driver2.msg import CustomMsg
        from tf2_ros import Buffer, TransformListener
        rclpy.init()
        node = Node('scurm_chain_preflight')
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        latest = {}
        yaw_guard = YawGuard(config['yaw'])
        def observed(topic, msg):
            sampled_at = sample_monotonic_time(msg.header.stamp,
                node.get_clock().now().nanoseconds, time.monotonic(),
                config['yaw'].get('feedback_timeout_s', .3) if topic == '/yaw/joint_states' else 2.)
            if topic == '/map':
                sampled_at = time.monotonic()  # Static map uses transient-local durability.
            if sampled_at is None:
                return
            latest[topic] = sampled_at
            if topic == '/yaw/joint_states':
                try:
                    index = msg.name.index(config['yaw'].get('joint_name', 'yaw_joint'))
                    yaw_guard.update(msg.position[index], msg.velocity[index], sampled_at)
                except (ValueError, IndexError):
                    pass
        expected = {'/livox/lidar': CustomMsg, '/livox/imu': Imu, '/yaw/joint_states': JointState}
        if args.mode != 'idle':
            expected.update({'/cloud_registered': PointCloud2, '/state_estimation': Odometry,
                             '/terrain_map_ext' if args.mode == 'mapping' else '/terrain_map': PointCloud2})
        if args.mode == 'mapping':
            expected['/projected_map'] = OccupancyGrid
        if args.mode == 'navigation':
            expected.update({'/map': OccupancyGrid, '/local_costmap/costmap': OccupancyGrid})
        subscriptions = []
        for topic, message_type in expected.items():
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic == '/map' else qos_profile_sensor_data
            subscriptions.append(node.create_subscription(message_type, topic,
                lambda msg, topic=topic: observed(topic, msg), qos))
        deadline = time.monotonic() + args.timeout
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            report = {'模式': args.mode, '话题': {}, 'TF': {}, '服务或动作': {}}
            for topic in expected:
                age = now-latest[topic] if topic in latest else None
                # A static map is expected to publish once with transient-local durability.
                report['话题'][topic] = {'收到': age is not None,
                    '新鲜': age is not None and (topic == '/map' or age < 2.),
                    '最后接收秒数': round(age, 2) if age is not None else None}
            transforms = [('base_link', 'imu_link'), ('base_link', 'yaw_link')]
            if args.mode != 'idle':
                transforms.append(('odom', 'base_link'))
            if args.mode == 'navigation':
                transforms.append(('map', 'odom'))
            for parent, child in transforms:
                name = parent + ' -> ' + child
                try:
                    tf = buffer.lookup_transform(parent, child, rclpy.time.Time())
                    report['TF'][name] = {'连通': True,
                        'xyz': [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]}
                except Exception as exc:
                    report['TF'][name] = {'连通': False, '错误': str(exc)}
            if args.mode == 'mapping':
                report['服务或动作']['/map_save'] = any(name == '/map_save' for name, types in node.get_service_names_and_types())
            if args.mode == 'navigation':
                from rclpy.action import get_action_names_and_types
                report['服务或动作']['/navigate_to_pose'] = any(name == '/navigate_to_pose' for name, types in get_action_names_and_types(node))
            publishers = node.get_publishers_info_by_topic('/cmd_vel')
            report['最终速度发布者'] = [info.node_name for info in publishers]
            report['最终速度仅门控发布'] = len(publishers) == 1 and publishers[0].node_name == 'scurm_web_panel'
            report['yaw锁角'] = yaw_guard.status()
            result = (all(v['新鲜'] for v in report['话题'].values()) and
                      all(v['连通'] for v in report['TF'].values()) and
                      all(report['服务或动作'].values()) and report['最终速度仅门控发布'] and report['yaw锁角']['ready'])
            report['通过'] = result
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    text = json.dumps(report, ensure_ascii=False, indent=2)
    print(text)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(text+'\n', encoding='utf-8')
    return 0 if result else 1


if __name__ == '__main__':
    raise SystemExit(main())
