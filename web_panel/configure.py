#!/usr/bin/env python3
"""Apply measured robot geometry, MID360 networking, and conservative startup limits."""
import argparse
import ipaddress
import json
import math
from pathlib import Path
import re
import shutil
import yaml
from robot_io import validate_config, optical_center_in_base


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, help='填写后的 robot.yaml；推荐使用')
    p.add_argument('--length', type=float, help='Full collision length, metres')
    p.add_argument('--width', type=float, help='Full collision width, metres')
    p.add_argument('--lidar-height', type=float, help='Optical centre above floor, m')
    p.add_argument('--imu-xyz', type=float, nargs=3,
                   help='MID360 IMU origin in yaw_link, metres')
    p.add_argument('--imu-rpy', type=float, nargs=3, default=[0, 0, 0], help='Mount roll/pitch/yaw, radians')
    p.add_argument('--host-ip', type=ipaddress.IPv4Address)
    p.add_argument('--lidar-ip', type=ipaddress.IPv4Address)
    p.add_argument('--yaw-axis-xyz', type=float, nargs=3)
    p.add_argument('--yaw-target', type=float)
    p.add_argument('--yaw-reference', choices=['base_link'])
    p.add_argument('--maps-dir', type=Path, default=Path.home() / 'scurm_maps')
    args = p.parse_args()
    if args.config:
        config = yaml.safe_load(args.config.read_text(encoding='utf-8'))
    else:
        config = {'version': 1, 'robot': {'length_m': args.length, 'width_m': args.width,
                  'lidar_height_m': args.lidar_height}, 'yaw': {
                  'reference': args.yaw_reference, 'target_rad': args.yaw_target,
                  'axis_xyz_m': args.yaw_axis_xyz, 'imu_xyz_m': args.imu_xyz,
                  'imu_rpy_rad': args.imu_rpy}, 'network': {
                  'host_ip': str(args.host_ip) if args.host_ip else None,
                  'lidar_ip': str(args.lidar_ip) if args.lidar_ip else None},
                  'hardware': {'transport': 'ros'}}
    if not isinstance(config, dict):
        p.error('robot.yaml 必须为配置字典')
    missing = validate_config(config)
    if missing:
        p.error('以下参数仍待测：' + '、'.join(missing))
    args.length, args.width = config['robot']['length_m'], config['robot']['width_m']
    args.lidar_height = config['robot']['lidar_height_m']
    args.imu_xyz, args.imu_rpy = config['yaw']['imu_xyz_m'], config['yaw']['imu_rpy_rad']
    args.yaw_axis_xyz, args.yaw_target = config['yaw']['axis_xyz_m'], config['yaw']['target_rad']
    args.host_ip, args.lidar_ip = config['network']['host_ip'], config['network']['lidar_ip']
    root = Path(__file__).resolve().parent.parent
    maps = args.maps_dir.expanduser().resolve()
    maps.mkdir(parents=True, exist_ok=True)
    config['maps_dir'] = str(maps)
    backup_root = root / 'web_panel' / 'original_configs'

    def write(path, text):
        backup = backup_root / path.relative_to(root)
        if path.exists() and not backup.exists():
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, backup)
        path.write_text(text, encoding='utf-8')

    def update_yaml(path, fn):
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
        fn(data)
        write(path, yaml.safe_dump(data, sort_keys=False, allow_unicode=True))

    params = root / 'sentry_bringup' / 'params'
    for name in ('fast_lio_mapping_param.yaml', 'fast_lio_relocalization_param.yaml'):
        def update_lio(data):
            cfg = data['/**']['ros__parameters']
            cfg['map_file_path'] = str(maps / 'site.pcd')
            if cfg['locate_in_prior_map']:
                cfg['prior_map_path'] = str(maps / 'site.pcd')
            cfg['filter_size_surf'] = cfg['filter_size_map'] = 0.15
            cfg['mapping']['extrinsic_est_en'] = False
            # MID360 internal LiDAR->IMU extrinsics from hku-mars/FAST_LIO mid360.yaml.
            cfg['mapping']['extrinsic_T'] = [-0.011, -0.02329, 0.04412]
            cfg['mapping']['extrinsic_R'] = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        update_yaml(params / name, update_lio)

    def update_nav(data):
        def real_clock(item):
            if isinstance(item, dict):
                for key, value in item.items():
                    if key == 'use_sim_time':
                        item[key] = False
                    else:
                        real_clock(value)
        real_clock(data)
        nav = data['bt_navigator']['ros__parameters']
        for key, filename in [('default_nav_to_pose_bt_xml', 'navigate_to_pose_w_replanning_and_recovery.xml'),
                              ('default_nav_through_poses_bt_xml', 'navigate_through_pose_w_replanning_and_recovery.xml')]:
            nav[key] = str(root / 'sentry_bringup' / 'behavior_tree' / filename)
        controller = data['controller_server']['ros__parameters']
        controller.update(controller_frequency=15.0, min_x_velocity_threshold=0.01,
                          min_y_velocity_threshold=0.01, min_theta_velocity_threshold=0.01)
        controller['progress_checker'].update(required_movement_radius=0.1, movement_time_allowance=10.0)
        controller['general_goal_checker'].update(xy_goal_tolerance=0.15, yaw_goal_tolerance=0.25)
        controller['FollowPath'].update(min_vel_x=-0.3, max_vel_x=0.3, min_vel_y=-0.3,
            max_vel_y=0.3, max_vel_theta=0.5, min_speed_xy=0.0, max_speed_xy=0.3,
            min_speed_theta=0.0, acc_lim_x=0.3, acc_lim_y=0.3, acc_lim_theta=0.5,
            decel_lim_x=-0.3, decel_lim_y=-0.3, decel_lim_theta=-0.5,
            vx_samples=10, vy_samples=10, vtheta_samples=16, sim_time=1.5,
            trans_stopped_velocity=0.03)
        x, y = args.length / 2, args.width / 2
        footprint = str([[x, y], [x, -y], [-x, -y], [-x, y]])
        for name in ('local_costmap', 'global_costmap'):
            cfg = data[name][name]['ros__parameters']
            cfg.pop('robot_radius', None)
            cfg['footprint'] = footprint
            cfg['footprint_padding'] = 0.03
            cfg['resolution'] = 0.05
            cfg['local_inflation_layer' if name == 'local_costmap' else 'global_inflation_layer']['inflation_radius'] = max(0.4, math.hypot(x, y) + 0.15)
        data['map_server']['ros__parameters']['yaml_filename'] = str(maps / 'site.yaml')
        data['velocity_smoother']['ros__parameters'].update(feedback='OPEN_LOOP',
            odom_topic='/state_estimation', smoothing_frequency=20.0,
            max_velocity=[0.3, 0.3, 0.5], min_velocity=[-0.3, -0.3, -0.5],
            max_accel=[0.3, 0.3, 0.5], max_decel=[-0.3, -0.3, -0.5], velocity_timeout=0.3)
    update_yaml(params / 'nav2_params.yaml', update_nav)

    path = root / 'sentry_bringup' / 'launch' / 'relocalization.launch.py'
    source = path.read_text(encoding='utf-8')
    # repr preserves valid Python literals even for apostrophes in a filesystem path.
    source = re.sub(r"\{'map_path':[^\n]+", lambda _: "{'map_path':" + repr(str(maps / 'site.pcd')) + "},", source)
    write(path, source)
    path = root / 'livox_ros_driver2' / 'config' / 'MID360_config.json'
    data = json.loads(path.read_text(encoding='utf-8'))
    host = data['MID360']['host_net_info']
    for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
        host[key] = str(args.host_ip)
    data['lidar_configs'][0]['ip'] = str(args.lidar_ip)
    write(path, json.dumps(data, indent=2) + '\n')

    for package, launch in [('terrain_analysis', 'terrain_analysis.launch'),
                            ('terrain_analysis_ext', 'terrain_analysis_ext.launch')]:
        path = root / 'autonomous_exploration_development_environment' / 'src' / package / 'launch' / launch
        source = path.read_text(encoding='utf-8')
        # /state_estimation is already the BASE pose, not the sensor pose.
        # Adding the sensor translation again would move the robot self-filter.
        parameters = {'vehicleLength': args.length, 'vehicleWidth': args.width,
                      'vehicleHeight': args.lidar_height, 'sensorOffsetX': 0.0, 'sensorOffsetY': 0.0}
        if package == 'terrain_analysis_ext':
            optical_z = optical_center_in_base(config['yaw'])[2]
            ground_relative_to_base = optical_z - args.lidar_height
            parameters.update(lowerBoundZ=min(-.5, ground_relative_to_base-.3),
                              upperBoundZ=max(1., args.lidar_height),
                              terrainUnderVehicle=ground_relative_to_base)
        for key, value in parameters.items():
            source = re.sub(r'(<param name="' + key + r'" value=")[^"]+',
                            lambda m, v=value: m[1] + str(v), source)
        write(path, source)

    runtime = root / 'web_panel' / 'runtime.json'
    write(runtime, json.dumps(config, ensure_ascii=False, indent=2) + '\n')
    tf_cmd = ['python3', str(root / 'web_panel' / 'yaw_tf.py'), '--config', str(runtime)]
    tf_source = (
        'from launch import LaunchDescription\nfrom launch.actions import ExecuteProcess\n\n'
        'def generate_launch_description():\n'
        '    return LaunchDescription([ExecuteProcess(cmd=' + repr(tf_cmd) + ', output="screen")])\n')
    write(root / 'web_panel' / 'robot_tf.launch.py', tf_source)
    print('已生成实车配置。重新 colcon 编译后运行 bringup.py。')
    print('Map directory:', maps)
    print('First originals:', backup_root)


if __name__ == '__main__':
    main()
