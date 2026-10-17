#!/usr/bin/env python3
"""Exercise real Humble callbacks, FAST-LIO, mapping, ICP, Nav2 and Web in isolation."""
import json
import copy
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[2]
PANEL = ROOT/'web_panel'
sys.path.insert(0, str(PANEL))
from server import stop_process
from serial_mcu import PtyMCU


def main():
    if os.environ.get('SCURM_SYNTHETIC_VALIDATION') != '1' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        sys.exit('This test requires the isolated validation environment.')
    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.action import ActionClient
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import PointCloud2, JointState
    from nav_msgs.msg import Odometry, OccupancyGrid
    from nav2_msgs.action import NavigateToPose
    from lifecycle_msgs.srv import GetState
    from tf2_ros import Buffer, TransformListener

    evidence = Path(os.environ.get('SCURM_VALIDATION_EVIDENCE', '/evidence'))
    evidence.mkdir(parents=True, exist_ok=True)
    maps = evidence/'synthetic_maps'
    maps.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load((PANEL/'robot.example.yaml').read_text())
    config['robot'].update(length_m=.6, width_m=.4, lidar_height_m=.7)
    config['yaw'].update(reference='base_link', target_rad=0., axis_xyz_m=[0., 0., .3],
                         imu_xyz_m=[.011, .02329, .35588], imu_rpy_rad=[0., 0., 0.])
    config['network'].update(host_ip='192.168.1.50', lidar_ip='192.168.1.196')
    mcu = PtyMCU()
    config['hardware'].update(transport='serial_jsonl_v1', serial_port=mcu.port, baud=460800,
                             yaw_encoder_sign=mcu.sign, yaw_encoder_zero_rad=mcu.zero)
    # These values belong only to this generated virtual vehicle, never to the user's car.
    configuration = Path('/workspace/validation_virtual_robot.yaml')
    configuration.write_text(yaml.safe_dump(config, allow_unicode=True))
    subprocess.run([sys.executable, str(PANEL/'configure.py'), '--config', str(configuration),
                    '--maps-dir', str(maps)], check=True)
    processes, handles = [], []
    env = dict(os.environ, SCURM_WEB_TOKEN='isolated-ros-validation')

    def launch(name, command):
        log = (evidence/(name+'.log')).open('wb')
        handles.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=env)
        processes.append((name, process))
        return process

    def api(path='/api/state', body=None):
        request = Request('http://127.0.0.1:18080'+path,
            data=json.dumps(body).encode() if body is not None else None,
            headers={'Authorization':'Bearer isolated-ros-validation', 'Content-Type':'application/json'})
        with urlopen(request, timeout=3) as response:
            return json.load(response)

    def wait_for(check, timeout, description):
        deadline = time.monotonic()+timeout
        last = None
        while time.monotonic() < deadline:
            for name, process in processes:
                if process.poll() is not None:
                    raise RuntimeError(f'{name} exited; see {name}.log')
            try:
                result = check()
                if result:
                    print('PASS: '+description, flush=True)
                    return result
            except Exception as exc:
                last = exc
            time.sleep(.1)
        raise TimeoutError(description+': '+str(last))

    rclpy.init()
    node = Node('scurm_ros_integration_observer')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    latest, counts, velocities = {}, {}, []
    def received(topic, msg):
        latest[topic] = (time.monotonic(), msg)
        counts[topic] = counts.get(topic, 0)+1
        if topic == '/cmd_vel':
            velocities.append((time.monotonic(), msg.linear.x, msg.linear.y, msg.angular.z))
    subscriptions = []
    for topic, kind in [('/cloud_registered', PointCloud2), ('/state_estimation', Odometry),
                        ('/terrain_map_ext', PointCloud2), ('/terrain_map', PointCloud2),
                        ('/projected_map', OccupancyGrid), ('/wheel/odom', Odometry), ('/cmd_vel', Twist)]:
        subscriptions.append(node.create_subscription(kind, topic, lambda msg, topic=topic: received(topic, msg),
                                                       qos_profile_sensor_data))
    manual = node.create_publisher(Twist, '/web/cmd_vel_manual', 1)
    auto = node.create_publisher(Twist, '/web/cmd_vel_auto', 1)
    yaw_fault = node.create_publisher(JointState, '/yaw/joint_states', 1)
    cloud_replay = node.create_publisher(PointCloud2, '/cloud_registered', 1)
    odom_replay = node.create_publisher(Odometry, '/state_estimation', 1)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    controller = node.create_client(GetState, '/controller_server/get_state')
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    def publish_velocity(publisher, vx, duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            command = Twist()
            command.linear.x = vx
            publisher.publish(command)
            time.sleep(.05)

    def future_result(future, timeout):
        wait_for(lambda: future.done(), timeout, 'ROS future completed')
        return future.result()

    report = {'simulation_only': True, 'ros_distro': os.environ.get('ROS_DISTRO'), 'steps': []}
    try:
        launch('serial_chassis', [sys.executable, str(PANEL/'chassis_bridge.py')])
        launch('yaw_tf', [sys.executable, str(PANEL/'yaw_tf.py')])
        sensor = launch('synthetic_mid360', [sys.executable, str(Path(__file__).with_name('synthetic_mid360.py'))])
        launch('web_ros', [sys.executable, '-u', str(PANEL/'server.py'), '--port', '18080', '--mock-hardware'])
        wait_for(lambda: api()['yaw']['ready'], 15, 'real ROS yaw feedback reaches Web')
        report['steps'].append('yaw_feedback_web')
        report['steps'].append('actual_pyserial_transport_with_virtual_mcu')
        api('/api/mode', {'mode':'mapping'})
        wait_for(lambda: '/cloud_registered' in latest and latest['/cloud_registered'][1].width > 100 and
                        '/state_estimation' in latest, 45, 'FAST-LIO produces cloud and base odometry')
        wait_for(lambda: '/projected_map' in latest and latest['/projected_map'][1].info.width > 0,
                 30, 'terrain/OctoMap produces a 2D map')
        cloud = api('/api/cloud')
        assert len(cloud['points']) > 100 and cloud['frame'] == 'odom', cloud
        start_x = latest['/wheel/odom'][1].pose.pose.position.x
        publish_velocity(manual, .1, 1.)
        publish_velocity(manual, 0., .5)
        assert latest['/wheel/odom'][1].pose.pose.position.x-start_x > .03
        report['steps'].append('mapping_cloud_terrain_web_manual_velocity')
        subprocess.run([sys.executable, str(PANEL/'preflight.py'), '--mode', 'mapping',
                        '--report', str(evidence/'preflight-mapping.json')], check=True)
        api('/api/mode', {'mode':'idle'})
        wait_for(lambda: api()['mode'] == 'idle' and not api()['switching'], 75, 'mapping saved and stopped')
        state = api()
        assert not state['error'], state
        for name in ('site.pcd', 'site.yaml', 'site.png'):
            assert (maps/name).is_file() and (maps/name).stat().st_size > 0, name
        report['steps'].append('pcd_and_2d_map_saved')
        api('/api/mode', {'mode':'navigation'})
        wait_for(lambda: navigator.server_is_ready() and controller.service_is_ready(), 60, 'Nav2 servers discovered')
        wait_for(lambda: future_result(controller.call_async(GetState.Request()), 5).current_state.id == 3,
                 30, 'Nav2 controller lifecycle is ACTIVE')
        wait_for(lambda: tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()), 30, 'ICP map-to-base TF')
        subprocess.run([sys.executable, str(PANEL/'preflight.py'), '--mode', 'navigation',
                        '--report', str(evidence/'preflight-navigation.json')], check=True)
        current = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = current.transform.translation.x+.6
        goal.pose.pose.position.y = current.transform.translation.y
        goal.pose.pose.orientation.w = 1.
        accepted = future_result(navigator.send_goal_async(goal), 10)
        assert accepted.accepted, 'Nav2 rejected the synthetic goal'
        result = future_result(accepted.get_result_async(), 90)
        assert result.status == 4, f'Nav2 did not succeed: {result.status}'
        actual = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        error = math.hypot(actual.transform.translation.x-goal.pose.pose.position.x,
                           actual.transform.translation.y-goal.pose.pose.position.y)
        assert error < .22, error
        report['navigation_position_error_m'] = error
        report['steps'].append('icp_nav2_action_actual_simulated_motion')
        # Verify actual callbacks reject replayed old data, even with fresh arrivals.
        started = time.monotonic()
        publish_velocity(auto, .1, .4)
        assert any(abs(v[1]) > .01 for v in velocities if v[0] > started), 'gate never opened'
        os.killpg(sensor.pid, signal.SIGSTOP)
        try:
            wait_for(lambda: (api()['cloud_age'] or 0.) > 1.2, 10, 'sensor loss reaches watchdog timeout')
            began_loss = time.monotonic()
            publish_velocity(auto, .1, .5)
            stopped = [v for v in velocities if v[0] > began_loss+.1]
            assert stopped and all(max(abs(x) for x in v[1:]) == 0. for v in stopped)
            report['steps'].append('actual_ros_sensor_loss_blocks_velocity')
            cloud_old = copy.deepcopy(latest['/cloud_registered'][1])
            odom_old = copy.deepcopy(latest['/state_estimation'][1])
            cloud_old.header.stamp.sec -= 5
            odom_old.header.stamp.sec -= 5
            began_replay = time.monotonic()
            while time.monotonic()-began_replay < 1.:
                cloud_replay.publish(cloud_old)
                odom_replay.publish(odom_old)
                command = Twist()
                command.linear.x = .1
                auto.publish(command)
                time.sleep(.05)
            replay_velocity = [v for v in velocities if v[0] > began_replay+.2]
            assert replay_velocity and all(max(abs(x) for x in v[1:]) == 0. for v in replay_velocity)
            report['steps'].append('actual_ros_stale_stamp_replay_blocks_velocity')
        finally:
            os.killpg(sensor.pid, signal.SIGCONT)
        wait_for(lambda: (api()['cloud_age'] is not None and api()['cloud_age'] < .5 and
                          api()['odom_age'] is not None and api()['odom_age'] < .5),
                 15, 'fresh sensor stream recovers after outage')
        # Exercise the actual ROS callback/gate while an explicit synthetic auto input is live.
        publish_velocity(auto, .15, .4)
        began_fault = time.monotonic()
        while time.monotonic()-began_fault < .8:
            fault = JointState()
            fault.header.stamp = node.get_clock().now().to_msg()
            fault.name, fault.position, fault.velocity = ['yaw_joint'], [.2], [.1]
            yaw_fault.publish(fault)
            command = Twist()
            command.linear.x = .15
            auto.publish(command)
            time.sleep(.01)
        after_fault = [item for item in velocities if item[0] > began_fault+.3]
        assert after_fault and all(max(abs(v) for v in item[1:]) < 1e-9 for item in after_fault), after_fault
        publish_velocity(auto, 0., .4)
        report['steps'].append('actual_ros_yaw_fault_blocks_velocity')
        api('/api/mode', {'mode':'idle'})
        wait_for(lambda: api()['mode'] == 'idle' and not api()['switching'], 30, 'navigation stopped via Web idle')
        assert all(max(abs(v) for v in item[1:]) == 0. for item in velocities[-3:])
        report['steps'].append('idle_zero_velocity')
        wait_for(lambda: len(mcu.commands) >= 3 and all(
            all(p[k] == 0. for k in ('vx','vy','wz')) for _, p in mcu.commands[-3:]),
            3, 'Web idle reaches serial zero commands')
        assert all(abs(p['yaw_target_encoder_rad']-.7) < 1e-9 for _, p in mcu.commands)
        report['serial_commands_received'] = len(mcu.commands)
        report['steps'].append('web_idle_and_yaw_target_verified_on_serial')
        for path in (maps/'web_logs').glob('*.log'):
            log = path.read_text(errors='replace')
            assert not re.search(r'process has died|failed to terminate|terminate called', log), path
        report['steps'].append('ros_mode_processes_exit_cleanly')
        report['passed'] = True
    except Exception as exc:
        report.update(passed=False, error=repr(exc), topic_counts=counts)
        raise
    finally:
        (evidence/'ros-integration.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        for name, process in reversed(processes):
            try:
                stop_process(process)
            except Exception as exc:
                print(f'Cleanup {name}: {exc}', flush=True)
        executor.shutdown()
        spin.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for handle in handles:
            handle.close()
        mcu.close()
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
