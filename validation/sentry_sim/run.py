#!/usr/bin/env python3
"""Actual Gazebo physics + Nav2 + AMCL + Shaoshao task/output integration.

LaserScan and local odometry are synthetic adapters over the simulated geometry
and body motion. This does not test MID360 rendering or FAST-LIO estimation.
"""
import argparse
from collections import deque
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import yaml

SOURCE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOURCE/'web_panel'))
from scene import make_scene, ranges_at, clearance, COMMIT
from navcar_navigation import NavigationBridge, build_navigation_config
from server import Panel


def wait_for(test, seconds, message):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        value = test()
        if value:
            return value
        time.sleep(.05)
    raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', choices=['smooth', 'narrow', 'reject', 'stuck', 'localization', 'cancel','kidnap'], default='smooth')
    parser.add_argument('--width', type=float, default=.8)
    parser.add_argument('--start-offset', type=float, default=0., help='Initial lateral offset from passage center (m)')
    parser.add_argument('--timeout', type=float, default=55.)
    parser.add_argument('--stall-stacks',action='store_true',help='Capture all thread stacks when moving output stalls')
    parser.add_argument('--state-delay',type=float,default=0.,help='Inject slow MCU status replies, leaving motion replies fast')
    parser.add_argument('--relocate-to',type=float,nargs=3,default=[4.,-3.,-1.1],metavar=('X','Y','YAW'))
    parser.add_argument('--odom-mode',choices=['blind','reset','tracked'],default='blind')
    parser.add_argument('--repeat-kidnap',type=float,nargs=3,metavar=('X','Y','YAW'))
    parser.add_argument('--cancel-after-kidnap',action='store_true')
    parser.add_argument('--hide-scan-after-kidnap',action='store_true')
    parser.add_argument('--pointcloud-localization',action='store_true',help='Use saved PCD/keyframes and synthetic 3D scans')
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('GZ_PARTITION', '').startswith('shaoshao-sentry-test-'):
        raise RuntimeError('Use run.sh in an isolated ROS domain and GZ partition')
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    from ament_index_python.packages import get_package_prefix
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.twist_pb2 import Twist as GzTwist
    from gz.msgs10.odometry_pb2 import Odometry as GzOdometry
    from gz.msgs10.pose_pb2 import Pose as GzPose
    from gz.msgs10.boolean_pb2 import Boolean as GzBoolean

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    maps = output/'maps'
    start_y=-2.+args.start_offset
    obstacles = make_scene(maps, args.width,start_y=start_y)
    cloud_fixture=None
    if args.pointcloud_localization:
        from cloud_scene import make_cloud_map,cloud_at
        cloud_fixture=make_cloud_map(maps,obstacles)
    root = output/'runtime'
    shadow = root/'shaoshao/web_panel'
    shadow.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load((SOURCE/'web_panel/robot.yaml').read_text())
    config['navigation']['max_speed_mps'] = .5
    (shadow/'robot.yaml').write_text(yaml.safe_dump(config))
    (shadow/'navcar_translation.xml').write_text((SOURCE/'web_panel/navcar_translation.xml').read_text())
    mapping = root/'mapping_runtime'
    mapping.mkdir(exist_ok=True)
    (mapping/'fast_lio.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'mapping': {'extrinsic_T': [-.011, -.02329, .04412], 'extrinsic_R': [1., 0., 0., 0., 1., 0., 0., 0., 1.]},
        'common': {}, 'pcd_save': {}, 'publish': {}, 'preprocess': {}}}}))
    (root/'navcar_move.token').write_text('isolated-simulator')
    generated = build_navigation_config(root, maps, {'x': 0., 'y': start_y, 'yaw': 0.,
        'reference_offset_xy_m': [.00318, -.02329]})
    frames = yaml.safe_load((generated/'frames.yaml').read_text())
    params = yaml.safe_load((generated/'nav2.yaml').read_text())
    amcl = params['amcl']['ros__parameters']
    amcl.update(initial_cov_xx=.0004, initial_cov_yy=.0004, initial_cov_aa=.0004,
                alpha1=.0001, alpha2=.0001, alpha3=.0001, alpha4=.0001, alpha5=.0001,
                laser_sigma_hit=.03, max_beams=180)
    # The simulator supplies LaserScan directly; no pointcloud conversion node runs.
    paramfile = generated/'nav2.yaml'
    paramfile.write_text(yaml.safe_dump(json.loads(json.dumps(params)), sort_keys=False))
    processes, logs = [], []
    rclpy.init()
    node = rclpy.create_node('shaoshao_sentry_sim')
    panel = Panel(NS(maps_dir=maps, demo=False, max_speed=.5, max_yaw_rate=.5, max_points=2000))
    panel.mode = 'navigation'
    panel.navigation_control = NS(running=True, frames=frames, max_speed=.5,
                                  mapping=NS(maps_dir=maps))
    transport = GzNode()
    command_pub = transport.advertise('/shaoshao/cmd_vel', GzTwist)
    truth = {'pose': (0., start_y, 0.), 'received': 0.,'velocity':(0.,0.)}
    truth_lock = threading.Lock()
    odometry={'transform':(0.,0.,0.),'suspended':False,'generation':0}
    relocated=False
    def pose_received(msg):
        for pose in msg.pose:
            if pose.name == 'shaoshao':
                q = pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                with truth_lock:
                    truth.update(pose=(pose.position.x, pose.position.y, yaw), received=time.monotonic())
    transport.subscribe(Pose_V, '/model/shaoshao/pose', pose_received)
    def velocity_received(msg):
        with truth_lock:
            truth['velocity']=(msg.twist.linear.x,msg.twist.linear.y)
    transport.subscribe(GzOdometry,'/shaoshao/odometry',velocity_received)
    samples, commands, states, timings, stops, callback_timings = [], [], [], [], [], []
    capturing=True
    stall_process=stall_memory=stall_stream=stall_log=None
    if args.stall_stacks:
        import faulthandler,mmap,struct
        stall_log=(output/'stall-stacks.log').open('w')
        faulthandler.register(signal.SIGUSR2,file=stall_log,all_threads=True)
        stall_stream=(output/'output-heartbeat.bin').open('w+b')
        stall_stream.truncate(16)
        stall_memory=mmap.mmap(stall_stream.fileno(),16)
        stall_process=subprocess.Popen([sys.executable,str(Path(__file__).parent/'stall_monitor.py'),
                                        str(os.getpid()),str(output/'output-heartbeat.bin')])
    mcu = {'lease': None, 'target': (0., 0.), 'at': 0., 'blocked': False}
    relay_done=threading.Event()
    def relay_commands():
        # The simulated MCU watchdog is independent of ROS callbacks, matching
        # the real lower controller. Sensor callback latency cannot hold motion.
        while not relay_done.wait(.02):
            now=time.monotonic()
            target=mcu['target'] if mcu['lease'] and now-mcu['at']<.2 and not mcu['blocked'] else (0.,0.)
            cmd=GzTwist();cmd.linear.x,cmd.linear.y=target
            command_pub.publish(cmd)
    relay_thread=threading.Thread(target=relay_commands,daemon=True)
    relay_thread.start()
    class SimBridge(NavigationBridge):
        def record_collision_stop(self,grid,pose,candidate,measured):
            path=output/'first-collision-stop.npz'
            if not path.exists():
                np.savez(path,values=grid.values,resolution=grid.resolution,origin=grid.origin,
                         pose=pose,candidate=candidate,measured=measured,pending=self.pending_velocity)
            path=output/'passage-collision-stop.npz'
            if pose[0]>.6 and time.monotonic()-grid.stamp<.4 and not path.exists():
                np.savez(path,values=grid.values,resolution=grid.resolution,origin=grid.origin,
                         pose=pose,candidate=candidate,measured=measured,pending=self.pending_velocity)
        def record_output_timing(self,values):
            timings.append(values)
        def request(self, path, data=None):
            if path == '/api/state':
                if args.state_delay:time.sleep(args.state_delay)
                return {'connected': True, 'error': '', 'control_source': 'navigation' if mcu['lease'] else 'manual'}
            if path == '/api/navigation/acquire':
                mcu['lease'] = 'test-lease'
                return {'lease': mcu['lease']}
            if path == '/api/navigation/release':
                mcu.update(lease=None, target=(0., 0.))
                return {}
            if path == '/api/navigation/command':
                if data['lease'] != mcu['lease']:
                    raise RuntimeError('Simulator lease revoked')
                tiny_settled=(math.hypot(*mcu['target'])<1e-5 and
                              (not commands or math.hypot(*commands[-1][4:6])<1e-4))
                if (capturing and panel.nav_status!='succeeded' and math.hypot(data['forward'],data['left'])==0.
                        and math.hypot(*mcu['target'])>1e-8 and not tiny_settled):
                    now=time.monotonic()
                    stops.append({'time':now,'reason':self.smoother.reset_reason,'status':panel.nav_status,
                                  'motion':self.motion_status,'odom_age':now-panel.odom_time,
                                  'grid_age':now-self.local_grid.stamp if self.local_grid else None,
                                  'localization_age':now-self.localization_at,'scan_age':now-self.match_at})
                mcu.update(target=(data['forward'], data['left']), at=time.monotonic())
                if stall_memory is not None:
                    struct.pack_into('dd',stall_memory,0,time.monotonic(),float(math.hypot(*mcu['target'])>1e-8))
                if capturing:
                    commands.append((time.monotonic(), *mcu['target'], self.smoother.sample_time or 0.,
                                     *self.smoother.a))
                return {}
            raise RuntimeError('Unexpected hardware endpoint')
    def measured_callback(name, method):
        def invoke(self,*values,**kwargs):
            begin=time.monotonic();cpu=time.thread_time()
            try:
                return method(self,*values,**kwargs)
            finally:
                callback_timings.append((begin,name,time.monotonic()-begin,time.thread_time()-cpu))
        return invoke
    for name in ('tick','odom','localized','command','scan_match','raw_costmap','costmap','checked_velocity','final_velocity'):
        setattr(SimBridge,name,measured_callback(name,getattr(NavigationBridge,name)))
    bridge = SimBridge(node, panel, root)
    panel.bridge = bridge
    odom_pub = node.create_publisher(Odometry, '/navcar/state_estimation', 10)
    scan_pub = node.create_publisher(LaserScan, '/navcar/scan', 10)
    if args.pointcloud_localization:
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
        from std_msgs.msg import Header
        cloud_pub=node.create_publisher(PointCloud2,'/cloud_registered_body',10)
    tf_pub, static_pub = TransformBroadcaster(node), StaticTransformBroadcaster(node)
    static = []
    for item in frames['static_transforms']:
        tf = TransformStamped()
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = 'navcar_imu', item['child']
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = item['xyz']
        tf.transform.rotation.w = 1.
        static.append(tf)
    static_pub.sendTransform(static)
    history = deque(maxlen=200)
    def sensor_tick():
        now, stamp = time.monotonic(), node.get_clock().now().to_msg()
        with truth_lock:
            pose, received, velocity = truth['pose'], truth['received'],truth['velocity']
            correction=odometry['transform'];suspended=odometry['suspended']
            generation=odometry['generation']
        if suspended or not received or now-received > .3:
            return
        tx,ty,heading=correction;gc,gs=math.cos(heading),math.sin(heading)
        local=(tx+gc*pose[0]-gs*pose[1],ty+gs*pose[0]+gc*pose[1],pose[2]+heading)
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id = stamp, 'navcar_odom'
        msg.child_frame_id = 'navcar_imu'
        c, s = math.cos(local[2]), math.sin(local[2])
        ox, oy = frames['center_offset'][:2]
        msg.pose.pose.position.x = local[0]-c*ox+s*oy
        msg.pose.pose.position.y = local[1]-s*ox-c*oy
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(local[2]/2), math.cos(local[2]/2)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = velocity
        odom_pub.publish(msg)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = 'navcar_imu'
        tf.transform.translation.x, tf.transform.translation.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        tf.transform.rotation = msg.pose.pose.orientation
        tf_pub.sendTransform(tf)
        history.append((stamp, pose,generation))
        panel.yaw_guard.update(0., 0.)
        if capturing:
            samples.append((now, *pose, *velocity, clearance(pose, obstacles)))
    def scan_tick():
        if len(history) < 4:
            return
        stamp, pose, generation = history[-4]
        if generation!=odometry['generation']:return
        # 60 ms delayed, correctly stamped measurement; TF must use that sample.
        c, s = math.cos(pose[2]), math.sin(pose[2])
        optical = (pose[0]-.01418*c, pose[1]-.01418*s, pose[2])
        msg = LaserScan()
        msg.header.stamp, msg.header.frame_id = stamp, 'navcar_lidar'
        msg.angle_min, msg.angle_max, msg.angle_increment = -math.pi, math.pi, math.pi/360
        msg.range_min, msg.range_max, msg.scan_time = .2, 15., .1
        msg.ranges = [float(v) for v in ranges_at(optical, obstacles)]
        if relocated and args.hide_scan_after_kidnap:msg.ranges=[math.inf]*len(msg.ranges)
        scan_pub.publish(msg)
        if args.pointcloud_localization:
            points=cloud_at(optical,obstacles)
            # Production FAST-LIO body output uses the IMU frame. Respect the
            # measured optical->IMU extrinsic; no map pose enters this message.
            points+=np.asarray(frames['lidar_to_imu_xyz'])
            if relocated and args.hide_scan_after_kidnap:points=np.empty((0,3))
            cloud_pub.publish(create_cloud_xyz32(Header(stamp=stamp,frame_id='navcar_imu'),points))
        panel.cloud_time = time.monotonic()
    # Independent producers must not share the bridge's mutually exclusive
    # callback group; real FAST-LIO and the LiDAR driver are separate processes.
    motion_group,scan_group=MutuallyExclusiveCallbackGroup(),MutuallyExclusiveCallbackGroup()
    node.create_timer(.02, sensor_tick,callback_group=motion_group)
    node.create_timer(.1, scan_tick,callback_group=scan_group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    def launch(cmd, name):
        log = (output/(name+'.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process
    result = {'case': args.case, 'width_m': args.width, 'start_offset_m':args.start_offset,
              'state_delay_s':args.state_delay,'upstream_commit': COMMIT,
              'odometry_fault_mode':args.odom_mode if args.case=='kidnap' else None,
              'simulator': 'Gazebo Harmonic, original room.world wall collisions',
              'sensor_adapter': 'Analytic LaserScan + synthetic local odometry; no FAST-LIO or MID360 validation',
              'passed': False}
    result['localization_backend']=frames['localization_backend']
    if cloud_fixture:result['pointcloud_fixture']=cloud_fixture
    result['source_sha256']={name:hashlib.sha256((SOURCE/name).read_bytes()).hexdigest() for name in
        ['web_panel/navigation_runtime.py','web_panel/navigation_recovery.py','web_panel/navcar_navigation.py',
         'web_panel/global_relocalization.py','web_panel/pointcloud_relocalization.py','web_panel/pointcloud_index.py',
         'web_panel/server.py','deployment/lidar_mapping_web.py',
         'validation/sentry_sim/scene.py','validation/sentry_sim/cloud_scene.py','validation/sentry_sim/run.py']}
    (output/'input-source.json').write_text(json.dumps(result,indent=2)+'\n')
    started = time.monotonic()
    try:
        launch([sys.executable, str(Path(__file__).parent/'gz_server.py'), str(maps/'scene.sdf')], 'gazebo')
        wait_for(lambda: truth['received'], 15., 'No actual Gazebo body pose received')
        for package, executable, name in [('nav2_map_server', 'map_server', 'map_server'),
                ('nav2_amcl', 'amcl', 'amcl'), ('nav2_planner', 'planner_server', 'planner_server'),
                ('nav2_controller', 'controller_server', 'controller_server'),
                ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator')]:
            binary = Path(get_package_prefix(package))/'lib'/package/executable
            launch([str(binary), '--ros-args', '-r', '__node:='+name, '--params-file', str(paramfile),
                    '-r', 'cmd_vel:=/navcar/cmd_vel_auto'], name)
        lifecycle = output/'lifecycle.yaml'
        lifecycle.write_text(yaml.safe_dump({'navigation_lifecycle': {'ros__parameters': {
            'autostart': True, 'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator']}}}))
        package = 'nav2_lifecycle_manager'
        launch([str(Path(get_package_prefix(package))/'lib'/package/'lifecycle_manager'), '--ros-args',
                '-r', '__node:=navigation_lifecycle', '--params-file', str(lifecycle)], 'lifecycle')
        wait_for(lambda: not panel.navigation_reason() and bridge.local_grid, 30., 'Navigation/localization gate not ready')
        revision = panel.saved_map()[0]['revision']
        goal_x = 2. if args.case == 'reject' else 4.
        panel.send_navigation_goal(goal_x, -2., revision)
        goal = dict(panel.nav_goal)
        goal_start = time.monotonic()
        injected = restored = False
        relocation_events=[];global_states=set();global_trace=[];cancel_at=0.;release_actuators_at=0.
        def relocate(destination):
            nonlocal relocated,release_actuators_at
            mcu['blocked']=True
            with panel.lock:
                previous=dict(panel.pose)
            with truth_lock:odometry['suspended']=True
            request=GzPose();request.name='shaoshao'
            request.position.x,request.position.y,request.position.z=destination[0],destination[1],.4
            request.orientation.z,request.orientation.w=math.sin(destination[2]/2),math.cos(destination[2]/2)
            ok=False;request_transport=GzNode()
            def arrived():
                with truth_lock:current=truth['pose']
                return (math.hypot(current[0]-destination[0],current[1]-destination[1])<.02 and
                        abs(math.atan2(math.sin(current[2]-destination[2]),math.cos(current[2]-destination[2])))<.02)
            for _ in range(3):
                ok,response=request_transport.request('/world/shaoshao_test/set_pose',request,GzPose,GzBoolean,2000)
                time.sleep(.03)
                if arrived():break
            wait_for(arrived,2.,'Actual Gazebo body did not teleport')
            desired=(0.,0.,0.) if args.odom_mode=='reset' else (previous['x'],previous['y'],previous['yaw'])
            if args.odom_mode=='tracked':transform=(0.,0.,0.)
            else:
                a=desired[2]-destination[2];c,s=math.cos(a),math.sin(a)
                transform=(desired[0]-c*destination[0]+s*destination[1],desired[1]-s*destination[0]-c*destination[1],a)
            with truth_lock:
                odometry.update(transform=transform,suspended=False,generation=odometry['generation']+1)
                history.clear()
            relocated=True
            at=time.monotonic();release_actuators_at=at+.3
            relocation_events.append({'time':at,'destination':list(destination),'odometry_mode':args.odom_mode,
                                      'generation_before':bridge.global_generation})
        seen_states = set()
        deadline = goal_start+args.timeout
        while time.monotonic() < deadline:
            now = time.monotonic()
            state = bridge.task.state if bridge.task else panel.nav_status
            seen_states.add(state)
            states.append((now, state, panel.nav_status, panel.nav_message, bridge.match_ratio,bridge.motion_status))
            with truth_lock:
                actual = truth['pose']
            global_state=bridge.global_status.get('state','idle');global_states.add(global_state)
            global_trace.append({'time':now,**bridge.global_status})
            if args.case=='kidnap' and not injected and actual[0]>.4:
                relocate(args.relocate_to)
                fault_at,injected=relocation_events[-1]['time'],True
            if release_actuators_at and now>=release_actuators_at:
                mcu['blocked']=False;release_actuators_at=0.
            if args.case=='kidnap' and relocation_events:
                event=relocation_events[-1]
                if (global_state=='verified' and panel.nav_localized and bridge.global_generation>event['generation_before']
                        and now>event['time'] and 'verified_at' not in event):
                    event.update(verified_at=now,position_error_m=math.hypot(panel.map_pose['x']-actual[0],panel.map_pose['y']-actual[1]),
                                 yaw_error_rad=abs(math.atan2(math.sin(panel.map_pose['yaw']-actual[2]),math.cos(panel.map_pose['yaw']-actual[2]))))
                if args.repeat_kidnap and len(relocation_events)==1 and event.get('verified_at') and now-event['verified_at']>1.5:
                    relocate(args.repeat_kidnap)
                if args.cancel_after_kidnap and not cancel_at and bridge.global_locator.future is not None:
                    panel.cancel_navigation();cancel_at=now
            if args.case == 'stuck' and not injected and actual[0] > .15:
                mcu['blocked'] = True
                fault_at, injected = now, True
            if args.case == 'stuck' and injected and not restored and now-fault_at > 8.:
                mcu['blocked'], restored = False, True
            if args.case == 'localization' and not injected and actual[0] > .4:
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.pose.pose.position.x, msg.pose.pose.position.y = actual[0]+1., actual[1]+1.
                msg.pose.pose.orientation.w = 1.
                msg.pose.covariance[0] = msg.pose.covariance[7] = .0004
                msg.pose.covariance[35] = .0004
                bridge.initial_pose_pub.publish(msg)
                fault_at, injected = now, True
            if args.case == 'cancel' and not injected and actual[0] > .2:
                panel.cancel_navigation()
                fault_at, injected = now, True
            if panel.nav_status == 'succeeded':
                break
            if args.case == 'reject' and now-goal_start > 12.:
                break
            if args.case == 'cancel' and injected and now-fault_at > 3.:
                break
            if args.case=='kidnap' and args.cancel_after_kidnap and cancel_at and now-cancel_at>5.:break
            if args.case=='kidnap' and args.hide_scan_after_kidnap and injected and now-fault_at>12.:break
            if any(process.poll() is not None for process in processes):
                raise AssertionError('Required simulation/navigation process exited')
            time.sleep(.05)
        with truth_lock:
            final_pose = truth['pose']
        capturing=False
        result.update(status=panel.nav_status, final_pose=final_pose, recovery_states=sorted(seen_states),
                      original_goal=goal, final_goal=panel.nav_goal, injected=injected,
                      elapsed_s=time.monotonic()-goal_start,
                      min_physical_clearance_m=min(row[-1] for row in samples))
        result['safety_stops']=stops
        result['global_states']=sorted(global_states)
        if args.case=='kidnap':result['relocations']=relocation_events
        passage=np.asarray([row for row in samples if 1.3<row[1]<2.7])
        if len(passage):
            result['passage']={'max_center_offset_m':float(np.abs(passage[:,2]+2.).max()),
                               'max_body_speed_mps':float(np.linalg.norm(passage[:,4:6],axis=1).max()),
                               'min_physical_clearance_m':float(passage[:,-1].min())}
        if commands:
            trace = np.asarray(commands)
            # Nonuniform wall-clock derivatives; stops for safety are reported separately.
            dt = np.diff(trace[:, 0])
            acceleration = np.diff(trace[:, 1:3], axis=0)/dt[:, None]
            jerk = np.diff(acceleration, axis=0)/((dt[1:]+dt[:-1])/2)[:, None]
            result['motion'] = {'max_speed_mps': float(np.linalg.norm(trace[:, 1:3], axis=1).max()),
                'max_acceleration_mps2': float(np.linalg.norm(acceleration, axis=1).max()) if len(acceleration) else 0.,
                'max_jerk_mps3': float(np.linalg.norm(jerk, axis=1).max()) if len(jerk) else 0.}
            valid=np.ones(len(dt),dtype=bool)
            for stop in stops:
                # Exclude only intervals crossing an explicitly recorded safety
                # stop, including the neighbouring jerk stencil. Raw peaks stay.
                valid &= ~((trace[:-1,0]<=stop['time']+.04)&(trace[1:,0]>=stop['time']-.04))
            normal_jerk=valid[:-1]&valid[1:]
            result['motion']['normal_max_acceleration_mps2']=float(np.linalg.norm(acceleration[valid],axis=1).max()) if valid.any() else 0.
            result['motion']['normal_max_jerk_mps3']=float(np.linalg.norm(jerk[normal_jerk],axis=1).max()) if normal_jerk.any() else 0.
            result['uninterrupted_motion']=not bool(stops)
        assert result['min_physical_clearance_m'] >= -.002, 'Physical wall contact/penetration'
        assert result['final_goal'] == goal, 'Original navigation target changed'
        if args.case == 'reject':
            assert panel.nav_status != 'succeeded' and final_pose[0] < 1., 'Undersized passage was entered'
        elif args.case == 'cancel' or (args.case=='kidnap' and args.cancel_after_kidnap):
            assert injected and bridge.task is None and panel.nav_status == 'canceled', 'Cancel did not revoke recovery'
        elif args.case=='kidnap' and args.hide_scan_after_kidnap:
            assert injected and panel.nav_status!='succeeded' and bridge.task is not None,'Insufficient scan did not retain the target'
        else:
            assert panel.nav_status == 'succeeded', 'Goal not reached: '+panel.nav_message
            assert math.hypot(final_pose[0]-goal_x, final_pose[1]+2) < .25, 'Simulator truth did not reach goal'
        if args.case == 'stuck':
            assert 'recovering' in seen_states and injected, 'Stuck recovery was not exercised'
        if args.case == 'localization':
            assert 'relocalizing' in seen_states and injected, 'Automatic relocalization was not exercised'
        if args.case=='kidnap':
            assert injected and 'relocalizing' in seen_states,'Kidnap recovery was not exercised'
            for event in relocation_events:
                end=event.get('verified_at',time.monotonic())
                unsafe=[row for row in commands if event['time']+.3<row[0]<end and math.hypot(row[1],row[2])>.005]
                assert not unsafe,'Motion was authorized before global localization verification'
            if not args.cancel_after_kidnap and not args.hide_scan_after_kidnap:
                assert len(relocation_events)==(2 if args.repeat_kidnap else 1),'Repeated relocation was not exercised'
                assert all(e.get('verified_at') and e['position_error_m']<.15 and e['yaw_error_rad']<.15 for e in relocation_events),'Recovered global pose did not match physical pose'
        if args.case in ('smooth', 'narrow'):
            assert result['motion']['normal_max_acceleration_mps2'] <= .33, 'Normal acceleration continuity failed'
            assert result['motion']['normal_max_jerk_mps3'] <= 1.2, 'Normal jerk bound failed'
        if args.case=='narrow' and args.width<=.65:
            assert 'passage' in result, 'No actual body samples inside the throat'
            assert result['passage']['max_body_speed_mps']<=.12, 'Narrow passage did not slow down'
            assert result['passage']['max_center_offset_m']<=.02, 'Narrow passage centering failed'
        result['passed'] = True
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
        result['debug'] = {'status': panel.nav_status, 'message': panel.nav_message,
                           'reason': panel.navigation_reason(), 'localization': bridge.localization_info,
                           'recovery': bridge.recovery_status(), 'local_grid': bridge.local_grid is not None}
    finally:
        if stall_memory is not None:
            struct.pack_into('dd',stall_memory,0,time.monotonic(),0.)
        panel.cancel_navigation()
        bridge.close()
        executor.shutdown(timeout_sec=3.)
        spin.join(3.)
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.)
        for log in logs:
            log.close()
        relay_done.set();relay_thread.join(1.)
        node.destroy_node()
        rclpy.shutdown()
        if stall_process is not None:
            stall_process.terminate();stall_process.wait(timeout=3.)
            faulthandler.unregister(signal.SIGUSR2)
            stall_memory.close();stall_stream.close();stall_log.close()
        for filename, rows in [('body.csv', samples), ('commands.csv', commands), ('states.csv', states),
                               ('timing.csv',timings),('callback-timing.csv',callback_timings)]:
            with (output/filename).open('w', newline='') as stream:
                csv.writer(stream).writerows(rows)
        if 'global_trace' in locals():
            (output/'global-localization.jsonl').write_text(''.join(json.dumps(row,ensure_ascii=False)+'\n' for row in global_trace))
        (output/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
