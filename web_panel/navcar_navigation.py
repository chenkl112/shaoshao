"""LiDAR-localized, rotation-free Nav2 adapter for the Navcar UART controller.

Navigation uses the measured IMU pose or a calibrated joint-axis reference,
with a circle enclosing every chassis yaw. No MCU target echo is used as pose.
"""
import json
import math
from pathlib import Path
import shlex
import subprocess
import threading
import time
from urllib.request import Request, urlopen

import yaml
from robot_io import finite_number, sample_monotonic_time
from navcar_settings import navigation_speed, limit_translation
from navcar_reference import navigation_frames,shift_pose,initial_reference_pose,shift_planar_covariance


def localization_quality(frame, covariance,position_std_limit=.01):
    if frame!='map' or len(covariance)!=36:
        return {'ready':False,'message':'地图定位消息无效，等待有效的 map 定位结果'}
    values=[covariance[i] for i in (0,7,35)]
    # AMCL's weighted second moments can round a zero variance just below zero.
    # Tolerate only numerical roundoff; genuinely negative/large variances remain invalid.
    if not all(math.isfinite(v) and v>=-1e-12 for v in values):
        return {'ready':False,'message':'地图定位方差无效，等待新的定位结果'}
    deviations=[math.sqrt(max(0.,v)) for v in values]
    ready=all(v<=position_std_limit**2 for v in values[:2]) and values[2]<=.09
    message=('地图定位就绪' if ready else
             f'定位尚未收敛：位置不确定度 {max(deviations[:2]):.2f} 米，方向 {math.degrees(deviations[2]):.1f}°；请在建图结束位置附近缓慢移动以匹配地图')
    if not ready:message+=f'；导航要求位置标准差 ≤ {position_std_limit*1000:g} mm'
    return {'ready':ready,'message':message,'position_std_m':max(deviations[:2]),'yaw_std_rad':deviations[2]}


def navigation_geometry(config,lidar_to_imu_xyz=None):
    robot,yaw=config['robot'],config['yaw']
    for key in ('lidar_height_m',):
        if not finite_number(robot.get(key)) or not 0 < robot[key] <= 10:
            raise ValueError('请填写有效的 robot.'+key)
    measured_radius=robot.get('circumscribed_radius_m')
    if measured_radius is not None:
        if not finite_number(measured_radius) or not 0 < measured_radius <= 10:
            raise ValueError('请填写有效的 robot.circumscribed_radius_m')
        chassis_radius=measured_radius
    else:
        for key in ('length_m','width_m'):
            if not finite_number(robot.get(key)) or not 0 < robot[key] <= 10:
                raise ValueError('请填写有效的 robot.'+key)
        chassis_radius=math.hypot(robot['length_m']/2,robot['width_m']/2)
    optical_offset=yaw.get('lidar_offset_radius_m')
    keys=('axis_xyz_m','imu_rpy_rad') if optical_offset is not None else ('axis_xyz_m','imu_xyz_m','imu_rpy_rad')
    for key in keys:
        values=yaw.get(key)
        count=2 if key!='imu_rpy_rad' else 3
        if not isinstance(values,list) or len(values)!=3 or not all(finite_number(v) for v in values[:count]):
            raise ValueError('请填写 yaw.'+key+' 的平面位置/朝向')
    if max(abs(v) for v in yaw['imu_rpy_rad'][:2])>.05:
        raise ValueError('当前二维导航要求雷达水平安装')
    # Any chassis yaw and sensor yaw joint angle fit in this enclosing circle.
    if optical_offset is not None:
        if not finite_number(optical_offset) or not 0 <= optical_offset <= 10:
            raise ValueError('请填写有效的 yaw.lidar_offset_radius_m')
        if not isinstance(lidar_to_imu_xyz,(list,tuple)) or len(lidar_to_imu_xyz)!=3 or not all(finite_number(v) for v in lidar_to_imu_xyz):
            raise ValueError('雷达光学中心偏心模式需要有效的 mapping.extrinsic_T')
        # The legacy IMU reference uses triangle bounds, without needing an
        # offset direction, to enclose every chassis/joint angle.
        sensor_offset=optical_offset+math.hypot(*lidar_to_imu_xyz[:2])
    else:
        sensor_offset=math.hypot(*yaw['imu_xyz_m'][:2])
    radius=chassis_radius+math.hypot(*yaw['axis_xyz_m'][:2])+sensor_offset
    if config.get('navigation',{}).get('reference_point')=='yaw_axis':
        # The frame origin is on the actual joint axis; its orientation follows
        # the sensor. The circle covers every chassis heading about that axis.
        radius=chassis_radius+math.hypot(*yaw['axis_xyz_m'][:2])
    return radius,robot['lidar_height_m']


def build_navigation_config(root, maps, initial_pose=None):
    root,maps=Path(root),Path(maps)
    repo=root/'shaoshao/web_panel'
    config=yaml.safe_load((repo/'robot.yaml').read_text())
    speed=navigation_speed(config)
    if not (maps/'site.yaml').is_file():raise ValueError('请先完成建图并保存地图')
    output=root/'navigation_runtime';output.mkdir(exist_ok=True)
    lio=yaml.safe_load((root/'mapping_runtime/fast_lio.yaml').read_text())
    p=lio['/**']['ros__parameters']
    radius,height=navigation_geometry(config,p['mapping']['extrinsic_T'])
    frames=navigation_frames(config,p);base=frames['base_frame']
    metadata=yaml.safe_load((maps/'site.yaml').read_text())
    resolution=metadata.get('resolution') if isinstance(metadata,dict) else None
    if not finite_number(resolution) or not 0 < resolution <= frames['resolution']+1e-9:
        raise ValueError('导航需要新建并保存的 1 cm 地图；不能仅修改旧地图 YAML 的分辨率')
    p['preprocess']={**p.get('preprocess',{}),'blind':frames['min_range']}
    (output/'frames.yaml').write_text(yaml.safe_dump(frames,sort_keys=False))
    p['common'].update(odom_frame_id='navcar_odom',sensor_frame_id='navcar_imu',send_odom_base_tf=False)
    p['pcd_save']['pcd_save_en']=False
    # Static export is only needed while mapping. Keep live scans/obstacle
    # detection, but do not maintain the finer export voxels during navigation.
    p.setdefault('map_filter',{})['dynamic_enabled']=False
    p['locate_in_prior_map']=False
    p['publish'].update(scan_bodyframe_pub_en=True,map_en=False)
    p['map_file_path']=str(output/'navigation-unused.pcd')
    (output/'fast_lio.yaml').write_text(yaml.safe_dump(lio,sort_keys=False))
    initial_pose=initial_reference_pose(initial_pose,frames['center_offset'])
    min_range=frames['min_range'];resolution=frames['resolution']
    common={'use_sim_time':False}
    obstacle={'plugin':'nav2_costmap_2d::ObstacleLayer','enabled':True,'observation_sources':'scan',
              'scan':{'topic':'/navcar/scan','data_type':'LaserScan','clearing':True,'marking':True,
                      'max_obstacle_height':2.,'raytrace_max_range':15.,'raytrace_min_range':min_range,
                      'obstacle_max_range':10.,'obstacle_min_range':min_range,'inf_is_valid':False}}
    inflation={'plugin':'nav2_costmap_2d::InflationLayer','cost_scaling_factor':5.,'inflation_radius':radius+.2}
    local={**common,'global_frame':'navcar_odom','robot_base_frame':base,'update_frequency':10.,
           'publish_frequency':1.,'rolling_window':True,'width':2,
           'height':2,'resolution':resolution,
           'robot_radius':radius,'footprint_padding':.02,'transform_tolerance':.3,
           'plugins':['obstacle_layer','inflation_layer'],'obstacle_layer':obstacle,'inflation_layer':inflation}
    glob={**local,'global_frame':'map','rolling_window':False,'update_frequency':1.,'resolution':resolution,
          'track_unknown_space':True,'plugins':['static_layer','obstacle_layer','inflation_layer'],
          'static_layer':{'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True}}
    params={
      'navigation_scan':{'ros__parameters':{**common,'target_frame':'navcar_lidar' if base=='navcar_center' else 'navcar_imu','transform_tolerance':.1,
        'min_height':.10-(height if base=='navcar_center' else height-p['mapping']['extrinsic_T'][2]),'max_height':1.2,
        'angle_min':-math.pi,'angle_max':math.pi,'angle_increment':math.pi/360,
        'scan_time':.1,'range_min':min_range,'range_max':15.,'use_inf':True}},
      'map_server':{'ros__parameters':{**common,'yaml_filename':str(maps/'site.yaml'),'frame_id':'map'}},
      'amcl':{'ros__parameters':{**common,'global_frame_id':'map','odom_frame_id':'navcar_odom',
        'base_frame_id':base,'scan_topic':'/navcar/scan','robot_model_type':'nav2_amcl::OmniMotionModel',
        'min_particles':500,'max_particles':2000,'max_beams':120,'laser_min_range':min_range,'laser_max_range':15.,
        'laser_model_type':'likelihood_field','transform_tolerance':.3,'tf_broadcast':True,
        'update_min_d':.02,'update_min_a':.02,'resample_interval':1,'set_initial_pose':True,
        'initial_pose':{'x':float(initial_pose['x']),'y':float(initial_pose['y']),'z':0.,'yaw':float(initial_pose['yaw'])}}},
      'planner_server':{'ros__parameters':{**common,'expected_planner_frequency':1.,'planner_plugins':['GridBased'],
        'GridBased':{'plugin':'nav2_navfn_planner::NavfnPlanner','tolerance':.02,'use_astar':True,'allow_unknown':False}}},
      'controller_server':{'ros__parameters':{**common,'controller_frequency':20.,'odom_topic':'/navcar/odom',
        'enable_stamped_cmd_vel':False,'min_x_velocity_threshold':.001,'min_y_velocity_threshold':.001,
        'min_theta_velocity_threshold':.001,'failure_tolerance':.3,'progress_checker_plugins':['progress_checker'],
        'goal_checker_plugins':['goal_checker'],'controller_plugins':['FollowPath'],
        'progress_checker':{'plugin':'nav2_controller::SimpleProgressChecker','required_movement_radius':.01,'movement_time_allowance':20.},
        'goal_checker':{'plugin':'nav2_controller::SimpleGoalChecker','xy_goal_tolerance':.03,'yaw_goal_tolerance':6.3,'stateful':True},
        'FollowPath':{'plugin':'dwb_core::DWBLocalPlanner','min_vel_x':-speed,'max_vel_x':speed,
          'min_vel_y':-speed,'max_vel_y':speed,'min_speed_xy':0.,'max_speed_xy':speed,'min_speed_theta':0.,'max_vel_theta':0.,
          'acc_lim_x':.3,'acc_lim_y':.3,'acc_lim_theta':.3,'decel_lim_x':-.3,'decel_lim_y':-.3,'decel_lim_theta':-.3,
          'vx_samples':9,'vy_samples':9,'vtheta_samples':1,'sim_time':2.,'linear_granularity':.005,
          'angular_granularity':.025,'transform_tolerance':.3,'xy_goal_tolerance':.03,'trans_stopped_velocity':.01,
          'short_circuit_trajectory_evaluation':True,'stateful':True,
          # At the legacy 0.1 m/s the horizon covers only 0.15 m. A .1 obstacle weight
          # makes entering even the outer inflation band cost more than progress,
          # so DWB repeatedly selects zero. Keep collision rejection enabled while
          # balancing nonlethal inflation against this short, slow trajectory.
          'critics':['BaseObstacle','PathDist','GoalDist'],'BaseObstacle.scale':.001,'PathDist.scale':32.,'GoalDist.scale':24.}}},
      'local_costmap':{'local_costmap':{'ros__parameters':local}},
      'global_costmap':{'global_costmap':{'ros__parameters':glob}},
      'bt_navigator':{'ros__parameters':{**common,'global_frame':'map','robot_base_frame':base,
        'odom_topic':'/navcar/odom','bt_loop_duration':20,'default_server_timeout':1000,
        'wait_for_service_timeout':5000,'navigators':['navigate_to_pose'],
        'navigate_to_pose':{'plugin':'nav2_bt_navigator::NavigateToPoseNavigator'},
        'default_nav_to_pose_bt_xml':str(repo/'navcar_translation.xml')}}}
    # ROS parameter YAML does not accept YAML anchors/aliases.
    (output/'nav2.yaml').write_text(yaml.safe_dump(json.loads(json.dumps(params)),sort_keys=False))
    return output


class NavigationControl:
    def __init__(self,root,panel,mapping):
        self.root,self.panel,self.mapping=Path(root),panel,mapping
        self.starting=self.stopping=False
        self.error='';self.process=None;self.worker=None
        self.initial_pose=None
        self.max_speed=.5
        self.frames={'base_frame':'navcar_imu','center_offset':[0.,0.,0.],'position_std_limit':.01}

    @property
    def running(self):return self.process is not None and self.process.poll() is None
    @property
    def active(self):return self.running or self.starting or self.stopping

    def request_start(self):
        with self.panel.lock:
            if self.starting or self.running:return
            if self.stopping or self.mapping.starting:raise RuntimeError('正在切换模式，请稍候')
            self.starting=True;self.error=''
            saved=self.mapping.maps_dir/'last_pose.json'
            self.initial_pose=dict(self.panel.pose) if self.panel.pose else None
            if self.initial_pose is None and saved.is_file():
                try:self.initial_pose=json.loads(saved.read_text())
                except (ValueError,OSError):pass
            self.worker=threading.Thread(target=self._start,daemon=True);self.worker.start()

    def _start(self):
        try:
            from ament_index_python.packages import get_package_prefix
            for name in ('nav2_controller','nav2_planner','nav2_amcl','nav2_bt_navigator','pointcloud_to_laserscan'):
                get_package_prefix(name)
            output=build_navigation_config(self.root,self.mapping.maps_dir,self.initial_pose)
            params=yaml.safe_load((output/'nav2.yaml').read_text())
            self.max_speed=params['controller_server']['ros__parameters']['FollowPath']['max_speed_xy']
            self.frames=yaml.safe_load((output/'frames.yaml').read_text())
            if self.mapping.is_running():
                self.mapping.request_stop()
            deadline=time.monotonic()+60
            while self.mapping.stopping and time.monotonic()<deadline:time.sleep(.1)
            if self.mapping.error or self.mapping.is_running() or self.mapping.stopping:
                raise RuntimeError(self.mapping.error or '建图尚未停止，无法启动导航')
            with self.panel.lock:
                self.panel.nav_localized=False;self.panel.map_pose_time=0.;self.panel.commands.clear()
                if self.panel.bridge:self.panel.bridge.previous_odom=None
                if self.panel.bridge:self.panel.bridge.localization_info={'ready':False,'message':'等待雷达匹配已保存地图'}
                self.panel.clear_navigation_plan();self.panel.navigation_costmap={'available':False}
                self.panel.cloud_time=self.panel.odom_time=0.
            setup='source /opt/ros/jazzy/setup.bash\nsource "$HOME/Zzj_slam/ws/out/install-livox/setup.bash"\nsource "$HOME/ccc/mapping_install/setup.bash"\n'
            cmd=['ros2','launch',str(self.root/'shaoshao/web_panel/navcar_navigation.launch.py')]
            with (self.root/'navigation.log').open('a') as log:
                self.process=subprocess.Popen(['bash','-c',setup+'exec '+shlex.join(cmd)],stdin=subprocess.DEVNULL,
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            (self.root/'navigation.pid').write_text(str(self.process.pid)+'\n')
            time.sleep(2)
            if not self.running:raise RuntimeError('导航启动失败，请查看 ~/ccc/navigation.log')
        except Exception as e:self.error=str(e)
        finally:self.starting=False

    def request_stop(self,start_mapping=False):
        with self.panel.lock:
            if self.starting:raise RuntimeError('正在启动导航，请稍候')
            if self.stopping:return
            self.stopping=True
            if self.panel.map_pose:
                try:(self.mapping.maps_dir/'last_pose.json').write_text(json.dumps(self.panel.map_pose))
                except OSError:pass
            self.panel.cancel_navigation();self.panel.nav_hold=True;self.panel.commands.clear()
            self.panel.clear_navigation_plan();self.panel.navigation_costmap={'available':False}
            self.worker=threading.Thread(target=self._stop,args=(start_mapping,),daemon=True);self.worker.start()

    def _stop(self,start_mapping=False):
        from server import stop_process
        try:
            if self.panel.bridge:self.panel.bridge.release()
            stop_process(self.process);self.process=None
            with self.panel.lock:
                self.panel.nav_serial+=1;self.panel.nav_status='idle'
                self.panel.nav_message='请在地图上选择目标位置'
                self.panel.nav_goal=self.panel.nav_handle=None
                self.panel.nav_localized=False;self.panel.nav_server_ready=False
                self.panel.map_pose=None;self.panel.map_pose_time=0.;self.panel.pose=None
            if start_mapping:self.mapping.request_start()
        except Exception as e:self.error=str(e)
        finally:self.stopping=False

    def reason(self):
        p=self.panel
        if self.error:return self.error
        if self.starting:return '正在启动地图定位和导航…'
        if self.stopping:return '正在停止导航…'
        if not self.running:return '请先点击顶部“导航”，启动已保存地图的定位和路径规划'
        if not p.nav_server_ready:return '等待 Nav2 导航服务就绪'
        if not p.nav_localized:
            return getattr(p.bridge,'localization_info',{}).get('message') or '等待雷达匹配已保存地图；请将车放回建图结束位置附近'
        if not p.map_pose or time.monotonic()-p.map_pose_time>1.:
            return '地图位置更新超时，等待雷达定位坐标变换恢复'
        if not p.cloud_time or not p.odom_time or time.monotonic()-p.cloud_time>1. or time.monotonic()-p.odom_time>1.:
            return '点云或雷达里程计已超时'
        bridge=p.bridge
        if bridge is None or time.monotonic()-bridge.serial_checked>1. or not bridge.serial_ready:
            return '等待下位机通讯，请确认8081控制服务已运行'
        return ''

    def close(self):
        if self.worker and self.worker.is_alive():self.worker.join(65)
        self._stop()


class NavigationBridge:
    def __init__(self,node,panel,root):
        from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry,Path as NavPath,OccupancyGrid
        from nav2_msgs.action import NavigateToPose,ComputePathToPose
        from rclpy.action import ActionClient
        from tf2_ros import Buffer,TransformListener
        self.node,self.panel,self.root=node,panel,Path(root)
        self.client=ActionClient(node,NavigateToPose,'/navigate_to_pose')
        self.planner=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        self.tf=Buffer();self.listener=TransformListener(self.tf,node)
        self.serial_ready=False;self.serial_checked=0.
        self.localization_info={'ready':False,'message':'等待雷达匹配已保存地图'}
        self.lease=None;self.seq=0;self.io_lock=threading.RLock();self.done=threading.Event()
        self.token=(self.root/'navcar_move.token').read_text().strip()
        self.odom_pub=node.create_publisher(Odometry,'/navcar/odom',10)
        self.previous_odom=None
        self.goal_stamp_ns=0
        from rclpy.qos import QoSProfile,DurabilityPolicy
        self.subs=[node.create_subscription(Twist,'/navcar/cmd_vel_auto',self.command,1),
                   node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.localized,
                                            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
                   node.create_subscription(Odometry,'/navcar/state_estimation',self.odom,10),
                   node.create_subscription(NavPath,'/plan',self.path,10),
                   node.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.costmap,
                                            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
        self.timer=node.create_timer(.1,self.tick)
        self.thread=threading.Thread(target=self.output,daemon=True);self.thread.start()

    def request(self,path,data=None):
        req=Request('http://127.0.0.1:8081'+path,data=json.dumps(data).encode() if data is not None else None,
                    headers={'Authorization':'Bearer '+self.token,'Content-Type':'application/json'})
        with urlopen(req,timeout=.25) as response:return json.load(response)

    def send_goal(self,target,feedback):
        from nav2_msgs.action import NavigateToPose
        with self.io_lock:
            self.release()
            self.lease=self.request('/api/navigation/acquire',{})['lease'];self.seq=0
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
        now=self.node.get_clock().now();self.goal_stamp_ns=now.nanoseconds;goal.pose.header.stamp=now.to_msg()
        goal.pose.pose.position.x=target['x'];goal.pose.pose.position.y=target['y']
        goal.pose.pose.orientation.z=math.sin(target['yaw']/2);goal.pose.pose.orientation.w=math.cos(target['yaw']/2)
        return self.client.send_goal_async(goal,feedback_callback=feedback)

    def plan_goal(self,target):
        from nav2_msgs.action import ComputePathToPose
        if not self.planner.server_is_ready():raise RuntimeError('等待路径规划器就绪')
        goal=ComputePathToPose.Goal();goal.goal.header.frame_id='map'
        goal.goal.header.stamp=self.node.get_clock().now().to_msg()
        goal.goal.pose.position.x=target['x'];goal.goal.pose.position.y=target['y']
        goal.goal.pose.orientation.w=1.;goal.planner_id='GridBased';goal.use_start=False
        # ComputePathToPose only plans. It never acquires the MCU lease or follows the path.
        return self.planner.send_goal_async(goal)

    def path(self,msg):
        p=self.panel
        if msg.header.frame_id!='map' or not msg.poses:return
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if stamp<self.goal_stamp_ns:return
        points=[[v.pose.position.x,v.pose.position.y] for v in msg.poses]
        if not all(math.isfinite(v) for xy in points for v in xy):return
        with p.lock:
            if p.nav_status not in ('sending','active') or not p.nav_goal:return
            if math.hypot(points[-1][0]-p.nav_goal['x'],points[-1][1]-p.nav_goal['y'])>.3:return
            stride=max(1,math.ceil(len(points)/2000))
            p.nav_path=points[::stride]
            if p.nav_path[-1]!=points[-1]:p.nav_path.append(points[-1])

    def costmap(self,msg):
        if msg.header.frame_id!='map' or not 0<msg.info.width*msg.info.height<=250000:return
        if not 0<msg.info.resolution<=10:return
        pos,q=msg.info.origin.position,msg.info.origin.orientation
        origin=[pos.x,pos.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
        if not all(math.isfinite(v) for v in origin):return
        # 99 is the inscribed collision margin; 100 is a measured/static obstacle.
        blocked=[i for i,v in enumerate(msg.data) if v==99]
        lethal=[i for i,v in enumerate(msg.data) if v==100]
        with self.panel.lock:
            self.panel.navigation_costmap={'available':True,'frame':'map','resolution':msg.info.resolution,
                'width':msg.info.width,'height':msg.info.height,'origin':origin,'blocked':blocked,'lethal':lethal}

    def release(self):
        with self.io_lock:
            lease,self.lease=self.lease,None
            if lease:
                try:self.request('/api/navigation/release',{'lease':lease})
                except Exception:pass # Independent 200ms controller watchdog still stops output.

    def command(self,msg):
        p=self.panel
        now=sample_monotonic_time(self.node.get_clock().now().to_msg(),self.node.get_clock().now().nanoseconds,time.monotonic(),1.)
        values=(msg.linear.x,msg.linear.y,msg.angular.z)
        if all(math.isfinite(v) for v in values):
            with p.lock:p.commands['auto']=(now or 0.,values)

    def localized(self,msg):
        p=self.panel
        # AMCL is the localization source; MCU target echoes are never used as observations.
        quality=localization_quality(msg.header.frame_id,msg.pose.covariance,p.navigation_control.frames['position_std_limit'])
        with p.lock:
            self.localization_info=quality;p.nav_localized=quality['ready']

    def odom(self,msg):
        p=self.panel
        sampled=sample_monotonic_time(msg.header.stamp,self.node.get_clock().now().nanoseconds,time.monotonic(),1.)
        pos=msg.pose.pose.position;q=msg.pose.pose.orientation
        values=(pos.x,pos.y,pos.z,q.x,q.y,q.z,q.w)
        if sampled is None or not all(math.isfinite(v) for v in values):return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        reference=p.navigation_control.frames
        offset=reference['center_offset']
        pos.x,pos.y=shift_pose(pos.x,pos.y,yaw,offset)
        msg.pose.covariance=shift_planar_covariance(list(msg.pose.covariance),yaw,offset)
        msg.child_frame_id=reference['base_frame']
        with p.lock:
            p.pose={'x':pos.x,'y':pos.y,'z':pos.z,'yaw':yaw,'frame':msg.header.frame_id,
                    'reference_offset_xy_m':offset[:2]};p.odom_time=sampled
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.previous_odom:
            t,x,y,heading=self.previous_odom;dt=stamp-t
            if .01<dt<.5:
                dx,dy=(pos.x-x)/dt,(pos.y-y)/dt
                msg.twist.twist.linear.x=math.cos(yaw)*dx+math.sin(yaw)*dy
                msg.twist.twist.linear.y=-math.sin(yaw)*dx+math.cos(yaw)*dy
                msg.twist.twist.angular.z=math.atan2(math.sin(yaw-heading),math.cos(yaw-heading))/dt
        self.previous_odom=(stamp,pos.x,pos.y,yaw);self.odom_pub.publish(msg)

    def tick(self):
        import rclpy
        p=self.panel;control=p.navigation_control
        if not control.running:return
        with p.lock:
            p.nav_server_ready=self.client.server_is_ready()
            try:
                tf=self.tf.lookup_transform('map',control.frames['base_frame'],rclpy.time.Time())
                sample=sample_monotonic_time(tf.header.stamp,self.node.get_clock().now().nanoseconds,time.monotonic(),1.)
                pos,q=tf.transform.translation,tf.transform.rotation
                if sample is None or not all(math.isfinite(v) for v in (pos.x,pos.y,q.x,q.y,q.z,q.w)):return
                p.map_pose={'x':pos.x,'y':pos.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),'frame':'map',
                            'reference_offset_xy_m':control.frames['center_offset'][:2]}
                p.map_pose_time=sample
            except Exception:p.map_pose_time=0.

    def output(self):
        while not self.done.wait(.05):
            p=self.panel
            try:
                if time.monotonic()-self.serial_checked>.3:
                    state=self.request('/api/state');self.serial_ready=state['connected'] and not state['error']
                    self.serial_checked=time.monotonic()
                with p.lock:
                    active=p.nav_status in ('sending','active','canceling') and p.navigation_control.running
                    stamp,values=p.commands.get('auto',(0.,(0.,0.,0.)))
                    moving=(active and p.nav_status=='active' and not p.nav_hold and not p.switching and
                            not p.navigation_reason() and time.monotonic()-stamp<.2)
                    vx,vy=values[:2] if moving else (0.,0.)
                    vx,vy=limit_translation(vx,vy,p.navigation_control.max_speed)
                with self.io_lock:
                    if self.lease and active:
                        self.seq+=1
                        self.request('/api/navigation/command',{'lease':self.lease,'seq':self.seq,'forward':vx,'left':vy})
                    elif self.lease:self.release()
            except Exception as exc:
                self.serial_ready=False;self.serial_checked=time.monotonic()
                with p.lock:
                    if p.nav_status in ('sending','active'):
                        p.cancel_navigation();p.nav_message='导航已停止：下位机断连或已被手动接管';p.nav_hold=True
                self.release()

    def close(self):
        self.done.set();self.thread.join(1);self.release()
