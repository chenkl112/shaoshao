"""Task recovery on the existing Nav2/AMCL bridge and its single MCU lease."""
import math
from pathlib import Path
import time
import threading
import numpy as np
import yaml

from navigation_runtime import CollisionGrid, RecoveryTask, SmoothVelocity, map_odom_correction, correction_jump
from global_relocalization import GlobalRelocalizer, angle_difference
from robot_io import sample_monotonic_time


class RecoveryBridgeMixin:
    def init_recovery(self):
        from nav2_msgs.msg import Costmap
        from nav2_msgs.srv import ClearEntireCostmap
        from sensor_msgs.msg import LaserScan,PointCloud2
        from rclpy.qos import qos_profile_sensor_data
        from std_srvs.srv import Empty
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self.task = None
        self.smoother = SmoothVelocity(acceleration=.25,jerk=.6,speed=self.panel.navigation_control.max_speed)
        self.motion_status = 'stopped'
        self.passage_speed_limit = None
        self.local_grid = None
        self.grid_condition=threading.Condition()
        self.pending_grid=None
        self.grid_epoch=0
        self.grid_thread=threading.Thread(target=self.build_grids,daemon=True)
        self.grid_thread.start()
        self.recovery_velocity = (0., 0.)
        self.pending_velocity=(0.,0.)
        self.output_permitted=False
        self.last_travel = (0., 0.)
        self.localization_at = self.localization_stamp = 0.
        self.match_at = self.match_ratio = 0.
        self.match_frames = 0
        self.last_scan_stamp = 0
        self.good_since = self.bad_since = 0.
        self.last_nomotion = self.last_global = self.last_seed = 0.
        self.last_good_pose = None
        self.last_good_uncertainty=(.03,.08)
        self.good_map_odom = None
        self.progress_anchor = None
        self.progress_reset_anchor=None
        self.progress_at = time.monotonic()
        self.localization_future = None
        self.resuming = False
        self.static_map = None
        self.static_map_path = None
        self.static_known = self.static_occupied = None
        self.latest_scan = None
        self.latest_cloud = None
        self.global_locator = GlobalRelocalizer()
        self.global_generation = 0
        self.global_needed = self.uses_pointcloud()
        self.global_candidate = self.global_proposal = self.global_job = None
        self.global_search_at = self.last_odom_jump = 0.
        self.global_audit_at=time.monotonic()
        self.global_status = {'state':'idle'}
        self.map_clear_futures=[];self.map_clear_at=self.map_clear_ack_at=0.
        self.global_costmap_at=0.
        self.localization_map_revision=None
        self.pointcloud_map_revision=None
        self.map_clear_clients=[self.node.create_client(ClearEntireCostmap,name) for name in
            ('/global_costmap/clear_entirely_global_costmap','/local_costmap/clear_entirely_local_costmap')]
        self.nomotion_client = self.node.create_client(Empty, '/request_nomotion_update')
        self.global_client = self.node.create_client(Empty, '/reinitialize_global_localization')
        self.initial_pose_pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.subs.extend([
            self.node.create_subscription(Costmap, '/local_costmap/costmap_raw', self.raw_costmap, 1,
                                          callback_group=self.callbacks['grid']),
            self.node.create_subscription(LaserScan, '/navcar/scan', self.scan_match, 1,
                                          callback_group=self.callbacks['scan']),
            self.node.create_subscription(PointCloud2,'/cloud_registered_body',self.localization_cloud,qos_profile_sensor_data,
                                          callback_group=self.callbacks['cloud'])])

    def begin_task(self, goal, revision):
        self.task = RecoveryTask(dict(goal), revision, since=time.monotonic())
        self.smoother = SmoothVelocity(acceleration=.25,jerk=.6,speed=self.panel.navigation_control.max_speed)
        self.progress_anchor = None
        self.progress_reset_anchor=None
        self.progress_at = time.monotonic()

    def cancel_task(self):
        self.task = None
        self.recovery_velocity = (0., 0.)
        self.pending_velocity=(0.,0.)
        self.smoother.reset()
        if hasattr(self,'global_locator'):
            self.global_generation+=1
            self.global_locator.invalidate()
            self.global_candidate=self.global_proposal=None
            self.map_clear_futures=[];self.map_clear_at=self.map_clear_ack_at=0.

    def reset_recovery(self):
        self.cancel_task()
        self.local_grid = None
        with self.grid_condition:
            self.grid_epoch+=1
            self.pending_grid=None
        self.localization_at = self.match_at = self.match_frames = 0.
        self.good_since = self.bad_since = 0.
        self.last_good_pose = self.good_map_odom = None
        self.last_good_uncertainty=(.03,.08)
        self.latest_scan=None
        self.latest_cloud=None
        self.global_needed=self.uses_pointcloud()
        self.global_status={'state':'idle'}
        self.pointcloud_map_revision=None

    def recovery_status(self):
        status = self.task.status() if self.task else {'state': 'idle'}
        return {**status, 'scan_match_ratio': float(self.match_ratio), 'motion':self.motion_status,
                'passage_speed_limit_mps':self.passage_speed_limit,'global_localization':{
                    **self.global_status,'map_yaml':str(self.localization_map_path()),'map_revision':self.localization_map_revision,
                    'map_pcd':str(self.localization_map_path().with_suffix('.pcd')) if self.uses_pointcloud() else None,
                    'configured_backend':'pcd_gicp' if self.uses_pointcloud() else '2d_scan'}}

    def localization_map_path(self):
        control=self.panel.navigation_control
        return Path(control.frames.get('localization_map_yaml',Path(control.mapping.maps_dir)/'site.yaml'))

    def uses_pointcloud(self):
        return getattr(self.panel.navigation_control,'frames',{}).get('localization_backend')=='pcd_gicp'

    def global_packet(self):
        return self.latest_cloud if self.uses_pointcloud() else self.latest_scan

    def submit_global_search(self,token,revision,packet):
        path=self.localization_map_path()
        kwargs={}
        if self.uses_pointcloud():
            from pointcloud_relocalization import cloud_revision
            revision=cloud_revision(path);kwargs['cloud']=True
        return self.global_locator.submit(token,path,revision,self.panel.navigation_control.frames['collision_radius'],
                                          packet[2],packet[3],**kwargs)

    def localization_cloud(self,msg):
        """Fresh undistorted lidar points, with only timestamped local TF."""
        if not self.uses_pointcloud():return
        import rclpy
        from scipy.spatial.transform import Rotation
        from sensor_msgs_py.point_cloud2 import read_points_numpy
        sampled=sample_monotonic_time(msg.header.stamp,self.node.get_clock().now().nanoseconds,time.monotonic(),.5)
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if sampled is None or msg.width*msg.height>200000:return
        epoch,generation=self.grid_epoch,self.global_generation
        try:
            frames=self.panel.navigation_control.frames;base=frames['base_frame']
            def lookup(target,source):
                value=self.tf.lookup_transform(target,source,rclpy.time.Time.from_msg(msg.header.stamp)).transform
                t,q=value.translation,value.rotation
                matrix=np.eye(4);matrix[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
                matrix[:3,3]=[t.x,t.y,t.z]
                return matrix
            base_from_body=lookup(base,msg.header.frame_id)
            odom_from_base=lookup('navcar_odom',base)
            imu_from_lidar=np.eye(4)
            imu_from_lidar[:3,:3]=np.asarray(frames['lidar_to_imu_rotation']).reshape(3,3)
            imu_from_lidar[:3,3]=frames['lidar_to_imu_xyz']
            base_from_lidar=lookup(base,'navcar_imu')@imu_from_lidar
            lidar_from_body=np.linalg.inv(base_from_lidar)@base_from_body
            points=np.asarray(read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True)).reshape(-1,3)
            points=points[np.isfinite(points).all(axis=1)]
            points=points[np.linspace(0,len(points)-1,min(60000,len(points)),dtype=int)]
            points=points@lidar_from_body[:3,:3].T+lidar_from_body[:3,3]
            odom={'x':float(odom_from_base[0,3]),'y':float(odom_from_base[1,3]),
                  'yaw':float(math.atan2(odom_from_base[1,0],odom_from_base[0,0]))}
        except Exception:
            return
        with self.panel.lock:
            if epoch!=self.grid_epoch or generation!=self.global_generation:return
            if self.latest_cloud and stamp<=self.latest_cloud[1]:return
            self.latest_cloud=(sampled,stamp,points,base_from_lidar,odom) if len(points)>=1000 else None

    def action_result(self, status, code, detail):
        task, p = self.task, self.panel
        if task is None:
            return False
        if status == 4:
            if np.linalg.norm(self.smoother.v)>1e-5 or np.linalg.norm(self.smoother.a)>1e-4:
                task.state = 'settling'
                p.nav_status,p.nav_hold,p.nav_handle='recovering',True,None
                p.commands.pop('auto',None)
                p.nav_message='已接近目标，正在平稳停止'
                return True
            self.cancel_task()
            return False
        if status == 5 and task.state not in ('pausing', 'stuck_pausing'):
            self.cancel_task()
            return False
        now = time.monotonic()
        if task.state == 'pausing' or not p.nav_localized:
            task.state, task.since = 'relocalizing', now
            task.reason = '定位不可信，已停车，正在自动重定位'
            self.good_since = 0.
            self.match_frames = 0
        else:
            recover_code=105 if task.state=='stuck_pausing' else code
            if code==205 and self.local_grid and p.pose and p.pose.get('frame')==self.local_grid.frame:
                point=(p.pose['x'],p.pose['y'])
                radius=p.navigation_control.frames['collision_radius']
                if now-self.local_grid.stamp<.5 and self.local_grid.safe_segment(point,point,radius):
                    recover_code=105
            task.failed(recover_code, now)
            task.reason = ('正在低速搜索脱困方向' if task.state == 'recovering' else
                           '暂时无法通行，保留目标等待重新规划')
            if recover_code in (101, 201, 203, 204, 205, 206):
                task.reason = '目标或导航配置需要检查，已停车并保留目标'
        p.nav_status, p.nav_hold = 'recovering', True
        p.nav_handle = p.nav_remaining = None
        p.nav_path = []
        p.commands.pop('auto', None)
        p.nav_message = task.reason
        self.smoother.reset()
        self.recovery_velocity = (0., 0.)
        return True

    def raw_costmap(self, msg):
        epoch=self.grid_epoch
        meta = msg.metadata
        sampled = sample_monotonic_time(msg.header.stamp, self.node.get_clock().now().nanoseconds,
                                        time.monotonic(), .5)
        if sampled is None or meta.size_x*meta.size_y != len(msg.data):
            return
        if not 0 < len(msg.data) <= 1000000:
            return
        q, pos = meta.origin.orientation, meta.origin.position
        origin = (pos.x, pos.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        values=np.asarray(msg.data).reshape(meta.size_y,meta.size_x).copy()
        with self.grid_condition:
            if epoch!=self.grid_epoch:return
            self.pending_grid=(epoch,values,meta.resolution,origin,sampled,msg.header.frame_id)
            self.grid_condition.notify()

    def build_grids(self):
        # Keep only the newest costmap. EDT/subdivision must not occupy the ROS
        # callback that receives commands, local odometry and localization.
        while not self.done.is_set():
            with self.grid_condition:
                self.grid_condition.wait_for(lambda:self.pending_grid is not None or self.done.is_set(),timeout=.2)
                if self.done.is_set():return
                job,self.pending_grid=self.pending_grid,None
            if job is None:continue
            epoch,values,resolution,origin,stamp,frame=job
            try:
                grid=CollisionGrid(values,resolution,origin,stamp,frame)
                grid.prepare(self.panel.navigation_control.frames['collision_radius'])
            except ValueError:
                continue
            if self.done.is_set():return
            with self.panel.lock:
                if epoch!=self.grid_epoch or time.monotonic()-stamp>.5:
                    continue
                if self.local_grid is None or grid.stamp>self.local_grid.stamp:
                    self.local_grid=grid

    def load_static_map(self):
        from PIL import Image
        path = self.localization_map_path()
        key = (str(path), path.stat().st_mtime_ns)
        if self.static_map_path == key:
            return
        meta = yaml.safe_load(path.read_text())
        with Image.open(path.parent/meta['image']) as image:
            pixels = np.flipud(np.asarray(image.convert('L')))
        occupancy = pixels/255. if meta.get('negate') else (255-pixels.astype(float))/255.
        occupied = occupancy > meta['occupied_thresh']
        self.static_occupied=occupied
        self.static_known=occupied|(occupancy<meta['free_thresh'])
        from scipy.ndimage import distance_transform_edt
        self.static_map = (distance_transform_edt(~occupied)*meta['resolution'], meta)
        self.static_map_path = key

    def scan_match(self, msg):
        # Independently check a candidate AMCL pose against measured wall returns.
        # This supplements covariance; no truth pose enters the localization gate.
        import rclpy
        epoch=self.grid_epoch
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        sampled = sample_monotonic_time(msg.header.stamp, self.node.get_clock().now().nanoseconds,
                                        time.monotonic(), .5)
        if sampled is None or stamp <= self.last_scan_stamp:
            return
        self.last_scan_stamp = stamp
        packet=None;ratio=0.
        try:
            self.load_static_map()
            base=self.panel.navigation_control.frames['base_frame']
            sensor_tf=self.tf.lookup_transform(base,msg.header.frame_id,rclpy.time.Time.from_msg(msg.header.stamp))
            sensor_pos,sensor_q=sensor_tf.transform.translation,sensor_tf.transform.rotation
            sensor_yaw=math.atan2(2*(sensor_q.w*sensor_q.z+sensor_q.x*sensor_q.y),1-2*(sensor_q.y**2+sensor_q.z**2))
            all_ranges=np.asarray(msg.ranges,dtype=float)
            all_angles=msg.angle_min+np.arange(len(all_ranges))*msg.angle_increment
            valid=np.isfinite(all_ranges)&(all_ranges>msg.range_min)&(all_ranges<msg.range_max-.05)
            selected=np.flatnonzero(valid)
            if len(selected)<30:
                with self.panel.lock:
                    if epoch==self.grid_epoch:
                        self.latest_scan=None;self.match_ratio=0.;self.match_frames=0
                        self.match_at=sampled;self.panel.nav_localized=False
                return
            selected=selected[np.linspace(0,len(selected)-1,min(144,len(selected)),dtype=int)]
            ranges=all_ranges[selected];body_angles=all_angles[selected]+sensor_yaw
            points=np.column_stack((sensor_pos.x+ranges*np.cos(body_angles),sensor_pos.y+ranges*np.sin(body_angles)))
            packet=(sampled,stamp,points.tolist(),(sensor_pos.x,sensor_pos.y))
            tf = self.tf.lookup_transform('map', msg.header.frame_id,
                                         rclpy.time.Time.from_msg(msg.header.stamp))
            pos, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            angle=all_angles[selected]+yaw
            x, y = pos.x+ranges*np.cos(angle), pos.y+ranges*np.sin(angle)
            distances, meta = self.static_map
            ox, oy, heading = meta['origin']
            c, s = math.cos(heading), math.sin(heading)
            ix = np.floor((c*(x-ox)+s*(y-oy))/meta['resolution']).astype(int)
            iy = np.floor((-s*(x-ox)+c*(y-oy))/meta['resolution']).astype(int)
            inside = (ix >= 0) & (iy >= 0) & (ix < distances.shape[1]) & (iy < distances.shape[0])
            matched = np.zeros(len(ranges), dtype=bool)
            matched[inside] = (distances[iy[inside], ix[inside]] <= .12)&self.static_known[iy[inside],ix[inside]]
            ratio = float(np.mean(matched))
        except Exception:
            # TF may not yet cover this scan. It cannot authorize localization.
            pass
        with self.panel.lock:
            if epoch!=self.grid_epoch:return
            if packet is not None:self.latest_scan=(*packet,dict(self.panel.pose) if self.panel.pose else None)
            self.match_at, self.match_ratio = sampled, ratio
            threshold=.8 if self.global_needed else .55
            self.match_frames = self.match_frames+1 if ratio >= threshold else 0
            if ratio<threshold:self.panel.nav_localized=False

    def force_global_relocalization(self,reason):
        self.global_generation+=1
        self.global_locator.invalidate()
        self.global_needed=True
        self.global_candidate=self.global_proposal=None
        self.global_status={'state':'required','reason':reason}
        self.map_clear_futures=[];self.map_clear_at=self.map_clear_ack_at=0.
        self.good_since=0.;self.match_frames=0
        self.panel.nav_localized=False

    def candidate_pose(self,correction,odom):
        x,y,yaw=correction;c,s=math.cos(yaw),math.sin(yaw)
        return {'x':x+c*odom['x']-s*odom['y'],'y':y+s*odom['x']+c*odom['y'],'yaw':yaw+odom['yaw']}

    def global_tick(self,now,revision):
        p=self.panel;token=(self.grid_epoch,self.global_generation,revision)
        result=self.global_locator.poll(token)
        if result is not None:
            job=self.global_job;self.global_search_at=now
            self.global_status={k:v for k,v in result.items() if k!='alternatives'}
            if (result.get('state')=='candidate' and job and p.pose and now-job['at']<8.
                    and now-p.odom_time<.3 and now-self.last_odom_jump>=1.
                    and math.hypot(*p.measured_velocity)<.025):
                pose=dict(zip(('x','y','yaw'),result['pose']))
                correction=map_odom_correction(pose,job['odom'])
                current=self.candidate_pose(correction,p.pose)
                previous=self.global_proposal
                agrees=False
                if previous and job['stamp']>previous['stamp']:
                    old=self.candidate_pose(previous['correction'],p.pose)
                    agrees=(math.hypot(current['x']-old['x'],current['y']-old['y'])<.12 and
                            angle_difference(current['yaw'],old['yaw'])<.12)
                self.global_proposal={'correction':correction,'stamp':job['stamp']}
                if agrees:
                    self.publish_localization_seed(current,.04,.08)
                    self.global_candidate={'correction':correction,'at':now}
                    self.last_seed=now;self.good_since=0.;self.match_frames=0
                    self.global_status.update(state='verifying',pose=[current[k] for k in ('x','y','yaw')])
                else:self.global_status['state']='confirming'
        if self.global_candidate and (self.global_candidate.get('confirmed_at') or now-self.global_candidate['at']<=5.):
            return
        if self.global_candidate:
            self.global_candidate=None
            self.global_status={'state':'verification_failed'}
        packet=self.global_packet()
        cooldown=.15 if self.global_status.get('state')=='confirming' else 3.
        if packet is None or now-packet[0]>.5:
            if self.global_locator.future is None:
                self.global_status={'state':'insufficient_cloud' if self.uses_pointcloud() else 'insufficient_scan'}
            return
        if (now-self.global_search_at<cooldown or
                not p.pose or packet[4] is None or p.pose.get('frame')!='navcar_odom' or
                now-p.odom_time>.3 or math.hypot(*p.measured_velocity)>=.025 or now-self.last_odom_jump<1.):
            return
        if self.global_proposal and packet[1]<=self.global_proposal['stamp']:return
        if self.submit_global_search(token,revision,packet):
            self.global_job={'at':packet[0],'stamp':packet[1],'odom':packet[4]}
            self.global_status['state']='searching'

    def clear_relocalized_maps(self,now):
        """Remove observations made under the old map transform before motion."""
        from nav2_msgs.srv import ClearEntireCostmap
        if not self.map_clear_futures:
            if not all(client.service_is_ready() for client in self.map_clear_clients):return False
            self.map_clear_at=now;self.map_clear_ack_at=0.
            self.map_clear_futures=[client.call_async(ClearEntireCostmap.Request()) for client in self.map_clear_clients]
            return False
        if not all(future.done() for future in self.map_clear_futures):return False
        try:
            for future in self.map_clear_futures:future.result()
        except Exception as exc:
            self.global_status.update(state='costmap_clear_failed',detail=str(exc))
            self.map_clear_futures=[]
            return False
        if not self.map_clear_ack_at:
            self.map_clear_ack_at=now
            self.local_grid=None;self.global_costmap_at=0.
            with self.grid_condition:
                self.grid_epoch+=1;self.pending_grid=None
            self.global_locator.invalidate()
            return False
        grid,pose=self.local_grid,self.panel.pose
        return bool(grid and pose and grid.frame==pose.get('frame') and
                    min(grid.stamp,self.global_costmap_at)>self.map_clear_ack_at and
                    grid.safe_segment((pose['x'],pose['y']),(pose['x'],pose['y']),
                                      self.panel.navigation_control.frames['collision_radius']))

    def audit_global_pose(self,now,revision):
        """An independent map-wide check may revoke tracking, never grant it."""
        p=self.panel;token=(self.grid_epoch,self.global_generation,revision,'audit')
        result=self.global_locator.poll(token)
        if result is not None:
            self.global_status['audit_state']=result.get('state')
            job=self.global_job
            if result.get('state')=='candidate' and job and p.pose and p.map_pose and now-job['at']<8.:
                pose=dict(zip(('x','y','yaw'),result['pose']))
                predicted=self.candidate_pose(map_odom_correction(pose,job['odom']),p.pose)
                if (math.hypot(predicted['x']-p.map_pose['x'],predicted['y']-p.map_pose['y'])>.3 or
                        angle_difference(predicted['yaw'],p.map_pose['yaw'])>.25):
                    self.force_global_relocalization('独立全地图核对与跟踪位置不一致')
                    return
        packet=self.global_packet()
        if now-self.global_audit_at<5. or packet is None or now-packet[0]>.5 or packet[4] is None:return
        if self.submit_global_search(token,revision,packet):
            self.global_audit_at=now
            self.global_job={'at':packet[0],'stamp':packet[1],'odom':packet[4]}

    def publish_localization_seed(self,pose,std,yaw_std):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map'
        msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x,msg.pose.pose.position.y=pose['x'],pose['y']
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w=math.sin(pose['yaw']/2),math.cos(pose['yaw']/2)
        msg.pose.covariance[0]=msg.pose.covariance[7]=std**2
        msg.pose.covariance[35]=yaw_std**2
        self.initial_pose_pub.publish(msg)

    def _request_update(self, client):
        from std_srvs.srv import Empty
        if self.localization_future and not self.localization_future.done():
            return False
        if not client.service_is_ready():
            return False
        self.localization_future = client.call_async(Empty.Request())
        return True

    def _seed_last_pose(self, now):
        if self.last_good_pose is None:
            return
        from geometry_msgs.msg import PoseWithCovarianceStamped
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        pose = self.last_good_pose
        local = self.panel.pose
        if self.good_map_odom is not None and local and local.get('frame')=='navcar_odom':
            # Keep the last trusted MAP->ODOM correction, then propagate with
            # fresh local odometry. A stale map pose is not the current seed.
            x,y,angle=self.good_map_odom
            c,s=math.cos(angle),math.sin(angle)
            pose={'x':x+c*local['x']-s*local['y'],'y':y+s*local['x']+c*local['y'],
                  'yaw':angle+local['yaw']}
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose['x'], pose['y']
        msg.pose.pose.orientation.z = math.sin(pose['yaw']/2)
        msg.pose.pose.orientation.w = math.cos(pose['yaw']/2)
        std,yaw_std=self.last_good_uncertainty
        msg.pose.covariance[0] = msg.pose.covariance[7] = max(.03,std)**2
        msg.pose.covariance[35] = max(.08,yaw_std)**2
        self.initial_pose_pub.publish(msg)
        self.last_seed = now

    def recovery_tick(self, *, revision):
        p, now = self.panel, time.monotonic()
        self.localization_map_revision=revision
        task = self.task
        if not p.navigation_control.running or p.closed or p.switching:
            self.cancel_task();p.nav_localized=False
            return
        if task and revision!=task.revision:
            p.cancel_navigation();p.nav_message='地图已变化，已停止原任务'
            return
        search_revision=revision
        if self.uses_pointcloud():
            from pointcloud_relocalization import cloud_revision
            current=cloud_revision(self.localization_map_path())
            if self.pointcloud_map_revision is not None and current!=self.pointcloud_map_revision:
                self.force_global_relocalization('点云地图或关键帧索引已变化')
            self.pointcloud_map_revision=current
            search_revision=(revision,current)
        if not self.global_needed and p.nav_status=='active' and p.nav_localized:
            self.audit_global_pose(now,search_revision)
        if self.global_needed:self.global_tick(now,search_revision)
        if now-self.last_nomotion > .5 and self._request_update(self.nomotion_client):
            self.last_nomotion = now
        trustworthy = (self.localization_info.get('ready',False) and 0 <= now-self.localization_at <= 2.
                       and 0 <= now-self.match_at <= .7 and self.match_ratio >= .55)
        candidate_ok=False
        if self.global_needed:
            candidate=self.global_candidate
            if candidate and p.map_pose and p.pose:
                predicted=self.candidate_pose(candidate['correction'],p.pose)
                candidate_ok=(math.hypot(predicted['x']-p.map_pose['x'],predicted['y']-p.map_pose['y'])<.12 and
                              angle_difference(predicted['yaw'],p.map_pose['yaw'])<.12 and
                              min(self.localization_at,self.match_at)>candidate['at'] and self.match_ratio>=.8)
            trustworthy=bool(trustworthy and candidate_ok)
        correction = None
        if trustworthy and p.map_pose and p.pose and p.pose.get('frame') == 'navcar_odom':
            correction = map_odom_correction(p.map_pose,p.pose)
            if self.good_map_odom is not None:
                old = self.good_map_odom
                jump = correction_jump(old,correction)
                searched = candidate_ok or (task is not None and task.state=='relocalizing' and not self.global_needed and
                            max(self.last_seed,self.last_global) >= task.since and
                            self.localization_at > max(self.last_seed,self.last_global))
                if jump and not searched:
                    trustworthy = False
        if not trustworthy:
            self.good_since = 0.
            self.bad_since = self.bad_since or now
        else:
            self.good_since = self.good_since or now
            self.bad_since = 0.
        stable=bool(self.good_since and now-self.good_since>=1. and self.match_frames>=3)
        if stable and trustworthy and self.global_needed:
            self.global_candidate.setdefault('confirmed_at',now)
            self.global_status['state']='refreshing_costmaps'
            if self.clear_relocalized_maps(now):
                self.global_needed=False
                self.global_status['state']='verified'
            else:trustworthy=False
        if (self.global_needed and self.global_candidate and self.global_candidate.get('confirmed_at') and
                self.bad_since and now-self.bad_since>2.):
            self.force_global_relocalization('候选位置确认后观测再次失效')
            trustworthy=False
        acquiring=self.global_needed or self.last_good_pose is None or bool(task and task.state=='relocalizing')
        if trustworthy and (stable or not acquiring) and p.map_pose and now-p.map_pose_time<.5:
            self.last_good_pose=dict(p.map_pose)
            self.good_map_odom=correction
            self.last_good_uncertainty=(self.localization_info.get('position_std_m',.03),
                                        self.localization_info.get('yaw_std_rad',.08))
        # Scan/pose consistency is part of motion permission even before a task.
        p.nav_localized = bool(trustworthy and (stable or not acquiring))
        if task is None:
            if (not trustworthy and self.bad_since and now-self.bad_since > 5.
                    and now-p.cloud_time < .5 and now-p.odom_time < .3 and now-self.last_global > 10.):
                if not self.global_needed:self.force_global_relocalization('启动或空闲时定位不可信')
            return
        if not trustworthy and self.bad_since and now-self.bad_since > .5:
            if self.match_ratio<.35 and not self.global_needed:
                self.force_global_relocalization('扫描与旧位置严重不一致')
            if task.state not in ('pausing', 'relocalizing'):
                task.state, task.since = 'pausing', now
                p.cancel_navigation(automatic=True)
                if p.nav_handle is None and p.nav_status != 'sending':
                    task.state = 'relocalizing'
                    p.nav_status = 'recovering'
                self.recovery_velocity = (0., 0.)
                self.smoother.reset()
        if task.state == 'relocalizing':
            p.nav_message = '定位丢失，正在自动重定位；原目标已保留'
            if self.global_needed:
                state=self.global_status.get('state')
                if state=='ambiguous':p.nav_message='全局定位存在相似位置，已停车等待更多观测；原目标保留'
                elif state in ('insufficient_scan','insufficient_scan_coverage','no_known_space',
                               'insufficient_cloud','insufficient_cloud_coverage','insufficient_cloud_slice'):
                    p.nav_message='扫描或地图特征不足，保持停车；原目标保留'
                elif state in ('map_error','worker_error','no_map_features','backend_unavailable'):
                    p.nav_message='地图或全局定位计算不可用，保持停车；原目标保留'
                elif state=='no_match':p.nav_message='当前观测与建图地图未可靠匹配，保持停车；原目标保留'
                elif state in ('refreshing_costmaps','costmap_clear_failed'):
                    p.nav_message='位置已确认，正在刷新实时障碍地图；原目标保留'
                else:p.nav_message='正在全地图搜索并验证位置和朝向；原目标保留'
            if trustworthy and self.match_frames >= 3 and now-self.good_since >= 1.:
                task.localized(now)
            elif not trustworthy and not self.global_needed and now-task.since > 1. and self.last_seed < task.since:
                self._seed_last_pose(now)
            elif not trustworthy and not self.global_needed and now-task.since > 3.:
                self.force_global_relocalization('局部初值未能恢复定位')
            return
        if not trustworthy or task.state == 'pausing':
            return
        if task.state == 'settling':
            if np.linalg.norm(self.smoother.v)<=1e-5 and np.linalg.norm(self.smoother.a)<=1e-4:
                p.nav_status,p.nav_hold,p.nav_message='succeeded',True,'已到达目标'
                p.nav_remaining=None
                self.cancel_task()
            return
        if task.state=='recovery_stopping':
            if (np.linalg.norm(self.smoother.v)<=1e-5 and np.linalg.norm(self.smoother.a)<=1e-4
                    and math.hypot(*p.measured_velocity)<.025):
                task.state,task.next_attempt='waiting',now+.5
                p.nav_message='脱困完成，正在重新规划原目标'
            return
        if task.state == 'running':
            pose = p.pose
            if pose and pose.get('frame') == 'navcar_odom' and now-p.odom_time < .3:
                point = (pose['x'], pose['y'])
                if self.progress_reset_anchor is None:
                    self.progress_reset_anchor=point
                elif math.dist(point,self.progress_reset_anchor)>.25:
                    task.retries=task.recoveries=0
                    self.progress_reset_anchor=point
                if self.progress_anchor is None or math.dist(point, self.progress_anchor) > .04:
                    self.progress_anchor, self.progress_at = point, now
                if now-self.progress_at > 6. and p.nav_status == 'active':
                    # Nav2's result chooses the recovery branch after cancellation.
                    task.state = 'stuck_pausing'
                    p.cancel_navigation(automatic=True)
            return
        if task.state == 'stuck_pausing':
            return
        if task.state == 'recovering':
            pose, grid = p.pose, self.local_grid
            if pose is None or grid is None or pose.get('frame') != grid.frame or now-grid.stamp > .5:
                self.recovery_velocity = (0., 0.)
                return
            point = (pose['x'], pose['y'])
            radius = p.navigation_control.frames['collision_radius']
            if task.escape_goal is None:
                task.escape_goal = grid.escape((pose['x'], pose['y'], pose['yaw']), radius, self.last_travel)
            goal = task.escape_goal
            if goal and grid.safe_segment(point, goal, radius) and now-task.since < 4. and math.dist(point, goal) > .035:
                dx, dy = goal[0]-point[0], goal[1]-point[1]
                speed = min(.08, math.sqrt(2*.15*math.hypot(dx, dy)))
                c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
                distance = math.hypot(dx, dy)
                self.recovery_velocity = (speed*(c*dx+s*dy)/distance, speed*(-s*dx+c*dy)/distance)
                p.nav_message = '正在低速脱困，随后继续原目标'
                return
            self.recovery_velocity = (0., 0.)
            task.state='recovery_stopping'
            p.nav_message='脱困动作结束，正在平稳停止'
        if task.state in ('waiting', 'blocked') and now >= task.next_attempt:
            if task.state == 'blocked':
                p.nav_message = task.reason+'；原目标保留'
                if task.permanent:
                    return
                # Exhausted motion attempts stay stopped. Passive replanning
                # remains limited to one attempt every five seconds.
            try:
                self.resuming = True
                p.send_navigation_goal(task.goal['x'], task.goal['y'], task.revision, resume=True)
                task.state, task.since = 'running', now
                self.progress_anchor, self.progress_at = None, now
            except (RuntimeError, ValueError) as exc:
                task.reason = str(exc)
                task.next_attempt = now+2.
            finally:
                self.resuming = False

    def checked_velocity(self, target, now, dt, permitted):
        p, grid = self.panel, self.local_grid
        self.output_permitted=bool(permitted)
        pose = p.pose
        measured=getattr(p,'measured_velocity',None)
        if (not permitted or grid is None or pose is None or pose.get('frame') != grid.frame
                or now-p.odom_time > .3 or measured is None):
            self.motion_status = 'permission:'+str(p.navigation_reason() or p.nav_status) if not permitted else 'local_data'
            self.pending_velocity=(0.,0.)
            self.output_permitted=False
            return self.smoother.reset('permission_or_local_data')
        radius = p.navigation_control.frames['collision_radius']
        position = (pose['x'], pose['y'], pose['yaw'])
        previous = tuple(self.smoother.v)
        cruise=.8*p.navigation_control.max_speed
        self.passage_speed_limit=grid.passage_speed_limit(position,target,radius,cruise)
        target=SmoothVelocity.clip(target,self.passage_speed_limit)
        if self.task and self.task.state=='running':
            target=grid.centered_velocity(position,target,radius)
        target=SmoothVelocity.clip(target,self.passage_speed_limit)
        self.pending_velocity=tuple(target)
        preview=self.smoother.preview(target,.02)
        result=preview[0] if preview is not None else (0.,0.)
        if not all(grid.safe_velocity(position, v, radius, now,
                                      self.smoother.amax, self.smoother.jmax)
                   for v in (previous, result,measured)):
            if hasattr(self,'record_collision_stop'):
                self.record_collision_stop(grid,position,result,measured)
            self.motion_status = 'braking_space'
            self.output_permitted=False
            self.pending_velocity=(0.,0.)
            return self.smoother.reset('braking_sweep')
        self.motion_status = 'narrow_following' if self.passage_speed_limit<cruise-.001 else 'following'
        if math.hypot(*result) > .03:
            c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
            self.last_travel = (c*result[0]-s*result[1], s*result[0]+c*result[1])
        return result

    def final_velocity(self):
        # Preview and commit at the wire boundary using actual elapsed time.
        p, now = self.panel, time.monotonic()
        task, grid, pose = self.task, self.local_grid, p.pose
        permitted = (p.nav_status=='active' and not p.nav_hold) or (
            p.nav_status=='recovering' and task and task.state in ('recovering','settling','recovery_stopping'))
        if (not self.output_permitted or not permitted or not p.nav_localized or p.switching or not p.navigation_control.running
                or not self.serial_ready or now-self.serial_checked>1.
                or now-p.odom_time>.3 or now-p.cloud_time>1. or pose is None or grid is None
                or pose.get('frame')!=grid.frame or getattr(p,'measured_velocity',None) is None):
            return self.smoother.reset('final_permission')
        radius=p.navigation_control.frames['collision_radius']
        for _ in range(4):
            stamp=time.monotonic()
            dt=.02 if self.smoother.sample_time is None else stamp-self.smoother.sample_time
            preview=self.smoother.preview(self.pending_velocity,dt)
            if preview is None:
                self.motion_status='output_deadline'
                return self.smoother.reset('output_deadline')
            if not all(grid.safe_velocity((pose['x'],pose['y'],pose['yaw']),velocity,radius,stamp,
                                          self.smoother.amax,self.smoother.jmax)
                       for velocity in (preview[0],p.measured_velocity)):
                self.motion_status='braking_space'
                return self.smoother.reset('final_sweep')
            if time.monotonic()-stamp<.001:
                return self.smoother.commit(preview,stamp)
        self.motion_status='output_deadline'
        return self.smoother.reset('sampling_latency')
