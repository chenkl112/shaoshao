import math
from pathlib import Path
import struct
import tempfile
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import yaml

from navcar_move_web import Controller
from navcar_navigation import NavigationBridge,build_navigation_config,navigation_geometry,localization_quality
from navcar_protocol import STATE,frame
from navcar_reference import navigation_frames,shift_pose,initial_reference_pose,shift_planar_covariance
from navcar_settings import navigation_options,navigation_speed,load_navigation_speed


def config():
    return {'robot':{'circumscribed_radius_m':.297658,'lidar_height_m':.327},
            'yaw':{'axis_xyz_m':[0,0,.280],'lidar_offset_radius_m':.01418,
                   'lidar_offset_xy_m':[-.01418,0.],'imu_rpy_rad':[0,0,0]},
            'navigation':{'reference_point':'yaw_axis','max_speed_mps':.5}}


def lio():
    return {'common':{},'pcd_save':{},'publish':{},'preprocess':{'blind':.5},
            'map_filter':{'dynamic_enabled':True},
            'mapping':{'extrinsic_T':[-.011,-.02329,.04412],'extrinsic_R':[1,0,0,0,1,0,0,0,1]}}


class ReferenceTests(unittest.TestCase):
    def test_odometry_callback_reports_center_velocity_instead_of_sensor_orbit(self):
        frames=navigation_frames(config(),lio());offset=frames['center_offset']
        panel=NS(navigation_control=NS(frames=frames),lock=__import__('threading').RLock())
        published=[]
        bridge=NS(panel=panel,node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=2_000_000_000))),
                  previous_odom=None,odom_pub=NS(publish=published.append))
        with patch('navcar_navigation.sample_monotonic_time',return_value=10.):
            for index,angle in enumerate((0.,.1,.2)):
                x,y=shift_pose(1.,2.,angle,[-offset[0],-offset[1]])
                msg=NS(header=NS(frame_id='navcar_odom',stamp=NS(sec=1,nanosec=index*100_000_000)),
                       child_frame_id='navcar_imu',
                       pose=NS(pose=NS(position=NS(x=x,y=y,z=.3),
                                       orientation=NS(x=0.,y=0.,z=math.sin(angle/2),w=math.cos(angle/2))),covariance=[0.]*36),
                       twist=NS(twist=NS(linear=NS(x=0.,y=0.,z=0.),angular=NS(x=0.,y=0.,z=0.))))
                NavigationBridge.odom(bridge,msg)
        self.assertEqual(len(published),3)
        for msg in published:
            self.assertEqual(msg.child_frame_id,'navcar_center')
            self.assertAlmostEqual(msg.pose.pose.position.x,1.)
            self.assertAlmostEqual(msg.pose.pose.position.y,2.)
            self.assertAlmostEqual(msg.twist.twist.linear.x,0.)
            self.assertAlmostEqual(msg.twist.twist.linear.y,0.)
        self.assertEqual(panel.pose['reference_offset_xy_m'],offset[:2])

    def test_true_axis_position_is_stationary_while_offset_sensor_rotates(self):
        cfg=config();frames=navigation_frames(cfg,lio());offset=frames['center_offset']
        self.assertAlmostEqual(offset[0],.00318)
        self.assertAlmostEqual(offset[1],-.02329)
        radius,_=navigation_geometry(cfg,lio()['mapping']['extrinsic_T'])
        self.assertEqual(radius,.297658)
        for degree in range(0,360,5):
            angle=math.radians(degree)
            imu=shift_pose(1.,2.,angle,[-offset[0],-offset[1]])
            center=shift_pose(*imu,angle,offset)
            self.assertAlmostEqual(center[0],1.)
            self.assertAlmostEqual(center[1],2.)
            for chassis_degree in range(0,360,15):
                for vertex in range(8):
                    heading=math.radians(chassis_degree+vertex*45)
                    r=.550/(2*math.cos(math.pi/8))
                    point=(1+r*math.cos(heading),2+r*math.sin(heading))
                    self.assertLessEqual(math.hypot(point[0]-center[0],point[1]-center[1]),radius)

    def test_seed_migration_applies_offset_once_and_supports_returning_to_imu(self):
        offset=navigation_frames(config(),lio())['center_offset']
        original={'x':1.,'y':2.,'yaw':1.2}
        shifted=initial_reference_pose(original,offset)
        saved={**shifted,'reference_offset_xy_m':offset[:2]}
        self.assertEqual(initial_reference_pose(saved,offset),shifted)
        back=initial_reference_pose(saved,[0,0,0])
        self.assertAlmostEqual(back['x'],original['x'])
        self.assertAlmostEqual(back['y'],original['y'])

    def test_shifted_pose_covariance_keeps_position_heading_correlations(self):
        offset=[.00318,-.02329,0]
        covariance=[0.]*36;covariance[35]=.04
        result=shift_planar_covariance(covariance,0.,offset)
        self.assertAlmostEqual(result[0],offset[1]**2*.04)
        self.assertAlmostEqual(result[7],offset[0]**2*.04)
        self.assertAlmostEqual(result[5],-offset[1]*.04)
        self.assertAlmostEqual(result[11],offset[0]*.04)
        self.assertEqual(result[35],.04)
        self.assertEqual(shift_planar_covariance(covariance,0.,[0,0,0]),covariance)

    def test_missing_calibration_or_unsupported_rotation_cannot_enable_center(self):
        cfg=config();cfg['yaw'].pop('lidar_offset_xy_m')
        with self.assertRaises(ValueError):navigation_frames(cfg,lio())
        cfg=config();cfg['yaw']['lidar_offset_xy_m']=[-.01,0]
        with self.assertRaises(ValueError):navigation_frames(cfg,lio())
        params=lio();params['mapping']['extrinsic_R']=[0,-1,0,1,0,0,0,0,1]
        with self.assertRaises(ValueError):navigation_frames(config(),params)
        cfg=config();cfg['navigation']['reference_point']='invalid'
        with self.assertRaises(ValueError):navigation_options(cfg)

    def test_profile_rejects_old_map_and_generates_consistent_sensor_and_center_frames(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);repo=root/'shaoshao/web_panel';repo.mkdir(parents=True)
            path=repo/'robot.yaml';path.write_text(yaml.safe_dump(config()))
            runtime=root/'mapping_runtime';runtime.mkdir()
            (runtime/'fast_lio.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':lio()}}))
            maps=root/'maps';maps.mkdir()
            (maps/'site.yaml').write_text('image: site.png\nresolution: 0.05\n')
            with self.assertRaisesRegex(ValueError,'2.5 cm'):build_navigation_config(root,maps)
            (maps/'site.yaml').write_text('image: site.png\nresolution: 0.025\n')
            output=build_navigation_config(root,maps,{'x':1.,'y':2.,'yaw':0.})
            frames=yaml.safe_load((output/'frames.yaml').read_text())
            self.assertNotIn('narrow',frames)
            params=yaml.safe_load((output/'nav2.yaml').read_text())
            fast=yaml.safe_load((output/'fast_lio.yaml').read_text())['/**']['ros__parameters']
            self.assertEqual(fast['preprocess']['blind'],.2)
            self.assertFalse(fast['map_filter']['dynamic_enabled'])
            transforms={v['child']:v['xyz'] for v in frames['static_transforms']}
            self.assertEqual(transforms['navcar_lidar'],[-.011,-.02329,.04412])
            self.assertAlmostEqual(transforms['navcar_center'][0],.00318)
            for name in ('global_costmap','local_costmap'):
                cost=params[name][name]['ros__parameters']
                self.assertEqual(cost['robot_base_frame'],'navcar_center')
                self.assertEqual(cost['robot_radius'],.297658)
                self.assertEqual(cost['footprint_padding'],.02)
                self.assertEqual(cost['resolution'],.025)
                self.assertFalse(cost['obstacle_layer']['scan']['inf_is_valid'])
            scan=params['navigation_scan']['ros__parameters']
            self.assertEqual(scan['target_frame'],'navcar_lidar')
            self.assertEqual(scan['range_min'],.2)
            self.assertAlmostEqual(scan['min_height'],-.227)
            amcl=params['amcl']['ros__parameters']
            self.assertEqual(amcl['base_frame_id'],'navcar_center')
            self.assertAlmostEqual(amcl['initial_pose']['x'],1.00318)
            self.assertAlmostEqual(amcl['initial_pose']['y'],1.97671)
            self.assertEqual(params['bt_navigator']['ros__parameters']['robot_base_frame'],'navcar_center')
            controller=params['controller_server']['ros__parameters']['FollowPath']
            self.assertEqual(controller['max_speed_xy'],load_navigation_speed(path))
            self.assertEqual(controller['max_speed_xy'],.5)
            self.assertEqual(controller['max_vel_theta'],0.)
            self.assertEqual(controller['critics'][0],'BaseObstacle')
            self.assertNotIn('&id',(output/'nav2.yaml').read_text())

    def test_global_speed_reaches_uart_once_and_localization_gate_is_tight(self):
        speed=navigation_speed(config())
        controller=Controller(dict(forward_units_per_mps=10,left_units_per_mps=10,rot_units_per_radps=1,
                                   max_translation_units=30,max_rot_units=6),clock=lambda:10.,
                              navigation_max_speed_mps=speed)
        controller.session.receive(frame(STATE,1,struct.pack('<4f',0,0,0,17)),10.);controller.yaw=17
        lease=controller.acquire_navigation()
        controller.navigation_command(dict(lease=lease,seq=1,forward=.03,left=.04))
        self.assertEqual(controller.desired(),(.3,-.4,0.,17))
        controller.navigation_command(dict(lease=lease,seq=2,forward=.3,left=.4))
        forward,left,rotation,yaw=controller.desired()
        self.assertAlmostEqual(forward,3.)
        self.assertAlmostEqual(left,-4.)
        self.assertEqual((rotation,yaw),(0.,17))
        with self.assertRaises(ValueError):
            controller.navigation_command(dict(lease=lease,seq=3,forward=.51,left=0))
        lower=config();lower['navigation']['max_speed_mps']=.1
        self.assertEqual(navigation_speed(lower),.1)
        legacy=config();legacy['navigation'].update(narrow_passage=False,max_speed_mps=.8)
        self.assertEqual(navigation_speed(legacy),.5)
        self.assertEqual(navigation_options(legacy)['resolution'],.025)
        covariance=[0.]*36;covariance[0]=covariance[7]=.000025
        self.assertTrue(localization_quality('map',covariance,.01)['ready'])
        covariance[7]=.000121
        self.assertFalse(localization_quality('map',covariance,.01)['ready'])
        self.assertFalse(localization_quality('map',covariance)['ready'])


if __name__=='__main__':unittest.main()
