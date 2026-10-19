import math
from pathlib import Path
import struct
import tempfile
import unittest

import yaml
from navcar_move_web import Controller
from navcar_navigation import navigation_geometry,build_navigation_config,localization_quality
from navcar_protocol import STATE,frame
from navcar_settings import navigation_speed,load_navigation_speed,limit_translation


class NavigationTests(unittest.TestCase):
    def test_shared_speed_limits_diagonal_and_uart_conversion_once(self):
        config={'navigation':{'max_speed_mps':.5}}
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'robot.yaml'
            self.assertEqual(load_navigation_speed(path),.1)
            path.write_text(yaml.safe_dump(config),encoding='utf-8')
            speed=load_navigation_speed(path)
        controller=Controller(dict(forward_units_per_mps=10,left_units_per_mps=10,rot_units_per_radps=1,
                                   max_translation_units=30,max_rot_units=6),clock=lambda:10.,
                              navigation_max_speed_mps=speed)
        controller.session.receive(frame(STATE,1,struct.pack('<4f',0,0,0,17)),10.)
        controller.yaw=17
        lease=controller.acquire_navigation()
        vx,vy=limit_translation(.5,.5,speed)
        self.assertAlmostEqual(math.hypot(vx,vy),.5)
        controller.navigation_command(dict(lease=lease,seq=1,forward=vx,left=vy))
        forward,right,rot,yaw=controller.desired()
        self.assertAlmostEqual(math.hypot(forward,right),5.)
        self.assertEqual((rot,yaw),(0,17))
        self.assertEqual(controller.state()['navigation_max_speed_mps'],.5)
        with self.assertRaises(ValueError):
            controller.navigation_command(dict(lease=lease,seq=2,forward=.5,left=.5))
        controller.navigation_command(dict(lease=lease,seq=2,forward=.5,left=0))
        self.assertEqual(controller.desired(),(5.,0.,0.,17))
        limited=Controller(dict(forward_units_per_mps=10,left_units_per_mps=20,rot_units_per_radps=1,
                                max_translation_units=3,max_rot_units=6),navigation_max_speed_mps=.5)
        self.assertEqual(limited.navigation_max_speed,.15)
        for invalid in (0,-1,True,None,float('nan'),float('inf'),3.01):
            with self.assertRaises(ValueError):navigation_speed({'navigation':{'max_speed_mps':invalid}})
        with self.assertRaises(ValueError):navigation_speed({'navigation':None})

    def test_measured_octagon_encloses_all_yaws_about_actual_imu_origin(self):
        config={'robot':{'circumscribed_radius_m':.276011,'lidar_height_m':.327},
                'yaw':{'axis_xyz_m':[0,0,.280],'lidar_offset_radius_m':.01418,'imu_rpy_rad':[0,0,0]}}
        extrinsic=[-.011,-.02329,.04412]
        radius,height=navigation_geometry(config,extrinsic)
        chassis_radius=.510/(2*math.cos(math.pi/8))
        self.assertGreaterEqual(config['robot']['circumscribed_radius_m'],chassis_radius)
        self.assertAlmostEqual(radius,.276011+.01418+math.hypot(*extrinsic[:2]))
        self.assertEqual(height,.327)
        # All octagon vertices under independent chassis/sensor angles and
        # every sampled direction of the measured 14.18 mm optical eccentricity.
        for chassis in range(0,360,15):
            for sensor in range(0,360,30):
                for eccentric in range(0,360,30):
                    offset=math.radians(eccentric)
                    ox,oy=.01418*math.cos(offset)+extrinsic[0],.01418*math.sin(offset)+extrinsic[1]
                    for vertex in range(8):
                        angle=math.radians(chassis-sensor+45*vertex)
                        self.assertLessEqual(math.hypot(chassis_radius*math.cos(angle)-ox,chassis_radius*math.sin(angle)-oy),radius+1e-12)
        with self.assertRaises(ValueError):navigation_geometry(config)
        config['robot']['circumscribed_radius_m']=0
        with self.assertRaises(ValueError):navigation_geometry(config,extrinsic)

    def test_amcl_zero_variance_roundoff_is_valid_but_real_invalid_variances_are_not(self):
        covariance=[0.]*36
        covariance[0]=-4.0332320816460765e-17
        covariance[7]=-1.1535911115245767e-15
        covariance[35]=8.01581023779379e-14
        quality=localization_quality('map',covariance)
        self.assertTrue(quality['ready']);self.assertEqual(quality['position_std_m'],0.)
        self.assertFalse(localization_quality('odom',covariance)['ready'])
        for value in (-1e-6,float('nan'),float('inf'),.091):
            covariance[0]=value
            self.assertFalse(localization_quality('map',covariance)['ready'])
        self.assertIn('尚未收敛',localization_quality('map',covariance)['message'])

    def setUp(self):
        self.now=10.
        self.c=Controller(dict(forward_units_per_mps=1,left_units_per_mps=1,rot_units_per_radps=1,
                               max_translation_units=3.,max_rot_units=.5,yaw_target_deg=0),clock=lambda:self.now)
        self.c.session.receive(frame(STATE,1,struct.pack('<4f',0,0,0,17)),self.now);self.c.yaw=17

    def move(self,lease,seq=1,forward=.06,left=.08):
        self.c.navigation_command(dict(lease=lease,seq=seq,forward=forward,left=left))

    def test_lease_does_not_move_until_command_and_limits_diagonal(self):
        lease=self.c.acquire_navigation()
        self.assertFalse(self.c.armed)
        self.assertEqual(self.c.desired(),(0,0,0,17))
        self.move(lease)
        self.assertEqual(self.c.desired(),(.06,-.08,0,17))
        with self.assertRaises(ValueError):self.move(lease,2,.1,.1)
        with self.assertRaises(ValueError):self.move(lease,2,float('nan'),0)
        self.now+=.201
        self.assertEqual(self.c.desired(),(0,0,0,17))

    def test_manual_takeover_rejects_late_auto_and_release_preserves_manual(self):
        lease=self.c.acquire_navigation();self.move(lease)
        self.c.command(dict(action='left',strength=.2,client='phone',seq=1))
        self.assertEqual(self.c.desired(),(0,-.2,0,17))
        with self.assertRaises(RuntimeError):self.move(lease,2)
        self.c.release_navigation(lease)
        self.assertEqual(self.c.desired(),(0,-.2,0,17))

    def test_reordered_auto_cannot_restart_stopped_motion(self):
        lease=self.c.acquire_navigation();self.move(lease,1)
        self.move(lease,3,0,0);self.move(lease,2)
        self.assertEqual(self.c.desired(),(0,0,0,17))
        self.c.release_navigation(lease)
        with self.assertRaises(RuntimeError):self.move(lease,4)

    def test_manual_stop_revokes_and_busy_manual_blocks_acquire(self):
        self.c.command(dict(action='forward',strength=.2,client='phone',seq=1))
        with self.assertRaises(RuntimeError):self.c.acquire_navigation()
        self.c.command(dict(action='stop',strength=.2,client='phone',seq=2))
        lease=self.c.acquire_navigation();self.move(lease)
        self.c.command(dict(action='stop',strength=.2,client='phone',seq=3))
        with self.assertRaises(RuntimeError):self.move(lease,2)
        self.assertEqual(self.c.desired(),(0,0,0,17))

    def test_geometry_encloses_chassis_and_uses_measured_pose_frame(self):
        config={'robot':{'length_m':.55,'width_m':.55,'lidar_height_m':.32},
                'yaw':{'axis_xyz_m':[0,0,.3],'imu_xyz_m':[.02,0,None],'imu_rpy_rad':[0,0,0]},
                'navigation':{'max_speed_mps':.5}}
        radius,height=navigation_geometry(config)
        self.assertAlmostEqual(radius,math.hypot(.275,.275)+.02)
        self.assertEqual(height,.32)
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);repo=root/'shaoshao/web_panel';repo.mkdir(parents=True)
            (repo/'robot.yaml').write_text(yaml.safe_dump(config))
            runtime=root/'mapping_runtime';runtime.mkdir()
            lio={'/**':{'ros__parameters':{'common':{},'pcd_save':{},'publish':{},
                 'map_filter':{'dynamic_enabled':True},'mapping':{'extrinsic_T':[0,0,.04412]}}}}
            (runtime/'fast_lio.yaml').write_text(yaml.safe_dump(lio))
            maps=root/'maps';maps.mkdir();(maps/'site.yaml').write_text('image: saved.png')
            out=build_navigation_config(root,maps,{'x':1.,'y':2.,'yaw':.3})
            params=yaml.safe_load((out/'nav2.yaml').read_text())
            self.assertNotIn('&id',(out/'nav2.yaml').read_text(),'ROS parameter files cannot contain YAML aliases')
            ctrl=params['controller_server']['ros__parameters']['FollowPath']
            self.assertEqual(ctrl['max_vel_theta'],0.)
            self.assertEqual(ctrl['max_speed_xy'],.5)
            self.assertEqual(ctrl['max_vel_x'],.5)
            self.assertEqual(ctrl['min_vel_y'],-.5)
            # Complete DWB scores recorded at an actual 105 stall (2026-10-18).
            # Moving toward the goal was legal but entered cost 100 inflation;
            # the old .1 weight preferred stopping indefinitely at cost zero.
            # Path/goal scales are converted to grid units by DWB (resolution / 2).
            resolution=params['local_costmap']['local_costmap']['ros__parameters']['resolution']
            observed=[(0.,0.,1.,10.),(-.1,100.,1.,7.),(-.05,86.,0.,9.),(.1,0.,4.,13.)]
            def score(row,obstacle_scale):
                _,obstacle,path,goal=row
                return (obstacle*obstacle_scale+resolution/2*
                        (path*ctrl['PathDist.scale']+goal*ctrl['GoalDist.scale']))
            self.assertEqual(min(observed,key=lambda row:score(row,.1))[0],0.)
            best=min(observed,key=lambda row:score(row,ctrl['BaseObstacle.scale']))
            self.assertLess(best[0],0.,'legal progress must beat stopping at the inflation boundary')
            self.assertGreater(ctrl['BaseObstacle.scale'],0.)
            self.assertEqual(ctrl['critics'][0],'BaseObstacle')
            self.assertEqual(params['amcl']['ros__parameters']['initial_pose']['x'],1.)
            cost=params['global_costmap']['global_costmap']['ros__parameters']
            self.assertEqual(cost['resolution'],.05)
            self.assertAlmostEqual(cost['robot_radius'],radius)
            self.assertEqual(cost['robot_base_frame'],'navcar_imu')
            self.assertFalse(yaml.safe_load((out/'fast_lio.yaml').read_text())['/**']['ros__parameters']['pcd_save']['pcd_save_en'])
            self.assertFalse(yaml.safe_load((out/'fast_lio.yaml').read_text())['/**']['ros__parameters']['map_filter']['dynamic_enabled'])
            config['robot']['circumscribed_radius_m']=.276011
            config['robot']['lidar_height_m']=.327
            config['yaw']['lidar_offset_radius_m']=.01418
            config['yaw']['axis_xyz_m']=[0,0,.280]
            config['robot']['length_m']=config['robot']['width_m']=None
            config['yaw']['imu_xyz_m']=[None,None,None]
            (repo/'robot.yaml').write_text(yaml.safe_dump(config))
            lio['/**']['ros__parameters']['mapping']['extrinsic_T']=[-.011,-.02329,.04412]
            (runtime/'fast_lio.yaml').write_text(yaml.safe_dump(lio))
            params=yaml.safe_load((build_navigation_config(root,maps)/'nav2.yaml').read_text())
            measured=params['global_costmap']['global_costmap']['ros__parameters']
            self.assertAlmostEqual(measured['robot_radius'],.276011+.01418+math.hypot(.011,.02329))
            self.assertEqual(measured['footprint_padding'],.02)
            self.assertAlmostEqual(params['navigation_scan']['ros__parameters']['min_height'],-.18288)
        config['robot']['width_m']=None
        config['robot'].pop('circumscribed_radius_m')
        with self.assertRaises(ValueError):navigation_geometry(config)


if __name__=='__main__':unittest.main()
