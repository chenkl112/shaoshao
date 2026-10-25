"""Independent synthetic scans, ambiguity rejection and localization authority."""
from concurrent.futures import Future
import math
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image
import yaml

from global_relocalization import GlobalRelocalizer,MapScanMatcher,angle_difference
from navigation_recovery import RecoveryBridgeMixin
from navigation_runtime import RecoveryTask


class GlobalLocalizationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.res=.05;self.origin=(-4.,-3.,0.)
        xx,yy=np.meshgrid(-4+(np.arange(160)+.5)*self.res,-3+(np.arange(120)+.5)*self.res)
        self.xx,self.yy=xx,yy
        self.walls=(abs(xx)>=3.5)|(abs(yy)>=2.5)

    def tearDown(self):self.temp.cleanup()

    def save_map(self,asymmetric=True,origin=None,unknown=False):
        occupied=self.walls.copy()
        if asymmetric:
            occupied|=((self.xx>.5)&(self.xx<1.3)&(self.yy>-1.7)&(self.yy<-.9))
            occupied|=((self.xx>-1.4)&(self.xx<-.8)&(self.yy>.8)&(self.yy<2.2))
            # A distinct north-wall alcove breaks the near-180-degree room alias.
            occupied|=((self.xx>-.6)&(self.xx<.5)&(self.yy>1.8)&(self.yy<2.5))
        self.occupied=occupied
        pixels=np.where(occupied,0,205 if unknown else 254).astype(np.uint8)
        Image.fromarray(np.flipud(pixels)).save(self.root/'site.png')
        path=self.root/'site.yaml'
        path.write_text(yaml.safe_dump({'image':'site.png','resolution':self.res,'origin':list(origin or self.origin),
                                      'negate':0,'occupied_thresh':.65,'free_thresh':.196}))
        return path

    def scan(self,pose):
        # Ray sampling is independent of the matcher's endpoint distance field.
        angles=np.linspace(-math.pi,math.pi,360,endpoint=False)
        distance=np.arange(.025,10.,.0125)
        c,s=math.cos(pose[2]),math.sin(pose[2]);origin=np.array(pose[:2])+np.array([-.01418*c,-.01418*s])
        bearing=angles+pose[2]
        xx=origin[0]+np.cos(bearing)[:,None]*distance
        yy=origin[1]+np.sin(bearing)[:,None]*distance
        ix=np.floor((xx+4)/self.res).astype(int);iy=np.floor((yy+3)/self.res).astype(int)
        inside=(ix>=0)&(iy>=0)&(ix<160)&(iy<120)
        hit=np.ones(ix.shape,dtype=bool);hit[inside]=self.occupied[iy[inside],ix[inside]]
        ranges=distance[hit.argmax(axis=1)]
        return np.column_stack((ranges*np.cos(angles)-.01418,ranges*np.sin(angles)))

    def test_finds_translation_and_heading_without_any_initial_pose(self):
        matcher=MapScanMatcher(self.save_map(),.297658)
        for expected in [(-2.,-1.,1.2),(2.,1.,-2.3)]:
            with self.subTest(pose=expected):
                result=matcher.search(self.scan(expected),(-.01418,0.))
                self.assertEqual(result['state'],'candidate',result)
                self.assertLess(math.dist(result['pose'][:2],expected[:2]),.1)
                self.assertLess(angle_difference(result['pose'][2],expected[2]),.08)

    def test_symmetric_map_is_ambiguous_and_cannot_supply_a_seed(self):
        matcher=MapScanMatcher(self.save_map(asymmetric=False),.297658)
        result=matcher.search(self.scan((1.,-.7,.5)),(-.01418,0.))
        self.assertEqual(result['state'],'ambiguous',result)
        self.assertLess(result['margin'],.07)

    def test_rotated_map_origin_preserves_world_pose(self):
        expected=(-2.,-1.,1.2)
        self.save_map();points=self.scan(expected)
        angle=.4;c,s=math.cos(angle),math.sin(angle)
        origin=(c*-4-s*-3,s*-4+c*-3,angle)
        matcher=MapScanMatcher(self.save_map(origin=origin),.297658)
        result=matcher.search(points,(-.01418,0.))
        target=(c*expected[0]-s*expected[1],s*expected[0]+c*expected[1],expected[2]+angle)
        self.assertEqual(result['state'],'candidate',result)
        self.assertLess(math.dist(result['pose'][:2],target[:2]),.1)
        self.assertLess(angle_difference(result['pose'][2],target[2]),.08)

    def test_unknown_map_and_one_sided_scan_do_not_guess(self):
        matcher=MapScanMatcher(self.save_map(unknown=True),.297658)
        self.assertEqual(matcher.search(self.scan((-2.,-1.,1.2)),(-.01418,0.))['state'],'no_known_space')
        matcher=MapScanMatcher(self.save_map(),.297658)
        angles=np.linspace(-.2,.2,60)
        points=np.column_stack((2*np.cos(angles),2*np.sin(angles)))
        self.assertEqual(matcher.search(points)['state'],'insufficient_scan_coverage')

    def test_coarse_search_keeps_thin_free_passages_between_lattice_samples(self):
        path=self.save_map()
        pixels=np.where(self.walls|((abs(self.xx)<.9)&(abs(self.yy)>.325)),0,254).astype(np.uint8)
        Image.fromarray(np.flipud(pixels)).save(self.root/'site.png')
        matcher=MapScanMatcher(path,.297658)
        centers=matcher.coarse_centers()
        self.assertTrue(np.any((abs(centers[:,0])<.7)&(abs(centers[:,1])<.05)),
                        'coarse search skipped a passage that fits the real hull')

    def test_conflicting_scan_does_not_supply_a_confident_global_seed(self):
        matcher=MapScanMatcher(self.save_map(),.297658)
        points=self.scan((-2.,-1.,1.2));sensor=np.array([-.01418,0.])
        points[::5]=sensor+(points[::5]-sensor)*.7
        self.assertNotEqual(matcher.search(points,sensor)['state'],'candidate')

    def test_result_from_canceled_or_changed_session_is_discarded(self):
        worker=GlobalRelocalizer();worker.future=Future();worker.token=('old-map',1)
        worker.invalidate();worker.future.set_result({'state':'candidate','pose':[3.,2.,1.]})
        self.assertIsNone(worker.poll(('new-map',2)))
        self.assertIsNone(worker.future)

    def test_covariance_alone_cannot_reenable_a_pending_global_recovery(self):
        from navcar_navigation import NavigationBridge
        panel=NS(lock=threading.RLock(),nav_localized=False,navigation_control=NS(frames={
            'position_std_limit':.05,'position_std_stop_limit':.075}))
        bridge=NS(panel=panel,match_frames=0)
        msg=NS(header=NS(frame_id='map'),pose=NS(covariance=[0.]*36))
        NavigationBridge.localized(bridge,msg)
        self.assertFalse(panel.nav_localized)

    def test_global_audit_can_revoke_a_wrong_tracking_pose_but_never_authorize(self):
        now=time.monotonic();worker=GlobalRelocalizer()
        worker.token=(0,0,'map','audit');worker.future=Future()
        worker.future.set_result({'state':'candidate','pose':[4.,-3.,-1.1]})
        class Bridge(RecoveryBridgeMixin):pass
        bridge=Bridge();bridge.panel=NS(pose={'x':0.,'y':0.,'yaw':0.},map_pose={'x':0.,'y':0.,'yaw':0.},nav_localized=True)
        bridge.global_locator=worker;bridge.grid_epoch=bridge.global_generation=0
        bridge.global_status={'state':'idle'};bridge.global_job={'at':now,'odom':{'x':0.,'y':0.,'yaw':0.}}
        bridge.global_candidate=bridge.global_proposal=None
        bridge.good_map_odom=(0.,0.,0.)
        bridge.audit_global_pose(now,'map')
        self.assertTrue(bridge.global_needed)
        self.assertFalse(bridge.panel.nav_localized)
        self.assertEqual(bridge.good_map_odom,(0.,0.,0.))

    def test_unverified_global_candidate_does_not_replace_last_trusted_correction(self):
        now=time.monotonic();goal={'x':4.,'y':-2.}
        panel=NS(navigation_control=NS(running=True),closed=False,switching=False,
                 pose={'x':0.,'y':0.,'yaw':0.,'frame':'navcar_odom'},map_pose={'x':3.,'y':2.,'yaw':.8},
                 map_pose_time=now,odom_time=now,cloud_time=now,measured_velocity=(0.,0.),nav_localized=False,
                 nav_status='recovering',nav_message='')
        class Bridge(RecoveryBridgeMixin):pass
        bridge=Bridge();bridge.panel=panel
        bridge.task=RecoveryTask(goal,'map',state='relocalizing',since=now-2.)
        bridge.global_needed=True;bridge.global_candidate={'correction':(3.,2.,.8),'at':now-.2}
        bridge.global_status={'state':'verifying'};bridge.global_tick=lambda *args:None
        bridge.last_nomotion=now;bridge.nomotion_client=None
        bridge.localization_info={'ready':True};bridge.localization_at=bridge.match_at=now-.01
        bridge.match_ratio=.95;bridge.match_frames=3;bridge.good_since=now-.2;bridge.bad_since=0.
        bridge.good_map_odom=(0.,0.,0.);bridge.last_good_pose={'x':0.,'y':0.,'yaw':0.}
        bridge.last_seed=now-.2;bridge.last_global=0.;bridge.last_good_uncertainty=(.03,.08)
        bridge.recovery_tick(revision='map')
        self.assertFalse(panel.nav_localized)
        self.assertEqual(bridge.good_map_odom,(0.,0.,0.))
        self.assertEqual(bridge.last_good_pose,{'x':0.,'y':0.,'yaw':0.})

    def test_confirmed_pose_waits_for_both_costmaps_after_clear_ack(self):
        class Bridge(RecoveryBridgeMixin):pass
        bridge=Bridge();futures=[Future(),Future()]
        bridge.map_clear_clients=[NS(service_is_ready=lambda:True,call_async=lambda req,f=f:f) for f in futures]
        bridge.map_clear_futures=[];bridge.map_clear_at=bridge.map_clear_ack_at=0.
        bridge.local_grid=None;bridge.global_costmap_at=0.;bridge.global_status={}
        bridge.grid_condition=threading.Condition();bridge.grid_epoch=0;bridge.pending_grid='old'
        bridge.global_locator=NS(invalidate=lambda:None)
        bridge.panel=NS(pose={'x':0.,'y':0.,'frame':'odom'},navigation_control=NS(frames={'collision_radius':.297658}))
        service=NS(ClearEntireCostmap=NS(Request=lambda:None))
        with patch.dict('sys.modules',{'nav2_msgs':NS(srv=service),'nav2_msgs.srv':service}):
            self.assertFalse(bridge.clear_relocalized_maps(10.))
            futures[0].set_result(None)
            self.assertFalse(bridge.clear_relocalized_maps(11.))
            futures[1].set_result(None)
            self.assertFalse(bridge.clear_relocalized_maps(12.))
            self.assertIsNone(bridge.pending_grid)
            self.assertEqual(bridge.grid_epoch,1)
            bridge.local_grid=NS(stamp=11.,frame='odom',safe_segment=lambda *args:True)
            bridge.global_costmap_at=13.
            self.assertFalse(bridge.clear_relocalized_maps(13.))
            bridge.local_grid.stamp=13.;bridge.global_costmap_at=11.
            self.assertFalse(bridge.clear_relocalized_maps(13.))
            bridge.global_costmap_at=13.
            self.assertTrue(bridge.clear_relocalized_maps(13.))


if __name__=='__main__':unittest.main()
