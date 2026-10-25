"""Native GICP tests: map-bound retrieval, ambiguity and extrinsic geometry."""
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np

from global_relocalization import angle_difference
from pointcloud_index import IndexWriter,save_pcd,transform
from pointcloud_relocalization import PointCloudMatcher,cloud_revision,search_saved_cloud
from navigation_recovery import RecoveryBridgeMixin


class PointCloudLocalizationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        rng=np.random.default_rng(124);surfaces=[]
        for center in [[-3,-2,0],[2,-1,.2],[1,3,.5],[-2,2,1]]:
            p=rng.uniform([-1,-.7,-.5],[1,.7,.5],(2000,3));axis=rng.integers(0,3,2000)
            p[np.arange(2000),axis]=np.where(rng.random(2000)<.5,-.5,.5)
            surfaces.append(p+center)
        self.cloud=np.concatenate(surfaces)
        self.pose=np.eye(4);self.pose[:3,3]=[-.7,-.3,.4]

    def tearDown(self):self.temp.cleanup()

    def write(self,cloud=None,poses=None):
        cloud=self.cloud if cloud is None else cloud
        save_pcd(self.root/'site.pcd',cloud)
        writer=IndexWriter(self.root/'keyframes',{'rings':20,'sectors':60,'radius_m':15.,'distance_m':.25,'angle_rad':.17})
        for i,pose in enumerate(poses or [self.pose]):
            local=transform(self.cloud,np.linalg.inv(pose))
            writer.add(local,pose,i+1)
        writer.finalize(self.root/'site.pcd')

    def query(self,yaw=1.4,translation=None):
        pose=self.pose.copy();c,s=math.cos(yaw),math.sin(yaw)
        pose[:2,:2]=[[c,-s],[s,c]]
        if translation is not None:pose[:3,3]=translation
        noise=np.random.default_rng(987).normal(0,.002,self.cloud.shape)
        return transform(self.cloud,np.linalg.inv(pose))+noise,pose

    def test_native_pcd_search_recovers_unknown_yaw_and_reference_offset(self):
        self.write();points,pose=self.query()
        extrinsic=np.eye(4);extrinsic[:3,3]=[.12,-.06,.3]
        expected=pose@np.linalg.inv(extrinsic)
        # No PNG/YAML is present: the successful result must come from PCD.
        result=PointCloudMatcher(self.root/'site.yaml').search(points,extrinsic,.297658)
        self.assertEqual(result['state'],'candidate',result)
        self.assertEqual(result['retrieval'],'keyframes')
        self.assertLess(np.linalg.norm(np.array(result['pose'][:2])-expected[:2,3]),.01)
        self.assertLess(angle_difference(result['pose'][2],1.4),.01)
        self.assertGreater(result['overlap'],.98)

    def test_repeated_geometry_at_two_distant_places_remains_ambiguous(self):
        offset=np.array([24.,0.,0.]);pose2=self.pose.copy();pose2[:3,3]+=offset
        # Both recorded local scans are identical, but belong to distinct places.
        save_pcd(self.root/'site.pcd',np.concatenate([self.cloud,self.cloud+offset]))
        writer=IndexWriter(self.root/'keyframes',{'rings':20,'sectors':60,'radius_m':15.,'distance_m':.25,'angle_rad':.17})
        writer.add(transform(self.cloud,np.linalg.inv(self.pose)),self.pose,1)
        writer.add(transform(self.cloud+offset,np.linalg.inv(pose2)),pose2,2)
        writer.finalize(self.root/'site.pcd')
        points,_=self.query()
        result=PointCloudMatcher(self.root/'site.yaml').search(points,np.eye(4),.297658)
        self.assertEqual(result['state'],'ambiguous',result)

    def test_incomplete_or_changed_map_index_cannot_downgrade_to_planar(self):
        self.write();points,_=self.query();path=self.root/'site.yaml'
        save_pcd(self.root/'site.pcd',self.cloud+[.001,0.,0.])
        result=search_saved_cloud(path,cloud_revision(path),.3,points,np.eye(4))
        self.assertEqual(result['state'],'map_error',result)
        self.assertIn('Map/index hash mismatch',result['detail'])
        meta=self.root/'keyframes/manifest.json';value=json.loads(meta.read_text());value['complete']=False
        meta.write_text(json.dumps(value))
        result=search_saved_cloud(path,cloud_revision(path),.3,points,np.eye(4))
        self.assertEqual(result['state'],'map_error',result)
        self.assertIn('not finalized',result['detail'])

    def test_low_overlap_and_insufficient_coverage_never_supply_a_seed(self):
        self.write();matcher=PointCloudMatcher(self.root/'site.yaml');points,_=self.query()
        points[::2]*=2.5
        self.assertNotEqual(matcher.search(points,np.eye(4),.3)['state'],'candidate')
        one_sided=np.random.default_rng(456).uniform([2.,-.02,-.2],[3.,.02,.2],(2000,3))
        self.assertEqual(matcher.search(one_sided,np.eye(4),.3)['state'],'insufficient_cloud_coverage')

    def test_selected_pcd_backend_requires_a_fresh_3d_packet(self):
        bridge=RecoveryBridgeMixin();bridge.panel=NS(navigation_control=NS(frames={'localization_backend':'pcd_gicp'}))
        bridge.latest_scan=('fresh','planar');bridge.latest_cloud=None
        self.assertIsNone(bridge.global_packet())
        bridge.latest_cloud=('fresh','3d')
        self.assertEqual(bridge.global_packet(),('fresh','3d'))


class PassageVisibilityTests(unittest.TestCase):
    def test_pcd_retrieval_keeps_the_true_place_after_exiting_a_narrow_passage(self):
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'validation/sentry_sim'))
        from scene import make_scene
        from cloud_scene import make_cloud_map,cloud_at
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);obstacles=make_scene(root,.65)
            make_cloud_map(root,obstacles)
            matcher=PointCloudMatcher(root/'site.yaml')
            # Regression: the former top-10/3 shortlist omitted this place and
            # proposed the opposite room after the passage's visibility changed.
            expected=[3.8462241540790565,-1.99591227364609,1.57]
            optical=[expected[0]-.01418*math.cos(expected[2]),expected[1]-.01418*math.sin(expected[2]),expected[2]]
            extrinsic=np.eye(4);extrinsic[:3,3]=[-.01418,0,.04412]
            result=matcher.search(cloud_at(optical,obstacles),extrinsic,.297658)
            self.assertEqual(result['state'],'candidate',result)
            self.assertLess(math.dist(result['pose'][:2],expected[:2]),.02)
            self.assertLess(angle_difference(result['pose'][2],expected[2]),.02)
            # Legacy maps without keyframes must also work with the real
            # installation height: ground returns are not 2D wall returns.
            from pointcloud_index import read_pcd
            import shutil
            static=read_pcd(root/'site.pcd');static[:,2]*=.327/.6
            save_pcd(root/'site.pcd',static);shutil.rmtree(root/'keyframes')
            points=cloud_at(optical,obstacles);points[:,2]*=.327/.6
            result=PointCloudMatcher(root/'site.yaml').search(points,extrinsic,.297658)
            self.assertIn(result['state'],('candidate','ambiguous'),result)
            self.assertEqual(result['retrieval'],'2d_candidates')
            self.assertLess(math.dist(result['pose'][:2],expected[:2]),.02)
            self.assertLess(result['normalized_error'],.005)


if __name__=='__main__':unittest.main()
