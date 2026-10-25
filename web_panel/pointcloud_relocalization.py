"""Saved PCD / keyframe global search in the isolated localization process.

Search has no access to the last map pose. The lidar/base extrinsic is the
only live transform supplied. AMCL remains the sole map->odom broadcaster.
"""
from functools import lru_cache
from pathlib import Path
import math
import time
import numpy as np

from pointcloud_index import RetrievalIndex,read_pcd,transform,delta


def cloud_revision(path):
    root=Path(path).parent
    result=[]
    for name in ('site.pcd','keyframes/manifest.json','keyframes/index.npz'):
        file=root/name
        stat=file.stat() if file.exists() else None
        result.append((name,(stat.st_mtime_ns,stat.st_size) if stat else None))
    return tuple(result)


def rigid_matrix(value):
    value=np.asarray(value,float)
    if (value.shape!=(4,4) or not np.isfinite(value).all() or
            not np.allclose(value[3],[0,0,0,1],atol=1e-6) or
            not np.allclose(value[:3,:3].T@value[:3,:3],np.eye(3),atol=1e-5) or
            abs(np.linalg.det(value[:3,:3])-1)>1e-5):
        raise ValueError('Invalid rigid transform')
    return value


def planar_scan(points,extrinsic):
    """Wall-height nearest returns; ground height follows the installation."""
    base_points=transform(points,extrinsic)
    floor=float(np.quantile(base_points[:,2],.05))
    selected=base_points[(base_points[:,2]>floor+.1)&(base_points[:,2]<floor+1.5),:2]
    if len(selected)<30:return np.empty((0,2))
    angle=np.arctan2(selected[:,1],selected[:,0]);bins=((angle+math.pi)*72/math.pi).astype(int)%144
    order=np.lexsort((np.linalg.norm(selected,axis=1),bins))
    _,first=np.unique(bins[order],return_index=True)
    return selected[order[first]]


class PointCloudMatcher:
    def __init__(self,path):
        import small_gicp
        self.path=Path(path);self.root=self.path.parent
        self.points=read_pcd(self.root/'site.pcd')
        if len(self.points)<500:raise ValueError('PCD has fewer than 500 finite points')
        self.target,self.tree=small_gicp.preprocess_points(
            np.asarray(self.points,dtype=np.float64),downsampling_resolution=.05,num_neighbors=20,num_threads=2)
        if self.target.size()<500:raise ValueError('PCD too sparse after voxel filtering')
        root=self.root/'keyframes'
        # An existing incomplete/mismatched index must fail, never downgrade.
        self.index=RetrievalIndex(root,self.root/'site.pcd') if root.exists() else None

    def search(self,cloud,base_from_lidar,radius,budget=5.):
        import small_gicp
        from global_relocalization import cached_matcher,map_revision
        started=time.monotonic();deadline=started+budget
        points=np.asarray(cloud,float);extrinsic=rigid_matrix(base_from_lidar)
        if points.ndim!=2 or points.shape[1]!=3 or len(points)<1000 or not np.isfinite(points).all():
            return {'state':'insufficient_cloud','backend':'pcd_gicp'}
        if len(points)>180000:return {'state':'oversized_cloud','backend':'pcd_gicp'}
        bearings=np.arctan2(points[:,1],points[:,0])
        sectors=np.bincount(((bearings+math.pi)*6/math.pi).astype(int)%12,minlength=12)
        if (sectors>=10).sum()<6:return {'state':'insufficient_cloud_coverage','backend':'pcd_gicp'}
        planar=(cached_matcher(str(self.path),map_revision(self.path),radius) if self.path.is_file() else None)
        scan=planar_scan(points,extrinsic) if planar is not None else None
        if planar is not None and len(scan)<30:return {'state':'insufficient_cloud_slice','backend':'pcd_gicp'}
        if self.index is not None:
            seeds=self.index.candidates(points,np.eye(4))
            retrieval='keyframes'
        else:
            # Older maps have no keyframes. Their 2D map supplies global XY/yaw
            # hypotheses; the saved 3D map must still validate them with GICP.
            if planar is None:return {'state':'map_error','detail':'Legacy PCD needs its matching 2D map','backend':'pcd_gicp'}
            result=planar.search(scan,extrinsic[:2,3],budget=max(.1,deadline-time.monotonic()))
            alternatives=result.get('alternatives',[])
            seeds=[]
            # The mapping origin is at the lidar's initial pose; a fresh
            # registration solves height as well. This fallback cannot rescue
            # maps whose 2D projection has no usable known-free observations.
            for candidate in alternatives:
                x,y,yaw=candidate['pose'];c,s=math.cos(yaw),math.sin(yaw)
                base=np.eye(4);base[:2,:2]=[[c,-s],[s,c]];base[:2,3]=[x,y]
                seed=base@extrinsic;seed[2,3]=0.
                seeds.append(seed)
            retrieval='2d_candidates'
            if not seeds:return {**result,'backend':'pcd_gicp','retrieval':retrieval}
        source,_=small_gicp.preprocess_points(points,downsampling_resolution=.05,num_neighbors=20,num_threads=2)
        if source.size()<500:return {'state':'insufficient_cloud','backend':'pcd_gicp'}
        candidates=[];rejections=[]
        for seed in seeds:
            if time.monotonic()>deadline:return {'state':'search_timeout','backend':'pcd_gicp'}
            result=small_gicp.align(self.target,source,self.tree,rigid_matrix(seed),
                max_correspondence_distance=.5,num_threads=2,max_iterations=32,
                rotation_epsilon=.001,translation_epsilon=.001)
            matrix=np.asarray(result.T_target_source)
            h=np.asarray(result.H);eigen=np.linalg.eigvalsh((h+h.T)*.5)
            overlap=float(result.num_inliers)/source.size()
            error=float(result.error)/max(1,result.num_inliers)
            condition=float(eigen[-1]/eigen[0]) if eigen[0]>0 else math.inf
            finite=lambda value:float(value) if math.isfinite(value) else None
            metrics={'overlap':finite(overlap),'normalized_error':finite(error),'min_eigenvalue':finite(eigen[0]),
                     'condition_number':finite(condition),'converged':bool(result.converged)}
            if (not result.converged or not np.isfinite(matrix).all() or not np.isfinite(eigen).all()
                    # Global guesses need a tighter fit than HW's .30 gate for
                    # local correction around an already known position.
                    or not math.isfinite(error) or error>.05 or overlap<.80 or eigen[0]<1000 or condition>1e6):
                rejections.append(metrics);continue
            base=matrix@np.linalg.inv(extrinsic)
            # AMCL is planar; don't flatten a large roll/pitch or flipped pose.
            if base[2,2]<math.cos(.12):continue
            pose=[float(base[0,3]),float(base[1,3]),float(math.atan2(base[1,0],base[0,0]))]
            # Point-to-surface overlap alone can make parallel rooms look
            # equivalent. Rank the actual fit and the observed free rays in
            # the saved 2D map together, before declaring ambiguity.
            quality=planar.quality(pose,scan,extrinsic[:2,3]) if planar is not None else None
            score=overlap*math.exp(-error/.05)
            if quality is not None:
                if quality['matched_ratio']<.85 or quality['ray_conflict']>.02 or quality['known_coverage']<.9:continue
                score*=quality['score']
            candidates.append({'pose':pose,'sensor_pose':matrix.tolist(),'overlap':overlap,
                               'normalized_error':error,'min_eigenvalue':float(eigen[0]),
                               'condition_number':condition,'score':score,'planar_quality':quality})
        ordered=[]
        for candidate in sorted(candidates,key=lambda c:-c['score']):
            if any(delta(np.asarray(candidate['sensor_pose']),np.asarray(old['sensor_pose']))[0]<.35 and
                   delta(np.asarray(candidate['sensor_pose']),np.asarray(old['sensor_pose']))[1]<.2 for old in ordered):continue
            ordered.append(candidate)
        common={'backend':'pcd_gicp','retrieval':retrieval,'elapsed_s':time.monotonic()-started}
        if not ordered:return {'state':'no_match',**common,'rejections':rejections}
        best=ordered[0]
        # Two plausible, distinct places remain ambiguous even if one has a
        # slightly better fitness. Descriptors alone cannot break that tie.
        margin=best['score']-(ordered[1]['score'] if len(ordered)>1 else 0.)
        state='no_match' if best['score']<.8 else 'ambiguous' if margin<.07 else 'candidate'
        return {'state':state,**best,**common,'margin':margin,'alternatives':ordered}


@lru_cache(maxsize=1)
def cached_cloud_matcher(path,revision):
    if cloud_revision(path)!=revision:raise ValueError('Pointcloud map changed before search')
    result=PointCloudMatcher(path)
    if cloud_revision(path)!=revision:raise ValueError('Pointcloud map changed during search')
    return result


def search_saved_cloud(path,revision,radius,points,base_from_lidar):
    try:
        matcher=cached_cloud_matcher(path,revision)
        result=matcher.search(points,base_from_lidar,radius)
        if cloud_revision(path)!=revision:return {'state':'stale_map','backend':'pcd_gicp'}
        return result
    except ImportError as exc:
        return {'state':'backend_unavailable','backend':'pcd_gicp','detail':str(exc)}
    except (OSError,ValueError,KeyError,RuntimeError) as exc:
        return {'state':'map_error','backend':'pcd_gicp','detail':str(exc)}
