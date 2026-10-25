"""Analytic 3D rays against the same room/fixture boxes and ground plane.

Like the existing LaserScan adapter, this tests localization integration,
not MID360 firmware or FAST-LIO. The global map is built before test motion.
"""
import math
from pathlib import Path
import numpy as np


def cloud_at(pose,obstacles):
    azimuth,elevation=np.meshgrid(np.linspace(-math.pi,math.pi,360,endpoint=False),
                                  np.deg2rad(np.linspace(-35,35,16)))
    azimuth,elevation=azimuth.ravel(),elevation.ravel()
    directions=np.column_stack((np.cos(elevation)*np.cos(azimuth+pose[2]),
                                np.cos(elevation)*np.sin(azimuth+pose[2]),np.sin(elevation)))
    origin=np.array([pose[0],pose[1],.6])
    distance=np.full(len(directions),math.inf)
    down=directions[:,2]<-1e-8;distance[down]=-.6/directions[down,2]
    for matrix,size in obstacles:
        rotation=matrix[:2,:2]
        local_origin=np.r_[rotation.T@(origin[:2]-matrix[:2,2]),origin[2]-size[2]/2]
        local_directions=np.column_stack((directions[:,:2]@rotation,directions[:,2]))
        half=np.asarray(size)/2
        with np.errstate(divide='ignore',invalid='ignore'):
            first=(-half-local_origin)/local_directions;second=(half-local_origin)/local_directions
        parallel=abs(local_directions)<1e-10
        lower=np.minimum(first,second);upper=np.maximum(first,second)
        lower[parallel]=-math.inf;upper[parallel]=math.inf
        outside=np.any(parallel&(abs(local_origin)>half),axis=1)
        near=lower.max(axis=1);far=upper.min(axis=1)
        hit=(~outside)&(far>=np.maximum(near,0))&(far>0)
        entry=np.where(near>0,near,far)
        distance[hit]=np.minimum(distance[hit],entry[hit])
    valid=(distance>.2)&(distance<15.)
    relative=directions[valid]*distance[valid,None]
    c,s=math.cos(pose[2]),math.sin(pose[2])
    relative[:,:2]=relative[:,:2]@np.array([[c,-s],[s,c]])
    return relative


def make_cloud_map(directory,obstacles):
    from global_relocalization import MapScanMatcher
    from pointcloud_index import IndexWriter,save_pcd,transform
    root=Path(directory);matcher=MapScanMatcher(root/'site.yaml',.297658)
    centers=matcher.coarse_centers()
    groups=np.floor(centers/.4).astype(int)
    _,selected=np.unique(groups,axis=0,return_index=True)
    writer=IndexWriter(root/'keyframes',{'rings':20,'sectors':60,'radius_m':15.,'distance_m':.25,'angle_rad':.17})
    accumulated=[]
    for i,position in enumerate(centers[selected]):
        optical=[position[0]-.01418,position[1],0.]
        points=cloud_at(optical,obstacles)
        pose=np.eye(4);pose[:2,3]=optical[:2]
        writer.add(points,pose,i+1);accumulated.append(transform(points,pose))
    points=np.concatenate(accumulated)
    _,keep=np.unique(np.floor(points/.025).astype(np.int32),axis=0,return_index=True)
    save_pcd(root/'site.pcd',points[keep]);writer.finalize(root/'site.pcd')
    return {'virtual_keyframes':len(writer.frames),'pcd_points':len(keep)}
