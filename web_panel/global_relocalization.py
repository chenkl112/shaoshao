"""Map-wide planar scan matching. Candidates seed AMCL, never publish a TF.

The worker receives only a saved map and a scan expressed in the robot frame.
It has no last pose, global odometry, simulator truth or navigation target.
"""
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from functools import lru_cache
import math
import multiprocessing
from pathlib import Path
import time

import numpy as np
import yaml

from navigation_runtime import CollisionGrid


def angle_difference(a,b):
    return abs(math.atan2(math.sin(a-b),math.cos(a-b)))


def separated(a,b,distance=.45,angle=.26):
    return math.dist(a[:2],b[:2])>distance or angle_difference(a[2],b[2])>angle


def map_revision(path):
    path=Path(path)
    meta=yaml.safe_load(path.read_text())
    image=path.parent/meta['image'];stat=image.stat()
    return f'{path.stat().st_mtime_ns}-{stat.st_mtime_ns}-{stat.st_size}'


class MapScanMatcher:
    def __init__(self,path,radius):
        from PIL import Image
        from scipy.ndimage import distance_transform_edt
        path=Path(path);meta=yaml.safe_load(path.read_text())
        self.resolution=float(meta['resolution'])
        self.origin=tuple(meta['origin']);self.radius=float(radius)
        if not .005<=self.resolution<=.2:raise ValueError('Unsupported map resolution')
        with Image.open(path.parent/meta['image']) as image:
            pixels=np.flipud(np.asarray(image.convert('L'))).astype(float)
        probability=pixels/255. if meta.get('negate') else (255-pixels)/255.
        self.free=probability<float(meta['free_thresh'])
        self.occupied=probability>float(meta['occupied_thresh'])
        self.known=self.free|self.occupied
        self.height,self.width=pixels.shape
        if pixels.size>16000000:raise ValueError('Map exceeds grid limit')
        self.distance=distance_transform_edt(~self.occupied)*self.resolution
        self.center_distance=distance_transform_edt(np.pad(self.free,1))[1:-1,1:-1]*self.resolution
        self.fields={sigma:np.exp(-.5*(np.minimum(self.distance,2.)/sigma)**2).astype(np.float32)
                     for sigma in (.22,.07)}
        values=np.where(self.occupied,254,np.where(self.free,0,255)).astype(np.uint8)
        self.grid=CollisionGrid(values,self.resolution,self.origin,0.,'map')
        self.centers=None

    def coarse_centers(self):
        if self.centers is not None:return self.centers
        rows,cols=np.nonzero(self.free&(self.center_distance>=self.radius))
        stride=max(1,math.ceil(.1/self.resolution))
        if not len(rows):return np.empty((0,2))
        # Pick a known center from EVERY occupied coarse block instead of
        # sampling one fixed lattice cell, which can skip a thin corridor.
        while True:
            groups=(rows//stride)*math.ceil(self.width/stride)+cols//stride
            clearance=np.minimum(self.center_distance[rows,cols],self.radius+.2)
            tie=((rows%stride)-(stride-1)/2)**2+((cols%stride)-(stride-1)/2)**2
            order=np.lexsort((tie,-clearance,groups))
            _,first=np.unique(groups[order],return_index=True)
            chosen=order[first]
            if len(chosen)<=16000:break
            stride+=1
        xx=(cols[chosen]+.5)*self.resolution;yy=(rows[chosen]+.5)*self.resolution
        ox,oy,angle=self.origin;c,s=math.cos(angle),math.sin(angle)
        self.centers=np.column_stack((ox+c*xx-s*yy,oy+s*xx+c*yy))
        return self.centers

    def indices(self,xx,yy):
        ox,oy,yaw=self.origin;c,s=math.cos(yaw),math.sin(yaw)
        ix=np.floor((c*(xx-ox)+s*(yy-oy))/self.resolution).astype(np.int32)
        iy=np.floor((-s*(xx-ox)+c*(yy-oy))/self.resolution).astype(np.int32)
        inside=(ix>=0)&(iy>=0)&(ix<self.width)&(iy<self.height)
        return ix,iy,inside

    def rank(self,poses,points,sigma):
        poses=np.asarray(poses);c=np.cos(poses[:,2,None]);s=np.sin(poses[:,2,None])
        xx=poses[:,0,None]+c*points[None,:,0]-s*points[None,:,1]
        yy=poses[:,1,None]+s*points[None,:,0]+c*points[None,:,1]
        ix,iy,inside=self.indices(xx,yy)
        values=np.zeros(xx.shape,dtype=np.float32)
        values[inside]=self.fields[sigma][iy[inside],ix[inside]]*self.known[iy[inside],ix[inside]]
        return values.mean(axis=1)

    def quality(self,pose,points,sensor):
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
        rotated=np.column_stack((c*points[:,0]-s*points[:,1],s*points[:,0]+c*points[:,1]))+[x,y]
        ix,iy,inside=self.indices(rotated[:,0],rotated[:,1])
        distance=np.full(len(points),math.inf);known=np.zeros(len(points),dtype=bool)
        distance[inside]=self.distance[iy[inside],ix[inside]]
        known[inside]=self.known[iy[inside],ix[inside]]
        matched=float(np.mean((distance<=.12)&known))
        endpoint=float(np.mean(np.exp(-.5*(distance/.07)**2)*known))
        # Sample the observed free ray interiors as well as their endpoints.
        fractions=np.array([.15,.3,.5,.7,.85])
        interior=sensor+(points-sensor)[:,None,:]*fractions[None,:,None]
        xx=x+c*interior[:,:,0]-s*interior[:,:,1]
        yy=y+s*interior[:,:,0]+c*interior[:,:,1]
        rx,ry,valid=self.indices(xx,yy)
        blocked=np.ones(xx.shape,dtype=bool);seen=np.zeros(xx.shape,dtype=bool)
        blocked[valid]=self.occupied[ry[valid],rx[valid]]
        seen[valid]=self.known[ry[valid],rx[valid]]
        contradiction=float(blocked.mean());coverage=float(seen.mean())
        score=endpoint*max(0.,1.-3.*contradiction)*coverage
        return {'score':score,'matched_ratio':matched,'ray_conflict':contradiction,'known_coverage':coverage}

    def polish(self,pose,points):
        from scipy.ndimage import map_coordinates
        from scipy.optimize import least_squares
        initial=np.asarray(pose,dtype=float)
        ox,oy,angle=self.origin;gc,gs=math.cos(angle),math.sin(angle)
        def residual(value):
            x,y,yaw=value;c,s=math.cos(yaw),math.sin(yaw)
            xx=x+c*points[:,0]-s*points[:,1]-ox
            yy=y+s*points[:,0]+c*points[:,1]-oy
            return map_coordinates(self.distance,[( -gs*xx+gc*yy)/self.resolution-.5,
                                                  (gc*xx+gs*yy)/self.resolution-.5],order=1,mode='constant',cval=1.)
        result=least_squares(residual,initial,bounds=(initial-[.04,.04,.02],initial+[.04,.04,.02]),
                             loss='soft_l1',f_scale=.06,max_nfev=25,diff_step=1e-4)
        return result.x

    def search(self,points,sensor=(0.,0.),budget=5.):
        started=time.monotonic();deadline=started+budget
        points=np.asarray(points,dtype=float);sensor=np.asarray(sensor,dtype=float)
        if points.ndim!=2 or points.shape[1]!=2 or len(points)<30 or not np.isfinite(points).all():
            return {'state':'insufficient_scan'}
        points=points[np.linspace(0,len(points)-1,min(144,len(points)),dtype=int)]
        angles=np.arctan2(points[:,1]-sensor[1],points[:,0]-sensor[0])
        sectors=np.bincount(np.floor((angles+math.pi)/(math.pi/4)).astype(int)%8,minlength=8)
        if (sectors>=3).sum()<4:return {'state':'insufficient_scan_coverage'}
        if not self.occupied.any():return {'state':'no_map_features'}
        centers=self.coarse_centers()
        if not len(centers):return {'state':'no_known_space'}
        coarse=points[::2]
        ranked=[]
        for yaw in np.arange(-math.pi,math.pi,math.pi/36):
            if time.monotonic()>deadline:return {'state':'search_timeout'}
            poses=np.column_stack((centers,np.full(len(centers),yaw)))
            scores=self.rank(poses,coarse,.22)
            keep=np.argsort(scores)[-min(48,len(scores)):]
            ranked.extend((float(scores[i]),poses[i]) for i in keep)
        seeds=[]
        for score,pose in sorted(ranked,key=lambda item:item[0],reverse=True):
            if all(separated(pose,old,.4,.25) for old in seeds):seeds.append(pose)
            if len(seeds)>=40:break
        offsets=np.array([(x,y,a) for x in np.arange(-.1,.101,.025) for y in np.arange(-.1,.101,.025)
                          for a in np.arange(-3.,3.01,.5)*math.pi/180])
        refined=[]
        for seed in seeds:
            if time.monotonic()>deadline:return {'state':'search_timeout'}
            poses=seed+offsets;scores=self.rank(poses,points,.07)
            for i in np.argsort(scores)[-40:][::-1]:
                candidate=poses[i]
                if self.grid.safe_segment(tuple(candidate[:2]),tuple(candidate[:2]),self.radius):
                    fitted=self.polish(candidate,points)
                    if (self.rank([fitted],points,.07)[0]>=scores[i] and
                            self.grid.safe_segment(tuple(fitted[:2]),tuple(fitted[:2]),self.radius)):
                        candidate=fitted
                    quality=self.quality(candidate,points,sensor)
                    refined.append({**quality,'pose':[float(v) for v in candidate]})
                    break
        ordered=[]
        for candidate in sorted(refined,key=lambda item:item['score'],reverse=True):
            if all(separated(candidate['pose'],old['pose'],.35,.20) for old in ordered):ordered.append(candidate)
        if not ordered:return {'state':'no_match'}
        best=ordered[0];runner=next((c for c in ordered[1:] if separated(c['pose'],best['pose'])),None)
        margin=best['score']-(runner['score'] if runner else 0.)
        state=('candidate' if best['matched_ratio']>=.85 and best['score']>=.80 and
               best['ray_conflict']<=.02 and best['known_coverage']>=.9 else 'no_match')
        if state=='candidate' and margin<.07:state='ambiguous'
        return {'state':state,**best,'margin':margin,'alternatives':ordered[:4],
                'elapsed_s':time.monotonic()-started}


@lru_cache(maxsize=1)
def cached_matcher(path,revision,radius):
    if map_revision(path)!=revision:raise ValueError('Map changed before search')
    result=MapScanMatcher(path,radius)
    if map_revision(path)!=revision:raise ValueError('Map changed during search')
    return result


def search_saved_map(path,revision,radius,points,sensor):
    try:
        result=cached_matcher(path,revision,radius).search(points,sensor)
        if map_revision(path)!=revision:return {'state':'stale_map'}
        return result
    except (OSError,ValueError,yaml.YAMLError) as exc:
        return {'state':'map_error','detail':str(exc)}


class GlobalRelocalizer:
    """One isolated worker and one pending request; stale results are discarded."""
    def __init__(self):
        self.pool=None;self.future=None;self.token=None

    def submit(self,token,path,revision,radius,points,sensor,*,cloud=False):
        if self.future is not None:return False
        if self.pool is None:
            self.pool=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'))
        self.token=token
        if cloud:
            from pointcloud_relocalization import search_saved_cloud
            search=search_saved_cloud
        else:search=search_saved_map
        try:self.future=self.pool.submit(search,str(path),revision,radius,points,sensor)
        except (BrokenProcessPool,RuntimeError):
            self.pool.shutdown(wait=False,cancel_futures=True);self.pool=None;self.token=None
            return False
        return True

    def poll(self,token):
        if self.future is None or not self.future.done():return None
        future,saved=self.future,self.token;self.future=self.token=None
        try:result=future.result()
        except Exception as exc:result={'state':'worker_error','detail':str(exc)}
        return result if token==saved else None

    def invalidate(self):
        self.token=None

    def close(self):
        self.invalidate()
        if self.pool is not None:self.pool.shutdown(wait=False,cancel_futures=True)
