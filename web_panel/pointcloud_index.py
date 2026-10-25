"""Map-bound ScanContext retrieval, adapted from shaoshao-hw/index.py (GPL-2.0).

Only recorded, finalized frames belonging to this exact PCD may supply poses.
Descriptors propose candidates; they never authorize motion.
"""
from pathlib import Path
import hashlib
import json
import math
import numpy as np

def transform(points,m):return np.asarray(points)@m[:3,:3].T+m[:3,3]

def delta(a,b):
    rotation=a[:3,:3].T@b[:3,:3]
    angle=math.acos(float(np.clip((np.trace(rotation)-1)/2,-1,1)))
    return float(np.linalg.norm(a[:3,3]-b[:3,3])),angle

def read_pcd(path):
    if Path(path).stat().st_size>512*1024*1024:raise ValueError('PCD exceeds 512 MiB limit')
    with Path(path).open('rb') as f:
        header={}
        while True:
            line=f.readline()
            if not line:raise ValueError('Incomplete PCD')
            parts=line.decode('ascii').strip().split()
            if parts and parts[0]!='#':header[parts[0]]=parts[1:]
            if parts and parts[0]=='DATA':break
        names=header['FIELDS']
        if len(set(names))!=len(names) or not all(n in names for n in ('x','y','z')):
            raise ValueError('PCD requires unique XYZ fields')
        if any(int(n)!=1 for n in header.get('COUNT',['1']*len(names))):
            raise ValueError('Only scalar PCD fields are supported')
        if header['DATA']==['binary']:
            types={('F','4'):'<f4',('F','8'):'<f8',('U','4'):'<u4',('I','4'):'<i4'}
            dtype=np.dtype([(n,types[(t,s)]) for n,t,s in zip(names,header['TYPE'],header['SIZE'])])
            data=np.frombuffer(f.read(),dtype=dtype)
            points=np.column_stack([data[n] for n in ('x','y','z')])
        elif header['DATA']==['ascii']:
            data=np.loadtxt(f,ndmin=2);points=data[:,[names.index(n) for n in ('x','y','z')]]
        else:raise ValueError('Only binary/ascii PCD supported')
    if len(points)!=int(header['POINTS'][0]):raise ValueError('PCD count mismatch')
    return points[np.isfinite(points).all(axis=1)]

def save_pcd(path,points):
    p=np.asarray(points,dtype='<f4').reshape(-1,3)
    header=f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(p)}\nHEIGHT 1\nPOINTS {len(p)}\nDATA binary\n'
    Path(path).write_bytes(header.encode()+p.tobytes())

def descriptor(points,rings=20,sectors=60,radius=15.):
    p=np.asarray(points,float);r=np.hypot(p[:,0],p[:,1])
    valid=np.isfinite(p).all(axis=1)&(r>=.2)&(r<radius)
    p,r=p[valid],r[valid]
    out=np.zeros((rings,sectors),np.float32)
    if not len(p):return out
    ring=np.minimum((r/radius*rings).astype(int),rings-1)
    sector=((np.arctan2(p[:,1],p[:,0])+math.pi)/(2*math.pi)*sectors).astype(int)%sectors
    # Positive height above a robust local floor, invariant to sensor world z.
    floor=float(np.quantile(p[:,2],.05))
    np.maximum.at(out,(ring,sector),np.clip(p[:,2]-floor+.1,.01,4.))
    return out

def descriptor_distance(query,target):
    qnorm=np.linalg.norm(query,axis=0);best=(float('inf'),0)
    for shift in range(query.shape[1]):
        moved=np.roll(target,shift,axis=1);tnorm=np.linalg.norm(moved,axis=0)
        valid=(qnorm>0)&(tnorm>0)
        if valid.sum()<6:continue
        similarity=np.sum(query[:,valid]*moved[:,valid],axis=0)/(qnorm[valid]*tnorm[valid])
        value=float(1-np.mean(similarity))
        if value<best[0]:best=(value,shift)
    return best

class IndexWriter:
    def __init__(self,directory,settings):
        self.root=Path(directory);self.root.mkdir(parents=True,exist_ok=True)
        self.settings=settings;self.frames=[];self.last_pose=None
    def add(self,points,sensor_pose,stamp):
        if len(points)<500:return False
        if self.last_pose is not None:
            d,a=delta(self.last_pose,sensor_pose)
            if d<self.settings['distance_m'] and a<self.settings['angle_rad']:return False
        name=f'frame-{len(self.frames):06d}.npz'
        path=self.root/name;tmp=path.with_suffix('.tmp')
        with tmp.open('wb') as f:np.savez_compressed(f,points=np.asarray(points,np.float32),pose=sensor_pose)
        tmp.replace(path)
        self.frames.append({'file':name,'stamp_ns':int(stamp)})
        self.last_pose=np.array(sensor_pose,copy=True)
        self._manifest(False)
        return True
    def _manifest(self,complete,map_hash=None):
        value={'version':1,'complete':complete,'frames':self.frames,'settings':self.settings,'map_sha256':map_hash}
        tmp=self.root/'manifest.tmp';tmp.write_text(json.dumps(value),encoding='utf-8');tmp.replace(self.root/'manifest.json')
    def finalize(self,map_path):
        from scipy.spatial import cKDTree
        points=read_pcd(map_path)
        if len(points)<500:raise ValueError('Static map is too sparse for global localization')
        tree=cKDTree(points);contexts=[];poses=[];kept=[]
        for frame in self.frames:
            with np.load(self.root/frame['file'],allow_pickle=False) as data:
                local=data['points'];sensor_pose=data['pose']
            distance,_=tree.query(transform(local,sensor_pose),workers=1)
            static=local[distance<=.1]
            if len(static)<500:continue
            contexts.append(descriptor(static,self.settings['rings'],self.settings['sectors'],self.settings['radius_m']))
            poses.append(sensor_pose);kept.append(frame)
        if not contexts:raise ValueError('No static keyframes; collect more overlapping observations')
        tmp=self.root/'index.tmp'
        with tmp.open('wb') as f:np.savez_compressed(f,contexts=contexts,poses=poses)
        tmp.replace(self.root/'index.npz');self.frames=kept
        self._manifest(True,hashlib.sha256(Path(map_path).read_bytes()).hexdigest())

class RetrievalIndex:
    def __init__(self,directory,map_path):
        root=Path(directory);meta=json.loads((root/'manifest.json').read_text(encoding='utf-8'))
        if meta.get('version')!=1 or not meta.get('complete'):raise ValueError('Map index not finalized')
        if meta.get('map_sha256')!=hashlib.sha256(Path(map_path).read_bytes()).hexdigest():
            raise ValueError('Map/index hash mismatch; rebuild or finalize the matching map')
        self.settings=meta['settings']
        for name,low,high in [('rings',4,100),('sectors',12,360),('radius_m',1,100)]:
            value=self.settings[name]
            if not isinstance(value,(int,float)) or not math.isfinite(value) or not low<=value<=high:
                raise ValueError('Invalid descriptor setting: '+name)
        if not isinstance(self.settings['rings'],int) or not isinstance(self.settings['sectors'],int):
            raise ValueError('Descriptor dimensions must be integers')
        with np.load(root/'index.npz',allow_pickle=False) as data:
            self.contexts=data['contexts'];self.poses=data['poses']
        count=len(meta.get('frames',[]))
        if (not 0<count<=20000 or self.contexts.shape!=(count,self.settings['rings'],self.settings['sectors']) or
                self.poses.shape!=(count,4,4) or not np.isfinite(self.contexts).all() or
                not np.isfinite(self.poses).all()):raise ValueError('Empty/invalid index')
        if (not np.allclose(self.poses[:,3,:],np.array([0,0,0,1]),atol=1e-6) or
                not np.allclose(np.swapaxes(self.poses[:,:3,:3],1,2)@self.poses[:,:3,:3],np.eye(3),atol=1e-5) or
                np.any(abs(np.linalg.det(self.poses[:,:3,:3])-1)>1e-5)):
            raise ValueError('Invalid keyframe poses')
        self.keys=self.contexts.mean(axis=2)
    def candidates(self,points,current_sensor_pose):
        q=descriptor(points,self.settings['rings'],self.settings['sectors'],self.settings['radius_m'])
        # Global localization must retain distant aliases for registration and
        # ambiguity checks. A short local-tracking shortlist can omit the true
        # place when visibility changes sharply at a passage exit.
        order=np.argsort(np.linalg.norm(self.keys-q.mean(axis=1),axis=1))[:40]
        ranked=[]
        for i in order:
            distance,shift=descriptor_distance(q,self.contexts[i])
            if not math.isfinite(distance):continue
            # roll(target,+shift) aligns target with query; query->target is -shift.
            angle=-shift*2*math.pi/self.settings['sectors']
            yaw=np.eye(4);yaw[:2,:2]=[[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]
            guess=self.poses[i]@yaw@np.linalg.inv(current_sensor_pose)
            ranked.append((distance,guess))
        result=[]
        for distance,guess in sorted(ranked,key=lambda x:x[0]):
            if any(delta(guess,prior)[0]<.5 and delta(guess,prior)[1]<.25 for prior in result):continue
            result.append(guess)
            if len(result)==12:break
        return result
