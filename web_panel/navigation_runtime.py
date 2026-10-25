"""ROS-independent recovery policy and checked, smooth planar command profiles."""
from dataclasses import dataclass
import math
import numpy as np


def map_odom_correction(map_pose, odom_pose):
    angle = map_pose['yaw']-odom_pose['yaw']
    c, s = math.cos(angle), math.sin(angle)
    return (map_pose['x']-c*odom_pose['x']+s*odom_pose['y'],
            map_pose['y']-s*odom_pose['x']-c*odom_pose['y'], angle)


def correction_jump(previous, current):
    return (math.dist(previous[:2], current[:2]) > .25 or
            abs(math.atan2(math.sin(current[2]-previous[2]), math.cos(current[2]-previous[2]))) > .3)


class SmoothVelocity:
    """Critically damped velocity tracking with vector acceleration/jerk bounds.

    Preview is side-effect free; commit only the command actually sent. Trapezoid
    integration bounds acceleration and jerk even with unequal sample intervals.
    A speed reserve permits jerk-limited braking before the hard speed envelope.
    """
    def __init__(self, acceleration=.3, jerk=1., speed=.5):
        if not all(math.isfinite(x) and x > 0 for x in (acceleration, jerk, speed)):
            raise ValueError('Invalid motion limits')
        self.amax, self.jmax, self.vmax = min(acceleration,math.sqrt(.4*jerk*speed)), jerk, speed
        self.reset()

    def reset(self,reason='reset'):
        self.reset_reason=reason
        self.v = (0.,0.)
        self.a = (0.,0.)
        self.sample_time = None
        return (0., 0.)

    @staticmethod
    def clip(value,limit):
        length=math.hypot(*value)
        scale=min(1.,limit/max(length,1e-15))
        return value[0]*scale,value[1]*scale

    def preview(self,target,dt):
        if len(target)!=2 or not all(math.isfinite(x) for x in target) or not 0<dt<=.18:
            return None
        target=self.clip(target,.8*self.vmax)
        jerk=self.clip(tuple(16*(target[i]-self.v[i])-8*self.a[i] for i in range(2)),self.jmax)
        acceleration=self.clip(tuple(self.a[i]+jerk[i]*dt for i in range(2)),self.amax)
        velocity=tuple(self.v[i]+.5*(self.a[i]+acceleration[i])*dt for i in range(2))
        return (velocity,acceleration) if math.hypot(*velocity)<=self.vmax+1e-9 else None

    def commit(self,preview,stamp=None):
        self.v,self.a=preview
        self.sample_time=stamp
        return self.v

    def step(self,target,dt,emergency=False):
        preview=None if emergency else self.preview(target,dt)
        return self.reset('emergency' if emergency else 'invalid_step') if preview is None else self.commit(preview)


class CollisionGrid:
    """Raw Nav2 costs, with known space and circular swept-footprint checks."""
    def __init__(self, values, resolution, origin, stamp, frame):
        from scipy.ndimage import distance_transform_edt,binary_erosion
        self.values = np.asarray(values)
        if (self.values.ndim != 2 or not self.values.size or not .005 <= resolution <= .2
                or not all(math.isfinite(x) for x in (*origin, stamp))):
            raise ValueError('Invalid collision grid')
        self.resolution, self.origin, self.stamp, self.frame = resolution, origin, stamp, frame
        self.height, self.width = self.values.shape
        occupied=self.values>=254
        self.boundary=occupied&~binary_erosion(occupied,structure=np.ones((3,3)),border_value=0)
        # Subdivide occupied SQUARES, preserving their full area. This refines
        # collision geometry only; it creates no new map/free-space observations.
        factor=4 if self.values.size<=65536 else 2 if self.values.size<=250000 else 1
        self.check_resolution=resolution/factor
        free=np.repeat(np.repeat(self.values<254,factor,axis=0),factor,axis=1)
        self.check_height,self.check_width=free.shape
        padded=np.pad(free,1,constant_values=False)
        self.distance=distance_transform_edt(padded)[1:-1,1:-1]*self.check_resolution
        self._safe_radius=None
        self._safe_cells=None

    def cell(self, x, y):
        ox, oy, angle = self.origin
        c, s = math.cos(angle), math.sin(angle)
        dx, dy = x-ox, y-oy
        ix = math.floor((c*dx+s*dy)/self.resolution)
        iy = math.floor((-s*dx+c*dy)/self.resolution)
        return (ix, iy) if 0 <= ix < self.width and 0 <= iy < self.height else None

    def clearance(self, point):
        ox,oy,yaw=self.origin;c,s=math.cos(yaw),math.sin(yaw)
        x,y=point[0]-ox,point[1]-oy
        ix=math.floor((c*x+s*y)/self.check_resolution)
        iy=math.floor((-s*x+c*y)/self.check_resolution)
        if not 0<=ix<self.check_width or not 0<=iy<self.check_height:
            return -math.inf
        return float(self.distance[iy,ix])-self.check_resolution*math.sqrt(2)

    def passage_speed_limit(self, pose, velocity, radius, cruise):
        """Slow before low-clearance space; soft inflation never forbids motion.

        Check 0.7 m ahead so normal jerk-limited braking starts before a throat.
        Full-hull checks remain separate from this speed preference.
        """
        speed=math.hypot(*velocity)
        if speed<1e-6:
            return cruise
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
        ux,uy=(c*velocity[0]-s*velocity[1])/speed,(s*velocity[0]+c*velocity[1])/speed
        gap=min(self.clearance((x+ux*d,y+uy*d))-radius for d in np.linspace(0.,.7,8))
        fraction=max(0.,min(1.,(gap-.025)/.16))
        fraction=fraction*fraction*(3.-2.*fraction)
        slow=min(.08,cruise)
        return slow+(cruise-slow)*fraction

    def centered_velocity(self, pose, velocity, radius):
        """Small lateral correction only where opposing walls form a throat.

        Sample cross-sections ahead in the commanded travel direction. A single
        wall or an opening cannot pull the robot sideways. The result remains a
        target for the smooth profile and the final swept-hull collision check.
        """
        speed=math.hypot(*velocity)
        if speed<1e-6:
            return velocity
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
        ux,uy=(c*velocity[0]-s*velocity[1])/speed,(s*velocity[0]+c*velocity[1])/speed
        nx,ny=-uy,ux
        offsets=np.arange(0.,radius+.26,self.check_resolution*.5)
        # Include the current hull span. Looking only ahead can follow a clear
        # future centerline while drifting into a protruding cell beside the
        # present body, especially after map->odom rotates during relocalization.
        ahead=np.array([-.15,0.,.15,.3,.45,.6])[:,None,None]
        side=np.array([-1.,1.])[None,:,None]*offsets[None,None,:]
        xx=x+ahead*ux+side*nx;yy=y+ahead*uy+side*ny
        ox,oy,angle=self.origin;gc,gs=math.cos(angle),math.sin(angle)
        ix=np.floor((gc*(xx-ox)+gs*(yy-oy))/self.resolution).astype(int)
        iy=np.floor((-gs*(xx-ox)+gc*(yy-oy))/self.resolution).astype(int)
        inside=(ix>=0)&(iy>=0)&(ix<self.width)&(iy<self.height)
        blocked=np.ones(ix.shape,dtype=bool)
        blocked[inside]=self.values[iy[inside],ix[inside]]>=254
        found=blocked.any(axis=2)
        distances=offsets[blocked.argmax(axis=2)]
        widths=distances.sum(axis=1)
        valid=found.all(axis=1)&(distances.min(axis=1)>radius*.5)&(widths>2*radius)&(widths<2*radius+.4)
        if not valid.any():
            return velocity
        # Use the narrowest section; average equivalent sections to avoid a
        # cell-edge tie changing the selected side on successive updates.
        width=widths[valid].min()
        selected=valid&(widths<=width+self.resolution)
        errors=(distances[selected,1]-distances[selected,0])/2
        progress=ahead[:,0,0][selected]
        midpoints=np.column_stack((x+progress*ux+errors*nx,y+progress*uy+errors*ny))
        if len(midpoints)>=2 and np.linalg.norm(midpoints[-1]-midpoints[0])>.08:
            # The midpoint line identifies the corridor axis even when DWB's
            # sampled velocity approaches it diagonally. Follow that axis,
            # rather than retaining a velocity component toward the wall.
            tangent=midpoints[-1]-midpoints[0];tangent/=np.linalg.norm(tangent)
            if tangent@[ux,uy]<0:tangent=-tangent
            normal=np.array([-tangent[1],tangent[0]])
            error=float((midpoints.mean(axis=0)-[x,y])@normal)
            correction=max(-.035,min(.035,.8*error))
            world=tangent*speed+normal*correction
            return c*world[0]+s*world[1],-s*world[0]+c*world[1]
        error=float(np.mean(errors))
        correction=max(-.035,min(.035,.8*error))
        return (velocity[0]+(c*nx+s*ny)*correction,
                velocity[1]+(-s*nx+c*ny)*correction)

    def safe_segment(self, start, end, radius):
        distance = math.dist(start, end)
        count = max(1, math.ceil(distance/(self.check_resolution*.5)))
        self.prepare(radius)
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        dx,dy=end[0]-start[0],end[1]-start[1]
        for i in range(count+1):
            x,y=start[0]+dx*i/count-ox,start[1]+dy*i/count-oy
            gx,gy=c*x+s*y,-s*x+c*y
            ix,iy=math.floor(gx/self.check_resolution),math.floor(gy/self.check_resolution)
            if not 0<=ix<self.check_width or not 0<=iy<self.check_height:
                return False
            if not self._safe_cells[iy*self.check_width+ix]:
                return self.exact_safe_segment(start,end,radius)
        return True

    def exact_safe_segment(self, start, end, radius):
        """Distance from a swept circle to full occupied/unknown grid squares.

        EDT is a cheap sufficient test. Near its discretization bound, compute
        the actual capsule/square distance instead of adding numerical padding.
        """
        ox,oy,angle=self.origin;c,s=math.cos(angle),math.sin(angle)
        points=np.asarray([start,end],dtype=float)-[ox,oy]
        points=points@np.array([[c,-s],[s,c]])
        cell=np.floor(points[0]/self.resolution).astype(int)
        if (not 0<=cell[0]<self.width or not 0<=cell[1]<self.height or
                self.values[cell[1],cell[0]]>=254):return False
        low=points.min(axis=0)-radius;high=points.max(axis=0)+radius
        if np.any(low<=0.) or np.any(high>=np.array([self.width,self.height])*self.resolution):
            return False
        # Include squares whose upper edge just touches the capsule bounds.
        first=np.maximum(0,np.floor((low-1e-10)/self.resolution).astype(int))
        last=np.floor(high/self.resolution).astype(int)+1
        # From a free starting center, a sweep can reach an occupied interior
        # only by crossing a boundary cell. Keep full boundary squares, while
        # avoiding repeated distance calculations for deep wall/unknown cells.
        rows,cols=np.nonzero(self.boundary[first[1]:last[1],first[0]:last[0]])
        if not len(rows):
            return True
        lower=np.column_stack((cols+first[0],rows+first[1]))*self.resolution
        upper=lower+self.resolution
        a,b=points;delta=b-a
        enter=np.zeros(len(rows));leave=np.ones(len(rows));overlap=np.ones(len(rows),dtype=bool)
        for axis in range(2):
            if abs(delta[axis])<1e-12:
                overlap&=(a[axis]>=lower[:,axis])&(a[axis]<=upper[:,axis])
            else:
                t1=(lower[:,axis]-a[axis])/delta[axis];t2=(upper[:,axis]-a[axis])/delta[axis]
                enter=np.maximum(enter,np.minimum(t1,t2));leave=np.minimum(leave,np.maximum(t1,t2))
        if np.any(overlap&(enter<=leave)):
            return False
        squared=math.inf
        for point in points:
            offset=np.maximum(np.maximum(lower-point,point-upper),0.)
            squared=min(squared,float(np.sum(offset**2,axis=1).min()))
        corners=np.stack((lower,upper,np.column_stack((lower[:,0],upper[:,1])),
                          np.column_stack((upper[:,0],lower[:,1])))).reshape(-1,2)
        length=float(delta@delta)
        t=np.clip((corners-a)@delta/length,0.,1.) if length>1e-20 else np.zeros(len(corners))
        squared=min(squared,float(np.sum((corners-a-t[:,None]*delta)**2,axis=1).min()))
        return squared>radius**2+1e-12

    def prepare(self,radius):
        if radius!=self._safe_radius:
            # Half a sampling step also covers the space between sampled poses.
            margin=self.check_resolution*(math.sqrt(2)+.25)
            self._safe_cells=(self.distance-margin>=radius).tobytes()
            self._safe_radius=radius

    def safe_velocity(self, pose, velocity, radius, now, acceleration=.3, jerk=1.):
        if not 0 <= now-self.stamp <= .5:
            return False
        x, y, yaw = pose
        vx, vy = velocity
        speed = math.hypot(vx, vy)
        # The collision response is an emergency stop. Certify its stopping
        # distance including 150 ms reaction and worst forward acceleration
        # during that latency. Normal comfort jerk does not delay that stop.
        reaction=.15
        peak=speed+acceleration*reaction
        distance=speed*reaction+.5*acceleration*reaction**2+peak**2/(2*acceleration)
        c, s = math.cos(yaw), math.sin(yaw)
        scale=distance/speed if speed>1e-9 else 0.
        end = (x+(c*vx-s*vy)*scale, y+(s*vx+c*vy)*scale)
        return self.safe_segment((x, y), end, radius)

    def escape(self, pose, radius, previous=(0., 0.), distance=.25):
        x, y, yaw = pose
        best = None
        # Reverse the previous map-frame travel first; lateral alternatives are
        # evaluated on the same swept footprint, never a fixed blind reverse.
        preferred = math.atan2(-previous[1], -previous[0]) if math.hypot(*previous) > .01 else yaw+math.pi
        reverse = (x+distance*math.cos(preferred), y+distance*math.sin(preferred))
        if self.safe_segment((x, y), reverse, radius):
            return reverse
        for length in (distance, .15):
            for angle in [preferred]+[preferred+i*math.pi/12 for i in range(1, 24)]:
                end = (x+length*math.cos(angle), y+length*math.sin(angle))
                if not self.safe_segment((x, y), end, radius):
                    continue
                score = self.clearance(end)-.01*abs(math.atan2(math.sin(angle-preferred), math.cos(angle-preferred)))
                if best is None or score > best[0]:
                    best = (score, end)
            if best is not None:
                break
        return None if best is None else best[1]


@dataclass
class RecoveryTask:
    goal: dict
    revision: str
    state: str = 'running'
    since: float = 0.
    retries: int = 0
    recoveries: int = 0
    next_attempt: float = 0.
    reason: str = ''
    escape_goal: tuple | None = None
    permanent: bool = False

    def failed(self, code, now):
        self.retries += 1
        self.escape_goal = None
        self.since = now
        self.permanent=code in (101,201,203,204,205,206)
        if code in (105, 104, 106, 107) and self.recoveries < 3:
            self.state = 'recovering'
            self.recoveries += 1
        else:
            self.state = 'waiting'
        self.next_attempt = now+min(5., .5*2**min(self.retries-1, 4))
        if self.retries >= 8 or self.permanent:
            self.state = 'blocked'
            self.next_attempt = now+5.

    def localized(self, now):
        self.state, self.since, self.next_attempt = 'waiting', now, now+.5
        self.escape_goal = None

    def status(self):
        return {'state': self.state, 'reason': self.reason, 'retries': self.retries,
                'recoveries': self.recoveries, 'goal': dict(self.goal),'requires_attention':self.permanent}
