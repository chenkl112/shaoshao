"""Measured planar reference points; no chassis heading is inferred."""
import math

from navcar_settings import navigation_options
from robot_io import finite_number


def navigation_frames(config,lio):
    options=navigation_options(config)
    result={**options,'base_frame':'navcar_imu','center_offset':[0.,0.,0.],
            'static_transforms':[]}
    if options['reference']=='imu':return result
    yaw=config['yaw'];offset=yaw.get('lidar_offset_xy_m')
    if not isinstance(offset,list) or len(offset)!=2 or not all(finite_number(v) for v in offset):
        raise ValueError('中心参考点需要实测 yaw.lidar_offset_xy_m：[x向前,y向左]，单位米')
    length=yaw.get('lidar_offset_radius_m')
    if not finite_number(length) or abs(math.hypot(*offset)-length)>1e-6:
        raise ValueError('雷达偏心向量长度与 lidar_offset_radius_m 不一致')
    matrix=lio['mapping'].get('extrinsic_R',[1.,0.,0.,0.,1.,0.,0.,0.,1.])
    identity=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
    # This installation has aligned LiDAR/IMU axes. Refuse an unhandled rotation
    # instead of silently using the identity with a different calibration.
    if len(matrix)!=9 or any(not finite_number(v) or abs(v-w)>1e-8 for v,w in zip(matrix,identity)):
        raise ValueError('当前中心参考点实现要求 MID360 内部 extrinsic_R 为单位阵')
    t=lio['mapping']['extrinsic_T']
    if len(t)!=3 or not all(finite_number(v) for v in t):raise ValueError('无效的 mapping.extrinsic_T')
    center=[t[0]-offset[0],t[1]-offset[1],0.]
    result.update(base_frame='navcar_center',center_offset=center,
                  static_transforms=[{'child':'navcar_center','xyz':center},
                                     {'child':'navcar_lidar','xyz':list(t)}])
    return result


def shift_pose(x,y,yaw,offset):
    return (x+math.cos(yaw)*offset[0]-math.sin(yaw)*offset[1],
            y+math.sin(yaw)*offset[0]+math.cos(yaw)*offset[1])


def initial_reference_pose(pose,offset):
    pose=pose or {'x':0.,'y':0.,'yaw':0.}
    previous=pose.get('reference_offset_xy_m',[0.,0.])
    values=[pose.get('x'),pose.get('y'),pose.get('yaw')]
    if not isinstance(previous,list) or len(previous)!=2 or not all(finite_number(v) for v in values+previous):
        raise ValueError('保存的初始位姿或参考点无效')
    x,y=shift_pose(*values,[offset[0]-previous[0],offset[1]-previous[1]])
    return {'x':x,'y':y,'yaw':values[2]}


def shift_planar_covariance(covariance,yaw,offset):
    # Jacobian of translating a planar pose. Preserve all original covariance
    # terms, including position/yaw correlations, rather than just relabeling it.
    j=[[float(i==k) for k in range(6)] for i in range(6)]
    j[0][5]=-math.sin(yaw)*offset[0]-math.cos(yaw)*offset[1]
    j[1][5]=math.cos(yaw)*offset[0]-math.sin(yaw)*offset[1]
    c=[covariance[i*6:i*6+6] for i in range(6)]
    left=[[sum(j[i][k]*c[k][v] for k in range(6)) for v in range(6)] for i in range(6)]
    return [sum(left[i][k]*j[v][k] for k in range(6)) for i in range(6) for v in range(6)]


def shift_body_twist(linear,angular,covariance,offset):
    """Translate a measured body twist to a rigidly attached reference point."""
    if len(covariance)!=36 or not all(math.isfinite(v) for v in (*linear,*angular,*offset,*covariance)):
        raise ValueError('Invalid measured odometry twist')
    x,y,z=offset;wx,wy,wz=angular
    velocity=(linear[0]+wy*z-wz*y,linear[1]+wz*x-wx*z,linear[2]+wx*y-wy*x)
    j=[[float(i==k) for k in range(6)] for i in range(6)]
    j[0][4]=z;j[0][5]=-y;j[1][3]=-z;j[1][5]=x;j[2][3]=y;j[2][4]=-x
    c=[covariance[i*6:i*6+6] for i in range(6)]
    left=[[sum(j[i][k]*c[k][v] for k in range(6)) for v in range(6)] for i in range(6)]
    return velocity,[sum(left[i][k]*j[v][k] for k in range(6)) for i in range(6) for v in range(6)]
