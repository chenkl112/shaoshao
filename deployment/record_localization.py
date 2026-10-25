#!/usr/bin/env python3
"""Record sensor-local mapping keyframes; finalize only against saved site.pcd."""
from pathlib import Path
import argparse
import math
import sys
import numpy as np
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parent
    code=root/'shaoshao/web_panel'
    if not code.is_dir():code=root.parent/'web_panel'
    sys.path.insert(0,str(code))
    from pointcloud_index import IndexWriter,transform
    import rclpy
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import read_points_numpy
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer,TransformListener
    from scipy.spatial.transform import Rotation
    parameters=yaml.safe_load(args.config.read_text())['/**']['ros__parameters']
    map_path=Path(parameters['map_file_path'])
    imu_from_lidar=np.eye(4)
    imu_from_lidar[:3,3]=parameters['mapping']['extrinsic_T']
    imu_from_lidar[:3,:3]=np.asarray(parameters['mapping']['extrinsic_R']).reshape(3,3)
    writer=IndexWriter(map_path.parent/'keyframes',{
        'rings':20,'sectors':60,'radius_m':15.,'distance_m':.25,'angle_rad':math.radians(10)})
    # Even a zero-frame recording is visibly incomplete; it cannot be confused
    # with a legacy map that never had a recorder.
    writer._manifest(False)
    rclpy.init(args=[]);node=rclpy.create_node('shaoshao_localization_recorder')
    buffer=Buffer();listener=TransformListener(buffer,node)
    frozen=False;last_stamp=0
    def receive(msg):
        nonlocal last_stamp
        if frozen or msg.width*msg.height>200000:return
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if stamp<=last_stamp:return
        try:
            tf=buffer.lookup_transform(parameters['common'].get('odom_frame_id','odom'),msg.header.frame_id,
                                       Time.from_msg(msg.header.stamp)).transform
            t,q=tf.translation,tf.rotation
            world_from_imu=np.eye(4);world_from_imu[:3,3]=[t.x,t.y,t.z]
            world_from_imu[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            points=np.asarray(read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True)).reshape(-1,3)
            points=transform(points,np.linalg.inv(imu_from_lidar))
            radius=np.linalg.norm(points[:,:2],axis=1)
            points=points[np.isfinite(points).all(axis=1)&(radius>=.2)&(radius<15.)]
            if len(points)>60000:points=points[np.linspace(0,len(points)-1,60000,dtype=int)]
            writer.add(points,world_from_imu@imu_from_lidar,stamp);last_stamp=stamp
        except Exception as exc:node.get_logger().debug('Keyframe not ready: '+str(exc))
    def finalize(request,response):
        nonlocal frozen
        frozen=True
        try:
            writer.finalize(map_path)
            response.success=True;response.message=f'Finalized {len(writer.frames)} static localization keyframes'
        except Exception as exc:
            frozen=False;response.success=False;response.message=str(exc)
        return response
    subscription=node.create_subscription(PointCloud2,'/cloud_registered_body',receive,qos_profile_sensor_data)
    service=node.create_service(Trigger,'/finalize_localization_index',finalize)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
