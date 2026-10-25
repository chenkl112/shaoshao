"""Isolated ROS probe of mapping cloud/extrinsic recording and finalization."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import yaml

SOURCE=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(SOURCE/'web_panel'))
from pointcloud_index import RetrievalIndex,read_pcd,save_pcd,transform


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=True)
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header
    from std_srvs.srv import Trigger
    from tf2_ros import TransformBroadcaster
    rng=np.random.default_rng(3);world=rng.uniform([-3,-3,-.6],[3,3,1.],(12000,3))
    save_pcd(root/'site.pcd',world)
    offset=np.array([-.011,-.02329,.04412])
    config={'/**':{'ros__parameters':{'map_file_path':str(root/'site.pcd'),'common':{'odom_frame_id':'odom'},
                                    'mapping':{'extrinsic_T':offset.tolist(),'extrinsic_R':np.eye(3).ravel().tolist()}}}}
    path=root/'fast_lio.yaml';path.write_text(yaml.safe_dump(config))
    rclpy.init(args=[]);node=rclpy.create_node('index_recorder_probe')
    pub=node.create_publisher(PointCloud2,'/cloud_registered_body',10);tfpub=TransformBroadcaster(node)
    client=node.create_client(Trigger,'/finalize_localization_index')
    log=(root/'recorder.log').open('w')
    process=subprocess.Popen([sys.executable,str(SOURCE/'deployment/record_localization.py'),'--config',str(path)],
                             stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+8
        while not client.service_is_ready() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        assert client.service_is_ready(),'Recorder service did not become ready'
        for x in [0.,.5,1.]:
            for _ in range(12):
                stamp=node.get_clock().now().to_msg()
                tf=TransformStamped();tf.header=Header(stamp=stamp,frame_id='odom');tf.child_frame_id='imu'
                tf.transform.translation.x=x;tf.transform.rotation.w=1.;tfpub.sendTransform(tf)
                # IMU cloud and odom pose must yield the same static XYZ map.
                points=world-[x,0.,0.]
                pub.publish(create_cloud_xyz32(Header(stamp=stamp,frame_id='imu'),points))
                rclpy.spin_once(node,timeout_sec=.05)
        future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=15)
        assert future.done() and future.result().success,str(future.result()) if future.done() else 'Finalize timed out'
        index=RetrievalIndex(root/'keyframes',root/'site.pcd')
        assert len(index.poses)>=2,'Mapping movement did not record independent keyframes'
        from scipy.spatial import cKDTree
        tree=cKDTree(read_pcd(root/'site.pcd'));errors=[]
        meta=json.loads((root/'keyframes/manifest.json').read_text())
        for frame in meta['frames']:
            with np.load(root/'keyframes'/frame['file']) as data:
                world_points=transform(data['points'],data['pose'])
            distance,_=tree.query(world_points);errors.append(float(distance.max()))
        assert max(errors)<1e-5,'Recorder changed the optical/IMU reference'
        result={'passed':True,'frames':len(index.poses),'max_static_point_error_m':max(errors),
                'index_hash_bound':True,'kind':'synthetic_ROS_recorder_probe'}
        (root/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    finally:
        if process.poll() is None:process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
