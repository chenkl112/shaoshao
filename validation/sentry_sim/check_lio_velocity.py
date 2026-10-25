#!/usr/bin/env python3
"""Probe the newly built FAST-LIO velocity with independent synthetic IMU/clouds."""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import numpy as np
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Use a separate localhost-only ROS domain')
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from nav_msgs.msg import Odometry
    source=Path(__file__).resolve().parents[2]
    spec=importlib.util.spec_from_file_location('synthetic_mid360',source/'web_panel/validation/synthetic_mid360.py')
    synthetic=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(synthetic)
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    config=yaml.safe_load((source/'FAST_LIO/config/mid360.yaml').read_text())
    p=config['/**']['ros__parameters']
    p.update(filter_size_surf=.15,filter_size_map=.15,map_file_path=str(out/'unused.pcd'))
    p['common'].update(send_odom_base_tf=False,odom_frame_id='probe_odom',sensor_frame_id='probe_imu')
    p['mapping'].update(extrinsic_T=[-.011,-.02329,.04412],extrinsic_est_en=False)
    p['publish'].update(map_en=False,path_en=False,effect_map_en=False)
    p['pcd_save']['pcd_save_en']=False
    p.setdefault('map_filter',{})['dynamic_enabled']=False
    params=out/'params.yaml';params.write_text(yaml.safe_dump(config))
    rclpy.init();sensors=synthetic.SyntheticMID360();observer=rclpy.create_node('native_lio_velocity_probe')
    samples=[];started=time.monotonic();last=started;position=0.
    def motion():
        nonlocal last,position
        now=time.monotonic();speed=min(.08,max(0.,now-started-3.)*.15)
        position+=speed*(now-last);last=now
        sensors.pose=(position,0.,0.,speed,0.,0.)
    observer.create_timer(.01,motion)
    def odom(msg):
        samples.append((time.monotonic()-started,msg.twist.twist.linear.x,msg.twist.twist.linear.y,
                        list(msg.twist.covariance),list(msg.pose.covariance)))
    observer.create_subscription(Odometry,'/Odometry',odom,20)
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(sensors);executor.add_node(observer)
    spin=threading.Thread(target=executor.spin,daemon=True);spin.start()
    result={'passed':False,'expected_forward_mps':.08,'sensor_source':'SyntheticMID360 + analytic IMU; no physical hardware'}
    with (out/'fast_lio.log').open('w') as log:
        process=subprocess.Popen([str(args.binary),'--ros-args','--params-file',str(params)],
                                 stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+16.
            while time.monotonic()<deadline:
                if process.poll() is not None:
                    raise AssertionError('FAST-LIO process exited')
                time.sleep(.1)
            steady=[row for row in samples if row[0]>10.]
            assert len(steady)>=10,'Too few native odometry observations'
            velocity=np.asarray([row[1:3] for row in steady])
            covariance=np.asarray([row[3] for row in steady]).reshape(-1,6,6)
            pose_cov=np.asarray([row[4] for row in steady]).reshape(-1,6,6)
            mean=float(velocity[:,0].mean())
            assert abs(mean-.08)<.035,'Native estimated velocity does not match synthetic motion'
            assert np.isfinite(covariance).all() and np.isfinite(pose_cov).all(),'Non-finite covariance'
            assert np.linalg.eigvalsh(covariance).min()>=-1e-8,'Twist covariance is not positive semidefinite'
            assert covariance[:,0,0].max()>0.,'Velocity covariance was not populated'
            result.update(passed=True,observations=len(steady),mean_forward_mps=mean,
                          mean_lateral_mps=float(velocity[:,1].mean()),
                          velocity_std_mps=float(velocity[:,0].std()),
                          min_twist_covariance_eigenvalue=float(np.linalg.eigvalsh(covariance).min()))
        except Exception as exc:
            result['error']=f'{type(exc).__name__}: {exc}'
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3.)
            executor.shutdown(timeout_sec=3.);spin.join(3.)
            sensors.destroy_node();observer.destroy_node();rclpy.shutdown()
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
