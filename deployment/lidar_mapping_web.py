#!/usr/bin/env python3
"""Live, lidar-only mapping panel. No motor-command publishers."""
import argparse,json,math,os,re,secrets,signal,struct,sys,threading,time,zlib
from pathlib import Path
from types import SimpleNamespace
from urllib.parse import urlparse
from http.server import ThreadingHTTPServer


def encode_grid(msg):
    import numpy as np
    w,h=msg.info.width,msg.info.height
    values=np.asarray(msg.data,dtype=np.int16).reshape(h,w)
    if not np.any(values>=0):return None
    pixels=np.full((h,w),205,dtype=np.uint8)
    pixels[(values>=0)&(values<50)]=254;pixels[values>=50]=0
    rows=b''.join(b'\0'+row.tobytes() for row in pixels[::-1])
    def chunk(kind,data):return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(rows))+chunk(b'IEND',b'')


def grid_metadata(msg,filename):
    q=msg.info.origin.orientation;o=msg.info.origin.position
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    # encode_grid writes unknown as gray 205, whose occupancy is 50/255.
    # A free threshold of .25 loads that unknown gray as FREE in Nav2.
    return {'image':filename,'resolution':msg.info.resolution,'origin':[o.x,o.y,yaw],
            'negate':0,'occupied_thresh':.65,'free_thresh':.196,'mode':'trinary'}


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from livox_ros_driver2.msg import CustomMsg
    from nav_msgs.msg import Odometry,OccupancyGrid
    from std_srvs.srv import Trigger
    import yaml
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bind',default='0.0.0.0');p.add_argument('--port',type=int,default=8080)
    p.add_argument('--maps-dir',type=Path,required=True);a=p.parse_args()
    root=Path(__file__).resolve().parent
    sys.path.insert(0,str(root/'shaoshao/web_panel'))
    import server as upstream
    from lidar_mapping_control import MappingControl,LidarMappingPanelMixin
    control=MappingControl(root,a.maps_dir)
    class Panel(LidarMappingPanelMixin,upstream.Panel):
        pass
    panel=Panel(SimpleNamespace(demo=False,maps_dir=a.maps_dir,max_points=6000,max_speed=.3,max_yaw_rate=.5))
    panel.mapping_control=control
    def mapping_started(maps):
        with panel.lock:
            panel.args.maps_dir=maps
            panel.logs=maps/'web_logs';panel.logs.mkdir(exist_ok=True)
            panel.pose=None;panel.odom_time=0.;panel.cloud_time=0.
            panel.cloud={'points':[],'frame':'','sequence':panel.cloud['sequence']+1}
    control.on_started=mapping_started
    panel.mode='mapping' if control.is_running() else 'idle'
    rclpy.init(args=[]);node=rclpy.create_node('shaoshao_lidar_mapping_web')
    navigation=None
    try:
        from navcar_navigation import NavigationControl,NavigationBridge
        from nav2_msgs.action import NavigateToPose
        panel.navigation_control=NavigationControl(root,panel,control)
        navigation=NavigationBridge(node,panel,root)
        panel.bridge=navigation
    except (ImportError,OSError) as exc:
        panel.navigation_control=None
        node.get_logger().warning('Navigation unavailable: '+str(exc))
    def navigation_running():return bool(getattr(panel,'navigation_control',None) and panel.navigation_control.running)
    def odom_cb(msg):
        if not control.is_running():return
        pos=msg.pose.pose.position;q=msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in [pos.x,pos.y,pos.z,q.x,q.y,q.z,q.w]):return
        with panel.lock:
            panel.pose={'x':pos.x,'y':pos.y,'z':pos.z,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),'frame':msg.header.frame_id}
            panel.odom_time=time.monotonic()
    last_cloud=0.
    def cloud_cb(msg):
        nonlocal last_cloud
        if not control.is_running() and not navigation_running():return
        now=time.monotonic()
        if now-last_cloud<.2:return
        last_cloud=now
        points=upstream.decode_cloud(msg,6000)
        with panel.lock:
            panel.cloud={'points':points,'frame':msg.header.frame_id,'sequence':panel.cloud['sequence']+1,'source':'mid360_registered'}
            panel.cloud_time=now if points else 0.
    last_raw=0.
    def raw_cb(msg):
        nonlocal last_raw
        if control.is_running() or navigation_running():return
        now=time.monotonic()
        if now-last_raw<.2:return
        last_raw=now
        step=max(1,math.ceil(len(msg.points)/6000))
        points=[[float(p.x),float(p.y),float(p.z)] for p in msg.points[::step]
                if all(math.isfinite(v) for v in (p.x,p.y,p.z))]
        with panel.lock:
            panel.pose=None;panel.odom_time=0.
            panel.cloud={'points':points,'frame':msg.header.frame_id,'sequence':panel.cloud['sequence']+1,'source':'mid360_raw'}
            panel.cloud_time=now if points else 0.
    def grid_cb(msg):
        if not control.is_running():return
        try:
            maps_dir=control.maps_dir
            if msg.info.width*msg.info.height>16000000:raise ValueError('Map exceeds grid limit')
            image=encode_grid(msg)
            if image is None:return
            filename='live-grid-'+str(time.time_ns())+'.png'
            target=maps_dir/filename;target.write_bytes(image)
            metadata=grid_metadata(msg,filename)
            tmp=maps_dir/'site.yaml.tmp';tmp.write_text(yaml.safe_dump(metadata,sort_keys=False));tmp.replace(maps_dir/'site.yaml')
            for old in sorted(maps_dir.glob('live-grid-*.png'))[:-8]:old.unlink(missing_ok=True)
        except Exception as e:node.get_logger().error('Cannot write live map: '+str(e))
    sub=node.create_subscription(PointCloud2,'/cloud_registered',cloud_cb,qos_profile_sensor_data)
    raw_sub=node.create_subscription(CustomMsg,'/livox/lidar',raw_cb,qos_profile_sensor_data)
    odom=node.create_subscription(Odometry,'/state_estimation',odom_cb,10)
    map_sub=node.create_subscription(OccupancyGrid,'/projected_map',grid_cb,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
    save_client=node.create_client(Trigger,'/map_save');save_future=None
    def save_tick():
        nonlocal save_future
        status=control.status()
        if status['stopping'] or status['starting'] or not status['running']:return
        if save_future is not None and not save_future.done():return
        if panel.cloud_time and time.monotonic()-panel.cloud_time<2 and save_client.service_is_ready():
            save_future=save_client.call_async(Trigger.Request())
    timer=node.create_timer(10.,save_tick)
    html=(root/'shaoshao/web_panel/index.html').read_text()
    html=html.replace('b.disabled=!connected','b.disabled=!!lastState?.preview||!connected')
    replacements={"  $('status').textContent=":"  $('status').textContent='真实雷达 · '+(s.mode==='navigation'?(s.navigation?.reason||s.navigation?.message||'导航就绪'):s.mapping_control?.message||'待机');","  $('demo').textContent=":"  $('demo').textContent='MID-360 · 建图 / 导航';","  $('yaw').textContent=":"  $('yaw').textContent='雷达坐标平移 · 小陀螺关闭';"}
    html='\n'.join(next((value for prefix,value in replacements.items() if line.startswith(prefix)),line) for line in html.splitlines())
    html=re.sub(r'<p class="note">.*?</p>','<p class="note">默认待机，只看实时点云。点击“开始建图”创建地图，结束时点击“停止建图并保存”。点击“导航”启动已保存地图的定位，等待定位就绪后在地图空白区域选点并点击“前往目标”。导航仅平移，指令上限 0.10，小陀螺关闭；8081 手动控制可接管并取消导航。</p>',html)
    page=html.encode();token=os.getenv('SCURM_WEB_TOKEN') or secrets.token_urlsafe(24)
    class Handler(upstream.handler_class(panel,token)):
        def do_GET(self):
            if urlparse(self.path).path=='/':self.reply(200,page,'text/html; charset=utf-8')
            else:super().do_GET()
    def spin_ros():
        try:rclpy.spin(node)
        except ExternalShutdownException:pass
    spin=threading.Thread(target=spin_ros,daemon=True);spin.start()
    http=ThreadingHTTPServer((a.bind,a.port),Handler);http.daemon_threads=True
    print(f'MID360 MAPPING: http://{a.bind}:{a.port}/?token={token}',flush=True)
    print('Map directory:',a.maps_dir,flush=True)
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:http.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:pass
    finally:
        http.server_close();panel.close();rclpy.shutdown();spin.join(timeout=3);node.destroy_node()
if __name__=='__main__':main()
