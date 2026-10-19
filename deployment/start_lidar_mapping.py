#!/usr/bin/env python3
from pathlib import Path
import argparse,fcntl,json,os,re,signal,subprocess,time,shlex,sys
import yaml
from lidar_runtime import ensure_driver
root=Path.home()/'ccc'
repo=root/'shaoshao'
parser=argparse.ArgumentParser()
parser.add_argument('--keep-web',action='store_true')
parser.add_argument('--expected-pid',type=int)
parser.add_argument('--expected-maps-dir',type=Path)
parser.add_argument('--filter-config',type=Path)
args=parser.parse_args()
sys.path.insert(0,str(repo/'web_panel'))
from navcar_settings import navigation_options
robot_file=repo/'web_panel/robot.yaml'
robot_config=yaml.safe_load(robot_file.read_text()) if robot_file.exists() else {}
navigation=navigation_options(robot_config)
start_lock=(root/'mapping-start.lock').open('a')
fcntl.flock(start_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
if args.expected_pid is not None and int((root/'mapping.pid').read_text())!=args.expected_pid:
    raise RuntimeError('Mapping session changed; refresh before starting')
if args.expected_maps_dir is not None and Path((root/'current_mapping_dir').read_text().strip()).resolve()!=args.expected_maps_dir.resolve():
    raise RuntimeError('Mapping directory changed; refresh before starting')
for binary in ['fast_lio/lib/fast_lio/fastlio_mapping','fast_lio/lib/fast_lio/static_map_projector','terrain_analysis_ext/lib/terrain_analysis_ext/terrainAnalysisExt']:
    if not (root/'mapping_install'/binary).is_file() or (root/'mapping_install'/binary).stat().st_size==0:raise RuntimeError('Mapping build incomplete: '+binary)
settings_file=args.filter_config or Path.home()/'scurm_maps/mapping_filter.json'
settings=json.loads(settings_file.read_text()) if settings_file.exists() else {'enabled':True,'height_m':1.5,'dynamic_enabled':True}
if not settings.get('dynamic_enabled',True):raise RuntimeError('This mapping runner requires dynamic filtering enabled')
for name in (['mapping'] if args.keep_web else ['mapping','web']):
    pidfile=root/f'{name}.pid'
    if not pidfile.exists():continue
    pid=int(pidfile.read_text());proc=Path(f'/proc/{pid}/cmdline')
    if not proc.exists():continue
    cmd=proc.read_bytes()
    if name=='mapping' and b'mapping_runtime' in cmd:raise RuntimeError('Lidar mapping already running')
    if name=='web' and (b'lidar_web.py' in cmd or b'lidar_mapping_web.py' in cmd):os.kill(pid,signal.SIGTERM)
# Wait briefly for UDP/socket owners from the old preview to exit.
for _ in range(30):
    active=[]
    for name,marker in ([] if args.keep_web else [('web',(b'lidar_web.py',b'lidar_mapping_web.py'))]):
        f=root/f'{name}.pid'
        if f.exists():
            p=Path('/proc')/f.read_text().strip()/'cmdline'
            if p.exists() and any(m in p.read_bytes() for m in marker):active.append(name)
    if not active:break
    time.sleep(.1)
else:raise RuntimeError('Old preview still stopping: '+str(active))
ensure_driver(root)
maps=Path.home()/'scurm_maps'/('mid360_mapping_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns()%1000000))
maps.mkdir(parents=True)
(maps/'mapping_filter.json').write_text(json.dumps(settings)+'\n')
config=yaml.safe_load((repo/'sentry_bringup/params/fast_lio_mapping_param.yaml').read_text())
c=config['/**']['ros__parameters']
c['map_file_path']=str(maps/'site.pcd')
c['filter_size_surf']=c['filter_size_map']=.15
c['preprocess']['lidar_type']=1
if navigation['narrow']:c['preprocess']['blind']=navigation['min_range']
c['common']['send_odom_base_tf']=False
c['mapping']['extrinsic_est_en']=False
c['mapping']['extrinsic_T']=[-.011,-.02329,.04412]
c['mapping']['extrinsic_R']=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
c['map_filter']={'enabled':settings['enabled'],'max_height':float(settings['height_m']),
                 'dynamic_enabled':True,'static_resolution':navigation['resolution']}
(root/'mapping_runtime/fast_lio.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
token=None
for f in ['web.log','web.previous.log']:
    p=root/f
    if p.exists():
        m=re.search(r'token=([\w-]+)',p.read_text(errors='replace'))
        if m:token=m.group(1);break
if token is None:
    import secrets
    token=secrets.token_urlsafe(24)
setup='source /opt/ros/jazzy/setup.bash\nsource "$HOME/Zzj_slam/ws/out/install-livox/setup.bash"\nsource "$HOME/ccc/mapping_install/setup.bash"\n'
commands={
 'mapping':['ros2','launch',str(root/'mapping_runtime/mapping.launch.py')],
 'web':['python3','-u',str(root/'lidar_mapping_web.py'),'--maps-dir',str(maps)]}
if args.keep_web:commands.pop('web')
mapping_pid=None
for name,command in commands.items():
    logfile=root/f'{name}.log'
    if logfile.exists():logfile.rename(root/f'{name}.previous.log')
    with logfile.open('w') as log:
        p=subprocess.Popen(['bash','-c',setup+'exec '+shlex.join(command)],cwd=root,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env={**os.environ,'SCURM_WEB_TOKEN':token})
    if name=='mapping':mapping_pid=p.pid
    print(name,'PID',p.pid)
    if args.keep_web:
        time.sleep(1)
        if p.poll() is not None:raise RuntimeError('Mapping launch exited; inspect ~/ccc/mapping.log')
    (root/f'{name}.pid').write_text(str(p.pid)+'\n')
    if name=='mapping':(root/'current_mapping_dir').write_text(str(maps)+'\n')
print('MAP_DIRECTORY',maps)
print('WEB','http://192.168.1.194:8080/?token='+token)
print('RESULT_JSON',json.dumps({'pid':mapping_pid,'maps_dir':str(maps)}))
