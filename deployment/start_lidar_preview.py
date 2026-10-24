#!/usr/bin/env python3
"""Start live LiDAR and the web panel without starting a mapping session."""
import fcntl
import json
import os
from pathlib import Path
import re
import secrets
import shlex
import subprocess
import time

from lidar_runtime import ensure_driver


def main():
    root=Path.home()/'ccc'
    root.mkdir(parents=True,exist_ok=True)
    lock=(root/'mapping-start.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    print('LIDAR_PID',ensure_driver(root))
    token=os.environ.get('SCURM_WEB_TOKEN')
    for name in ('web.log','web.previous.log'):
        if token:break
        log=root/name
        if log.exists():
            matches=re.findall(r'token=([\w-]+)',log.read_text(errors='replace'))
            if matches:token=matches[-1]
    token=token or secrets.token_urlsafe(24)
    pidfile=root/'web.pid'
    if pidfile.exists():
        try:
            pid=int(pidfile.read_text())
            running=str(root/'lidar_mapping_web.py').encode() in Path(f'/proc/{pid}/cmdline').read_bytes()
        except (ValueError,OSError):running=False
        if running:
            print('网页已运行，保留当前建图/待机状态。')
            print('WEB','http://192.168.1.194:8080/?token='+token)
            return
    current=root/'current_mapping_dir'
    if current.exists():maps=Path(current.read_text().strip())
    else:
        maps=Path.home()/'scurm_maps/mid360_preview'
        current.write_text(str(maps)+'\n')
    maps.mkdir(parents=True,exist_ok=True)
    if not (root/'mapping.pid').exists():(root/'mapping.pid').write_text('0\n')
    settings=maps/'mapping_filter.json'
    if not settings.exists():
        previous=Path.home()/'scurm_maps/mapping_filter.json'
        settings.write_text(previous.read_text() if previous.exists() else json.dumps({'enabled':True,'height_m':1.5,'dynamic_enabled':True})+'\n')
    setup='set -e\nsource "$HOME/ccc/ros_env.sh"\n'
    command=['python3','-u',str(root/'lidar_mapping_web.py'),'--maps-dir',str(maps)]
    with (root/'web.log').open('a') as log:
        proc=subprocess.Popen(['bash','-c',setup+'exec '+shlex.join(command)],cwd=root,
                              stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                              start_new_session=True,env={**os.environ,'SCURM_WEB_TOKEN':token})
    pidfile.write_text(str(proc.pid)+'\n')
    time.sleep(1)
    if proc.poll() is not None:raise RuntimeError('Web exited; inspect ~/ccc/web.log')
    print('实时点云网页已启动；本命令不启动建图，点击网页“开始建图”后创建新地图。')
    print('WEB','http://192.168.1.194:8080/?token='+token)


if __name__=='__main__':main()
