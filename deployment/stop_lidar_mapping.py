#!/usr/bin/env python3
"""Save the current real PCD before stopping the isolated lidar mapping launch."""
import argparse,os,signal,sys,time
from pathlib import Path


def command(pid):
    try:return Path(f'/proc/{pid}/cmdline').read_bytes()
    except FileNotFoundError:return b''


def group_alive(pgid):
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            fields=(p/'stat').read_text().rsplit(')',1)[1].split()
            if fields[0]!='Z' and int(fields[2])==pgid:return True
        except (FileNotFoundError,ProcessLookupError,PermissionError):pass
    return False


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--check',action='store_true');parser.add_argument('--expected-pid',type=int);parser.add_argument('--expected-maps-dir',type=Path);args=parser.parse_args()
    root=Path.home()/'ccc';pid=int((root/'mapping.pid').read_text());maps=Path((root/'current_mapping_dir').read_text().strip())
    if args.expected_pid is not None and pid!=args.expected_pid:raise RuntimeError('建图会话已改变，未停止任何进程')
    if args.expected_maps_dir is not None and maps.resolve()!=args.expected_maps_dir.resolve():raise RuntimeError('地图目录已改变，未停止任何进程')
    if b'mapping_runtime/mapping.launch.py' not in command(pid):
        print('建图已停止。地图目录：'+str(maps));return 0
    if os.getpgid(pid)!=pid:raise RuntimeError('进程组不匹配，未停止任何进程')
    import rclpy
    from std_srvs.srv import Trigger
    rclpy.init(args=[]);node=rclpy.create_node('shaoshao_mapping_stop_client')
    try:
        client=node.create_client(Trigger,'/map_save')
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('/map_save 未就绪，未停止建图')
        if args.check:
            print('CHECK OK：建图运行中，保存服务可用。地图目录：'+str(maps));return 0
        pcd=maps/'site.pcd';previous=pcd.stat().st_mtime_ns if pcd.exists() else None
        future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=30)
        if not future.done():raise RuntimeError('保存超时，建图仍在运行')
        result=future.result()
        if result is None or not result.success:raise RuntimeError('保存失败，建图仍在运行：'+(result.message if result else '无响应'))
        if not pcd.is_file() or pcd.stat().st_size==0 or pcd.stat().st_mtime_ns==previous:
            raise RuntimeError('PCD 文件未成功更新，建图仍在运行')
        # A recorder directory means this session promises a map-bound index.
        # Never label that session complete with an unfinished index.
        if (maps/'keyframes').is_dir():
            index=node.create_client(Trigger,'/finalize_localization_index')
            if not index.wait_for_service(timeout_sec=5):raise RuntimeError('点云已保存，但关键帧保存服务未就绪；建图仍在运行')
            future=index.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=30)
            if not future.done():raise RuntimeError('点云已保存，但关键帧索引保存超时；建图仍在运行')
            result=future.result()
            if result is None or not result.success:
                raise RuntimeError('点云已保存，但全局定位索引未完成；建图仍在运行：'+(result.message if result else '无响应'))
    finally:node.destroy_node();rclpy.shutdown()
    # The launch parent forwards SIGINT to its driver/LIO/terrain children.
    # The web server belongs to a separate group and keeps serving saved maps.
    if b'mapping_runtime/mapping.launch.py' in command(pid):os.kill(pid,signal.SIGINT)
    deadline=time.monotonic()+15
    while group_alive(pid) and time.monotonic()<deadline:time.sleep(.1)
    if group_alive(pid):raise RuntimeError('地图已保存，但部分建图进程尚未退出；查看 ~/ccc/mapping.log')
    os.sync()
    print('建图已停止，地图已保存。网页仍可查看已保存地图。')
    print('地图目录：'+str(maps))
    return 0
if __name__=='__main__':
    try:sys.exit(main())
    except Exception as e:print(str(e),file=sys.stderr);sys.exit(1)
