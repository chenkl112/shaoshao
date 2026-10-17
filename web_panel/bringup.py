#!/usr/bin/env python3
"""Start the fixed-yaw hardware/TF/Livox/Web chain; default Web mode is idle."""
import argparse
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from robot_io import load_runtime
from server import stop_process


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('runtime.json'))
    p.add_argument('--bind', default='0.0.0.0')
    p.add_argument('--port', type=int, default=8080)
    p.add_argument('--mock-hardware', action='store_true')
    p.add_argument('--external-driver', action='store_true', help='已有底盘/yaw ROS 驱动在另一终端运行')
    args = p.parse_args()
    config = load_runtime(args.config)
    if not shutil.which('ros2'):
        p.error('未找到 ros2，先 source ROS 2 和当前工作区')
    folder = Path(__file__).resolve().parent
    log_dir = Path(config['maps_dir']) / 'base_logs' / str(time.time_ns())
    log_dir.mkdir(parents=True, exist_ok=True)
    commands = []
    if args.mock_hardware or config['hardware']['transport'] == 'serial_jsonl_v1':
        commands.append(('chassis', [sys.executable, str(folder/'chassis_bridge.py'),
                                    '--config', str(args.config.resolve())] + (['--mock'] if args.mock_hardware else [])))
    elif not args.external_driver:
        command = config['hardware'].get('driver_command')
        if not isinstance(command, list) or not command or not all(isinstance(x, str) for x in command):
            p.error('已有驱动的启动命令待测。配置 hardware.driver_command，或先手动启动并加 --external-driver')
        commands.append(('chassis', command))
    commands.extend([
        ('yaw_tf', [sys.executable, str(folder/'yaw_tf.py'), '--config', str(args.config.resolve())]),
        ('livox', ['ros2', 'launch', 'livox_ros_driver2', 'msg_MID360_launch.py']),
        ('web', [sys.executable, '-u', str(folder/'server.py'), '--config', str(args.config.resolve()),
                 '--bind', args.bind, '--port', str(args.port)] + (['--mock-hardware'] if args.mock_hardware else [])),
    ])
    processes, files = [], []
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        for name, command in commands:
            log = (log_dir / (name+'.log')).open('wb')
            files.append(log)
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append((name, process))
            print(f'已启动 {name}，日志：{log_dir / (name+".log")}', flush=True)
        print('默认待机。Web 链接和访问令牌见 web.log。Ctrl+C 先停止 Web 和模式节点，再停止基础链路。', flush=True)
        while True:
            for name, process in processes:
                code = process.poll()
                if code is not None:
                    raise RuntimeError(f'{name} 退出，code={code}；检查 {log_dir}')
            time.sleep(.2)
    except KeyboardInterrupt:
        pass
    finally:
        # Keep feedback and the hardware driver alive until Web sends its final zero command.
        for name, process in reversed(processes):
            try:
                stop_process(process)
            except Exception as exc:
                print(f'停止 {name} 失败：{exc}', file=sys.stderr)
        for log in files:
            log.close()


if __name__ == '__main__':
    main()
