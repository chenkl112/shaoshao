"""Keep the CustomMsg Livox driver independent of the mapping process group."""
from pathlib import Path
import os
import subprocess
import time
import shlex


def ensure_driver(root):
    root = Path(root)
    pidfile = root / 'lidar.pid'
    own_pid = int(pidfile.read_text()) if pidfile.exists() else None
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            args = proc.joinpath('cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        if not args or Path(os.fsdecode(args[0])).name != 'livox_ros_driver2_node':continue
        if int(proc.name) == own_pid and b'xfer_format:=1' in args:
            return own_pid
        raise RuntimeError('A different Livox driver is already running; stop it before changing format')
    overlay = Path.home()/'Zzj_slam/ws/out/install-livox'
    binary = overlay/'livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node'
    if not binary.is_file():
        from ament_index_python.packages import get_package_prefix
        binary = Path(get_package_prefix('livox_ros_driver2'))/'lib/livox_ros_driver2/livox_ros_driver2_node'
    params = {'xfer_format':1, 'multi_topic':0, 'data_src':0, 'publish_freq':10.0,
              'output_data_type':0, 'frame_id':'livox_frame',
              'user_config_path':str(Path.home()/'.config/mid360-view/MID360_config.json')}
    command = [str(binary), '--ros-args']
    for name, value in params.items():command.extend(['-p', f'{name}:={value}'])
    with (root/'lidar.log').open('a') as log:
        script = 'source /opt/ros/jazzy/setup.bash\n'
        if (overlay/'setup.bash').is_file():script += 'source '+shlex.quote(str(overlay/'setup.bash'))+'\n'
        process = subprocess.Popen(['bash','-c',script+'exec '+shlex.join(command)], cwd=root, stdin=subprocess.DEVNULL,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    pidfile.write_text(str(process.pid)+'\n')
    time.sleep(1)
    if process.poll() is not None:raise RuntimeError('Livox driver exited; inspect ~/ccc/lidar.log')
    return process.pid
