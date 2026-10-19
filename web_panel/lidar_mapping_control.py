"""Save and stop the isolated LiDAR mapping launch without blocking HTTP."""
from pathlib import Path
import json
import subprocess
import threading


class MappingControl:
    def __init__(self, root, maps_dir, runner=None, on_started=None):
        self.root = Path(root)
        self.maps_dir = Path(maps_dir).resolve()
        self.pid = int((self.root / 'mapping.pid').read_text())
        self.runner = runner or subprocess.run
        self.lock = threading.RLock()
        self.stopping = False
        self.starting = False
        self.on_started = on_started
        self.saved = False
        self.error = ''
        self.worker = None

    def matches_session(self):
        try:
            return (int((self.root / 'mapping.pid').read_text()) == self.pid and
                    Path((self.root / 'current_mapping_dir').read_text().strip()).resolve() == self.maps_dir)
        except (OSError, ValueError):
            return False

    def is_running(self):
        if not self.matches_session():
            return False
        try:
            return b'mapping_runtime/mapping.launch.py' in Path(f'/proc/{self.pid}/cmdline').read_bytes()
        except OSError:
            return False

    def status(self):
        with self.lock:
            running = self.is_running()
            message = ('正在启动建图…' if self.starting else
                       '正在保存地图并停止建图…' if self.stopping else
                       '正在建图' if running else
                       '建图已停止，地图已保存' if self.saved else
                       '建图已停止，可查看最近保存的地图')
            return {'running': running, 'stopping': self.stopping, 'starting': self.starting,
                    'can_start': not running and not self.stopping and not self.starting,
                    'can_stop': running and not self.stopping and not self.starting,
                    'maps_dir': str(self.maps_dir),
                    'saved': self.saved, 'message': message, 'error': self.error}

    def request_stop(self):
        with self.lock:
            if self.starting:
                raise RuntimeError('正在启动建图，请稍候')
            if self.stopping:
                return
            if not self.matches_session():
                raise RuntimeError('网页对应的建图会话已改变，请刷新后重试')
            if not self.is_running():
                return
            self.stopping = True
            self.error = ''
            self.worker = threading.Thread(target=self._stop, daemon=True)
            self.worker.start()

    def request_start(self):
        with self.lock:
            if self.starting or self.is_running():return
            if self.stopping:raise RuntimeError('正在保存并停止建图，请稍候')
            if not self.matches_session():raise RuntimeError('网页对应的建图会话已改变，请刷新后重试')
            self.starting=True;self.error=''
            self.worker=threading.Thread(target=self._start,daemon=True)
            self.worker.start()

    def _start(self):
        try:
            command=[str(self.root/'start_mapping.sh'),'--keep-web',
                     '--expected-pid',str(self.pid),'--expected-maps-dir',str(self.maps_dir)]
            settings=self.maps_dir/'mapping_filter.json'
            if settings.is_file():command.extend(['--filter-config',str(settings)])
            result=self.runner(command,capture_output=True,text=True,timeout=30,start_new_session=True)
            if result.returncode:raise RuntimeError((result.stderr or result.stdout or '启动建图失败').strip()[-1200:])
            lines=[line[len('RESULT_JSON '):] for line in result.stdout.splitlines() if line.startswith('RESULT_JSON ')]
            if not lines:raise RuntimeError('启动命令未返回建图会话信息')
            session=json.loads(lines[-1]);pid=int(session['pid']);maps=Path(session['maps_dir']).resolve()
            with self.lock:
                self.pid,self.maps_dir,self.saved=pid,maps,False
            if not self.is_running():raise RuntimeError('建图进程未能启动，请检查 mapping.log')
            if self.on_started:self.on_started(maps)
        except subprocess.TimeoutExpired:
            with self.lock:self.error='启动建图超时，请检查 mapping.log'
        except Exception as exc:
            with self.lock:self.error=str(exc)
        finally:
            with self.lock:self.starting=False

    def _stop(self):
        try:
            result = self.runner(
                [str(self.root / 'stop_mapping.sh'), '--expected-pid', str(self.pid),
                 '--expected-maps-dir', str(self.maps_dir)],
                capture_output=True, text=True, timeout=55, start_new_session=True)
            if result.returncode:
                raise RuntimeError((result.stderr or result.stdout or '保存或停止建图失败').strip()[-1200:])
            if self.is_running():
                raise RuntimeError('停止命令已返回，但建图进程仍在运行')
            with self.lock:
                self.saved = True
        except subprocess.TimeoutExpired:
            with self.lock:
                self.error = '停止操作超时，请检查建图状态和日志后重试'
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
        finally:
            with self.lock:
                self.stopping = False


class LidarMappingPanelMixin:
    def navigation_reason(self):
        if getattr(self,'navigation_control',None):return self.navigation_control.reason()
        return '当前仅启用雷达建图；地图定位、路径规划和底盘自动控制尚未接通，暂不能前往目标'

    def state(self):
        with self.lock:
            status = self.mapping_control.status()
            nav=getattr(self,'navigation_control',None)
            if nav and nav.process is not None and not nav.running and not nav.stopping:
                nav.error=nav.error or '导航进程已退出，请查看 ~/ccc/navigation.log'
            self.mode = 'navigation' if nav and nav.running else 'mapping' if status['running'] or status['stopping'] else 'idle'
            self.switching = status['stopping'] or status['starting'] or bool(nav and (nav.starting or nav.stopping))
            self.error = (nav.error if nav else '') or status['error']
            result = super().state()
            result.update(preview=not bool(nav and nav.running), mapping_only=True,navigation_available=nav is not None,
                          point_topic='/cloud_registered' if status['running'] or (nav and nav.running) else '/livox/lidar',
                          mapping_control=status)
            return result

    def request_mode(self, mode):
        nav=getattr(self,'navigation_control',None)
        if mode == 'navigation':
            if nav:nav.request_start()
            else:raise RuntimeError(self.navigation_reason())
        elif mode == 'mapping':
            if nav and nav.active:nav.request_stop(start_mapping=True)
            else:self.mapping_control.request_start()
        elif mode == 'idle':
            if nav and nav.active:nav.request_stop()
            else:
                if self.mapping_control.is_running() and self.pose:
                    try:(self.mapping_control.maps_dir/'last_pose.json').write_text(json.dumps(self.pose))
                    except OSError:pass  # An optional localization seed cannot prevent stopping.
                self.mapping_control.request_stop()
        else:raise RuntimeError('当前为仅雷达建图，支持开始建图和停止保存')

    def close(self):
        nav=getattr(self,'navigation_control',None)
        if nav:nav.close()
        bridge=getattr(self,'bridge',None)
        if bridge and hasattr(bridge,'close'):bridge.close()
        super().close()
