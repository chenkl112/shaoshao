# 哨哨实车部署：Ubuntu 24.04 / ROS 2 Jazzy / Navcar

适用车辆：麦轮底盘，MID360 安装在 yaw 云台上；下位机通过 Navcar UART1 V1 保持启动世界航向，
上位机负责雷达、定位、规划、Web 和串口指令。底盘电机与 yaw 的实际闭环由下位机负责人实现。
本目录基于 `shaoshao-code-20261018.zip` 的车端脚本；源码放在 `~/ccc/shaoshao/`，
启动脚本和构建输出放在 `~/ccc/`。不要和 `bringup.py` 的旧 JSONL / 相对底盘锁角入口混用。

先用当前地图验证“路径预览”，再测试行驶。未知尺寸、安装和闭环误差保留为待测；
ACK 或目标回显不能作为实际姿态和实际运动的测量值。

## 1. 已经能运行的车：更新这次修正

这次修改了 C++，只复制 Python 或重启网页不会生效。新地图默认使用 5 cm 静态体素与二维投影，
并修正灰色未知区域的导出阈值；旧地图和个人配置不会自动转换。

先在 8080 点击“待机”，在 8081 点击“停止”。如果服务在终端前台运行，在原终端 Ctrl+C 退出。
若原来由后台脚本启动，可以在车端执行下面的检查与停止，只处理 PID 文件指向且命令匹配的两个 Web 服务：

```bash
python3 - <<'PY'
import os, signal, time
from pathlib import Path
root = Path.home() / 'ccc'
for name, marker in [('web', b'lidar_mapping_web.py'), ('move_web', b'navcar_move_web.py')]:
    record = root / (name + '.pid')
    if not record.exists():
        continue
    pid = int(record.read_text())
    proc = Path('/proc') / str(pid)
    if pid <= 1:
        raise RuntimeError('无效 PID：' + name)
    if not proc.exists():
        continue
    if marker not in (proc / 'cmdline').read_bytes():
        raise RuntimeError('进程不匹配，未停止：' + name)
    os.kill(pid, signal.SIGINT)
    for _ in range(150):
        if not proc.exists():
            break
        fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            break
        time.sleep(.1)
    else:
        raise RuntimeError('仍在退出，请检查日志，暂勿启动第二份：' + name)
    print('已停止：', name)
PY
```

保留已有实车配置、运行配置和启动脚本，再将收到的新源码同步到 `~/ccc/shaoshao/`。
源码包或本地修改不会自动出现在 GitHub；只有已发布的提交才能通过 `git pull --ff-only` 获取。

```bash
shaoshao_backup="$HOME/ccc/backups/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$shaoshao_backup"
cp -a "$HOME/ccc/mapping_runtime" "$shaoshao_backup/"
for item in robot.yaml serial.navcar.yaml; do
  if [ -f "$HOME/ccc/shaoshao/web_panel/$item" ]; then
    cp -a "$HOME/ccc/shaoshao/web_panel/$item" "$shaoshao_backup/"
  fi
done
for item in start_mapping.sh start_lidar_mapping.py lidar_mapping_web.py; do
  if [ -f "$HOME/ccc/$item" ]; then cp -a "$HOME/ccc/$item" "$shaoshao_backup/"; fi
done
```

同步源码后，在车端执行：

```bash
cd ~/ccc/shaoshao
cp -a deployment/. "$HOME/ccc/"
bash "$HOME/ccc/build_mapping.sh"
```

编译必须成功再启动。按第 5 节启动服务，在 8080 重新创建并保存地图；
不要把旧 `site.yaml` 的 `resolution` 改为 0.05，这会缩放地图的实际尺寸。
不要将旧稀疏 PCD 重采样成 5 cm 后当作恢复了丢失的观测。

## 2. 新电脑：系统、源码和驱动

系统使用 Ubuntu 24.04。按 [ROS 2 Jazzy 官方安装步骤](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
配置软件源并安装 ROS，然后安装本项目依赖：

```bash
sudo apt update
sudo apt install git build-essential cmake python3-colcon-common-extensions \
  python3-yaml python3-serial python3-numpy libeigen3-dev libpcl-dev \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-pointcloud-to-laserscan \
  ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-tf2-geometry-msgs
mkdir -p "$HOME/ccc"
git clone https://github.com/chenkl112/shaoshao.git "$HOME/ccc/shaoshao"
```

已有源码目录时保留个人配置，再获取所需版本，不要覆盖克隆到一个已有工程里。
先安装 Livox-SDK2 和 Jazzy 版 `livox_ros_driver2`，编译方法见
[Livox 官方驱动说明](https://github.com/Livox-SDK/livox_ros_driver2)。
当前车端脚本的驱动安装目录固定为 `~/Zzj_slam/ws/out/install-livox/`，应确保存在：

```bash
test -f "$HOME/Zzj_slam/ws/out/install-livox/setup.bash"
source /opt/ros/jazzy/setup.bash
source "$HOME/Zzj_slam/ws/out/install-livox/setup.bash"
ros2 pkg prefix livox_ros_driver2
```

如果现有驱动安装在别处，需统一修改 `deployment/` 启动和构建脚本中的路径；
单独在当前终端 source 另一个目录不会修改后台子进程的启动配置。

## 3. 恢复目录、网络与待测参数

在源码根目录执行：

```bash
mkdir -p "$HOME/ccc" "$HOME/.config/mid360-view"
cp -a deployment/. "$HOME/ccc/"
cp deployment/config/MID360_config.json "$HOME/.config/mid360-view/MID360_config.json"
cp -n web_panel/serial.navcar.example.yaml web_panel/serial.navcar.yaml
cp -n web_panel/robot.navcar.example.yaml web_panel/robot.yaml
```

复制雷达配置前核对其中的主机和雷达 IP；已有配置应先备份，更新已工作的车时保留现有雷达配置。
`mapping_runtime/` 恢复到 `~/ccc/mapping_runtime/`，与源码目录并列。

雷达配置样例中的主机网口为 `192.168.10.100`，雷达为 `192.168.10.101`，仅是现有配置，
请先核对实际值再设置雷达专用网卡。`host_net_info` 中各个 `*_ip` 必须对应这块网卡的地址。
网络验收可用 `ip -br addr`、`ping <实际雷达IP>` 和 `ros2 topic hz /livox/lidar`；
网页访问地址则使用手机能访问的车载电脑 Wi-Fi/有线局域网 IP，通常与雷达专网 IP 不同。
旧脚本打印的 `192.168.1.194` 只是原车地址，需要替换成当前电脑的实际地址，保留端口与 token。

编辑两个配置文件：

| 文件 | 需要填写或核对 |
| --- | --- |
| `web_panel/serial.navcar.yaml` | `serial_port` 为 `/dev/serial/by-id/...`，波特率 2000000；平移倍率先按已测初值，横移仍待复测 |
| `web_panel/robot.yaml` | 实际碰撞长宽、光学中心离地高度、yaw 轴和 IMU 安装位置、IMU 安装角；未测值保留 null |

当前雷达坐标导航读取 `robot` 的外接半径（或长宽）、光心高度和 `yaw` 的安装位置/角度，不使用旧配置里的
`yaw.reference`、编码器符号或编码器零位来伪造实际姿态。
新的 `robot.navcar.example.yaml` 专门用于这条入口，旧 `robot.example.yaml` 属于 JSONL 通用入口。
60 cm 通道联调使用已标定的中心参考点与独立低速细图配置，完整步骤见 [60 cm 通道联调](../web_panel/60cm通道联调.md)。
最新实测已修正：正八边形对边距为 510 mm，已含轮子和突出物；雷达底面及 yaw 安装面离地 280 mm，
水平偏心为 14.18 mm。旧的 29 cm 外接半径、32 cm 雷达高度、31 cm yaw 高度和 1 cm 偏心不再作为当前实车参数。
MID360 官方尺寸图的点云原点距底面 47 mm，因此 `lidar_height_m` 应为 `0.280 + 0.047 = 0.327 m`，
不能填成底面高度 0.280。[官方用户手册](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf)
的 Coordinates（印刷页 12）和附录尺寸图（印刷页 19）给出参考原点和尺寸；该换算要求水平正装。

当前用户已补充实测外形，Navcar 导航可使用外接半径代替未知长宽。在原 `robot.yaml` 中合并以下字段，
保留原有网络、串口和已标定安装角，不要用模板整份覆盖实车配置：

```yaml
robot:
  across_flats_m: 0.510       # 对边距测量记录，含全部轮子和突出物
  circumscribed_radius_m: 0.276011 # 0.510/(2*cos(pi/8))，向上取整到微米
  lidar_bottom_height_m: 0.280 # 底面高度测量记录
  lidar_height_m: 0.327       # 程序使用点云原点高度，280 + 47 mm
yaw:
  axis_xyz_m: [0.0, 0.0, 0.280] # yaw 轴居中，原点取与雷达底面同高的安装面
  lidar_offset_radius_m: 0.01418 # 雷达光学中心相对 yaw 轴的水平偏心
  lidar_offset_xy_m: [-0.01418, 0.0] # 从 yaw 轴指向光心，沿雷达 x 负向
navigation:
  max_speed_mps: 0.5          # 三处导航速度限制共用，单位 m/s
  reference_point: yaw_axis # 水平原点为真实 yaw 轴，方向跟随雷达
  narrow_passage: false     # 60 cm 联调设 true；有效限速为 0.05，需新建 1 cm 地图
```

长宽可继续待测；填写有效外接半径后当前 Navcar 导航不再依赖长宽。
`across_flats_m` 和 `lidar_bottom_height_m` 仅记录测量依据；程序直接使用显式的外接半径和点云原点高度，
不会自动把对边距当直径，也不会自动把底面高度换算成点云原点高度。
光学偏心模式也不依赖旧的 `yaw.imu_xyz_m`，使用 LIO 内部外参计入 IMU 与光学中心偏移；
`yaw.imu_rpy_rad` 仍需满足水平安装要求。旧配置未写速度时保留原 0.1 m/s。
重启两个 Web 服务后再进入导航，勿仅手改自动生成的 `navigation_runtime/nav2.yaml`。

串口权限与只读联调：

```bash
ls -l /dev/serial/by-id/
sudo usermod -aG dialout "$USER"
# 重新登录使用户组生效，再运行：
cd ~/ccc/shaoshao
python3 web_panel/navcar_bridge.py --check-config
python3 web_panel/navcar_bridge.py --no-ros
```

默认只读；Ctrl+C 退出后再启动 8081。不能让只读桥、旧桥和移动页同时占用同一个串口。
协议字段、ACK、看门狗与单位见 [Navcar 接入](../web_panel/Navcar接入.md)。

## 4. 编译与只读检查

```bash
bash ~/ccc/build_mapping.sh
source /opt/ros/jazzy/setup.bash
source "$HOME/Zzj_slam/ws/out/install-livox/setup.bash"
source "$HOME/ccc/mapping_install/setup.bash"
ros2 pkg prefix fast_lio
ros2 pkg prefix terrain_analysis_ext
python3 -m unittest discover -s "$HOME/ccc/shaoshao/web_panel" -p 'test*.py' -q
```

构建脚本只编译 `fast_lio` 和 `terrain_analysis_ext`，输出为 `~/ccc/mapping_install/`。
Nav2 与点云转扫描由系统 ROS 包提供。编译失败时先解决错误，不使用旧二进制冒充本次更新。

## 5. 启动、建图与导航

终端一，串口与移动页：

```bash
bash ~/ccc/start_move_web.sh
```

保持该终端运行，打开打印的 8081 链接。打开页面不会自动行驶；
以云台朝向为前方，低速检查前后左右、停止和 yaw 目标保持。
yaw 目标单位为度、参考为 MCU 启动航向，不是相对底盘的关节角。
实际闭环是否稳定仍需现场观察/测量；回显相同目标只说明目标一致。

终端二，雷达与 8080 网页：

```bash
bash ~/ccc/start_mapping.sh
```

命令默认只启动雷达和网页，处于待机；8080 链接的 token 和 8081 的 token 是独立的。
雷达已被另一个驱动占用时先处理已有实例，不能再启动第二份 Livox 驱动。

建图时：

1. 在网页点击“开始建图”，保持地面与障碍得到足够观测；新地图创建在 `~/scurm_maps/`。
2. 通过 8081 低速移动，导航链不依赖四轮编码器里程计；相邻观测需要有重叠。
3. 静态导出需要同一位置获得跨多个扫描、至少约 1.5 秒的持续观测。新版本体素更细，经过狭窄区域时应慢行、停留检查。
4. 点击“停止建图并保存”，等待保存成功；确认当前目录有非空 `site.pcd`、`site.yaml` 和 YAML 指向的 PNG。
5. 灰色为未知，不代表空地；应补充观测，不能把未扫到的区域填白。

当前地图目录记录在 `~/ccc/current_mapping_dir`。建图运行时检查：

```bash
ros2 param get /laser_mapping_node map_filter.static_resolution
ros2 param get /static_map_projector resolution
ros2 topic echo /projected_map --once --field info
```

普通配置两项精度都应为 0.05，消息中的 `resolution` 也应约为 0.05；
`narrow_passage: true` 时对应为 0.01，并要求新建的 1 cm 地图。
内部 LIO 匹配体素仍是 0.15，与新的静态导出精度用途不同。

导航时：

1. 保持车辆在建图结束位置附近；点击“导航”，等待定位和导航就绪。
2. 网页选一个附近、留有整车空间的目标，先查看路径预览。
3. 预览成功再点击“前往目标”；普通配置的导航向量速度上限读取 `navigation.max_speed_mps`，当前为 0.50 m/s；窄通道配置进一步限制为 0.05 m/s。只平移，不做最后朝向对齐。
4. “取消导航”停止该任务；“待机”停止定位与规划，保留雷达和网页。
5. 8081 任意手动方向/停止/yaw 设置会撤销导航控制权。导航完成前不要在另一个页面修改控制目标。

先验初始位置来自 `last_pose.json`，移动车辆后不能把旧位置当作准确定位。
实际 yaw/底盘测量缺失时，碰撞模型继续使用整车包围圆；两侧各留十多厘米的通道不保证可过，
先按 [窄通道排障](../web_panel/窄通道排障.md) 检查代价图和近场障碍。

## 6. 日志、回退与待验收

| 路径 | 内容 |
| --- | --- |
| `~/ccc/lidar.log` | MID360 驱动 |
| `~/ccc/web.log` | 8080 网页、建图模式切换 |
| `~/ccc/mapping.log` | LIO、地形分析与二维投影 |
| `~/ccc/navigation.log` | AMCL、Nav2、规划/跟随错误 |
| `~/ccc/navigation_runtime/nav2.yaml` | 实际生成并被 Nav2 使用的参数，改源配置后需退出导航并重新进入 |
| `~/ccc/mapping_runtime/fast_lio.yaml` | 本次建图生成的 LIO 参数，下次开始建图会重新生成 |
| `~/ccc/navcar_move.token` | 8081 本机接入令牌；不要把含令牌的日志作为公开文档发布 |

回退时先进入待机、停止两个 Web 服务，再恢复备份的源码/脚本和 `mapping_runtime`；
若恢复了旧 C++，需要重新编译与旧源码一致的二进制。保留新旧地图在各自目录，不能混用 PNG 与另一张地图的 YAML 原点/分辨率。

截至本次验证：Windows 92 项 Python 测试执行、90 项通过、2 项 POSIX 专用测试跳过；
Ubuntu 24.04 / WSL 92 项全通过；Windows 与 Linux 的 C++ 静态过滤/投影测试均通过，
包含窄通道栅格、保留障碍/未知区域和细体素导出。实车快照 200 项文件哈希核验通过。
本次没有完成 Jazzy 的整包重新编译或连接车辆；上车仍需按以上步骤编译和验收。

待测项包括实际几何、云台世界锁定误差、近场自体点、实际航向反馈、横移倍率、定位误差和停车距离。
没有这些测量依据时，不把半车宽当作完整车体包围半径，也不关闭碰撞拒绝规则。

实际 `robot.yaml`、`serial.yaml` 和 `serial.navcar.yaml` 不提交 Git。快照、点云和令牌没有作为源码提交。
