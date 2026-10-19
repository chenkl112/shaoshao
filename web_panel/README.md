# 麦轮车、闭环 yaw 云台与 MID360：中文上车部署指南

适用目标：i7-12800HX 车载电脑，四轮麦轮底盘，MID360 刚性安装在 yaw 云台上，网页提供点云与 **建图 / 导航 / 待机** 三个模式按键。

本目录提供 Web 面板、模式管理、yaw 锁角与反馈门控、实际角度 TF、参考底盘串口桥、实测配置生成、一键基础链路启动和只读检查工具。基于上游 `b187f2e4e29ae0f483aec48befbe39fe67a3a5eb` 修改。

**交付边界：** 已完成可在当前电脑检查的软件与演示链路。没有连接你的电机、编码器、雷达或车载 Ubuntu；不能把本地演示当作实车已调通。实测几何、yaw 参考系/零位、串口路径和波特率仍待测，均留在 `robot.example.yaml`；下位机按已选协议的固件由底盘负责人实现。未填写必填项时，实车配置生成和启动会报错并指出缺失项。

**2026-10-18 实车协议更新：** 下位机已提供 [Navcar UART1 V1](https://github.com/Li-chee/Navcar-Comm-Protocol)，请使用 [Navcar 接入说明](Navcar接入.md) 和 `navcar_bridge.py`。原 JSONL 参考桥仅保留兼容，不适用于该固件。Navcar 仅回传目标，不提供实测 yaw/轮式里程计，不能替代下文的实际角度反馈或直接启用整车导航。

**旧参考协议分工：** 你负责上位机和串口收发，底盘负责人实现下位机四轮/yaw 控制。双方已选 `serial_jsonl_v1`；将 [串口对接约定](串口对接约定.md) 交给对方实现。轮径、轮距、减速比和电机 PID 由对方负责，不是你这边生成导航配置的必填项。车体碰撞外形与雷达安装几何仍需测量，用于 footprint 和 TF。

## 一、先明确 yaw 的“不动”

本版实现 **保持相对底盘固定的 yaw 关节角**，配置为 `yaw.reference: base_link`。下位机用编码器做 yaw 位置/速度闭环，ROS 发送固定目标并读取真实角度。底盘转弯时，雷达与云台随底盘一起转，这是预期行为。

如果你指“底盘转弯时云台仍指向世界中的同一方向”，这是世界锁定，关节角必须反向变化。需另外实现世界航向目标、关节限位与动态转动期间的点云补偿；此版配置会拒绝启用世界锁定。暂时不能确认时，把该项保持“待测”。

即使 yaw 闭环保持不动，TF 仍使用编码器测到的角度：

```text
map                  # 建图时与 odom 对齐；导航时由 ICP 初始重定位给出
 └── odom
      └── base_link  # FAST-LIO 根据实时传感器安装 TF 换算
           └── yaw_link       # 实际 yaw 反馈，动态 TF
                └── imu_link # IMU 到 yaw 的实测刚性安装，静态 TF
```

LiDAR 到 IMU 的内部外参另在 FAST-LIO 中配置，不能把“离地高度”填进内部外参。底盘轮式里程计使用 `/wheel/odom`，不发布与 FAST-LIO 冲突的 `odom → base_link` TF。

## 二、现在就能看网页演示

在项目根目录运行：

```bash
python web_panel/server.py --demo --port 8080
```

打开终端打印的完整链接。Windows、Linux 都能运行此演示，不需要 ROS、不联网下载前端依赖。网页会注明“演示数据 · 未连接车辆”。可拖动旋转点云、滚轮缩放、双击重新定位视角，三个按钮切换演示模式。左侧“实时点云 / 建图地图”只切换视图，不切换车辆运行模式；地图页读取 `maps_dir/site.yaml` 指向的目录内 PNG，显示已保存的二维栅格地图，可拖动、缩放或双击适应窗口。保存地图后自动刷新，未保存时显示暂无地图，不使用仓库测试地图替代。演示不会连接串口或输出电机命令。

实时点云页默认开启“点云去顶”，隐藏参考高度以上 1.5 m 的点，可调整“高度上限”或取消勾选恢复全部点。原始 MID-360 点云以雷达原点高度为参考；点云坐标系与车辆位姿一致时以底盘 z 高度为参考，其他情况以当前点云坐标原点为参考，页面会标注。设置保存在当前浏览器会话中。勾选和高度输入先改变网页显示；点击“应用到建图”会将开关/高度保存到 `maps_dir/mapping_filter.json`，在下一次建图时同时用于三维 PCD 和二维地图，默认开启、1.5 m。设置只允许在待机时修改，建图期间保持不变，避免同一地图混入不同高度范围的数据。

建图去顶在 FAST-LIO 完成运动补偿和坐标变换后，按重力的反方向计算点的实际高度，以首次有效扫描的雷达位置为固定基准。高于基准加上限的点不会进入 `/cloud_registered`、累计 `/Laser_map` 和保存的 PCD，地形分析/OctoMap 也接收这一去顶点云生成二维地图。雷达原始数据和 LIO 内部估计仍保留完整观测。已有地图不会自动重写；新设置需要重新建图。此处修改了 C++，必须重新编译 `fast_lio` 和 `sentry_bringup` 后才能用于实车；当前雷达预览只保存设置，不代表建图链路已运行。

单独使用启动文件时，可指定同样参数：

```bash
ros2 launch sentry_bringup mapping.launch.py use_rviz:=false \
  map_ceiling_enabled:=true map_ceiling_height:=1.5
```

`fast_lio_mapping_param.yaml` 中的 `map_filter.enabled` / `map_filter.max_height` 是直接启动的默认值；网页启动时使用已保存的设置覆盖它们。导航定位配置默认不对观测点云裁剪。

## 三、完整链路与接口

```text
MID360 以太网 -> livox_ros_driver2 -> /livox/lidar CustomMsg + /livox/imu
                              -> FAST-LIO -> /cloud_registered + /state_estimation
                                         -> 地形分析 -> OctoMap /projected_map
                                         -> ICP + 先验地图定位 + Nav2

Nav2 平滑输出/恢复动作 -> /web/cmd_vel_auto   ┐
遥控器或键盘           -> /web/cmd_vel_manual ├-> 模式/yaw/新鲜度门控 -> /cmd_vel
                                             ┘                       -> 下位机麦轮控制
Web 固定目标           -> /yaw/target -> 下位机 yaw 闭环
下位机编码器           -> /yaw/joint_states -> yaw_tf.py -> 实际 yaw TF
                                          -> Web 到位判断与速度门控
Web HTTP               -> 浏览器点云、车位置、yaw 状态、三个模式按钮
```

| 接口 | 消息类型 | 约定 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 最终底盘命令，只有 `scurm_web_panel` 发布；vx 前进、vy 向左、wz 逆时针，m/s 与 rad/s |
| `/web/cmd_vel_manual` | `Twist` | 原有遥控器或键盘输入，仅建图模式允许通过 |
| `/web/cmd_vel_auto` | `Twist` | Nav2 平滑器及恢复动作，仅导航模式允许通过 |
| `/yaw/target` | `std_msgs/msg/Float64` | 标定后的 yaw 关节目标角，rad，保持相对底盘固定角 |
| `/yaw/joint_states` | `sensor_msgs/msg/JointState` | 包含 `yaw_joint` 对应的 position、velocity 和有效 ROS 时间戳 |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | 底盘可选反馈，须有真实速度；该驱动不发布全局定位 TF |
| `/state_estimation` | `Odometry` | 本方案 FAST-LIO 定位里程计；原源码未填有效 twist，不直接作为闭环速度反馈 |

使用已有 ROS 底盘驱动时，把它接到上述接口；适配器必须同时提供实际 yaw 反馈。没有已有驱动时可使用 `chassis_bridge.py`，但它使用本文定义的 **参考 JSONL 协议**，需要下位机固件匹配。它不会自动识别未知厂商协议。[下位机接口与麦轮公式](硬件接口.md)给出了报文、超时和验收要求。

## 四、车载系统与依赖

建议以 **Ubuntu 22.04 x86_64 + ROS 2 Humble** 为基线。当前仓库是 ROS 2，不能直接用 ROS 1 Noetic 的命令。12800HX 可作为 FAST-LIO、DWB 与 Web 的 CPU 平台，不要求 NVIDIA GPU；实际运行频率需在车上确认。

先按 [ROS 2 Humble 官方说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)安装 ROS，随后：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install git cmake build-essential python3-colcon-common-extensions \
  python3-rosdep python3-numpy python3-yaml python3-serial \
  libeigen3-dev libpcl-dev libcurl4-openssl-dev \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rviz2 \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-octomap-server \
  ros-humble-octomap-ros ros-humble-tf2-ros ros-humble-teleop-twist-keyboard
```

厂商 SDK 需要单独安装，rosdep 不会代替它：

```bash
mkdir -p ~/vendor
cd ~/vendor
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cmake -S Livox-SDK2 -B Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release
cmake --build Livox-SDK2/build -j4
sudo cmake --install Livox-SDK2/build
sudo ldconfig
```

将 **当前这份修改后的仓库** 复制到车上的 `~/sentry_ws/src/SCURM_Nav_Tutorial`。从 GitHub 重新克隆上游不会自动获得本次新增的 Web 与 yaw 文件；也可以对相同上游提交应用交付补丁。

第一次安装 rosdep 才执行 `sudo rosdep init`，已有配置不要重复初始化。然后：

```bash
source /opt/ros/humble/setup.bash
rosdep update
cd ~/sentry_ws
rosdep install --from-paths \
  src/SCURM_Nav_Tutorial/sentry_bringup \
  src/SCURM_Nav_Tutorial/FAST_LIO \
  src/SCURM_Nav_Tutorial/livox_ros_driver2 \
  src/SCURM_Nav_Tutorial/icp_relocalization \
  src/SCURM_Nav_Tutorial/autonomous_exploration_development_environment/src \
  src/SCURM_Nav_Tutorial/nav2_plugins/costmap_intensity \
  src/SCURM_Nav_Tutorial/nav2_plugins/behavior_ext_plugins \
  --ignore-src -r -y
```

解决实际依赖错误后再编译。初期不启动比赛裁判模拟 `control_panel` 和比赛决策 `rm_decision_cpp`，它们可能直接发布底盘命令而绕开当前门控。

## 五、测量与配置：未知项保持待测

```bash
cd ~/sentry_ws/src/SCURM_Nav_Tutorial
cp web_panel/robot.example.yaml web_panel/robot.yaml
python3 web_panel/preflight.py --offline --config web_panel/robot.yaml
```

离线检查会输出缺失清单，未测量时返回非零状态是预期结果。编辑 `robot.yaml`，不要把示例或估计值当成实测值。

| 测量项 | 定义与单位 |
| --- | --- |
| `robot.length_m / width_m` | 车体碰撞外轮廓，含突出的轮子、支架，m |
| `robot.lidar_height_m` | MID360 光学中心到地面的高度，m |
| `yaw.axis_xyz_m` | yaw 旋转轴原点在 base_link 中的位置，m；本版假设转轴平行 base_link 的 +z |
| `yaw.imu_xyz_m / imu_rpy_rad` | MID360 IMU 原点在 yaw_link 中的刚性安装位置/角度，m、rad |
| `yaw.reference / target_rad` | 锁角参考系及标定后目标，确认相对底盘锁角才填 base_link |
| `hardware.yaw_encoder_sign / zero` | 串口方案的原始编码器正负方向与零位，必须通过实际转动确认 |
| `network.host_ip / lidar_ip` | 主机雷达网口 IP 和设备实际 IP，不能照搬上游地址 |
| 轮径、轮距、轴距、减速比、编码器分辨率 | 由底盘驱动或 MCU 使用；记录在 pending 区，不能用猜测值设置轮速换算 |

`base_link` 使用 x 前、y 左、z 上。yaw_link 原点位于旋转轴，零位轴向与 base_link 一致；装配的固定角度归到 `imu_rpy_rad`。如转轴不是竖直轴，需要修改 yaw TF 模型，而不是将倾斜误差藏进零位。

MID360 光学中心与内置 IMU 原点不是同一点。当前采用 [FAST-LIO 官方 MID360 配置](https://github.com/hku-mars/FAST_LIO/blob/master/config/mid360.yaml)的内部平移 `[-0.011, -0.02329, 0.04412]` m、单位旋转矩阵，关闭在线外参估计。装配测量应对应上述原点定义；已有设备标定结果时替换内部外参。

MID360 专用网口与雷达在同一网段，通常不设默认网关；另一个网口或 Wi-Fi 连接看网页的电脑。通过 Ubuntu NetworkManager 持久设置网口地址。原仓库 `.50/.196` 是作者环境，不代表你的雷达地址。需要确保雷达点云和 IMU UDP 接收地址都是主机该网口地址。

填写完成后生成实车配置：

```bash
python3 web_panel/configure.py --config web_panel/robot.yaml \
  --maps-dir "$HOME/scurm_maps"
```

脚本会备份首次修改前的配置至 `original_configs/`，生成 `runtime.json` 和 `robot_tf.launch.py`，更新以下内容：

1. MID360 IP 与接收 IP；保留 `xfer_format=1`，FAST-LIO 使用 Livox CustomMsg。
2. FAST-LIO PCD 输出、先验地图、ICP 地图、Nav2 地图与行为树路径。
3. 实测矩形 footprint、地形分析尺寸与地面高度范围。地形分析输入已经是 base_link 位姿，因此 sensorOffsetX/Y 设为 0；安装偏移只在 TF 中计算，避免重复加入。
4. 低速 DWB、加减速限制，统一 `/state_estimation`，速度平滑器改为 OPEN_LOOP。

修改 YAML、安装位置或目标角后，重新生成配置并重新编译/重启。`runtime.json` 中的地图路径必须与 Web 使用的路径一致。不要把 Windows 生成的绝对路径配置拿到 Linux 直接用，要在车上重新生成。

## 六、编译

```bash
cd ~/sentry_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 4 \
  --packages-up-to sentry_bringup fast_lio icp_relocalization \
  terrain_analysis terrain_analysis_ext sensor_scan_generation \
  costmap_intensity behavior_ext_plugins \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

本次包含 C++ 修改，必须重新编译 `fast_lio` 和 `icp_relocalization`。FAST-LIO 查询点云时间对应的 yaw TF；ICP 将原始 LiDAR 匹配位姿转换到 IMU 位姿，避免把内部外参遗漏在初始定位链路中。

已在隔离的 Ubuntu 22.04 / ROS 2 Humble 容器中编译上述 9 个包，修正了上游插件遗漏的 nav2_costmap_2d 依赖与恢复动作时钟 API。车载系统版本、依赖版本不同仍需重新编译；遇到错误请保留完整日志与 `ros2 doctor --report` 输出。

## 七、yaw 和底盘先单独验收

MCU 上先验证编码器、电机方向、机械限位、位置环与速度环。**闭环运行在下位机**；Web 的 10 Hz 目标广播不是电机 PID。PID、电流和速度上限都待测，不提供通用增益。

已有 ROS 驱动：启动自己的驱动，让其接收 `/cmd_vel` 与 `/yaw/target`，输出 `/yaw/joint_states`；尽量让底盘通信与 yaw 编码器使用同一 ROS 时钟。将启动命令作为字符串数组写入 `hardware.driver_command`，或者单独启动后使用 `--external-driver`。

采用已约定的串口协议：底盘负责人按 [串口对接约定](串口对接约定.md) 实现 MCU 报文，你这边填写串口、波特率与双方确认的 yaw 编码器换算。优先使用 `/dev/serial/by-id/...` 稳定设备路径。串口权限按系统情况添加 dialout 组并重新登录。车体尺寸尚未测量时，可先使用独立串口配置：

```bash
cp web_panel/serial.example.yaml web_panel/serial.yaml
# 填好串口、波特率、yaw 保持角、编码器正方向/零位后：
python3 web_panel/chassis_bridge.py --config web_panel/serial.yaml --check-config
source /opt/ros/humble/setup.bash
python3 web_panel/chassis_bridge.py --config web_panel/serial.yaml
```

该方式只做串口和 ROS 话题转换，不启动建图、Nav2 或网页，不要求雷达 IP、安装 TF 和车体几何。`--check-config` 不打开串口或 ROS；参数未填完时会指出待测项。完整系统配置完成后，也可用生成的 runtime.json 单独运行：

```bash
source /opt/ros/humble/setup.bash
source ~/sentry_ws/install/setup.bash
cd ~/sentry_ws/src/SCURM_Nav_Tutorial
python3 web_panel/chassis_bridge.py --config web_panel/runtime.json
```

有完整几何但暂不连电机时，可加 `--mock` 模拟底盘和 yaw 状态。它不会打开串口，也不能证明电机方向或实际闭环正确。

yaw 反馈要求含关节名、position、velocity 和有效 ROS 时间戳；当前建议连续 50 Hz。只有角误差小于 0.02 rad、角速度小于 0.03 rad/s，并持续 0.5 s，才允许进入建图/导航。反馈超过 0.3 s 未更新会阻止速度。上述值是起始门控阈值，要根据编码器噪声、机械抖动和定位效果实测调整。

点云和定位按消息采样时间检查新鲜度，超过 1 s、零时间戳或明显来自未来的消息不能放行速度；yaw 超时也从采样时间计算，排队延迟不会延长有效期。空点云、非有限位姿或非法姿态四元数会阻止速度。已有驱动应保留实际采样时间，不能给旧数据重新盖当前时间戳。`Twist` 本身不带时间戳，因此遥控/导航输入按接收时间检查 0.3 s 超时。

## 八、一键启动基础链路

所有新终端先 source ROS 和工作区。配置完整后：

```bash
cd ~/sentry_ws/src/SCURM_Nav_Tutorial
python3 web_panel/bringup.py --config web_panel/runtime.json
```

它按顺序启动底盘/yaw 驱动、yaw TF、MID360、Web。默认为待机。每个进程日志在 `$HOME/scurm_maps/base_logs/<本次编号>/`，网页链接与令牌见该目录 `web.log`。已有驱动在另一终端运行时：

```bash
python3 web_panel/bringup.py --external-driver
```

使用模拟底盘配合真实 ROS/MID360 验证上层链路时：

```bash
python3 web_panel/bringup.py --mock-hardware
```

这仍需要 ROS、编译后的工作区、有效几何配置和真实雷达；网页会显示模拟底盘标记。它与纯网页 `--demo` 演示是两种不同用途。

浏览器打开 Web 日志给出的链接，把 `0.0.0.0` 换成车载电脑的**局域网 IP**，保留 `?token=...`。同一 ROS 环境使用一致的 `ROS_DOMAIN_ID`；启用防火墙时允许所需局域网访问 TCP 8080 与雷达 UDP 数据端口。

`bringup.py` 的 Ctrl+C 会先停止 Web 和模式节点，再停止基础驱动，以便最后的零速度仍可发给底盘。MCU 自身仍必须在命令超时后停车，因为上位机突然掉电时不能发送任何最后命令。

## 九、三个按钮怎么用

**默认只平移，yaw 固定角保持。** 最终 `/cmd_vel` 的 `angular.z` 默认为 0，遥控或 Nav2 输入的转向量不直接放行。yaw 仍由下位机位置闭环保持 `yaw.target_rad`，实际编码器反馈用于 TF 和到位检查；该角度相对底盘固定，底盘旋转时雷达随底盘转动。

Web 的“小陀螺”是独立开关，启动时关闭，只在手动/建图模式可开启。默认角速度 +0.5 rad/s，正值逆时针、负值顺时针，不能超过 Web 的角速度上限。开启后可原地旋转，也可叠加新鲜的平移指令；平移指令超时则平移归零。网页通过 `/api/spin/heartbeat` 维持 0.75 秒有效期，心跳超时后开关锁定为关闭，网络恢复不会自行重启。切换模式、进入待机、yaw 未锁定、点云或定位超时也关闭小陀螺。

导航采用全向平移：DWB 的 `max_vel_theta=0`、`vtheta_samples=1`，不要求到点后的最终朝向，不使用旋转恢复。导航模式不启用小陀螺，避免轨迹碰撞预测与实际旋转不一致。`configure.py` 会生成相同设置。

| 按钮 | 软件行为 | 底盘速度来源 |
| --- | --- | --- |
| 建图 | yaw 锁定后启动 FAST-LIO、地形分析与 OctoMap | 仅 `/web/cmd_vel_manual` |
| 导航 | 保存当前建图结果，检查已有地图，再启动 ICP、先验地图定位与 Nav2 | 仅 `/web/cmd_vel_auto` |
| 待机 | 停止模式节点，保留雷达、TF 和驱动，持续输出零底盘速度 | 两个来源均阻止 |

切换先关闭速度门控，并清除旧速度。离开建图时自动保存 `site.pcd`、`site.yaml`、`site.png`。建图到导航若保存失败，保留建图节点并显示错误；进入待机时即使保存失败也会停车并显示错误。地图保存会覆盖同名文件，保留历史版本请先复制地图目录。

初始待机没有启动 FAST-LIO，页面显示“尚无点云”是正常的。建图后显示当前帧 `/cloud_registered`，不是不断发送整幅累计地图。默认最多 6000 点、最高约 5 Hz，Web 展示采样不改变导航所用点云。

开始建图时静止完成 IMU 初始化，确认 yaw 已锁定、点云和里程计新鲜，再慢速移动。键盘输入接到门控前：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/web/cmd_vel_manual
```

按该工具提示使用 Shift 对应的全向移动键。原有遥控器也输出到 `/web/cmd_vel_manual`。不要同时直接发布最终 `/cmd_vel`。

导航模式按钮表示已选择并启动相应节点，不等于定位成功或已分配目标。回到建图起点附近启动 ICP，或通过 RViz `2D Pose Estimate` 提供接近真实位置的 `/initialpose`。ICP 是局部匹配，初始位姿相差很大时不应靠不断放宽阈值解决。

点击顶部“导航”会自动打开地图页，并启动原有定位与 Nav2 链路。定位成功、yaw 到位且点云/里程计新鲜后，在地图上单击选择位置，点击“前往目标”向 `/navigate_to_pose` 提交 `map` 坐标系下的 NavigateToPose 任务，目标朝向沿用当前地图航向。单击只选点，不发送运动命令；拖动用于平移地图。页面显示车辆地图位置、导航状态和剩余距离；“取消导航”立即关闭上位机速度输出并请求取消任务，确认取消后可以重新选点出发。切换运行模式也会取消任务，迟到的任务响应不会恢复旧任务。

网页通过 `POST /api/navigation/goal` 接收 `{"x":1.2,"y":3.4,"map_revision":"来自 /api/map 的 revision"}`，通过 `POST /api/navigation/cancel` 接收 `{}`；两者都需要 Bearer 令牌。地图变更、坐标越界、任务未结束、模式或传感器未就绪时会拒绝目标。接口只提交 Nav2 任务，底盘速度继续通过原有门控输出。未编译配置完整导航链路时，仅显示地图和未就绪原因，不能驱动车辆；当前独立雷达预览允许点击“导航”打开地图，仍不会启动实车导航。

网页任务开始后，后续目标请继续通过网页提交。网页的取消按钮针对本网页提交的任务；切换待机则停止整套导航节点。

## 十、按层检查所有链路

检查工具只订阅和查询，不发送运动命令。每一层通过后再进入下一层：

```bash
# 基础驱动、MID360、yaw 反馈与 TF；Web 保持待机
python3 web_panel/preflight.py --mode idle --report "$HOME/scurm_maps/检查_待机.json"

# 点击建图后检查点云、里程计、地形、投影地图与保存服务
python3 web_panel/preflight.py --mode mapping --report "$HOME/scurm_maps/检查_建图.json"

# 点击导航并完成 ICP 后检查地图、局部代价地图、TF、导航 action
python3 web_panel/preflight.py --mode navigation --report "$HOME/scurm_maps/检查_导航.json"
```

报告会检查 `/cmd_vel` 是否仅由模式门控发布，以及 yaw 是否实际到位稳定。话题/action 存在不等于导航任务成功，最后还需看实际轨迹与停车距离。

分层人工验收：

1. **通信与方向：** 验证串口/CAN/网口、编码器零位和 yaw 正方向；车辆前进、向左横移、逆时针转向必须与 ROS 坐标一致。轮式里程计方向与真实运动一致。
2. **yaw 保持：** 静止时到位；轻微扰动后能恢复；底盘旋转时相对底盘角不变。拔掉反馈或反馈偏角时网页显示问题、最终速度归零。
3. **雷达：** `ros2 topic hz /livox/lidar` 与 `/livox/imu` 连续输入。保留 CustomMsg，不要为了浏览器把 `xfer_format` 改为 0。
4. **时间与 TF：** 检查 `ros2 run tf2_ros tf2_echo base_link imu_link` 和 `odom base_link`；yaw 反馈时间戳与点云处于同一 ROS 时钟。若出现 TF extrapolation，先检查时间同步、反馈频率/延迟，不能改回恒为零的静态 yaw 角来掩盖问题。
5. **建图：** 检查 `/cloud_registered`、`/state_estimation` 新鲜；墙面稳定，横移/转弯时不出现明显重影。顶置雷达的车体遮挡和低矮障碍需要现场验证。
6. **地形与障碍：** 检查 `/terrain_map`、`/terrain_map_ext`、`/projected_map`。原代价层读取的是地形处理后的相对高度值，不能直接接原始雷达反射强度。根据离地间隙和可跨障碍高度调整地形阈值。
7. **保存与重定位：** 建图后进入待机，确认 PCD 与二维地图均存在且内容有效。导航时 `map → odom → base_link` 连通，车辆位置和实际位置吻合。
8. **导航与横移：** 在低速开放区域设置目标，确认避障、横向速度、路径跟随；确认 footprint 包含车体完整外形。
9. **退出与失联：** 验证导航到待机、速度输入停止、雷达输入停止、yaw 反馈停止、上位机进程退出后的实际停车。MCU 命令超时必须有效。

## 十一、起始参数与调参顺序

| 项目 | 当前起点 | 调整依据 |
| --- | --- | --- |
| 前后/横移速度 | 各方向最大 0.3 m/s，Web 另限制合速度 0.3 m/s | 停车距离、横移打滑、实际底盘能力 |
| 底盘角速度 | 普通/导航 0；小陀螺起始 0.5 rad/s | 小陀螺仅手动/建图开启，按轮胎抓地和定位效果调整 |
| 平面加速度 | 0.3 m/s² | 电机能力与载荷，横移通常更易打滑 |
| DWB | 15 Hz，10×10×16 采样，预测 1.5 s | 先测真实计算周期，再增采样或速度 |
| 点云体素 | 0.15 m | 定位精度、窄通道与点云密度 |
| 代价地图 | 0.05 m 分辨率，实测矩形 footprint + 0.03 m | 障碍尺寸、车体边界和通道宽度 |
| 膨胀半径 | 至少 0.4 m，随车体外接半径增大 | 宽通道避障余量与窄通道可通过性 |
| 速度平滑器 | OPEN_LOOP，20 Hz | 原 LIO 没有有效 twist；有真实底盘速度后再评估闭环 |

先调电机与 yaw，随后校准 TF/时间、验证建图、验证障碍，再调 Nav2。提高速度时同时更新控制器、平滑器、Web 限幅与下位机能力参数，不能只改一个 `max_vel_x`。

## 十二、验证范围、日志与回退

当前本地测试执行：

```bash
cd web_panel
python -m unittest discover -v
```

覆盖模式与 HTTP、保存失败、缺失地图、退出停车门控、命令/传感器超时、yaw 偏角/速度/断流/稳定时间、角度环绕、参考协议与零位换算、点云行填充和大端解码、配置路径与首次备份。Chrome 演示另验证点云和三个按钮。

另提供 [隔离 ROS 验证方法](validation/README.md)，用合成 MID360/IMU 与模拟底盘运行实际 FAST-LIO、地形、OctoMap、ICP、Nav2 和 Web 节点。该验证与真实车辆验收分开记录在 [验收状态](验收状态.md)；合成参数不会写入实车配置模板。

基础链路日志在 `base_logs/`，模式和地图保存日志在 `web_logs/`，检查工具输出可保存在地图目录。回退参数时先进入待机、停止启动程序，再从 `original_configs/` 逐项恢复，重新编译；代码回退可使用上游提交与交付补丁。不要在运行中只恢复一半参数，让 TF、地图路径和驱动设置互相矛盾。

Web 当前用于带令牌的局域网访问。需要外网时增加 VPN 或带 TLS/认证的反向代理，再验证控制权限与网络中断行为。

## 十三、通过验收后配置开机启动

提供了 `scurm-car.service.in` 模板。先手动跑通并验收，再复制模板为 `scurm-car.service`，替换三处占位符：`@CAR_USER@` 为运行 ROS 的普通用户，`@WORKSPACE@` 为工作区绝对路径，`@REPO@` 为仓库绝对路径。模板按常见无空格 Linux 家目录编写，若路径含空格，应按 systemd/bash 规则重新引用路径。同时将 `ROS_DOMAIN_ID=0` 改为与你其他 ROS 设备一致的值。

确认模板中的驱动方式与你填写的配置一致；已有驱动由外部服务管理时，可在 ExecStart 的 bringup 命令后加 `--external-driver`，并确保该驱动服务先启动。

在车上安装和启动填写后的文件：

```bash
sudo cp web_panel/scurm-car.service /etc/systemd/system/scurm-car.service
sudo systemctl daemon-reload
sudo systemctl enable --now scurm-car.service
systemctl status scurm-car.service
journalctl -u scurm-car.service -f
```

仍然默认待机；开机不会自动行驶。Web 链接与令牌保存在本次基础日志的 `web.log`。停止与关闭自启动：

```bash
sudo systemctl stop scurm-car.service
sudo systemctl disable scurm-car.service
```

若当前正在建图，先在网页点待机并确认地图已保存，再停止服务。进程异常被 systemd 重启后重新进入待机，确认定位和状态后再操作。

### 建图动态物体过滤

网页“实时点云”工具栏增加“建图动态过滤”（默认开启），点“应用到建图”保存；仅待机可修改，下次建图生效。与去顶设置一起保存在 `maps_dir/mapping_filter.json`，新增布尔字段 `dynamic_enabled`，旧文件默认开启。命令行可用 `map_dynamic_enabled:=false` 关闭。

过滤只作用于保存的静态地图：

- FAST-LIO 每个传感器扫描最多按 5 Hz 更新静态体素（0.15 m），同一时间戳不会重复确认；连续观测至少 1.5 秒且不少于 6 帧才进入 `/Laser_map` 和 PCD。建议缓慢行驶，新区域停留约 2 秒再保存。
- 后续激光实际穿过原体素、测到更远返回时，连续两次自由空间观测将其清除；遮挡、离开视野和雷达暂停本身不会删除已经确认的墙面。
- 二维 `/projected_map` 每秒根据当前静态点云完整重建，结合原 `terrain_analysis_ext` 地面高度判断：高于地面 0.15 m 为障碍，确认的地面为自由区域，没有地面证据的表面保守保留为障碍，未观察区域保持未知。动态过滤开启时不启动累积式 OctoMap，避免旧拖影残留；关闭后恢复原投影管线。
- `/livox/lidar`、LIO 内部估计及实时避障点云保持实时观测，行人仍会显示在实时点云并被避障模块感知。过滤不是语义识别人车；缓慢移动或长时间静止的目标仍可能进入静态地图，离开后需重新观察该位置才能清除。已保存的旧地图需要重新建图才有此效果。

静态导出半径每次扫描为 50 m，最多 200 万体素，二维栅格最多 1600 万格；达到体素上限会记录告警并停止加入新体素。超出二维上限会报错，应分区建图。动态建图时地形分析的下方裁剪范围扩展到 -3 m，以便采集地面用于栅格投影；这是观测范围，不是雷达安装高度。实车主机必须重新编译 `fast_lio`、`sentry_bringup` 和 `terrain_analysis_ext`；雷达网页预览本身不启动建图。

验证过滤行为（无 ROS / PCL 依赖）：

```bash
g++ -std=c++14 -O2 -I FAST_LIO/include FAST_LIO/test/test_static_map_filter.cpp -o /tmp/static-map-test
/tmp/static-map-test
python3 -m unittest discover -s web_panel -p 'test*.py'
```
