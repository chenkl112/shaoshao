# Navcar UART1 V1 上位机接入

协议来源：<https://github.com/Li-chee/Navcar-Comm-Protocol>，2026-10-17 版。
对照提交：`4eb14adf5d7bc463dca50c092a1b3be66f77ef9a`。
实际下位机使用该二进制协议，旧 `serial_jsonl_v1` 参考文档不适用。

串口 2000000、8N1、无流控，3.3V TTL 共地；MCU RX=PB7、TX=PA9。
控制帧为 `A5 5A / 01 / 01 / uint16序号 / 10 00 / 4个float32 / CRC16`，
所有多字节数小端，四个 float 顺序为 **vy、vx、rot、yaw_angle_deg**。
CRC16/MODBUS 只计算 VERSION 到 PAYLOAD，不包含帧头。

## 现在可以运行的只读验证

在 `~/ccc/shaoshao` 下编辑 `web_panel/serial.navcar.yaml` 的 `serial_port`，
填写 `ls -l /dev/serial/by-id/` 得到的实际路径。
首次从 GitHub 克隆后，先执行 `cp -n web_panel/serial.navcar.example.yaml web_panel/serial.navcar.yaml`。
实际串口配置由 Git 忽略，原压缩包中的配置已同步到本地。

```bash
cd ~/ccc/shaoshao
python3 web_panel/navcar_bridge.py --check-config
python3 web_panel/navcar_bridge.py --no-ros
```

默认只读，不发送控制帧，不接管遥控器。每秒显示5次最新目标及状态年龄。
Ctrl+C 退出只读监视也不发送任何命令。只读验证仅需 Python3、PyYAML、pyserial。
状态 `feedback_kind=targets_only`，不得将四个目标当作传感器实测值。
已有桥接入口也可用 `python3 web_panel/chassis_bridge.py --config web_panel/serial.navcar.yaml --no-ros`，
将自动选择 Navcar 适配器，默认同样只读。

需要向 ROS 发布状态时：

```bash
source /opt/ros/jazzy/setup.bash
python3 web_panel/navcar_bridge.py
```

状态发布到 `/navcar/target_feedback`（std_msgs/String，JSON），包括序号、
四个原生目标、状态年龄、ACK、控制是否启用；不发布 `/yaw/joint_states` 或 `/wheel/odom`。

## 运动适配与待测参数

根据用户实测，原生指令0.5运行10秒移动0.5米，即实际速度约0.05米/秒，
因此平移换算系数为10原生单位/(米/秒)。前后、左右暂统一使用10，横移需另行复测。
页面目标速度上限3.0米/秒，对应原生指令上限30.0；旋转系数为1、原生上限为6.0。
此倍率来自一次测量，实际速度随载荷、电量与方向变化，需通过后续实测修正。

| 输入 | MCU 目标 |
|---|---|
| 云台坐标前进速度 linear.x，m/s | vy = linear.x × forward_units_per_mps |
| 云台坐标向左速度 linear.y，m/s | vx = −linear.y × left_units_per_mps |
| ROS 逆时针旋转请求 angular.z，rad/s | rot = −angular.z × rot_units_per_radps |
| 固定云台目标 | yaw_target_deg，启动航向参考，归一化±180° |

`rot` 是叠加到轮速的量，并非实测底盘偏航速度，实际角速度换算仍待实测。
旋转输入默认0，小陀螺只通过8081明确开启，导航不输出旋转。输入超过原生限值，整条运动指令归零并报告。

```bash
# 仅检查是否满足控制参数，不打开串口
python3 web_panel/navcar_bridge.py --enable-control --check-config
# 显式启用发送（平移按实测倍率10换算）
source /opt/ros/jazzy/setup.bash
python3 web_panel/navcar_bridge.py --enable-control
```

控制输入是 `/navcar/cmd_vel_yaw`（geometry_msgs/Twist），明确约定为**云台坐标系**。
不要直接将 Nav2 的底盘坐标 `/cmd_vel` 重映射过来：云台与底盘夹角变化时需先做坐标变换。
当前 Web 的全车启动和导航门控保持实际 yaw 反馈要求；未绕过门控。

桥接每约20ms发送最新目标；输入超过150ms未更新时发送零平移/零旋转并保留yaw目标。
先收到有效状态 S，首帧使用 S+1；单条在途命令，50ms无匹配ACK时原样重发，最多3次。
ACK=0/1结束等待，下一帧递增序号，即使目标未变；不积压旧目标。
ACK=2～6、状态超时或重试耗尽后退出控制，不继续发送运动命令；重新启动进程重新同步。
MCU在300ms未接受新命令后归零平移和旋转并保留yaw。重复ACK=1本身不刷新MCU看门狗。
正常退出尝试新序号零速帧，确认ACK=0和对应零速状态后关闭；失败明确报告未确认。

## 当前协议对导航的限制

1. 状态回传的是**目标**，没有实际云台相对底盘角、云台角速度、轮速或电机在线状态。
   ACK仅表示命令接受，不证明车辆运动或锁角到位。
2. MCU的yaw是**相对启动朝向**的惯导目标，现有上位机固定yaw关节模型是
   **相对底盘角度**。底盘旋转时两者不同，不能直接替换。
3. 继续实现导航/移动小陀螺，需要下位机新增实际云台相对底盘角及采样时间，
   确认是否提供实际底盘速度；或将控制与定位链改成云台世界锁定模型并完成实测。
   本次只完成通讯与云台坐标控制接口，不生成假的 TF 或轮式里程计。

离线测试（包含协议文档的官方字节向量、拆包粘包、CRC、ACK重试及停止确认）：

```bash
python3 -m unittest discover -s web_panel -p 'test*.py'
```


## 独立前后左右移动页（8081）

```bash
# 小电脑上运行；不要同时运行另一个串口桥
~/ccc/start_move_web.sh
```

打开日志打印的带token完整链接。方向按钮按住移动、松开停止，也支持WASD/方向键，
空格停止。默认目标速度0.10米/秒，可调至3.00米/秒，按实测倍率10换算下发。
打开页面不会接管底盘，首次按方向/停止、设置yaw或开启小陀螺后开始控制；
云台默认保留接入时回传的目标角，yaw面板可设置−180°到180°、±5°微调或回到0°。
方向以云台朝向为准，yaw是相对MCU启动朝向的目标，不是关节实测角。
设置yaw会停止平移、关闭小陀螺并撤销导航控制权。
小陀螺默认关闭，旋转强度范围−6到6，正值逆时针、负值顺时针，尚未换算为rad/s。
可同时平移，松开方向键只停止平移；停止按钮、网页失焦或隐藏同时关闭小陀螺。
心跳超过200ms未更新时下发零平移和零旋转，yaw继续保持已设置目标角。
状态或ACK故障停止发帧，下位机300ms看门狗归零；此时需重启服务重新同步。
此服务直接管理串口，不依赖ROS，不更改8080点云/建图进程。
后台运行日志在 `~/ccc/move_web.log`，PID在 `~/ccc/move_web.pid`。

### 导航路线预览与失败原因

8080 点击地图目标后调用 `POST /api/navigation/plan`，请求体为
`{"x":1.0,"y":2.0,"map_revision":"GET /api/map 返回的 revision"}`。
此接口只调用 Nav2 ComputePathToPose，不获取串口控制权，也不发运动指令。
`GET /api/state` 的 `navigation.plan` 返回 planning/ready/failed、路线点、长度和失败原因。
蓝线显示预览路线；成功后才能点击“前往目标”。行驶中订阅 `/plan` 更新全局路线。
`GET /api/navigation/costmap` 返回实时障碍物和内切碰撞范围，网页分别叠加红色和橙色。
规划失败会保留 Nav2 错误码，例如 205 起点占用、206 目标占用、208 无可通行路径。
AMCL 位姿使用 transient-local 订阅，启动时也能接收最近一次有效定位结果。
车体碰撞范围仍依据 robot.yaml 计算，不通过减小车体或忽略障碍物绕过规划失败。

### yaw、小陀螺与调速接口

以下接口均在8081，使用 `Authorization: Bearer <navcar_move.token中的值>`。
请求为JSON，同一客户端所有控制接口共用递增的 `seq`；`client`是客户端唯一字符串。

```json
POST /api/yaw
{"angle_deg":30,"client":"my-controller","seq":1}

POST /api/spin
{"enabled":true,"rate":0.2,"client":"my-controller","seq":2}

POST /api/spin
{"enabled":true,"rate":-0.3,"client":"my-controller","seq":3}

POST /api/spin
{"enabled":false,"client":"my-controller","seq":4}
```

小陀螺运行时每约70ms重发开启请求并增加seq；停止续命200ms后归零。
`rate`是旋转指令强度，上位机再按 `rot_units_per_radps` 换算，当前系数为1。
`POST /api/command` 可同时携带 `spin_enabled`、`spin_rate`，同时控制平移与旋转；
平移参数 `strength` 是米/秒，带 `speed_unit:"m/s"`。
方向释放时发送 `action:"stop"` 并保留 `spin_enabled:true` 可继续旋转；
总停止发送 `action:"stop",spin_enabled:false`。
`GET /api/state` 的 `spin` 返回当前开关、指令强度和上限，`yaw_target_deg`为上位机目标，
`feedback_targets[3]`为下位机回显的目标；不能据此判断电机已到位。

## 雷达坐标下的二维平移导航

8080 的 MID-360 服务现在接入独立的 Nav2 导航链：FAST-LIO 提供真实雷达里程计，
AMCL 将二维扫描匹配到当前保存的 `site.yaml`，Navfn 规划路径，DWB 输出前后左右平移。
点击“导航”时才启动，若正在建图则先保存并停止建图。等待“导航就绪”，在地图上选点，
点击“前往目标”；“取消导航”停止目标，“待机”关闭定位和规划，雷达继续运行。
默认启动和打开网页均不会开始建图或行驶。

配置读取 `web_panel/robot.yaml` 的实测外接半径（未填写时回退到车长、车宽）、雷达高度及安装偏移。
此链以真实的雷达内置 IMU 坐标 `navcar_imu` 为控制与定位坐标系，使用包含整车和安装偏移的圆形碰撞范围，
不构造底盘或关节的虚假反馈。仅用于水平安装的雷达，底盘旋转量始终0，小陀螺关闭。
它与前文的底盘坐标全车启动链不同；不要求将目标回显冒充关节角。

导航通过8081已有的串口控制器发送向前/向左速度分量，向量总量由 `robot.yaml` 的
`navigation.max_speed_mps` 统一配置；当前实车模板为 0.50 米/秒，旧配置未填写时仍为 0.10 米/秒。
Nav2、导航转发和8081读取同一配置；8081仍受串口原生量程限制。
与手动控制共用倍率10，只在串口发送前换算一次；手动页面上限仍3.0米/秒。
8081授予独立导航控制权；任意有效手动方向或停止指令立即撤销该控制权，
迟到的导航指令无法重新接管。导航心跳失联200ms后归零，ACK与下位机看门狗仍生效。
地图定位、点云或里程计超时也会关闭运动输出。导航结束位置记录在地图的 `last_pose.json`，
下次定位用作初始猜测；待机移动过车辆时，请将车放回该位置附近再定位。

依赖 `ros-jazzy-navigation2`、`ros-jazzy-nav2-bringup`、`ros-jazzy-pointcloud-to-laserscan`。
运行时配置在 `~/ccc/navigation_runtime`，日志在 `~/ccc/navigation.log`。

当前 DWB 使用 `BaseObstacle.scale: 0.01`。原先 0.10 m/s 低速实车轨迹评分显示，旧权重 0.1
会让进入非致命膨胀区的扣分超过向目标前进的收益，从而持续选择零速，最终报 105。
此项只调整可通行轨迹的评分；车体半径、膨胀范围以及致命/内切障碍的轨迹拒绝规则保留。
评分回放已加入回归检查；修改速度、预测时长或地图分辨率后，应重新检查这组权重。

### 当前八边形车体的实测参数

用户确认：四轮斜向麦轮，整车外形近似八边形，含轮子最外侧距车体中心 0.29 m；
yaw 轴位于车体中心且离地 0.31 m，雷达安装在 yaw 上，光学中心离地 0.32 m，水平偏心 0.01 m。
使用 `robot.circumscribed_radius_m: 0.29`，不再从旧的 0.5 × 0.5 m 长宽对角线推算半径。
`yaw.lidar_offset_radius_m: 0.01` 是距离包络，不猜测偏心方向，也不把它当作 IMU 的偏移。

以雷达光学中心为圆心，整车包围半径为 0.30 m。当前 Nav2 原点实际是内置 IMU，
还需加上 LIO `mapping.extrinsic_T` 的水平长度。当前内部外参为 `[-0.011,-0.02329,0.04412]`，
故 `robot_radius` 约为 0.32576 m，另保留 0.02 m padding；高度 0.31 m 不计入水平半径。
这仍包住任何底盘/yaw 朝向，无需假装获得实际车身朝向。
此半径缩小来自实测外形修正，不代表已验证实际窄通道通行或 0.5 m/s 刹停表现。

修改配置后需在待机状态重启 8080 和 8081，再进入导航以重新生成参数。
8081 `GET /api/state` 的 `navigation_max_speed_mps` 应为 0.5；
`~/ccc/navigation_runtime/nav2.yaml` 的 `FollowPath.max_speed_xy` 应为 0.5。
手动页面的速度输入独立，默认仍为 0.10 m/s。

### 窄通道与地图精度

2026-10-19 起，新建地图的静态导出体素、二维投影和全局代价地图默认都为 0.05 m。
LIO 内部匹配的 `filter_size_surf` / `filter_size_map` 保留 0.15 m：静态导出使用去畸变后的原始观测，
无需同时加密里程计内部地图。`map_filter.static_resolution` 控制静态导出体素，
`static_map_projector.resolution` 控制二维投影。修改 C++ 后需要重新编译。
导航阶段不保存或发布静态累计地图，因此关闭 LIO 内部的静态导出过滤计算；实时点云、扫描与 Nav2 障碍层继续运行。

地图 PNG 的未知像素为灰度 205；对应 `free_thresh` 为 0.196，不能用原来的 0.25，
否则 Nav2 会将这类未知像素加载为空地。修正只影响新导出的地图；已有地图和实车配置不会自动重写。

全局代价地图中的 253 是车体内切碰撞边距，254 是障碍，255 是未知；
发布为 `OccupancyGrid` 后分别为 99、100、-1。软膨胀代价不等于禁止通行区域。
增加规划容差、降低 DWB 障碍评分或替换 A* 算法，都不能使被碰撞边距封住的通道自动变得可通行。
具体快照诊断、测量项与处理顺序见 [窄通道排障](窄通道排障.md)。


## 默认启动时只看点云，手动开始建图

```bash
~/ccc/start_mapping.sh
```

此入口现在只启动雷达与8080网页。网页默认待机，不创建新的建图会话。
点击“开始建图”才启动FAST-LIO并创建新地图；“停止建图并保存”保存后回到待机，雷达继续工作。
重复运行启动入口不会重复启动进程，也不会结束已有建图。
8081移动页独立运行：`~/ccc/start_move_web.sh`。目标速度上限3.0米/秒，默认0.10米/秒。
