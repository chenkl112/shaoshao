# 哨哨：麦轮车 + yaw 云台 + MID360 导航与 Web 面板

基于 SCURM 火锅战队 24 赛季哨兵导航，适配目标为 i7-12800HX、麦轮底盘与安装在 yaw 云台上的 MID360。网页提供实时点云、车辆位置、yaw 状态，以及 **建图 / 导航 / 待机** 三个模式按钮。

## 从这里开始

- [详细中文上车部署指南](web_panel/README.md)：系统安装、测量、网络、编译、闭环 yaw、启动、三个模式、分层验收、调参与日志。
- [待测参数模板](web_panel/robot.example.yaml)：未知值保留 null/待测，实车必填项未填完不会生成可启动配置。
- [底盘与 yaw 硬件接口](web_panel/硬件接口.md)：ROS 接口、参考串口协议、编码器标定、下位机职责与麦轮公式。
- [给底盘负责人的串口对接约定](web_panel/串口对接约定.md)：已选 JSONL 协议、双向报文、单位、频率和分工。
- [原项目说明](README_原项目.md)：上游比赛/仿真环境与相关项目来源。

## 无硬件先看网页

```bash
python web_panel/server.py --demo --port 8080
```

打开终端打印的链接。演示使用合成点云，不连接车辆。

## 实车路径

建议 Ubuntu 22.04 + ROS 2 Humble。先按详细指南安装 ROS、Livox-SDK2 和依赖，再在车上执行：

```bash
cp web_panel/robot.example.yaml web_panel/robot.yaml
python3 web_panel/preflight.py --offline --config web_panel/robot.yaml
# 填写实测几何、yaw 参考/目标、IP、串口和波特率后：
python3 web_panel/configure.py --config web_panel/robot.yaml --maps-dir "$HOME/scurm_maps"
# 按详细指南重新 colcon 编译，并 source 工作区后：
python3 web_panel/bringup.py
```

本版使用 **相对底盘的固定 yaw 关节角闭环**，读取实际编码器角发布动态 TF。实际需求已确认为“云台保持世界方向”，该控制方式尚未实现，不能仅修改配置启用。锁角闭环由下位机实现，上位机提供目标、反馈检查和底盘速度门控。

底盘由另一位负责人实现；你这边只负责上位机和串口接口。双方采用 `serial_jsonl_v1`，轮径、减速比、四轮 PID 等留给下位机。车体尺寸暂未测量时，可以先按对接约定使用 `serial.example.yaml` 单独联调串口。

## 新增内容

- Web 点云查看与三个模式按钮，局域网令牌接入。
- 模式进程管理、离开建图自动保存 PCD 与二维地图、命令/传感器/yaw 超时门控。
- `base_link → yaw_link → imu_link` 实测安装链，按真实 yaw 反馈发布 TF。
- 参考串口桥与模拟硬件入口；已有 ROS 驱动可按统一接口接入。
- 中文待测配置、实测参数生成、一键基础链路启动、只读链路检查。
- 实车低速 DWB/footprint 初始配置，FAST-LIO 按点云时间查询安装 TF，ICP 输出正确的初始 IMU 位姿。

## 验证与尚未完成的实车事项

本地可执行测试：

```bash
cd web_panel
python -m unittest discover -v
```

已在隔离 Ubuntu 22.04 / ROS 2 Humble 容器编译所需 9 个 ROS 包，并用合成 MID360/IMU 与模拟底盘验证建图、地图保存、ICP 重定位、Nav2 到点、yaw 异常阻断速度及待机。另已检查浏览器点云和三个按钮，详情见 [验收状态](web_panel/验收状态.md) 与 [复现方法](web_panel/validation/README.md)。

本次没有连接你的车辆和传感器。上位机车载版本、几何、实际串口连接、MID360 网络、定位精度和实际行驶仍待验收；下位机按已选协议实现和验证电机闭环由底盘负责人负责。待测参数不代表已标定。

上游项目来源与致谢见原项目说明；本次交付的修改以当前目录和增量补丁为准，重新从上游克隆不会自动获得新增功能。
