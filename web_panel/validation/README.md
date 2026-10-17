# 隔离 ROS 2 Humble 验证

这套工具验证软件链路，使用合成 MID360/IMU 与模拟麦轮底盘，不连接真实车辆，也不生成你的实车标定值。需要 Docker、可用网络及足够磁盘空间；镜像包含 PCL、Nav2 和编译依赖。

在仓库根目录执行以下 Bash 命令。源目录只读挂载，编译与虚拟参数写入容器的 `/workspace`；证据输出到当前目录的 `validation_evidence/`。

```bash
docker build -t scurm-humble-validation:local -f web_panel/validation/Dockerfile .
mkdir -p validation_evidence
docker run -d --name scurm-humble-verify \
  --mount "type=bind,src=$PWD,dst=/source,readonly" \
  --mount "type=bind,src=$PWD/validation_evidence,dst=/evidence" \
  -e ROS_DOMAIN_ID=71 -e ROS_LOCALHOST_ONLY=1 \
  -e SCURM_SYNTHETIC_VALIDATION=1 \
  scurm-humble-validation:local sleep infinity
docker exec scurm-humble-verify bash /source/web_panel/validation/compile.sh
docker exec -e SCURM_VALIDATION_EVIDENCE=/evidence/serial scurm-humble-verify bash -c \
  'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; python3 /workspace/src/SCURM_Nav_Tutorial/web_panel/validation/serial_smoke.py'
docker exec scurm-humble-verify bash -c \
  'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; python3 /workspace/src/SCURM_Nav_Tutorial/web_panel/validation/ros_smoke.py'
```

Windows PowerShell 可将 `$PWD` 改为仓库绝对路径，续行符改为反引号，或将每条 Docker 命令写成一行。验证容器没有映射串口、USB 或 HTTP 端口；ROS 仅在容器本机通信。

`compile.sh` 编译 9 个 ROS 包并运行 Python 测试。`ros_smoke.py` 依次验证：

1. 实际 pyserial 打开容器内部 PTY 虚拟串口，收发 JSONL 给虚拟下位机；yaw 反馈进入 Web，实际关节 TF 连通。
2. 网页 API 进入建图，FAST-LIO 输出点云/里程计，地形与 OctoMap 输出二维地图，手动速度通过门控驱动模拟底盘。
3. 进入待机，保存 PCD、PNG、YAML，停止建图进程。
4. 进入导航，ICP 重定位、Nav2 lifecycle 激活、导航前只读检查通过。
5. 发送真实 NavigateToPose action，模拟底盘运动并到达测试目标。
6. 暂停合成雷达，确认速度归零；持续重放旧时间戳点云/定位也不能恢复运动，恢复新数据后链路继续工作。
7. 注入异常 yaw 反馈后速度归零，网页待机后停止导航，检查模式进程正常退出。

检查 `ros-integration.json` 中的 `passed`，以及 `preflight-mapping.json`、`preflight-navigation.json`。日志包含 `colcon-build.log`、`tests-humble.log`、节点日志及 `synthetic_maps/web_logs/`。设置 `SCURM_VALIDATION_EVIDENCE` 可以选择容器内不同的证据子目录。

`serial_smoke.py` 独立检查三轴速度的串口字节、yaw 编码器方向/零位换算、分帧与坏帧恢复、重复反馈/反馈断流超时、ROS 命令超时以及桥退出时的零速度报文。它使用不含导航几何的最小配置，结果写入 serial-integration.json。PTY 不代表实际 USB/UART 电气连接、波特率精度或电机闭环已验收。

测试结束后停止并删除本次容器；源目录与证据不受影响：

```bash
docker stop scurm-humble-verify
docker rm scurm-humble-verify
```

合成场景没有真实雷达盲区、时钟漂移、振动、轮胎打滑或电机动力学。因此，即使此验证通过，仍应按主部署指南测量参数、适配控制板协议并完成实车验收。`robot.example.yaml` 里的待测项必须继续保留，不能抄用测试脚本里的虚拟车参数。
