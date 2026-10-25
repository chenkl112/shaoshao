# sentry_planning 场景中的 Jazzy 导航验证

场景来源：[zzt-180/sentry_planning](https://github.com/zzt-180/sentry_planning)，固定提交
`b9349d2ada3fe0299ccae9d1e483d5f3b79b1cc8`，原文件
`src/sentry_gazebo_2024/sentry/sentry_gazebo/worlds/room.world`。
原件和 MIT 许可证在 `assets/`。

原工程使用 ROS 1 Noetic + Gazebo Classic。本适配使用 **Gazebo Harmonic 的真实物理服务器、Nav2 和 AMCL**，
保留房间全部八段墙体的碰撞几何与位姿，将原差速机器人替换为本车外接圆半径 0.297658 m 的全向测试体。
额外添加两块具名障碍，形成可配置宽度的直通瓶颈。它不是原工程 OCS2 控制器或 RMUC 整场模型的运行结果。

局部里程计和 LaserScan 是仿真适配：从物理服务器的运动和同一组碰撞盒生成，遵守历史采样时间，
AMCL 自行估计地图位置。真实位姿只用于生成合成传感器及验收，不能进入地图定位门控。
这些测试不验证 MID360 的光学采样、FAST-LIO、轮胎打滑或真实底盘电机闭环。

## 运行

需要 ROS 2 Jazzy、Navigation2、Gazebo Harmonic/ros_gz、NumPy、SciPy、Pillow、PyYAML。
不需要图形界面、ROS 1 或真实串口。测试使用独立 ROS domain、Gazebo partition 和内存模拟的 MCU 租约，
不会调用实际 8081 服务，不改变个人 `robot.yaml`。生成配置均在输出目录。

```bash
python3 validation/sentry_sim/run_suite.py --output /tmp/shaoshao-sim/suite
bash validation/sentry_sim/run.sh --case smooth --width .8 --output /tmp/shaoshao-sim/smooth
bash validation/sentry_sim/run.sh --case narrow --width .75 --output /tmp/shaoshao-sim/narrow75
bash validation/sentry_sim/run.sh --case narrow --width .65 --output /tmp/shaoshao-sim/narrow65
bash validation/sentry_sim/run.sh --case narrow --width .65 --start-offset .04 --output /tmp/shaoshao-sim/narrow65-offset
bash validation/sentry_sim/run.sh --case reject --width .58 --output /tmp/shaoshao-sim/reject58
bash validation/sentry_sim/run.sh --case stuck --width .8 --output /tmp/shaoshao-sim/stuck
bash validation/sentry_sim/run.sh --case localization --width .8 --output /tmp/shaoshao-sim/localization
bash validation/sentry_sim/run.sh --case cancel --width .8 --output /tmp/shaoshao-sim/cancel
```

如使用独立 ROS overlay，通过 `SHAOSHAO_ROS_ENV=/path/to/environment.sh` 指定环境。
默认 ROS Domain 187；同时运行时必须分别指定不同的 `ROS_DOMAIN_ID`。

## 线程与延迟检查

导航的里程计/速度、定位、原始局部地图、扫描匹配、可视化、状态监管和动作回调使用独立回调组，
实车 `lidar_mapping_web.py` 与仿真均使用 4 个 ROS 执行线程。各组内部仍按顺序处理；
里程计和路径订阅仅保留最新帧，避免慢回调导致旧数据排队。
全局地图显示使用 NumPy 提取障碍索引，TF 查询与地图文件读取在共享状态锁之外执行。
切换地图后，旧会话的地图、扫描和 TF 结果不能写回新会话。

局部地图距离场预计算、MCU 状态查询、约 50 Hz 运动输出各有独立线程。
状态 HTTP 回复变慢不会占住运动输出；过期的查询回复也不能撤销新任务的租约。
真实的传感器/授权失效仍切零，180 ms 输出间隔、300 ms 里程计和 500 ms 局部地图门槛没有放宽。

完整套件还包含 `slow-status80`：将每次 MCU 状态回复延迟 250 ms，检查通行与输出连续性。
该项模拟状态调用的阻塞，不模拟 HTTP 超时报错；真实请求失败仍撤销输出许可。
手动复现与采样：

```bash
bash validation/sentry_sim/run.sh --case smooth --width .8 --state-delay .25 --output /tmp/shaoshao-sim/slow-status
bash validation/sentry_sim/run.sh --case narrow --width .65 --stall-stacks --output /tmp/shaoshao-sim/stall-probe
```

`callback-timing.csv` 为回调开始时间、名称、墙钟耗时、当前线程 CPU 耗时；
`--stall-stacks` 启用一个仅用于诊断的外部监视进程，在行驶输出超过 120 ms 未更新时请求线程栈。
它不发送运动指令，也不改变控制超时门槛。

## 验收与输出

- `smooth` / `narrow`：实际穿过瓶颈到达，目标不变、无墙体碰撞，检查正常行驶段下发速度、加速度和 jerk。
- `reject`：目标放在无法容纳完整车体的 58 cm 通道中，保持停止并保留目标。不能为了通过缩小包络。
- `stuck`：注入 8 s 的平移动作阻塞，检查检测、受控退让、重规划及原目标到达。
- `localization`：行驶中向 AMCL 注入偏移 1 m 的错误位姿，检查失定位停车、自动重定位和原目标续行。
- `cancel`：途中取消，检查恢复任务被撤销，迟到的定位/动作结果不能再次启动。

每次输出 `result.json`、原始命令 `commands.csv`、真实物理运动 `body.csv`、恢复状态 `states.csv`、
全部节点日志、生成的地图和 `scene.sdf`。`commands.csv` 列依次为：单调接收时间、vx、vy、
速度轨迹采样时间、轨迹 ax、ay；`body.csv` 为时间、x、y、yaw、vx、vy、实际外形至墙体的余量。
故障停车不服从舒适性 jerk 限制，故障场景仍完整记录这些峰值，不把它们计为正常行驶的平顺性通过。
即使非故障注入场景出现输出或感知超时，也会保留急停事件、全程峰值和 `uninterrupted_motion=false`。
`normal_max_*` 只排除明确记录的急停事件附近 40 ms；它不能代表整段无突停。

按用户要求，当前取消每侧 20 mm 的额外余量：Nav2 `footprint_padding=0`，
输出检查半径等于实测外接半径 0.297658 m，圆形包络直径 595.316 mm。
650 mm 通道有约 27.3 mm/侧的几何空间；栅格化和实时观测仍可能减少地图中的可用空间。
局部控制代价图使用 1 cm 或更细栅格，减少静态地图变换到滚动局部窗口时的再次离散损失；
保存地图和全局代价图保留真实分辨率，不修改旧图或填充未知区。
EDT 快速检查无法确认时，对完整占据/未知方格与圆形扫掠做精确距离检查，不额外叠加离散近似余量。
真实车体、完整墙格和未知区仍参与碰撞判定；不能从仿真结果推断实车验收完成。

窄道先检查行进方向前方 0.7 m 的净空，连续降低速度目标，最窄处目标约 0.08 m/s；
目标经过原加速度/jerk 平滑器后输出。全局 Smac 提高膨胀代价权重、局部 PathDist 跟随居中路径；
只有前方横截面存在两侧障碍时才施加至多 0.035 m/s 的横向居中修正，单侧墙不触发。
膨胀区允许通行，接触真实墙体的轨迹仍拒绝。`result.json` 的 `passage` 记录瓶颈内的
最大中心偏差、实际速度与最小物理净空，便于检验减速和居中是否实际生效。
65 cm 窄道验收还要求瓶颈内实际速度不超过 0.12 m/s、中心偏差不超过 2 cm，并包含偏置 4 cm 起步用例。

## 运行代码入口

- `web_panel/navigation_runtime.py`：速度轨迹、原始代价栅格、完整圆形扫掠及恢复策略。
- `web_panel/navigation_recovery.py`：保留任务、有限重试、全向退让、定位监管和 AMCL 自动重定位。
- `web_panel/navcar_navigation.py`：原有唯一 MCU 租约上的 50 Hz 输出，处理后的指令再次检查制动空间。
- `web_panel/server.py` / `index.html`：恢复状态、原目标显示、取消与回调代次隔离。

新定位门控同时检查协方差、消息新鲜度、独立扫描与地图的一致性，以及 `map → odom` 的校正跳变。
失定位先用最后可信位置重播初值，再根据需要调用 AMCL 全局重初始化；至少三个独立扫描和约一秒稳定后续行。
AMCL 仍是唯一地图 TF 发布者。上述为此前二维恢复流程；当前已增加保存 PCD/关键帧全局搜索，详见 [建图地图重定位](../../web_panel/建图地图重定位.md)。
全局规划采用 Smac 2D A*，避免测试中 NavFn 在窄道找到势场却无法提取路径的问题。
局部安全检查会细分原栅格的完整占据方格，提升圆形包络碰撞检查的几何精度，
不更改地图分辨率、不增加空闲观测、不缩小真实车体。

参考原工程的分层规划、轨迹优化和控制约束思路，以及 HWSentryNav26 的受困退让、恢复后重规划策略。
本次没有直接移植 ROS 1 OCS2、轮腿越障或参考工程的高速参数。
局部地图保留 unknown；碰撞预计算在独立线程处理，停止旧地图会话后不能接纳旧结果。
局部代价图同时融合已保存静态地图与实时障碍，避免当前扫描的空洞覆盖已知空闲区。
测试的传感器使用独立回调组，模拟 MCU 的 200 ms 看门狗独立于 ROS 回调运行。
导出 PNG 先完整写入临时文件再替换，随后更新 YAML，减少中断时留下空图像的情况。
运动脱困每轮有次数限制；临时无路时保留目标，以至多每五秒一次的被动重规划等待通路恢复。
脱困结束和到达目标都先平稳停止，急停及手动接管保持立即停止。

正常输出以 0.25 m/s²、0.6 m/s³ 为平顺目标，并保留 20% 的合速度余量用于连续制动；
配置硬上限 0.5 m/s 时，正常平顺速度目标不超过 0.4 m/s。
规划器短暂断更时，在感知和定位仍可信且制动通路有效的条件下平稳减速；
传感器、定位、授权失效或硬障碍会立即切零，优先保证停止。
FAST-LIO 同时改为发布滤波器速度及当前帧协方差，桥接层补偿到轴心参考点，
不再用异步位置差分生成局部控制速度；必须重新构建 FAST-LIO，不能仅更新 Python。
制动检查使用 150 ms 反应时间和配置减速度，包含反应期继续加速的距离；实车必须测量并校准制动能力。

新 FAST-LIO 二进制可单独验证（先加载含 Livox 消息的构建环境）：

```bash
ROS_DOMAIN_ID=190 ROS_LOCALHOST_ONLY=1 python3 validation/sentry_sim/check_lio_velocity.py \
  --binary /path/to/install/fast_lio/lib/fast_lio/fastlio_mapping --output /tmp/shaoshao-lio-probe
```

安装 Matplotlib 后可导出完整轨迹与速度/加速度/jerk 曲线：

```bash
python3 validation/sentry_sim/plot_results.py /tmp/shaoshao-sim/suite/narrow75
```

`motion.png/svg` 保留完整导航速度和急停峰值；`passage.png/svg` 单独显示瓶颈内实际中心偏差、
零 padding 的物理包络边界和实际速度（包含停车）。
