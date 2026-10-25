# 哨哨导航

## 简介

面向麦轮底盘、yaw 云台和 MID360 的导航工程，使用 Ubuntu 24.04 / ROS 2 Jazzy。网页提供实时点云、建图、导航和待机；支持窄道减速居中、受困退让、保留原目标重规划，以及使用保存的建图点云进行搬车后全局重定位。

导航只输出平移，由下位机负责底盘与 yaw 闭环。二维地图用于定位确认、规划和碰撞检查；三维点云与关键帧用于全局位置检索和配准。

## 部署启动

先安装 ROS 2 Jazzy、Livox-SDK2 和 Jazzy 版 `livox_ros_driver2`。首次配置、串口与雷达网络设置见 [完整部署指南](deployment/README.md)。车端目录统一使用 `~/ccc`，地图保存在 `~/scurm_maps`。

首次获取源码：

```bash
mkdir -p ~/ccc
git clone https://github.com/chenkl112/shaoshao.git ~/ccc/shaoshao
```

已有工程时，先保存建图、进入待机并退出旧服务，备份车端配置和部署脚本，再更新：

```bash
cd ~/ccc/shaoshao
git pull --ff-only origin master
```

安装依赖、复制部署脚本并编译：

```bash
sudo apt update
sudo apt install -y build-essential cmake python3-dev python3-venv \
  python3-colcon-common-extensions python3-yaml python3-serial \
  python3-numpy python3-scipy python3-pil libeigen3-dev libpcl-dev \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-smac-planner \
  ros-jazzy-pointcloud-to-laserscan ros-jazzy-pcl-ros \
  ros-jazzy-pcl-conversions ros-jazzy-tf2-geometry-msgs

cd ~/ccc/shaoshao
cp -a deployment/. ~/ccc/
cp -n web_panel/serial.navcar.example.yaml web_panel/serial.navcar.yaml
/usr/bin/python3 -m venv --system-site-packages ~/ccc/localization-venv
~/ccc/localization-venv/bin/pip install -r web_panel/requirements-localization.txt
bash ~/ccc/build_mapping.sh
```

编译前核对 `web_panel/robot.yaml` 的车辆尺寸、安装参数，以及 `web_panel/serial.navcar.yaml` 的实际串口；保留已有标定。驱动默认从 `~/my_test_ws/install` 或旧安装目录加载；安装在别处时，在构建和启动终端设置 `SCURM_LIVOX_PREFIX` 为实际 `install` 路径。

编译成功后，两个终端分别运行：

```bash
# 终端一：8081 底盘控制，保持前台运行
source ~/ccc/ros_env.sh
bash ~/ccc/start_move_web.sh
```

```bash
# 终端二：雷达与 8080 网页，默认待机
bash ~/ccc/start_mapping.sh
```

打开各终端打印的链接，使用车载电脑实际局域网 IP，保留各自的 token。在 8081 启用控制并确认底盘反馈正常，然后在 8080 操作：

**开始建图 → 停止建图并保存 → 导航 → 等待定位就绪 → 路径预览 → 前往目标。**

完整地图包括 `site.pcd`、`site.yaml`、其引用图像，以及完成保存的 `keyframes/manifest.json` 和 `index.npz`。旧稀疏地图应重新采集，不能靠修改分辨率或填白未知区恢复观测。导航期间操作 8081 会取消导航控制权；异常日志见 `~/ccc/web.log`、`mapping.log` 和 `navigation.log`。
