# 2026-10-18 车端部署脚本

本目录来自 `shaoshao-code-20261018.zip`，对应车端 `~/ccc/` 中的启动脚本。
源码位于 `~/ccc/shaoshao/`；当前脚本使用 Ubuntu 24.04 / ROS 2 Jazzy。

## 恢复目录

在车上的源码根目录执行：

```bash
mkdir -p "$HOME/ccc" "$HOME/.config/mid360-view"
cp -a deployment/. "$HOME/ccc/"
cp deployment/config/MID360_config.json "$HOME/.config/mid360-view/MID360_config.json"
cp -n web_panel/serial.navcar.example.yaml web_panel/serial.navcar.yaml
```

复制雷达配置前核对其中的主机和雷达 IP；已有配置应先备份。
`mapping_runtime/` 恢复到 `~/ccc/mapping_runtime/`，与源码目录并列。
脚本优先使用现有 Livox 安装 `~/Zzj_slam/ws/out/install-livox/`；新电脑需要先安装该驱动，或按实际安装位置修改脚本。

## 配置与入口

- 编辑 `web_panel/serial.navcar.yaml`，填写实际串口；协议和只读联调见 [Navcar 接入](../web_panel/Navcar接入.md)。
- `build_mapping.sh` 编译 FAST-LIO 和地形分析，输出到 `~/ccc/mapping_build/`、`~/ccc/mapping_install/`。
- `start_mapping.sh`、`stop_mapping.sh` 启停建图；地图写入 `~/scurm_maps/`。
- `start_lidar_preview.py` 启动雷达预览；`start_move_web.sh` 启动独立的 8081 底盘控制页。

原压缩包中的 `robot.yaml`、`serial.yaml` 和 `serial.navcar.yaml` 已恢复到本地工程，实际配置不提交 Git。
在另一台电脑上部署时，可从原压缩包恢复，或根据实测数据重新填写。
压缩包中的日志、临时点云、版本清单和补丁文件没有作为运行源码提交。

本次同步不代表已在当前电脑完成 ROS 编译或实车验收。
