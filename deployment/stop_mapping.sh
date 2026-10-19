#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source "$HOME/Zzj_slam/ws/out/install-livox/setup.bash"
source "$HOME/ccc/mapping_install/setup.bash"
exec python3 "$HOME/ccc/stop_lidar_mapping.py" "$@"
