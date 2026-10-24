#!/usr/bin/env bash
set -e
source "$HOME/ccc/ros_env.sh"
exec python3 "$HOME/ccc/stop_lidar_mapping.py" "$@"
