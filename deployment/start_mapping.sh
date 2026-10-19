#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
if [[ $# -eq 0 || ( $# -eq 1 && $1 == --idle ) ]]; then
  exec python3 "$HOME/ccc/start_lidar_preview.py"
fi
exec python3 "$HOME/ccc/start_lidar_mapping.py" "$@"
