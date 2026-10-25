#!/usr/bin/env bash
set -eo pipefail
sim_source="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${SHAOSHAO_ROS_ENV:-}" ]]; then
  source "$SHAOSHAO_ROS_ENV"
else
  source /opt/ros/jazzy/setup.bash
fi
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_IP=127.0.0.1
export GZ_PARTITION="shaoshao-sentry-test-$$"
sim_ros_prefix="$(ros2 pkg prefix gz_sim_vendor)"
sim_vendor_root="$sim_ros_prefix/opt"
for sim_vendor in gz_sim_vendor gz_transport_vendor gz_msgs_vendor gz_math_vendor; do
  export PYTHONPATH="$sim_vendor_root/$sim_vendor/lib/python:${PYTHONPATH:-}"
done
export GZ_SIM_SYSTEM_PLUGIN_PATH="$sim_vendor_root/gz_sim_vendor/lib/gz-sim-8/plugins"
export GZ_SIM_PHYSICS_ENGINE_PATH="$sim_vendor_root/gz_physics_vendor/lib/gz-physics-7/engine-plugins"
exec python3 "$sim_source/run.py" "$@"
