#!/usr/bin/env bash
# 在隔离工作区复制并编译，不改动 /source 中的用户待测配置。
set -eo pipefail
source /opt/ros/humble/setup.bash
mkdir -p /workspace/src/SCURM_Nav_Tutorial /evidence
for package in sentry_bringup FAST_LIO livox_ros_driver2 icp_relocalization autonomous_exploration_development_environment nav2_plugins web_panel; do
    cp -a "/source/$package" /workspace/src/SCURM_Nav_Tutorial/
done
cd /workspace
colcon build --symlink-install --parallel-workers 4 \
    --packages-up-to sentry_bringup fast_lio icp_relocalization \
    terrain_analysis terrain_analysis_ext sensor_scan_generation \
    costmap_intensity behavior_ext_plugins \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    2>&1 | tee /evidence/colcon-build.log
source /workspace/install/setup.bash
cd /workspace/src/SCURM_Nav_Tutorial/web_panel
python3 -m unittest discover -v 2>&1 | tee /evidence/tests-humble.log
