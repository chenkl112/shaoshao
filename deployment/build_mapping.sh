#!/usr/bin/env bash
set -eo pipefail
source "$HOME/ccc/ros_env.sh"
cd "$HOME/ccc/shaoshao"
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
colcon --log-base "$HOME/ccc/mapping_build_logs" build --base-paths FAST_LIO autonomous_exploration_development_environment/src/terrain_analysis_ext --build-base "$HOME/ccc/mapping_build" --install-base "$HOME/ccc/mapping_install" --packages-select fast_lio terrain_analysis_ext --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
