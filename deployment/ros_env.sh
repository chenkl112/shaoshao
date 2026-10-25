#!/usr/bin/env bash
# Source the active Livox installation and mapping packages without stale underlays.
source /opt/ros/jazzy/setup.bash || return $?
scurm_livox_found=false
for scurm_livox_candidate in "${SCURM_LIVOX_PREFIX:-}" "$HOME/my_test_ws/install" "$HOME/Zzj_slam/ws/out/install-livox"; do
  [[ -n "$scurm_livox_candidate" ]] || continue
  if [[ -f "$scurm_livox_candidate/local_setup.bash" && ( -x "$scurm_livox_candidate/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node" || -x "$scurm_livox_candidate/lib/livox_ros_driver2/livox_ros_driver2_node" ) ]]; then
    source "$scurm_livox_candidate/local_setup.bash" || return $?
    export SCURM_LIVOX_PREFIX="$scurm_livox_candidate"
    scurm_livox_found=true
    break
  fi
done
if [[ "$scurm_livox_found" != true ]]; then
  echo '未找到 Livox 驱动安装：请将 SCURM_LIVOX_PREFIX 设为有效的 install 目录。' >&2
  unset scurm_livox_candidate scurm_livox_found
  return 1
fi
# local_setup avoids the deleted driver underlay baked into setup.bash.
if [[ -f "$HOME/ccc/mapping_install/local_setup.bash" ]]; then
  source "$HOME/ccc/mapping_install/local_setup.bash" || return $?
fi
unset scurm_livox_candidate scurm_livox_found
if [[ -f "$HOME/ccc/localization-venv/bin/activate" ]]; then
  source "$HOME/ccc/localization-venv/bin/activate" || return $?
fi
