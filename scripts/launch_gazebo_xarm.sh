#!/usr/bin/env bash
set -eo pipefail

readonly PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f "${PROJECT_ROOT}/ros_ws/install/setup.bash" ]]; then
  echo 'The ROS workspace has not been built. Run build_ros_workspace.sh first.' >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/ros_ws/install/setup.bash"
bash "${PROJECT_ROOT}/scripts/fix_moveit_versions.sh"
bash "${PROJECT_ROOT}/.devcontainer/start-vnc.sh"
export DISPLAY=:1

# Gazebo simulation only. This launch file uses gazebo_ros2_control and does
# not accept or connect to a physical robot IP.
exec ros2 launch xarm_moveit_config xarm6_moveit_gazebo.launch.py
