#!/usr/bin/env bash
set -euo pipefail

readonly PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f "${PROJECT_ROOT}/ros_ws/install/setup.bash" ]]; then
  echo 'The ROS workspace has not been built. Run build_ros_workspace.sh first.' >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/ros_ws/install/setup.bash"
bash "${PROJECT_ROOT}/.devcontainer/start-vnc.sh"
export DISPLAY=:1

# Simulation/fake hardware only. Do not add a robot IP here.
exec ros2 launch xarm_planner xarm6_planner_fake.launch.py

