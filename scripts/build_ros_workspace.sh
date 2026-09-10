#!/usr/bin/env bash
set -euo pipefail

readonly PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ROS_WORKSPACE="${PROJECT_ROOT}/ros_ws"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble was not found. Run this script inside the HAND dev container.' >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
cd "${ROS_WORKSPACE}"
colcon build --symlink-install --parallel-workers 2

echo "Built ${ROS_WORKSPACE}."
echo 'Source it with: source /workspace/ros_ws/install/setup.bash'

