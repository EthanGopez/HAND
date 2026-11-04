#!/usr/bin/env bash
set -eo pipefail

readonly PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ROS_WORKSPACE="${PROJECT_ROOT}/ros_ws"
readonly XARM_SOURCE="${ROS_WORKSPACE}/src/xarm_ros2"
readonly XARM_REPOSITORY='https://github.com/xArm-Developer/xarm_ros2.git'
readonly XARM_BRANCH='humble'
readonly XARM_COMMIT='62936f7ea1846a85f7350de2c4c18f39e6d19715'

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble was not found. Run this script inside the HAND dev container.' >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
mkdir -p "${ROS_WORKSPACE}/src"

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi
rosdep update

if [[ ! -e "${XARM_SOURCE}" ]]; then
  echo 'Initializing the pinned xarm_ros2 submodule.'
  if ! git -C "${PROJECT_ROOT}" submodule update --init --recursive -- \
    ros_ws/src/xarm_ros2; then
    echo 'Submodule initialization was unavailable; cloning the same pinned source.'
    git clone --branch "${XARM_BRANCH}" --recurse-submodules \
      "${XARM_REPOSITORY}" "${XARM_SOURCE}"
  fi
fi

if ! git -C "${XARM_SOURCE}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "${XARM_SOURCE} exists but is not a Git checkout; refusing to overwrite it." >&2
  exit 1
fi

current_xarm_commit="$(git -C "${XARM_SOURCE}" rev-parse HEAD)"
if [[ "${current_xarm_commit}" != "${XARM_COMMIT}" ]]; then
  echo 'Checking out the xarm_ros2 revision used by xArm6_manipulation.'
  git -C "${XARM_SOURCE}" checkout "${XARM_COMMIT}"
fi
git -C "${XARM_SOURCE}" submodule update --init --recursive
echo "xarm_ros2 is ready at ${XARM_COMMIT}."

if ! rosdep install \
  --from-paths "${ROS_WORKSPACE}/src" \
  --ignore-src \
  --rosdistro humble \
  -r -y; then
  echo 'rosdep failed. Run rosdep update, then retry this script.' >&2
  exit 1
fi

bash "${PROJECT_ROOT}/scripts/fix_moveit_versions.sh"

echo 'ROS sources and dependencies are ready.'
echo 'Next: bash /workspace/scripts/build_ros_workspace.sh'
