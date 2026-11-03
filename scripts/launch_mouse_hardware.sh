#!/usr/bin/env bash
# Connect to a PHYSICAL xArm 6, discovering its address when omitted.
set -eo pipefail
readonly PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "Usage: bash scripts/launch_mouse_hardware.sh [ROBOT_IP] [max_speed:=0.05 mouse_gain:=0.0001]"
  echo "With no IP, discover a single xArm 6 on the local wired LAN. Connecting enables motors."
  exit 0
fi
ROBOT_ADDRESS=auto
if [[ $# -gt 0 && "$1" != *:=* && "$1" != -* ]]; then
  ROBOT_ADDRESS="$1"
  shift
fi
readonly ROBOT_ADDRESS
for argument in "$@"; do
  case "$argument" in
    max_speed:=*|mouse_gain:=*|show_rviz:=*) ;;
    *) echo "Unsupported hardware launch argument: $argument" >&2; exit 2 ;;
  esac
done
if [[ ! -f "${PROJECT_ROOT}/ros_ws/install/setup.bash" ]]; then
  echo 'Build the ROS workspace first with scripts/build_ros_workspace.sh.' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/ros_ws/install/setup.bash"
bash "${PROJECT_ROOT}/.devcontainer/start-vnc.sh"
export DISPLAY=:1
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
exec ros2 launch hand_bringup hardware_mouse.launch.py robot_ip:="$ROBOT_ADDRESS" "$@"
