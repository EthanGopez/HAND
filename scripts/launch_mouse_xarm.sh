#!/usr/bin/env bash
# Start the browser-accessible desktop and the mouse-controlled fake xArm.
set -eo pipefail

readonly PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/ros_ws/install/setup.bash"
bash "${PROJECT_ROOT}/.devcontainer/start-vnc.sh"
export DISPLAY=:1
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1

# The launch file explicitly selects fake hardware and never supplies a robot IP.
exec ros2 launch hand_bringup sim.launch.py input_mode:=mouse max_speed:=0.25 "$@"
