#!/usr/bin/env bash
# Start the browser-accessible desktop and the fake xArm driven by the palm CSV
# that vision_demos/hand_tracking_demo.py writes.
#
# The tracking demo is a separate program; start it yourself, in its own
# terminal, before or after this script. With no hand in frame the demo writes
# no rows and the arm correctly stands still.
set -eo pipefail

readonly PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f "${PROJECT_ROOT}/ros_ws/install/setup.bash" ]]; then
  echo 'The ROS workspace has not been built. Run build_ros_workspace.sh first.' >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/ros_ws/install/setup.bash"
bash "${PROJECT_ROOT}/.devcontainer/start-vnc.sh"
export DISPLAY=:1
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1

# The launch file explicitly selects fake hardware and never supplies a robot IP.
# An absolute CSV path keeps the node independent of its working directory.
exec ros2 launch hand_bringup sim.launch.py input_mode:=csv \
  csv_path:="${PROJECT_ROOT}/palm_center.csv" "$@"
