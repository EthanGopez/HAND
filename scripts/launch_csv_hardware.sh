#!/usr/bin/env bash
# Start the browser desktop and CSV hand-tracking control of the PHYSICAL Lite6.
#
# This connects to a real robot controller. Read the hardware section of
# ros_ws/src/hand_teleop/README.md and diagnostics/csv_teleop/README.md first.
# Unlike the mouse pad, this input has no deadman: the arm stops only because
# the tracking demo stopped writing rows. Keep the physical emergency stop in
# reach, and start the tracking demo yourself in a separate terminal.
set -eo pipefail

readonly PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ $# -lt 1 ]]; then
  echo "Usage: $(basename "$0") <lite6_controller_ipv4> [launch args...]" >&2
  echo 'The controller IP is required; there is no discovery for Lite6.' >&2
  exit 1
fi
readonly ROBOT_IP="$1"
shift

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

exec ros2 launch hand_bringup hardware_csv.launch.py \
  robot_ip:="${ROBOT_IP}" \
  csv_path:="${PROJECT_ROOT}/palm_center.csv" "$@"
