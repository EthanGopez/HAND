#!/usr/bin/env bash
# Physics simulation plus drag-speed mouse input. No physical arm connection.
set -eo pipefail

readonly PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/ros_ws/install/setup.bash"
bash "${PROJECT_ROOT}/.devcontainer/start-vnc.sh"
export DISPLAY=:1
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
# All Gazebo transport stays inside this container, including the GUI client.
export GAZEBO_IP=127.0.0.1
# Use the bundled Gazebo models without waiting for the online model database.
export GAZEBO_MODEL_DATABASE_URI=''

exec ros2 launch hand_bringup gazebo.launch.py "$@"
