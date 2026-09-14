#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y --no-install-recommends \
  dbus-x11 \
  git \
  novnc \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-venv \
  python3-vcstool \
  tigervnc-standalone-server \
  xfce4 \
  xfce4-goodies

if ! grep -qxF 'source /opt/ros/humble/setup.bash' /root/.bashrc; then
  echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
fi

if [[ -f /workspace/ros_ws/install/setup.bash ]] && \
   ! grep -qxF 'source /workspace/ros_ws/install/setup.bash' /root/.bashrc; then
  echo 'source /workspace/ros_ws/install/setup.bash' >> /root/.bashrc
fi

echo "HAND development container is ready."
echo "Next: bash /workspace/scripts/bootstrap_ros_workspace.sh"

