#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/src/ros2_ws"
ROS_DISTRO="${ROS_DISTRO:-}"
if [[ -z "$ROS_DISTRO" ]]; then
  ROS_DISTRO=jazzy
  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then ROS_DISTRO=humble; fi
fi
if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
  echo "ROS 2 $ROS_DISTRO not installed." >&2
  exit 2
fi
source "/opt/ros/$ROS_DISTRO/setup.bash"
mkdir -p "$WS/tmp" "$WS/log"
export TMPDIR="$WS/tmp" ROS_LOG_DIR="$WS/log"
cd "$WS"
colcon --log-base "$WS/log" build --packages-select rcauto --build-base "$WS/build" --install-base "$WS/install" --symlink-install
source "$WS/install/setup.bash"
exec ros2 launch rcauto rcauto.launch.py simulation:=true camera_source:=ros use_hailo:=false hardware_enabled:=false lane_style:=tape foxglove_enabled:=true foxglove_port:=8767
