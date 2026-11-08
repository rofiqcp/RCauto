#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT"
SOURCES="$ROOT/src"
ROS_DISTRO="${ROS_DISTRO:-$(find /opt/ros -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort -V | tail -n 1)}"
if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
  echo "ROS $ROS_DISTRO unavailable. Install the matching ROS distribution." >&2
  exit 2
fi
source "/opt/ros/$ROS_DISTRO/setup.bash"
mkdir -p "$WS/tmp" "$WS/log" "$WS/colcon"
export ROS_LOG_DIR="$WS/log" TMPDIR="$WS/tmp" COLCON_HOME="$WS/colcon"
export ROS_DOMAIN_ID="${RCAUTO_DOMAIN_ID:-88}"
MODE="${RCAUTO_MODE:-pid}"
STYLE="${RCAUTO_LANE_STYLE:-road}"
HARDWARE="${RCAUTO_HW_OUTPUT:-false}"
CAMERA="${RCAUTO_CAMERA_SOURCE:-ros}"
FOX_ADDR="${RCAUTO_FOXGLOVE_ADDRESS:-127.0.0.1}"
if [[ "$HARDWARE" == "true" && "${RCAUTO_LIVE_ACK:-}" != "CONFIRM_WHEELS_UP_EXTERNAL_ESTOP" ]]; then
  echo "LIVE ACTUATOR DENIED: physical kill switch & wheels-up verification required." >&2
  exit 3
fi
cd "$WS"
colcon --log-base "$WS/log" build --base-paths "$SOURCES" --packages-select rcauto_cpp \
  --build-base "$WS/build" --install-base "$WS/install" --symlink-install
source "$WS/install/setup.bash"
exec ros2 launch rcauto_cpp rcauto_cpp.launch.py simulation:=false \
  hardware_enabled:="$HARDWARE" live_ack:="${RCAUTO_LIVE_ACK:-}" \
  camera_source:="$CAMERA" lane_style:="$STYLE" \
  foxglove_enabled:=true foxglove_port:=8767 \
  foxglove_address:="$FOX_ADDR" controller_mode:="$MODE" \
  model_path:="$ROOT/src/models/line_policy_baseline.onnx"
