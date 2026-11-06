#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/src/ros2_ws"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
  echo "ROS $ROS_DISTRO is unavailable on this machine." >&2
  exit 2
fi
source "/opt/ros/$ROS_DISTRO/setup.bash"
mkdir -p "$WS/tmp" "$WS/log"
export TMPDIR="$WS/tmp" ROS_LOG_DIR="$WS/log"
MODE="${RCAUTO_MODE:-pid}"
LANE_STYLE="${RCAUTO_LANE_STYLE:-road}"
HAILO="${RCAUTO_USE_HAILO:-true}"
LIVE="${RCAUTO_HW_OUTPUT:-false}"
FOXGLOVE_ADDR="${RCAUTO_FOXGLOVE_ADDRESS:-127.0.0.1}"
if [[ "$LIVE" == "true" && "${RCAUTO_LIVE_ACK:-}" != "WHEELS_UP_ESC_CALIBRATED" ]]; then
  echo "LIVE PWM blocked: first lift wheels, verify kill switch and ESC calibration." >&2
  exit 3
fi
CAMERA_SOURCE=usb
if [[ "$HAILO" == "true" ]]; then CAMERA_SOURCE=ros; fi
cd "$WS"
colcon --log-base "$WS/log" build --packages-select rcauto --build-base "$WS/build" --install-base "$WS/install" --symlink-install
source "$WS/install/setup.bash"
exec ros2 launch rcauto rcauto.launch.py simulation:=false camera_source:="$CAMERA_SOURCE" use_hailo:="$HAILO" hardware_enabled:="$LIVE" lane_style:="$LANE_STYLE" mode:="$MODE" model_path:="$ROOT/src/models/line_policy_baseline.onnx" foxglove_port:=8767 foxglove_address:="$FOXGLOVE_ADDR"
