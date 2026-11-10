#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CV5="$ROOT/.opencv5/install"
BRIDGE="$ROOT/.opencv5/cv_bridge_install"
RC="$ROOT/.opencv5/rcauto_install"

for required in "$CV5/lib/cmake/opencv5/OpenCVConfig.cmake" \
                "$BRIDGE/cv_bridge/lib/libcv_bridge.so" \
                "$RC/rcauto_cpp/lib/rcauto_cpp/rcauto_vision"; do
  if [[ ! -e "$required" ]]; then
    printf 'Required OpenCV 5 component not built: %s\n' "$required" >&2
    exit 1
  fi
done

source /opt/ros/humble/setup.bash
source "$BRIDGE/setup.bash"
source "$RC/setup.bash"
export OpenCV_DIR="$CV5/lib/cmake/opencv5"
export LD_LIBRARY_PATH="$CV5/lib:$BRIDGE/cv_bridge/lib:${LD_LIBRARY_PATH:-}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID="${RCAUTO_DOMAIN_ID:-88}"
export ROS_LOG_DIR="$ROOT/log"

exec ros2 launch rcauto_cpp rcauto_cpp.launch.py \
  simulation:=false \
  camera_only:="${RCAUTO_CAMERA_ONLY:-true}" \
  hardware_enabled:="${RCAUTO_HARDWARE_ENABLED:-false}" \
  actuator_backend:=esp32 \
  esp32_port:="${RCAUTO_ESP32_PORT:-auto}" \
  camera_source:="${RCAUTO_CAMERA_SOURCE:-usb}" \
  lane_style:=road \
  controller_mode:="${RCAUTO_CONTROLLER_MODE:-pid}" \
  foxglove_enabled:=true \
  foxglove_port:=8767 \
  foxglove_address:="${RCAUTO_FOXGLOVE_BIND:-100.68.64.50}" \
  "$@"
