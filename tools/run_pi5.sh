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
# The host may have a stale CycloneDDS config path; use the installed
# Fast DDS implementation for this isolated camera-only process.
export RMW_IMPLEMENTATION="${RCAUTO_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ "$RMW_IMPLEMENTATION" == "rmw_fastrtps_cpp" ]]; then
  unset CYCLONEDDS_URI
fi
mkdir -p "$WS/tmp" "$WS/log" "$WS/colcon"
export ROS_LOG_DIR="$WS/log" TMPDIR="$WS/tmp" COLCON_HOME="$WS/colcon"
export ROS_DOMAIN_ID="${RCAUTO_DOMAIN_ID:-88}"
# A Foxglove-saved mode survives restart. An explicit RCAUTO_MODE
# environment value still overrides it for intentional one-time use.
if [[ -n "${RCAUTO_MODE:-}" ]]; then
  MODE="$RCAUTO_MODE"
else
  MODE="$(python3 - "$ROOT/src/rcauto_cpp/config/robot.yaml" <<'PYMODE'
import sys,yaml
with open(sys.argv[1],encoding="utf-8") as f:
    config=yaml.safe_load(f)
print(config["rcauto_cpp_controller"]["ros__parameters"]["controller"]["mode"])
PYMODE
)"
fi
STYLE="${RCAUTO_LANE_STYLE:-road}"
HARDWARE="${RCAUTO_HW_OUTPUT:-false}"
CAMERA="${RCAUTO_CAMERA_SOURCE:-ros}"
FOX_ADDR="${RCAUTO_FOXGLOVE_ADDRESS:-127.0.0.1}"
if [[ "$HARDWARE" == "true" && "${RCAUTO_LIVE_ACK:-}" != "CONFIRM_WHEELS_UP_EXTERNAL_ESTOP" ]]; then
  echo "LIVE ACTUATOR DENIED: physical kill switch & wheels-up verification required." >&2
  exit 3
fi
cd "$WS"
if [[ "${RCAUTO_SKIP_BUILD:-false}" != "true" ]]; then
  export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS="-j1 -l1"
  colcon --log-base "$WS/log" build --executor sequential --parallel-workers 1 \
    --base-paths "$SOURCES" --packages-select rcauto_cpp \
    --build-base "$WS/build" --install-base "$WS/install" --symlink-install
fi
source "$WS/install/setup.bash"
LAUNCH_ARGS=(
  simulation:=false
  camera_only:="${RCAUTO_CAMERA_ONLY:-false}"
  hardware_enabled:="$HARDWARE"
  camera_source:="$CAMERA"
  lane_style:="$STYLE"
  foxglove_enabled:=true
  foxglove_port:=8767
  foxglove_address:="$FOX_ADDR"
  controller_mode:="$MODE"
)
if [[ -n "${RCAUTO_AI_MODEL:-}" ]]; then
  LAUNCH_ARGS+=("model_path:=$RCAUTO_AI_MODEL")
fi
if [[ -n "${RCAUTO_LIVE_ACK:-}" ]]; then
  LAUNCH_ARGS+=("live_ack:=$RCAUTO_LIVE_ACK")
fi
exec ros2 launch rcauto_cpp rcauto_cpp.launch.py "${LAUNCH_ARGS[@]}"
