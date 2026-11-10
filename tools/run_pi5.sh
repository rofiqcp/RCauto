#!/usr/bin/env bash
set -Eeo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p "$ROOT/tmp" "$ROOT/log" "$ROOT/colcon"

# RCauto currently uses OpenCV/USB vision only. Keep Hailo integration source
# for future use, but never auto-start its inference service from this runner.
export RCAUTO_HAILO_ENABLE=false
HARDWARE="${RCAUTO_HW_OUTPUT:-false}"
if [[ "$HARDWARE" == "true" && "${RCAUTO_LIVE_ACK:-}" != "CONFIRM_WHEELS_UP_EXTERNAL_ESTOP" ]]; then
  echo "Hardware output denied: verify external E-stop and wheels off the ground." >&2
  exit 3
fi

export RCAUTO_DOMAIN_ID="${RCAUTO_DOMAIN_ID:-89}"
source "$ROOT/tools/activate_ros2_native.sh"
export ROS_LOG_DIR="$ROOT/log"
export TMPDIR="$ROOT/tmp"
export COLCON_HOME="$ROOT/colcon"

CPU_COUNT="$(nproc)"
MAX_CORES="$((CPU_COUNT / 2))"
(( MAX_CORES > 0 )) || MAX_CORES=1
BUILD_JOBS="${RCAUTO_BUILD_JOBS:-$MAX_CORES}"
if [[ ! "$BUILD_JOBS" =~ ^[1-9][0-9]*$ ]] || (( BUILD_JOBS > MAX_CORES )); then
  echo "RCAUTO_BUILD_JOBS must be between 1 and $MAX_CORES (50% of CPU cores)" >&2
  exit 2
fi
MEM_AVAILABLE_KIB="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
if (( BUILD_JOBS > 1 && MEM_AVAILABLE_KIB < 2500000 )); then
  BUILD_JOBS=1
fi
export MAKEFLAGS="-j$BUILD_JOBS -l$BUILD_JOBS"
export CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS"
CPU_SET="0-$((MAX_CORES - 1))"

if [[ "${RCAUTO_SKIP_BUILD:-false}" != "true" ]]; then
  printf 'Build rcauto_cpp: %s job(s), CPU affinity %s of %s cores, domain %s\n' \
    "$BUILD_JOBS" "$CPU_SET" "$CPU_COUNT" "$ROS_DOMAIN_ID"
  nice -n 10 ionice -c2 -n7 taskset -c "$CPU_SET" \
    colcon --log-base "$ROOT/log" build --executor sequential --parallel-workers 1 \
    --base-paths "$ROOT/src" --packages-select rcauto_cpp \
    --build-base "$ROOT/build" --install-base "$ROOT/install" --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
fi
source "$ROOT/install/setup.bash"

CAMERA="${RCAUTO_CAMERA_SOURCE:-usb}"
FOX_BIND="${RCAUTO_FOXGLOVE_BIND:-0.0.0.0}"
FOX_IP="$(tailscale ip -4 2>/dev/null | head -1 || true)"
FOX_IP="${FOX_IP:-10.41.18.54}"
MODE="${RCAUTO_MODE:-pid}"
LAUNCH_ARGS=(
  "simulation:=false"
  "camera_only:=$([[ "$HARDWARE" == "true" ]] && echo false || echo true)"
  "hardware_enabled:=$HARDWARE"
  "actuator_backend:=esp32"
  "esp32_port:=${RCAUTO_ESP32_PORT:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}"
  "camera_source:=$CAMERA"
  "lane_style:=${RCAUTO_LANE_STYLE:-road}"
  "foxglove_enabled:=true"
  "foxglove_port:=8767"
  "foxglove_address:=$FOX_BIND"
  "controller_mode:=$MODE"
)
[[ -z "${RCAUTO_LIVE_ACK:-}" ]] || LAUNCH_ARGS+=("live_ack:=$RCAUTO_LIVE_ACK")
[[ -z "${RCAUTO_MODEL_PATH:-}" ]] || LAUNCH_ARGS+=("model_path:=$RCAUTO_MODEL_PATH")

printf 'RCauto ROS %s | ws://%s:8767 | hardware=%s | camera=%s\n' \
  "$ROS_DOMAIN_ID" "$FOX_IP" "$HARDWARE" "$CAMERA"
printf 'Hailo inference disabled | USB camera + OpenCV active\n'
exec ros2 launch rcauto_cpp rcauto_cpp.launch.py "${LAUNCH_ARGS[@]}"
