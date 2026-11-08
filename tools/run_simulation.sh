#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT"
SOURCES="$ROOT/src"
F1_SOURCE="$ROOT/referensi/github/f1tenth_simulator/f1tenth_gym_ros"
BACKEND="${RCAUTO_SIM_BACKEND:-gazebo}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
  echo "ROS 2 distro missing: $ROS_DISTRO" >&2
  exit 2
fi
source "/opt/ros/$ROS_DISTRO/setup.bash"
mkdir -p "$WS/tmp" "$WS/log" "$WS/colcon"
export ROS_LOG_DIR="$WS/log" TMPDIR="$WS/tmp" COLCON_HOME="$WS/colcon"
export ROS_DOMAIN_ID="${RCAUTO_DOMAIN_ID:-88}"
cd "$WS"
BASE_PATHS=("$SOURCES")
PACKAGES=(rcauto_cpp)
if [[ "$BACKEND" == "f1tenth" ]]; then
  if [[ ! -f "$F1_SOURCE/package.xml" ]]; then
    echo "F1TENTH source missing from $F1_SOURCE" >&2
    exit 2
  fi
  BASE_PATHS+=("$F1_SOURCE")
  PACKAGES+=(f1tenth_gym_ros)
fi
colcon --log-base "$WS/log" build --base-paths "${BASE_PATHS[@]}" \
  --packages-select "${PACKAGES[@]}" \
  --build-base "$WS/build" --install-base "$WS/install" --symlink-install
source "$WS/install/setup.bash"
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-$ROOT/.cache}"
if [[ "$BACKEND" == "gazebo" ]]; then
  if [[ -z "${DISPLAY:-}" ]]; then
    command -v xvfb-run >/dev/null || {
      echo "Headless Gazebo requires xvfb-run" >&2
      exit 2
    }
    export LIBGL_ALWAYS_SOFTWARE=1
    export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
    export __GLX_VENDOR_LIBRARY_NAME=mesa
    exec xvfb-run -a -s '-screen 0 1280x720x24' \
      ros2 launch rcauto_cpp rcauto_gazebo.launch.py headless:=true \
      foxglove_enabled:=true controller_mode:="${RCAUTO_MODE:-pid}" \
      model_path:="$ROOT/src/models/line_policy_baseline.onnx"
  fi
  exec ros2 launch rcauto_cpp rcauto_gazebo.launch.py headless:=false \
    foxglove_enabled:=true controller_mode:="${RCAUTO_MODE:-pid}" \
    model_path:="$ROOT/src/models/line_policy_baseline.onnx"
elif [[ "$BACKEND" == "f1tenth" ]]; then
  VENV_SITE="$ROOT/.venv/f1tenth/lib/python3.10/site-packages"
  if [[ ! -d "$VENV_SITE" ]]; then
    echo "F1TENTH dependencies missing: $VENV_SITE" >&2
    exit 2
  fi
  export PYTHONPATH="$ROOT/referensi/github/f1tenth_simulator/gym:$ROOT/referensi/github/f1tenth_simulator/f1tenth_gym:$VENV_SITE:${PYTHONPATH:-}"
  export NUMBA_CACHE_DIR="$ROOT/.cache/numba"
  mkdir -p "$NUMBA_CACHE_DIR"
  if [[ -z "${DISPLAY:-}" ]]; then
    command -v xvfb-run >/dev/null || {
      echo "Headless F1TENTH requires xvfb-run" >&2
      exit 2
    }
    export LIBGL_ALWAYS_SOFTWARE=1
    exec xvfb-run -a -s '-screen 0 1280x720x24' \
      ros2 launch rcauto_cpp rcauto_f1tenth.launch.py rviz_enabled:=false \
      foxglove_enabled:=true controller_mode:="${RCAUTO_MODE:-pid}" \
      model_path:="$ROOT/src/models/line_policy_baseline.onnx"
  fi
  exec ros2 launch rcauto_cpp rcauto_f1tenth.launch.py \
    rviz_enabled:="${RCAUTO_F1TENTH_RVIZ:-true}" foxglove_enabled:=true \
    controller_mode:="${RCAUTO_MODE:-pid}" \
    model_path:="$ROOT/src/models/line_policy_baseline.onnx"
elif [[ "$BACKEND" == "synthetic" ]]; then
  exec ros2 launch rcauto_cpp rcauto_cpp.launch.py simulation:=true \
    hardware_enabled:=false camera_source:=ros lane_style:=tape \
    foxglove_enabled:=true foxglove_port:=8767 \
    controller_mode:="${RCAUTO_MODE:-pid}" \
    model_path:="$ROOT/src/models/line_policy_baseline.onnx"
else
  echo "Unknown simulation backend: $BACKEND (gazebo|f1tenth|synthetic)" >&2
  exit 2
fi
