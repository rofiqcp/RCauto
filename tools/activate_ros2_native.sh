#!/usr/bin/env bash
# Source this file to use native ROS 2 Humble on Raspberry Pi OS.
RCAUTO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RCAUTO_NATIVE_PREFIX="$RCAUTO_ROOT/.ros2_native/envs/humble"
RCAUTO_MAMBA="$RCAUTO_ROOT/.ros2_native/bin/micromamba"
if [[ ! -x "$RCAUTO_MAMBA" || ! -x "$RCAUTO_NATIVE_PREFIX/bin/ros2" ]]; then
  echo "Native ROS 2 Humble is missing in $RCAUTO_NATIVE_PREFIX" >&2
  return 2
fi
export MAMBA_ROOT_PREFIX="$RCAUTO_ROOT/.ros2_native"
if [[ "${CONDA_PREFIX:-}" != "$RCAUTO_NATIVE_PREFIX" ]]; then
  eval "$("$RCAUTO_MAMBA" shell hook --shell bash)" || return
  micromamba activate "$RCAUTO_NATIVE_PREFIX" || return
fi
export ROS_DOMAIN_ID="${RCAUTO_DOMAIN_ID:-89}"
export ROS_LOCALHOST_ONLY=0
export PYTHONDONTWRITEBYTECODE=1
export CMAKE_BUILD_PARALLEL_LEVEL="${RCAUTO_BUILD_JOBS:-1}"
export MAKEFLAGS="-j${RCAUTO_BUILD_JOBS:-1}"
if [[ -f "$RCAUTO_ROOT/install/setup.bash" ]]; then
  source "$RCAUTO_ROOT/install/setup.bash"
fi
