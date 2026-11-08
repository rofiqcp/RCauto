#!/usr/bin/env bash
set -euo pipefail

# Print-ready geometry regenerates from the existing Gazebo/ROS 2 package.
# All temporary dependencies remain under /media/sirobo/HDD/RCauto.
CAD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE="$(dirname "$CAD_DIR")"
ROOT="$(dirname "$(dirname "$PACKAGE")")"
DEPS="$ROOT/tmp/rcauto_cad_deps"

cleanup() {
  rm -rf "$DEPS"
}
trap cleanup EXIT

mkdir -p "$DEPS"
python3 -m pip install --quiet --disable-pip-version-check --no-input --no-deps \
  --target "$DEPS" 'trimesh>=4.3,<5' 'manifold3d>=3,<4'

export PYTHONPATH="$DEPS${PYTHONPATH:+:$PYTHONPATH}"
python3 "$CAD_DIR/generate_stl.py"
python3 "$CAD_DIR/sync_sim.py"

python3 - "$CAD_DIR/assembly.json" "$PACKAGE" <<'PY'
import json
import pathlib
import sys
import trimesh

manifest=json.loads(pathlib.Path(sys.argv[1]).read_text())
root=pathlib.Path(sys.argv[2])
for name, part in manifest["parts"].items():
    mesh=trimesh.load(root/part["file"], force="mesh")
    if not mesh.is_watertight or not mesh.is_volume or mesh.volume <= 0:
        raise SystemExit("Non-solid STL: "+name)
    if mesh.extents[0] > 220.001 or mesh.extents[1] > 220.001:
        raise SystemExit("STL exceeds 220mm square bed: "+name)
print("Validated",len(manifest["parts"]),"solid printable STL meshes")
PY

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  xacro "$PACKAGE/description/rcauto_350x200.urdf.xacro" >/dev/null
fi
if command -v gz >/dev/null; then
  SDF_PATH="$PACKAGE/sim" gz sdf -k "$PACKAGE/sim/rcauto_track.sdf"
fi

if python3 -c 'import vtk' 2>/dev/null; then
  if command -v xvfb-run >/dev/null && [[ -z "${DISPLAY:-}" ]]; then
    xvfb-run -a -s '-screen 0 1500x1100x24' \
      python3 "$CAD_DIR/preview.py"
  else
    python3 "$CAD_DIR/preview.py"
  fi
fi
