"""Read-only Raspberry Pi 5 + Hailo + ROS readiness check. Writes no files."""

import importlib.util
import os
from pathlib import Path
import platform
import shutil
import subprocess


def check_module(name):
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


def main():
    root = Path(__file__).resolve().parents[1]
    print("PROJECT:", root)
    print("PLATFORM:", platform.platform())
    print("ARCH:", platform.machine())
    print("ROS_DISTRO:", os.environ.get("ROS_DISTRO", "not sourced"))
    for name in ("rclpy", "cv2", "numpy", "ackermann_msgs", "onnxruntime",
                 "lgpio", "hailo", "hailo_apps"):
        print(f"MODULE {name}:", "OK" if check_module(name) else "UNAVAILABLE")
    print("ONNX_MODEL:", (root / "src/models/line_policy_baseline.onnx").exists())
    print("GPIO_CHIPS:", ", ".join(str(p) for p in Path("/dev").glob("gpiochip*")))
    print("CAMERAS:", ", ".join(str(p) for p in Path("/dev").glob("video*")))
    for cmd in ("ros2", "colcon", "rpicam-hello", "hailortcli"):
        print(f"COMMAND {cmd}:", shutil.which(cmd) or "UNAVAILABLE")
    cli = shutil.which("hailortcli")
    if cli:
        try:
            p = subprocess.run([cli, "fw-control", "identify"], capture_output=True,
                               text=True, timeout=8)
            print("HAILO_DEVICE:", p.stdout.strip()[:500] if p.returncode == 0
                  else "NOT VERIFIED")
        except (subprocess.SubprocessError, OSError):
            print("HAILO_DEVICE: NOT VERIFIED")
    print("Note: this check does not arm ESC/servo or validate PWM timing.")


if __name__ == "__main__":
    main()
