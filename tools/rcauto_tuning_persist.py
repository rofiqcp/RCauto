#!/usr/bin/env python3
"""Persist accepted ROS 2 RCauto tuning updates to the canonical robot.yaml."""

from __future__ import annotations

import os
import stat
import tempfile
from pathlib import Path
from time import monotonic

import rclpy
import yaml
from rcl_interfaces.msg import ParameterEvent, ParameterType
from rclpy.node import Node
from rclpy.qos import qos_profile_parameter_events
from std_msgs.msg import String


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROBOT_YAML = PROJECT_ROOT / "src/rcauto_cpp/config/robot.yaml"

CONTROLLER_PARAMETERS = frozenset(
    (
        "controller.mode",
        "controller.pid.kp",
        "controller.pid.ki",
        "controller.pid.kd",
        "controller.speed_pid.kp",
        "controller.speed_pid.ki",
        "controller.speed_pid.kd",
        "controller.ai.output_gain",
        "controller.fuzzy.error_weight",
        "controller.fuzzy.rate_weight",
        "controller.fuzzy.speed_aggressiveness",
        "drive.max_steering_rad",
        "drive.steering_scale",
        "drive.follow_speed_mps",
        "drive.max_speed_mps",
        "drive.corner_speed_scale",
        "drive.cone_speed_scale",
        "drive.steering_deadband_norm",
        "drive.steering_slew_rad_s",
        "drive.accel_mps2",
        "drive.decel_mps2",
    )
)
VISION_PARAMETERS = frozenset(
    (
        "vision.target_distance_ratio",
        "vision.lookahead_weight",
        "vision.error_smoothing_alpha",
        "vision.min_saturation",
        "vision.min_brightness",
        "vision.cone_avoidance_offset_ratio",
    )
)
ALLOWED = {
    "/rcauto_cpp_controller": CONTROLLER_PARAMETERS,
    "/rcauto_cpp_vision": VISION_PARAMETERS,
    "/rcauto_esp32_hotplug": frozenset((
        "max_motor_output_percent",
        "max_steering_output_percent",
    )),
}


def decode_parameter(parameter):
    if parameter.value.type == ParameterType.PARAMETER_DOUBLE:
        return float(parameter.value.double_value)
    if parameter.value.type == ParameterType.PARAMETER_INTEGER:
        return int(parameter.value.integer_value)
    if parameter.value.type == ParameterType.PARAMETER_STRING:
        return str(parameter.value.string_value)
    return None


def update_tree(document: dict, updates: dict[tuple[str, str], object]) -> int:
    changed = 0
    for (node_name, name), value in sorted(updates.items()):
        section = document[node_name.removeprefix("/")]["ros__parameters"]
        path = name.split(".")
        branch = section
        for part in path[:-1]:
            branch = branch[part]
            if not isinstance(branch, dict):
                raise ValueError(f"YAML parameter path is not nested: {name}")
        leaf = path[-1]
        if leaf not in branch:
            raise ValueError(f"Missing YAML parameter: {node_name}.{name}")
        existing = branch[leaf]
        if type(existing) is not type(value):
            raise ValueError(
                f"YAML/ROS parameter type mismatch: {node_name}.{name}"
            )
        if existing != value:
            branch[leaf] = value
            changed += 1
    return changed


def save_updates(path: Path, updates: dict[tuple[str, str], object]) -> int:
    with path.open("r", encoding="utf-8") as handle:
        source = yaml.safe_load(handle)
    if not isinstance(source, dict):
        raise ValueError("robot.yaml must contain ROS 2 node mappings")
    changed = update_tree(source, updates)
    if not changed:
        return 0
    serialized = yaml.safe_dump(
        source, sort_keys=False, allow_unicode=True, default_flow_style=False
    )
    checked = yaml.safe_load(serialized)
    if checked != source:
        raise ValueError("YAML round-trip verification failed")
    file_mode = stat.S_IMODE(path.stat().st_mode)
    temp_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=".robot-yaml-",
            suffix=".tmp",
            delete=False,
        ) as handle:
            temp_name = handle.name
            handle.write(serialized)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temp_name, file_mode)
        os.replace(temp_name, path)
        temp_name = None
        directory_fd = os.open(str(path.parent), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temp_name is not None:
            Path(temp_name).unlink(missing_ok=True)
    return changed


class TuningPersistence(Node):
    def __init__(self) -> None:
        super().__init__("rcauto_tuning_persistence")
        self.pending: dict[tuple[str, str], object] = {}
        self.last_change = monotonic()
        self.status = self.create_publisher(
            String, "/rcauto/tuning/persistence_status", 10
        )
        self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self.on_parameter_event,
            qos_profile_parameter_events,
        )
        self.create_timer(0.4, self.flush)

    def announce(self, message: str) -> None:
        payload = String()
        payload.data = message
        self.status.publish(payload)

    def on_parameter_event(self, event: ParameterEvent) -> None:
        if event.node not in ALLOWED:
            return
        for parameter in [*event.new_parameters, *event.changed_parameters]:
            if parameter.name not in ALLOWED[event.node]:
                continue
            value = decode_parameter(parameter)
            if value is None:
                continue
            self.pending[(event.node, parameter.name)] = value
            self.last_change = monotonic()

    def flush(self) -> None:
        if not self.pending or monotonic() - self.last_change < 0.35:
            return
        updates = dict(self.pending)
        try:
            count = save_updates(ROBOT_YAML, updates)
        except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
            message = "YAML SAVE FAILED: " + str(error)
            self.get_logger().error(message)
            self.announce(message)
            return
        for key, value in updates.items():
            if self.pending.get(key) == value:
                self.pending.pop(key)
        if count:
            self.announce(
                f"SAVED robot.yaml | {count} parameter"
                f"{'s' if count != 1 else ''}"
            )


def main() -> None:
    rclpy.init()
    node = TuningPersistence()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
