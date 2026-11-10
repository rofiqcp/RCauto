#!/usr/bin/env python3
"""Single-owner, hot-pluggable ESP32/BTS7960 ROS 2 bridge.

Physical button presses synchronize the ROS controller and Foxglove. In
preview mode, the bridge never sends a nonzero motor command or arms the ESP32.
"""

from __future__ import annotations

import glob
import math
import os
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, String
from serial import Serial, SerialException
from serial.tools import list_ports

from rcauto_esp32 import frame, control_frame, normalized

SUPPORTED_USB_UARTS = frozenset({
    (0x1A86, 0x7523), (0x1A86, 0x55D4), (0x10C4, 0xEA60),
    (0x303A, 0x1001), (0x303A, 0x0002), (0x0403, 0x6001),
})
TICK_S = 0.05
DRIVE_TIMEOUT_S = 0.25
OPERATOR_TIMEOUT_S = 1.25
STATUS_PERIOD_S = 0.5


def find_serial_port(preference: str) -> tuple[str, str]:
    if preference != "auto":
        if any(symbol in preference for symbol in "*?[]"):
            candidates = sorted(glob.glob(preference))
            return (candidates[0], "") if candidates else ("", "port USB belum tersambung")
        if os.path.exists(preference):
            return preference, ""
        return "", "port USB belum tersambung"
    candidates = sorted({
        port.device for port in list_ports.comports()
        if (port.vid, port.pid) in SUPPORTED_USB_UARTS
    })
    if len(candidates) == 1:
        return candidates[0], ""
    if len(candidates) > 1:
        return "", "beberapa adaptor USB; pilih port ESP32 secara spesifik"
    return "", "menunggu adaptor serial USB ESP32"


class Esp32Hotplug(Node):
    def __init__(self) -> None:
        super().__init__("rcauto_esp32_hotplug")
        self.declare_parameter("port", "auto")
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("max_steering_rad", 0.35)
        self.declare_parameter("max_speed_mps", 0.18)
        self.declare_parameter("max_motor_output_percent", 100)
        self.declare_parameter("max_steering_output_percent", 100)
        self.add_on_set_parameters_callback(self.validate_tuning)

        self.connection: Serial | None = None
        self.connected_port = ""
        self.rx = bytearray()
        self.state = ""
        self.command = (0, 0)
        self.last_drive = 0.0
        self.last_ui_heartbeat = 0.0
        self.last_status_request = 0.0
        self.last_physical_heartbeat = 0.0
        self.enabled = False
        self.physical_mode = False
        self.press_was_running = False
        self.firmware_armed = False

        self.status_pub = self.create_publisher(String, "/rcauto/esp32/status", 10)
        self.telemetry_pub = self.create_publisher(String, "/rcauto/firmware/status", 10)
        self.armed_pub = self.create_publisher(Bool, "/rcauto/firmware/armed", 10)
        self.enable_pub = self.create_publisher(Bool, "/rcauto/operator/enable", 10)
        self.create_subscription(Bool, "/rcauto/operator/enable", self.on_enable, 10)
        self.create_subscription(
            AckermannDriveStamped, "/rcauto/drive", self.on_drive, 10
        )
        self.create_timer(TICK_S, self.tick)
        self.get_logger().info("ESP32 USB hotplug active, initial motor DISARMED")

    def validate_tuning(self, parameters: list) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name in (
                "max_motor_output_percent", "max_steering_output_percent"
            ):
                if type(parameter.value) is not int or not 0 <= parameter.value <= 100:
                    return SetParametersResult(
                        successful=False, reason="Persentase ESP32 harus 0 sampai 100"
                    )
        return SetParametersResult(successful=True)

    @property
    def hardware_enabled(self) -> bool:
        return bool(self.get_parameter("hardware_enabled").value)

    def announce(self, message: str) -> None:
        if message != self.state:
            self.state = message
            self.get_logger().info(message)
        self.status_pub.publish(String(data=message))

    def publish_enable(self, value: bool) -> None:
        self.enable_pub.publish(Bool(data=value))

    def write(self, payload: bytes) -> None:
        if self.connection is None:
            return
        self.connection.write(payload)

    def control(self, command: str) -> None:
        self.write(control_frame(command))

    def disarm(self) -> None:
        if self.connection is not None:
            self.control("ARM,0")
            self.write(frame(0, 0))
        self.firmware_armed = False

    def disconnect(self, reason: str) -> None:
        if self.connection is not None:
            try:
                self.connection.close()
            except (OSError, SerialException):
                pass
        self.connection = None
        self.connected_port = ""
        self.rx.clear()
        self.command = (0, 0)
        self.last_drive = 0.0
        self.firmware_armed = False
        self.physical_mode = False
        if self.enabled:
            self.enabled = False
            self.publish_enable(False)
        self.announce("ESP32 DISCONNECTED | " + reason)

    def connect(self, port: str) -> None:
        serial = Serial(
            port=None, baudrate=115200, timeout=0, write_timeout=0.15, exclusive=True
        )
        serial.dtr = False
        serial.rts = False
        serial.port = port
        serial.open()
        self.connection = serial
        self.connected_port = port
        self.enabled = False
        self.physical_mode = False
        self.last_ui_heartbeat = 0.0
        self.last_status_request = 0.0
        self.command = (0, 0)
        self.disarm()
        self.publish_enable(False)
        self.announce("ESP32 CONNECTING | " + port + " | output motor nol")

    def on_enable(self, message: Bool) -> None:
        enabled = bool(message.data)
        if enabled:
            self.last_ui_heartbeat = time.monotonic()
        if enabled == self.enabled:
            return
        self.enabled = enabled
        if not enabled:
            self.physical_mode = False
            self.command = (0, 0)
            self.last_drive = 0.0
            self.disarm()
        elif self.hardware_enabled and self.connection is not None:
            self.control("ARM,1")
            self.write(frame(0, 0))
        else:
            # Preview START changes ROS state without applying motor PWM.
            self.disarm()

    def on_drive(self, message: AckermannDriveStamped) -> None:
        if not self.hardware_enabled:
            return
        steer = float(message.drive.steering_angle)
        speed = float(message.drive.speed)
        if not math.isfinite(steer) or not math.isfinite(speed):
            self.command = (0, 0)
            return
        self.command = (
            normalized(steer, float(self.get_parameter("max_steering_rad").value), 900),
            normalized(speed, float(self.get_parameter("max_speed_mps").value), 1000),
        )
        self.last_drive = time.monotonic()

    def parse_line(self, line: str) -> None:
        if line.startswith("ARMED,"):
            parts = line.split(",")
            if len(parts) >= 3:
                self.firmware_armed = parts[1] == "1"
                self.armed_pub.publish(Bool(data=self.firmware_armed))
                if parts[2] == "BUTTON" and not self.hardware_enabled:
                    # Preview mode is allowed to change ROS state but cannot
                    # retain an armed motor board, even after physical START.
                    self.disarm()
        elif line == "BUTTON,PRESSED":
            self.press_was_running = self.enabled
            if not self.enabled:
                self.enabled = True
                self.physical_mode = True
                self.last_ui_heartbeat = time.monotonic()
                self.last_physical_heartbeat = 0.0
                self.publish_enable(True)
                if not self.hardware_enabled:
                    self.disarm()
        elif line == "BUTTON,LONG":
            # A hold from the first START press must not also issue STOP.
            if self.press_was_running and self.enabled:
                self.enabled = False
                self.physical_mode = False
                self.command = (0, 0)
                self.last_drive = 0.0
                self.disarm()
                self.publish_enable(False)
            self.press_was_running = False
        elif line.startswith("STATUS,"):
            self.telemetry_pub.publish(String(data=line))
            fields = {}
            for item in line.split(",")[1:]:
                if item.count("=") == 1:
                    key, value = item.split("=", 1)
                    fields[key] = value
            self.firmware_armed = fields.get("armed") == "1"
            self.armed_pub.publish(Bool(data=self.firmware_armed))
            self.announce(
                "ESP32 CONNECTED | " + self.connected_port
                + " | " + ("ARMED" if self.firmware_armed else "DISARMED")
                + " | speed=" + fields.get("speed", "?")
                + " target=" + fields.get("target", "?")
            )
        elif line.startswith(("ERR,", "SAFE_STOP,")):
            self.get_logger().warn("Firmware: " + line)
        elif line.startswith("READY,"):
            self.disarm()
            if self.enabled:
                self.enabled = False
                self.physical_mode = False
                self.publish_enable(False)

    def receive(self) -> None:
        if self.connection is None:
            return
        count = min(self.connection.in_waiting, 2048)
        if count:
            self.rx.extend(self.connection.read(count))
        if len(self.rx) > 4096:
            self.rx.clear()
            self.get_logger().warn("ESP32 serial line overflow")
        while b"\n" in self.rx:
            line, _, remaining = self.rx.partition(b"\n")
            self.rx[:] = remaining
            self.parse_line(line.decode("ascii", "replace").strip())

    def tick(self) -> None:
        port, reason = find_serial_port(str(self.get_parameter("port").value))
        try:
            if self.connection is not None and port != self.connected_port:
                self.disconnect(reason or "USB dilepas")
            if not port:
                self.announce("ESP32 DISCONNECTED | " + reason)
                return
            if self.connection is None:
                try:
                    self.connect(port)
                except (OSError, SerialException) as error:
                    self.disconnect("port tidak tersedia: " + type(error).__name__)
                    return

            now = time.monotonic()
            self.receive()
            if self.enabled and self.physical_mode:
                if now - self.last_physical_heartbeat >= 0.25:
                    self.last_physical_heartbeat = now
                    self.publish_enable(True)
            elif self.enabled and now - self.last_ui_heartbeat > OPERATOR_TIMEOUT_S:
                self.enabled = False
                self.disarm()
                self.publish_enable(False)

            if self.hardware_enabled and self.enabled and (
                now - self.last_drive <= DRIVE_TIMEOUT_S
            ):
                motor_limit = int(self.get_parameter("max_motor_output_percent").value)
                steer_limit = int(self.get_parameter("max_steering_output_percent").value)
                steer, motor = self.command
                steer = round(steer * steer_limit / 100)
                motor = round(motor * motor_limit / 100)
            else:
                steer, motor = 0, 0
            self.write(frame(steer, motor))
            if now - self.last_status_request >= STATUS_PERIOD_S:
                self.write(b"STATUS\n")
                self.last_status_request = now
            self.receive()
        except (OSError, SerialException) as error:
            self.disconnect("USB hotplug: " + type(error).__name__)

    def destroy_node(self) -> bool:
        try:
            self.disarm()
        except (OSError, SerialException):
            pass
        if self.connection is not None:
            self.connection.close()
        self.connection = None
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Esp32Hotplug()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
