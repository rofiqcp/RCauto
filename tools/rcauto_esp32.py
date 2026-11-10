#!/usr/bin/env python3
"""RCauto serial protocol shared by ESP32 firmware, manual tools and ROS2."""

import argparse
import math
import time

import serial

STEERING_LIMIT = 900
THROTTLE_LIMIT = 1000
PERIOD_S = 0.05
DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
DEFAULT_TOPIC = "/rcauto/drive"


def control_frame(body: str) -> bytes:
    payload = body.encode("ascii")
    checksum = 0
    for value in payload:
        checksum ^= value
    return b"@" + payload + f"*{checksum:02X}\n".encode("ascii")


def frame(steering: int, throttle: int) -> bytes:
    if not -STEERING_LIMIT <= steering <= STEERING_LIMIT:
        raise ValueError("steering must be -900..900")
    if not -THROTTLE_LIMIT <= throttle <= THROTTLE_LIMIT:
        raise ValueError("throttle must be -1000..1000")
    return control_frame(f"RC,{steering},{throttle}")


def normalized(value: float, limit: float, units: int) -> int:
    if not math.isfinite(value):
        return 0
    return round(max(-1.0, min(1.0, value / limit)) * units)


class Esp32Link:
    def __init__(self, port: str, quiet: bool = False):
        self.connection = serial.Serial(
            port, 115200, timeout=0.01, write_timeout=0.4, exclusive=True
        )
        self.quiet = quiet

    def close(self) -> None:
        self.connection.close()

    def send(self, steering: int, throttle: int) -> list[str]:
        self.connection.write(frame(steering, throttle))
        return self.read_responses()

    def control(self, command: str) -> list[str]:
        self.connection.write(control_frame(command))
        return self.read_responses()

    def request(self, command: str) -> list[str]:
        self.connection.write((command + "\n").encode("ascii"))
        time.sleep(0.45 if command == "PWM" else 0.12)
        return self.read_responses()

    def read_responses(self) -> list[str]:
        results = []
        while self.connection.in_waiting:
            line = self.connection.readline().decode("utf-8", "replace").strip()
            if line:
                results.append(line)
                if not self.quiet or line.startswith(("ERR", "SAFE_STOP")):
                    print(line, flush=True)
        return results

    def stop(self, steering: int = 0) -> None:
        self.send(steering, 0)


def control_direct(link: Esp32Link, steering: int, throttle: int,
                   duration: float) -> None:
    until = time.monotonic() + duration
    try:
        while True:
            link.send(steering, throttle)
            remaining = until - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(PERIOD_S, remaining))
    finally:
        link.stop(steering)
        link.request("STATUS")


def control_ros(link: Esp32Link, topic: str, max_steering_rad: float,
                max_speed_mps: float) -> None:
    import os
    from pathlib import Path

    cyclone_uri = os.environ.get("CYCLONEDDS_URI", "")
    if cyclone_uri.startswith("file://") and not Path(cyclone_uri[7:]).is_file():
        os.environ.pop("CYCLONEDDS_URI")
        print("ROS: unavailable CycloneDDS configuration ignored", flush=True)

    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from rclpy.node import Node
    from std_msgs.msg import Bool, String

    class Bridge(Node):
        def __init__(self) -> None:
            super().__init__("rcauto_esp32_bridge")
            self.command = (0, 0)
            self.last_message = 0.0
            self.last_status_request = 0.0
            self.armed = False
            self.operator_enabled = False
            self.physical_button_mode = False
            self.last_enable_heartbeat = 0.0

            self.create_subscription(
                AckermannDriveStamped, topic, self.on_drive, 10)
            self.create_subscription(
                Bool, "/rcauto/operator/enable", self.on_enable, 10)
            self.armed_pub = self.create_publisher(
                Bool, "/rcauto/firmware/armed", 10)
            self.status_pub = self.create_publisher(
                String, "/rcauto/firmware/status", 10)
            self.button_pub = self.create_publisher(
                Bool, "/rcauto/operator/enable", 10)
            self.create_timer(PERIOD_S, self.transmit)

            # A newly opened serial bridge never inherits a prior motor arm.
            link.control("ARM,0")
            link.send(0, 0)
            self.get_logger().info(
                f"{topic} -> BTS7960, steering -900..900, speed -1000..1000")

        def on_drive(self, message: AckermannDriveStamped) -> None:
            steer = message.drive.steering_angle
            speed = message.drive.speed
            self.command = (
                normalized(steer, max_steering_rad, STEERING_LIMIT),
                normalized(speed, max_speed_mps, THROTTLE_LIMIT))
            self.last_message = time.monotonic()

        def on_enable(self, message: Bool) -> None:
            enabled = bool(message.data)
            if enabled:
                self.last_enable_heartbeat = time.monotonic()
            if enabled == self.operator_enabled:
                return
            self.operator_enabled = enabled
            if not enabled:
                self.physical_button_mode = False
                self.command = (0, 0)
                self.last_message = 0.0
            link.control("ARM,1" if enabled else "ARM,0")
            # Apply neutral once per transition, not on every heartbeat.
            link.send(0, 0)

        def handle_responses(self, lines: list[str]) -> None:
            for line in lines:
                if line.startswith("ARMED,"):
                    parts = line.split(",")
                    if len(parts) >= 3:
                        self.armed = parts[1] == "1"
                        self.armed_pub.publish(Bool(data=self.armed))
                        if parts[2] == "BUTTON":
                            # A physical START button updates the ROS controller.
                            self.command = (0, 0)
                            self.last_message = 0.0
                            self.operator_enabled = self.armed
                            self.physical_button_mode = self.armed
                            self.last_enable_heartbeat = time.monotonic()
                            link.send(0, 0)
                            self.button_pub.publish(Bool(data=self.armed))
                elif line.startswith("STATUS,"):
                    self.status_pub.publish(String(data=line))
                    for piece in line.split(","):
                        if piece.startswith("armed="):
                            self.armed = piece == "armed=1"
                            self.armed_pub.publish(Bool(data=self.armed))
                            break
                elif line.startswith("ESTOP,1"):
                    self.armed = False
                    self.armed_pub.publish(Bool(data=False))

        def transmit(self) -> None:
            try:
                now = time.monotonic()
                if self.operator_enabled and self.physical_button_mode:
                    if now - self.last_enable_heartbeat >= 0.25:
                        self.last_enable_heartbeat = now
                        self.button_pub.publish(Bool(data=True))
                elif self.operator_enabled and now - self.last_enable_heartbeat > 1.25:
                    self.on_enable(Bool(data=False))
                    self.button_pub.publish(Bool(data=False))
                fresh = now - self.last_message <= 0.25
                steering, throttle = self.command if fresh else (0, 0)
                self.handle_responses(link.send(steering, throttle))
                now = time.monotonic()
                if now - self.last_status_request >= 0.5:
                    link.connection.write(b"STATUS\n")
                    self.last_status_request = now
            except serial.SerialException as error:
                self.get_logger().error(f"ESP32 serial disconnected: {error}")
                rclpy.try_shutdown()

    rclpy.init()
    node = None
    try:
        node = Bridge()
        rclpy.spin(node)
    finally:
        try:
            link.control("ARM,0")
            link.stop()
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="RCauto ESP32 BTS7960: steer -900..900, speed -1000..1000")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--steer", type=int, default=0)
    parser.add_argument("--throttle", type=int, default=0)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--ros", action="store_true", help="ROS2 /rcauto/drive bridge")
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--max-steering-rad", type=float, default=0.35)
    parser.add_argument("--max-speed-mps", type=float, default=0.18)
    parser.add_argument("--status", action="store_true")
    parser.add_argument("--pwm", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()
    frame(args.steer, args.throttle)
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be positive")
    if not math.isfinite(args.max_steering_rad) or args.max_steering_rad <= 0:
        parser.error("--max-steering-rad must be positive")
    if not math.isfinite(args.max_speed_mps) or args.max_speed_mps <= 0:
        parser.error("--max-speed-mps must be positive")

    link = Esp32Link(args.port, quiet=args.quiet or args.ros)
    try:
        time.sleep(1.0)
        link.read_responses()
        if args.ros:
            control_ros(link, args.topic, args.max_steering_rad,
                        args.max_speed_mps)
        elif args.status:
            link.request("STATUS")
        elif args.pwm:
            link.request("PWM")
        else:
            control_direct(link, args.steer, args.throttle, args.duration)
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            link.stop()
            if not args.ros:
                link.control("ARM,0")
        finally:
            link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
