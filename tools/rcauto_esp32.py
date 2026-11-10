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


def main() -> int:
    parser = argparse.ArgumentParser(
        description="RCauto ESP32 BTS7960: steer -900..900, speed -1000..1000")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--steer", type=int, default=0)
    parser.add_argument("--throttle", type=int, default=0)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--status", action="store_true")
    parser.add_argument("--pwm", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()
    frame(args.steer, args.throttle)
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be positive")

    link = Esp32Link(args.port, quiet=args.quiet)
    try:
        time.sleep(1.0)
        link.read_responses()
        if args.status:
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
            link.control("ARM,0")
        finally:
            link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
