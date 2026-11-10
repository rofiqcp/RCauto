#!/usr/bin/env python3
"""Safe ESP32 USB hot-plug observer: neutral-only, never arms motor outputs."""

from __future__ import annotations

import glob
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from serial import Serial, SerialException
from serial.tools import list_ports

from rcauto_esp32 import frame


SUPPORTED_USB_UARTS = frozenset({
    (0x1A86, 0x7523),  # CH340 / CH341
    (0x1A86, 0x55D4),  # CH9102
    (0x10C4, 0xEA60),  # CP210x
    (0x303A, 0x1001),  # Espressif native USB CDC
    (0x303A, 0x0002),  # Espressif USB serial/JTAG
    (0x0403, 0x6001),  # FT232
})


def find_serial_port(preference: str) -> tuple[str, str]:
    if preference != "auto":
        if "*" in preference:
            paths = sorted(glob.glob(preference))
            return (paths[0], "") if paths else ("", "no matching port")
        return preference, ""

    matches = sorted(
        {port.device for port in list_ports.comports()
         if (port.vid, port.pid) in SUPPORTED_USB_UARTS}
    )
    if len(matches) == 1:
        return matches[0], ""
    if len(matches) > 1:
        return "", "multiple USB serial devices; select explicit port"
    return "", "no supported ESP32 USB serial adapter"



class Esp32Hotplug(Node):
    def __init__(self) -> None:
        super().__init__("rcauto_esp32_hotplug")
        self.declare_parameter(
            "port", "auto"
        )
        self.connection: Optional[Serial] = None
        self.state = ""
        self.rx = bytearray()
        self.status = self.create_publisher(String, "/rcauto/esp32/status", 10)
        self.timer = self.create_timer(0.5, self.poll)

    def announce(self, message: str) -> None:
        if self.state != message:
            self.state = message
            self.get_logger().info(message)
        payload = String()
        payload.data = self.state
        self.status.publish(payload)

    def disconnect(self) -> None:
        if self.connection is not None:
            try:
                self.connection.close()
            except (OSError, SerialException):
                pass
        self.connection = None
        self.rx.clear()

    def poll(self) -> None:
        preference = self.get_parameter("port").get_parameter_value().string_value
        port, reason = find_serial_port(preference)
        if self.connection is not None and self.connection.port != port:
            self.disconnect()
        if not port or not os.path.exists(port):
            self.disconnect()
            self.announce("ESP32 DISCONNECTED | " + (
                reason or "USB hot-plug scanning"))
            return
        if self.connection is None:
            try:
                conn = Serial(
                    port=None, baudrate=115200, timeout=0,
                    write_timeout=0.2, exclusive=True
                )
                conn.dtr = False
                conn.rts = False
                conn.port = port
                conn.open()
                self.connection = conn
                self.announce("ESP32 CONNECTING | motor stays DISARMED")
            except (OSError, SerialException) as error:
                self.disconnect()
                self.announce(
                    "ESP32 PORT BUSY | close the manual firmware/tools/tes.py tester"
                    if "lock" in str(error).lower() or "busy" in str(error).lower()
                    else "ESP32 UNAVAILABLE | " + type(error).__name__
                )
                return
        try:
            assert self.connection is not None
            # Never send ARM,1 or a nonzero drive command. Polling is
            # exclusively neutral, so restart and hot-plug are inert.
            self.connection.write(frame(0, 0))
            self.connection.write(b"STATUS\n")
            count = min(self.connection.in_waiting, 512)
            if count:
                self.rx.extend(self.connection.read(count))
            if len(self.rx) > 512:
                self.rx.clear()
            while b"\n" in self.rx:
                line, _, remaining = self.rx.partition(b"\n")
                self.rx[:] = remaining
                text = line.decode("ascii", "replace").strip()
                if not text.startswith("STATUS,"):
                    continue
                fields = dict(
                    part.split("=", 1)
                    for part in text.split(",")[1:]
                    if part.count("=") == 1
                )
                if not {"armed", "speed", "target"}.issubset(fields):
                    continue
                if fields["armed"] != "0":
                    # The physical button may arm independently; this
                    # observer reports that fact without creating motion.
                    self.announce(
                        "ESP32 CONNECTED | external ARM active; "
                        "USB monitor sends neutral only"
                    )
                else:
                    self.announce(
                        "ESP32 CONNECTED | DISARMED | "
                        f"speed={fields['speed']} target={fields['target']}"
                    )
            if self.state.startswith("ESP32 CONNECTING"):
                self.announce("ESP32 CONNECTING | awaiting firmware STATUS")
        except (OSError, SerialException):
            self.disconnect()
            self.announce("ESP32 DISCONNECTED | waiting for USB reconnect")

    def destroy_node(self) -> bool:
        self.disconnect()
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
