"""Raspberry Pi 5 BCM GPIO RC PWM output using lgpio, disabled by default.

Never enable live output until servo/ESC calibrated with wheels lifted.
lgpio.tx_servo() pulse widths are in microseconds.
"""

import math
import time

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rcauto.core import clamp


class ActuatorNode(Node):
    def __init__(self):
        super().__init__("rcauto_actuator")
        for name, value in [
            ("hardware_enabled", False), ("gpiochip", 0),
            ("steering_gpio_bcm", 18), ("esc_gpio_bcm", 19),
            ("steering_center_us", 1500), ("steering_min_us", 1100), ("steering_max_us", 1900),
            ("esc_neutral_us", 1500), ("esc_max_forward_us", 1600),
            ("max_steering_rad", 0.35), ("max_speed_mps", 0.18),
            ("command_timeout_s", 0.25),
        ]:
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.enabled = bool(p("hardware_enabled"))
        self.gpiochip = int(p("gpiochip"))
        self.steering_gpio = int(p("steering_gpio_bcm"))
        self.esc_gpio = int(p("esc_gpio_bcm"))
        self.steering_center = int(p("steering_center_us"))
        self.steering_min = int(p("steering_min_us"))
        self.steering_max = int(p("steering_max_us"))
        self.esc_neutral = int(p("esc_neutral_us"))
        self.esc_forward = int(p("esc_max_forward_us"))
        self.max_steering = float(p("max_steering_rad"))
        self.max_speed = float(p("max_speed_mps"))
        self.timeout_s = float(p("command_timeout_s"))
        if self.max_speed <= 0 or self.max_steering <= 0 or self.timeout_s <= 0:
            raise ValueError("Speed, steering and watchdog limits must be positive")
        if self.steering_gpio == self.esc_gpio:
            raise ValueError("Steering and ESC GPIO cannot overlap")
        if not (900 <= self.steering_min < self.steering_center < self.steering_max <= 2100):
            raise ValueError("Invalid servo limits")
        if not (1000 <= self.esc_neutral < self.esc_forward <= 1900):
            raise ValueError("Invalid ESC neutral/forward limits")
        self._chip_handle = None
        self.gpio = None
        self.last_command = -math.inf
        self.last_speed = 0
        self.last_steering = 0
        self.warned = False
        if self.enabled:
            import lgpio
            self.gpio = lgpio
            self._chip_handle = lgpio.gpiochip_open(self.gpiochip)
            try:
                for pin in (self.steering_gpio, self.esc_gpio):
                    lgpio.gpio_claim_output(self._chip_handle, pin, 0)
                self._write_pulses(self.steering_center, self.esc_neutral)
            except BaseException:
                lgpio.gpiochip_close(self._chip_handle)
                raise
            self.get_logger().warning(
                f"LIVE PWM ENABLED: steer GPIO{self.steering_gpio}, ESC GPIO{self.esc_gpio}")
        else:
            self.get_logger().warning("DRY RUN: no GPIO output; hardware_enabled=false")
        self.create_subscription(AckermannDriveStamped, "/rcauto/drive", self.on_drive, 10)
        self.output = self.create_publisher(String, "/rcauto/actuator_status", 10)
        self.create_timer(0.05, self.tick)

    def _write_pulses(self, steer_us, esc_us):
        if self._chip_handle is not None:
            self.gpio.tx_servo(self._chip_handle, self.steering_gpio, int(steer_us), 50)
            self.gpio.tx_servo(self._chip_handle, self.esc_gpio, int(esc_us), 50)

    def on_drive(self, msg):
        speed = float(msg.drive.speed)
        steer = float(msg.drive.steering_angle)
        if not math.isfinite(speed) or not math.isfinite(steer):
            self.last_speed = 0
            self.last_steering = 0
            return
        self.last_speed = clamp(speed, 0, self.max_speed)
        self.last_steering = clamp(steer, -self.max_steering, self.max_steering)
        self.last_command = time.monotonic()

    def tick(self):
        fresh = (time.monotonic() - self.last_command) < self.timeout_s
        speed = self.last_speed if fresh else 0.0
        steer = self.last_steering if fresh else 0.0
        offset = (self.steering_max - self.steering_center) if steer >= 0 else (self.steering_center - self.steering_min)
        steer_us = round(self.steering_center + steer / self.max_steering * offset)
        esc_us = round(self.esc_neutral + speed / self.max_speed * (self.esc_forward - self.esc_neutral))
        self._write_pulses(steer_us, esc_us)
        msg = String()
        msg.data = f"{'LIVE' if self.enabled else 'DRY'} servo={steer_us}us esc={esc_us}us watchdog={'OK' if fresh else 'STOP'}"
        self.output.publish(msg)

    def destroy_node(self):
        if self._chip_handle is not None:
            self._write_pulses(self.steering_center, self.esc_neutral)
            self.gpio.gpiochip_close(self._chip_handle)
            self._chip_handle = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
