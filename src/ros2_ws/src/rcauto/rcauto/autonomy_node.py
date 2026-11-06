"""ROS 2 central controller: vision watchdog, safety FSM and three steering modes."""

import math
import time

from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
import rclpy
from std_msgs.msg import Bool, Float32, String

from rcauto.core import AIController, DriveState, FuzzyController, ModeUnavailable, Observation, PIDController, SafetyFSM


class AutonomyNode(Node):
    def __init__(self):
        super().__init__("rcauto_autonomy")
        self.declare_parameter("mode", "pid")
        self.declare_parameter("model_path", "")
        self.declare_parameter("max_steering_rad", 0.35)
        self.declare_parameter("max_speed_mps", 0.18)
        self.declare_parameter("watchdog_s", 0.6)
        self.mode = self.get_parameter("mode").value.lower()
        self.model_path = self.get_parameter("model_path").value
        self.max_steering = float(self.get_parameter("max_steering_rad").value)
        self.max_speed = float(self.get_parameter("max_speed_mps").value)
        self.fsm = SafetyFSM(max_sensor_age=float(self.get_parameter("watchdog_s").value))
        self.controllers = {"pid": PIDController(), "fuzzy": FuzzyController()}
        self.ai_error = ""
        self._load_ai()
        self.armed = False
        self.obs = Observation()
        self.stamps = {"lane": -math.inf, "confidence": -math.inf, "intersection": -math.inf,
                       "hazard": -math.inf, "signal": -math.inf}
        self.prev_tick = time.monotonic()
        self.last_state = None
        self.create_subscription(Float32, "/rcauto/lane_error", self.on_lane, 10)
        self.create_subscription(Float32, "/rcauto/lane_confidence", self.on_confidence, 10)
        self.create_subscription(Bool, "/rcauto/hazard", self.on_hazard, 10)
        self.create_subscription(Bool, "/rcauto/intersection", self.on_intersection, 10)
        self.create_subscription(String, "/rcauto/signal", self.on_signal, 10)
        self.create_subscription(Bool, "/rcauto/operator/enable", self.on_enable, 10)
        self.create_subscription(Bool, "/rcauto/operator/estop", self.on_estop, 10)
        self.create_subscription(Bool, "/rcauto/operator/reset_estop", self.on_reset, 10)
        self.create_subscription(String, "/rcauto/operator/mode", self.on_mode, 10)
        self.drive = self.create_publisher(AckermannDriveStamped, "/rcauto/drive", 10)
        self.status = self.create_publisher(String, "/rcauto/status", 10)
        self.create_timer(0.05, self.tick)

    def _load_ai(self):
        try:
            self.controllers["ai"] = AIController(self.model_path)
            self.ai_error = ""
        except ModeUnavailable as exc:
            self.ai_error = str(exc)
            self.controllers.pop("ai", None)

    def on_lane(self, msg):
        self.obs.lane_error = float(msg.data)
        self.stamps["lane"] = time.monotonic()

    def on_confidence(self, msg):
        self.obs.lane_confidence = float(msg.data)
        self.stamps["confidence"] = time.monotonic()

    def on_hazard(self, msg):
        self.obs.hazard = bool(msg.data)
        self.stamps["hazard"] = time.monotonic()

    def on_intersection(self, msg):
        self.obs.intersection = bool(msg.data)
        self.stamps["intersection"] = time.monotonic()

    def on_signal(self, msg):
        self.obs.signal = msg.data.upper()
        self.stamps["signal"] = time.monotonic()

    def on_enable(self, msg):
        self.armed = bool(msg.data)
        if not self.armed:
            self._reset_controllers()

    def on_estop(self, msg):
        if msg.data:
            self.fsm.estop_latched = True
            self.armed = False

    def on_reset(self, msg):
        if msg.data and not self.armed:
            self.fsm.reset_estop()

    def on_mode(self, msg):
        candidate = msg.data.strip().lower()
        if candidate in self.controllers and not self.armed:
            self.mode = candidate
            self._reset_controllers()
        else:
            self.get_logger().warning("Disarm before mode change; AI needs a valid model")

    def _reset_controllers(self):
        for c in self.controllers.values():
            c.reset()

    def tick(self):
        now = time.monotonic()
        dt = max(0.001, min(now - self.prev_tick, 0.25))
        self.prev_tick = now
        self.obs.lane_age = max(now - self.stamps["lane"],
                                now - self.stamps["confidence"],
                                now - self.stamps["intersection"])
        self.obs.hazard_age = now - self.stamps["hazard"]
        self.obs.signal_age = now - self.stamps["signal"]
        state, speed = self.fsm.evaluate(self.obs, now, self.armed)
        steer = 0.0
        if speed > 0:
            controller = self.controllers.get(self.mode)
            if controller is None:
                state, speed = DriveState.SENSOR_STOP, 0.0
            else:
                try:
                    steer = controller.step(self.obs.lane_error, dt)
                except (ValueError, ModeUnavailable, RuntimeError, FloatingPointError) as exc:
                    self.get_logger().error("Steering controller failed: " + str(exc))
                    self.armed = False
                    state, speed = DriveState.SENSOR_STOP, 0.0
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.speed = float(min(speed, self.max_speed)) if speed > 0 else 0.0
        msg.drive.steering_angle = float(steer * self.max_steering)
        self.drive.publish(msg)
        status = String()
        status.data = f"{state.value} | mode={self.mode} | armed={self.armed} | error={self.obs.lane_error:.3f} | hazard={self.obs.hazard}"
        self.status.publish(status)
        if state != self.last_state:
            self.get_logger().info(status.data)
            self.last_state = state


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
