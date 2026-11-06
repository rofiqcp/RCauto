"""Low-cost CPU line perception. A Hailo node supplies object and signal detections."""

import cv2
import numpy as np
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from rcauto.road_lane import process_road


def process_lane(frame, line_color="yellow"):
    """Central tape-track follower. Returns signed offset, confidence, crossing and debug frame."""
    h, w = frame.shape[:2]
    y0 = int(h * 0.60)
    roi = frame[y0:h, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    if line_color == "black":
        mask = cv2.inRange(hsv, (0, 0, 0), (179, 255, 85))
    elif line_color == "white":
        mask = cv2.inRange(hsv, (0, 0, 150), (179, 75, 255))
    else:
        mask = cv2.inRange(hsv, (15, 70, 80), (42, 255, 255))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # Use only the near-field slice for steering, avoiding distant curves.
    near = mask[int(mask.shape[0] * 0.55):, :]
    count = np.count_nonzero(near)
    min_pixels = max(30, int(near.size * 0.002))
    error, confidence = 0.0, 0.0
    if count >= min_pixels:
        xs = np.where(near > 0)[1]
        # Median tolerates highlights and isolated false positives.
        center = float(np.median(xs))
        error = float(np.clip((center - w * 0.5) / (w * 0.5), -1.0, 1.0))
        min_span = max(6, int(0.018 * w))
        spread = float(np.percentile(xs, 90) - np.percentile(xs, 10))
        plausible = min_span <= spread <= 0.55 * w
        confidence = 0.75 if plausible else 0.25
    # A wide horizontal line in a mid-depth band marks stop/intersection.
    middle = mask[int(mask.shape[0] * 0.15):int(mask.shape[0] * 0.55), :]
    broad_rows = (np.count_nonzero(middle, axis=1) > int(w * 0.65))
    intersection = bool(np.count_nonzero(broad_rows) >= max(2, int(middle.shape[0] * 0.12)))
    annotated = frame.copy()
    cv2.line(annotated, (w // 2, y0), (w // 2, h), (200, 200, 200), 1)
    cv2.putText(annotated, f"offset={error:.2f} conf={confidence:.2f} intersection={intersection}",
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
    return error, confidence, intersection, annotated


class VisionNode(Node):
    def __init__(self):
        super().__init__("rcauto_vision")
        self.declare_parameter("source", "usb")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("line_color", "yellow")
        self.declare_parameter("lane_style", "tape")
        self.declare_parameter("fps", 15.0)
        self.source = self.get_parameter("source").value
        self.line_color = self.get_parameter("line_color").value
        self.lane_style = self.get_parameter("lane_style").value
        if self.lane_style not in ("road", "tape"):
            raise ValueError("lane_style must be road or tape")
        fps = float(self.get_parameter("fps").value)
        self.pub_err = self.create_publisher(Float32, "/rcauto/lane_error", 10)
        self.pub_conf = self.create_publisher(Float32, "/rcauto/lane_confidence", 10)
        self.pub_cross = self.create_publisher(Bool, "/rcauto/intersection", 10)
        self.pub_image = self.create_publisher(Image, "/rcauto/camera/annotated", 10)
        self.camera = None
        if self.source == "usb":
            self.camera = cv2.VideoCapture(int(self.get_parameter("camera_index").value))
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if not self.camera.isOpened():
                self.get_logger().error("Camera not available; stale sensor watchdog will stop car")
            self.create_timer(1.0 / fps, self._read_camera)
        elif self.source == "ros":
            self.create_subscription(Image, "/rcauto/camera/image", self._on_image, 10)
        else:
            raise ValueError("source must be usb or ros")

    def _read_camera(self):
        if self.camera is None or not self.camera.isOpened():
            return
        ok, frame = self.camera.read()
        if ok:
            self._analyze(frame)

    def _on_image(self, msg):
        try:
            if msg.encoding not in ("bgr8", "rgb8") or msg.step < msg.width * 3:
                return
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            frame = frame[:, :msg.width * 3].reshape(msg.height, msg.width, 3).copy()
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self._analyze(frame)
        except (ValueError, cv2.error) as exc:
            self.get_logger().warning("Invalid camera image: " + str(exc))

    def _analyze(self, frame):
        if self.lane_style == "road":
            error, confidence, crossing, debug = process_road(frame)
        else:
            error, confidence, crossing, debug = process_lane(frame, self.line_color)
        self.pub_err.publish(Float32(data=float(error)))
        self.pub_conf.publish(Float32(data=float(confidence)))
        self.pub_cross.publish(Bool(data=crossing))
        out = Image()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "camera_link"
        out.height, out.width = debug.shape[:2]
        out.encoding = "bgr8"
        out.is_bigendian = 0
        out.step = debug.shape[1] * 3
        out.data = debug.tobytes()
        self.pub_image.publish(out)

    def destroy_node(self):
        if self.camera is not None:
            self.camera.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
