"""SIMULATION ONLY: synthetic yellow stripe and synthetic safe environment.

This node must never be launched in a real-vehicle configuration.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class SimNode(Node):
    def __init__(self):
        super().__init__("rcauto_simulator")
        self.camera = self.create_publisher(Image, "/rcauto/camera/image", 10)
        self.hazard = self.create_publisher(Bool, "/rcauto/hazard", 10)
        self.signal = self.create_publisher(String, "/rcauto/signal", 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().warning("SIMULATED sensor data only. Never connect to live actuators.")

    def tick(self):
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        img[:] = (60, 60, 60)
        cv2.line(img, (145, 239), (157, 120), (0, 220, 255), 12)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "sim_camera"
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        self.camera.publish(msg)
        self.hazard.publish(Bool(data=False))
        self.signal.publish(String(data="UNKNOWN"))


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
