"""Bridge Hailo GStreamer detection callback into ROS 2 Image, hazard and signal.

Requires the official hailo-rpi5-examples + hailo_apps Python runtime on actual Pi.
Use with vision source:=ros so only Hailo holds the camera handle.
COCO traffic_light labels are NOT signal states; color is estimated from a boxed crop.
Unknown/conflicting signals are UNKNOWN and the vehicle must remain stopped.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


def classify_signal_crop(bgr):
    """Return red, yellow, green or UNKNOWN when color dominance is insufficient."""
    if bgr.size == 0 or min(bgr.shape[:2]) < 5:
        return "UNKNOWN"
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    bright = hsv[:, :, 2] >= 95
    sat = hsv[:, :, 1] >= 110
    colors = {
        "RED": (((hsv[:, :, 0] <= 10) | (hsv[:, :, 0] >= 170)) & bright & sat),
        "YELLOW": ((hsv[:, :, 0] >= 18) & (hsv[:, :, 0] <= 37) & bright & sat),
        "GREEN": ((hsv[:, :, 0] >= 40) & (hsv[:, :, 0] <= 88) & bright & sat),
    }
    counts = sorted(((int(np.count_nonzero(v)), k) for k, v in colors.items()), reverse=True)
    total = bgr.shape[0] * bgr.shape[1]
    if counts[0][0] < max(4, int(total * 0.06)):
        return "UNKNOWN"
    if counts[0][0] < max(3, counts[1][0] * 2):
        return "UNKNOWN"
    return counts[0][1]


class HailoBridge(Node):
    def __init__(self):
        super().__init__("rcauto_hailo_bridge")
        self.pub_camera = self.create_publisher(Image, "/rcauto/camera/image", 10)
        self.pub_hazard = self.create_publisher(Bool, "/rcauto/hazard", 10)
        self.pub_signal = self.create_publisher(String, "/rcauto/signal", 10)
        self.get_logger().info("Hailo camera + detection bridge initialized")

    def publish_detections(self, frame_rgb, detections):
        h, w = frame_rgb.shape[:2]
        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        hazard = False
        lights = []
        for label, score, x1, y1, x2, y2 in detections:
            if score < 0.40:
                continue
            xa = max(0, min(w, int(x1 * w)))
            xb = max(0, min(w, int(x2 * w)))
            ya = max(0, min(h, int(y1 * h)))
            yb = max(0, min(h, int(y2 * h)))
            if label.lower() in ("person", "car", "truck", "bus", "motorcycle", "bicycle"):
                # Conservative image-space collision corridor, not metric distance.
                if xb > 0.23 * w and xa < 0.77 * w and yb >= 0.42 * h:
                    hazard = True
            elif label.lower().replace(" ", "_") == "traffic_light":
                lights.append(classify_signal_crop(frame[ya:yb, xa:xb]))
        signal = "UNKNOWN"
        if len(lights) == 1:
            signal = lights[0]
        elif len(lights) > 1:
            if len(set(lights)) == 1:
                signal = lights[0]
            # Do not accept green if there are multiple conflicting lights.
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height, msg.width = h, w
        msg.encoding = "rgb8"
        msg.step = w * 3
        msg.data = frame_rgb.tobytes()
        self.pub_camera.publish(msg)
        self.pub_hazard.publish(Bool(data=hazard))
        self.pub_signal.publish(String(data=signal))


def main(args=None):
    import hailo
    from gi.repository import Gst
    from hailo_apps.hailo_app_python.core.common.buffer_utils import get_caps_from_pad, get_numpy_from_buffer
    from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
    from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import GStreamerDetectionApp

    rclpy.init(args=args)
    node = HailoBridge()
    class CallbackData(app_callback_class):
        pass

    cb_data = CallbackData()
    cb_data.use_frame = True

    def callback(pad, info, user_data):
        buf = info.get_buffer()
        if buf is None:
            return Gst.PadProbeReturn.OK
        pixel_format, width, height = get_caps_from_pad(pad)
        if pixel_format is None or width is None or height is None:
            return Gst.PadProbeReturn.OK
        raw = get_numpy_from_buffer(buf, pixel_format, width, height)
        if raw is None:
            return Gst.PadProbeReturn.OK
        frame = np.asarray(raw).copy()
        if frame.ndim != 3 or frame.shape[2] != 3:
            return Gst.PadProbeReturn.OK
        detections = []
        roi = hailo.get_roi_from_buffer(buf)
        for detection in roi.get_objects_typed(hailo.HAILO_DETECTION):
            box = detection.get_bbox()
            detections.append((detection.get_label(), float(detection.get_confidence()),
                               box.xmin(), box.ymin(), box.xmin() + box.width(),
                               box.ymin() + box.height()))
        node.publish_detections(frame, detections)
        return Gst.PadProbeReturn.OK

    try:
        GStreamerDetectionApp(callback, cb_data).run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
