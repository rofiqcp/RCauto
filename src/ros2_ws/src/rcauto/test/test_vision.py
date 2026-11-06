import cv2
import numpy as np
from rcauto.vision_node import process_lane
from rcauto.hailo_bridge_node import classify_signal_crop


def test_stripe_offset():
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.line(img, (180, 239), (180, 140), (0, 255, 255), 12)
    e, c, cross, _ = process_lane(img)
    assert 0.08 < e < 0.18 and c > 0.4 and not cross


def test_blank_frame_fails_closed():
    e, c, cross, _ = process_lane(np.zeros((240, 320, 3), dtype=np.uint8))
    assert c < 0.4 and not cross


def test_signal_classification():
    red = np.zeros((24, 16, 3), dtype=np.uint8)
    red[:] = (0, 0, 255)
    assert classify_signal_crop(red) == "RED"
    assert classify_signal_crop(np.zeros_like(red)) == "UNKNOWN"
