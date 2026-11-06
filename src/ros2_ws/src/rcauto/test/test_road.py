import cv2
import numpy as np
from rcauto.road_lane import process_road


def test_two_lanes_provide_road_center():
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    img[:] = (50, 50, 50)
    cv2.line(img, (70, 239), (135, 125), (255, 255, 255), 5)
    cv2.line(img, (245, 239), (185, 125), (255, 255, 255), 5)
    error, confidence, crossing, _ = process_road(img)
    assert abs(error) < 0.08
    assert confidence > 0.4
    assert crossing is False


def test_no_single_boundary_driving():
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.line(img, (70, 239), (135, 125), (255, 255, 255), 5)
    _, confidence, _, _ = process_road(img)
    assert confidence < 0.4
