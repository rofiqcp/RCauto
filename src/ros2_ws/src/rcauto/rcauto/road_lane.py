"""Classic two-boundary road lane detection with fail-closed confidence."""

import cv2
import numpy as np


def process_road(frame):
    h, w = frame.shape[:2]
    y0 = int(h * 0.51)
    roi = frame[y0:, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, (0, 0, 140), (179, 65, 255))
    yellow = cv2.inRange(hsv, (14, 70, 80), (44, 255, 255))
    mask = cv2.bitwise_or(white, yellow)
    edges = cv2.Canny(mask, 70, 140)
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180, threshold=22,
        minLineLength=max(14, w // 15), maxLineGap=max(10, w // 20),
    )
    left, right = [], []
    yref = 0.88 * roi.shape[0]
    debug = frame.copy()
    if lines is not None:
        for group in lines:
            # OpenCV 4/5 bindings may return (N, 1, 4) or (N, 4).
            x1, yy1, x2, yy2 = (int(v) for v in np.asarray(group).reshape(4))
            dy = yy2 - yy1
            dx = x2 - x1
            if abs(dy) < max(9, roi.shape[0] * 0.08):
                continue
            slope = dx / dy
            if not 0.10 < abs(slope) < 1.5:
                continue
            xref = x1 + (yref - yy1) * slope
            if not 0 <= xref < w:
                continue
            if xref < w / 2:
                left.append(xref)
            else:
                right.append(xref)
            cv2.line(debug, (x1, yy1 + y0), (x2, yy2 + y0), (0, 255, 0), 2)
    error, confidence = 0.0, 0.0
    if left and right:
        l, r = float(np.median(left)), float(np.median(right))
        width = (r - l) / w
        if 0.22 <= width <= 0.89:
            center = (l + r) / 2
            error = float(np.clip((center - w / 2) / (w / 2), -1, 1))
            confidence = 0.78
    # White/yellow stop bar spanning road, rather than any short horizontal highlight.
    broad = np.count_nonzero(mask[int(mask.shape[0]*0.15):int(mask.shape[0]*0.55)], axis=1)
    intersection = bool(np.count_nonzero(broad > int(w * 0.60)) > 4)
    cv2.putText(debug, f"road e={error:.2f} conf={confidence:.2f} crossing={intersection}",
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
    return error, confidence, intersection, debug
