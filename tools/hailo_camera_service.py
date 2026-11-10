#!/usr/bin/env python3
"""Single-owner USB camera, Hailo-8L YOLO inference, and supplementary cone proposals."""
from __future__ import annotations

import argparse
import json
import logging
import socket
import struct
import time

import cv2
import numpy as np
from hailo_platform import (
    ConfigureParams,
    FormatType,
    HailoStreamInterface,
    HEF,
    InferVStreams,
    InputVStreamParams,
    OutputVStreamParams,
    VDevice,
)

LOG = logging.getLogger("rcauto_hailo")
DEFAULT_HEF = "/usr/share/hailo-models/yolov8s_h8l.hef"


def cone_proposals(frame: np.ndarray) -> list[dict]:
    """Conservative orange-cone shape candidates, not neural-network predictions."""
    height, width = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (3, 115, 100), (23, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 13)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    result: list[dict] = []
    for contour in cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_SIMPLE)[-2]:
        x, y, box_w, box_h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if (area < width * height * 0.00065 or box_w < 9 or box_h < 24 or
                box_h < box_w * 0.85 or y + box_h < height * 0.34):
            continue
        region = mask[y:y + box_h, x:x + box_w]
        top_band = region[:max(1, box_h // 3)]
        bottom_band = region[-max(1, box_h // 3):]
        top_width = int(np.count_nonzero(np.any(top_band > 0, axis=0)))
        bottom_width = int(np.count_nonzero(np.any(bottom_band > 0, axis=0)))
        if bottom_width < 6 or top_width >= bottom_width * 0.85:
            continue
        fill_ratio = area / max(1.0, box_w * box_h)
        if fill_ratio < 0.11 or fill_ratio > 0.82:
            continue
        # Cones typically taper upwards. The confidence is a geometric heuristic.
        score = min(0.85, 0.50 + (bottom_width - top_width) / max(1, box_w) * 0.25)
        result.append({
            "class_id": "traffic_cone",
            "score": round(score, 3),
            "bbox": [x + box_w / 2, y + box_h / 2, box_w, box_h],
            "source": "orange_shape_heuristic",
        })
    return sorted(result, key=lambda obj: obj["score"], reverse=True)[:8]


def decode_yolo(nms: list, width: int, height: int, threshold: float,
                model_width: int, model_height: int,
                scale: float, pad_x: int, pad_y: int) -> list[dict]:
    """Hailo NMS: batch -> class -> [y_min,x_min,y_max,x_max,score]."""
    detections: list[dict] = []
    for class_idx, boxes in enumerate(nms[0]):
        for y1, x1, y2, x2, confidence in boxes:
            if float(confidence) < threshold:
                continue
            xmin = float(np.clip((x1 * model_width - pad_x) / scale, 0, width))
            xmax = float(np.clip((x2 * model_width - pad_x) / scale, 0, width))
            ymin = float(np.clip((y1 * model_height - pad_y) / scale, 0, height))
            ymax = float(np.clip((y2 * model_height - pad_y) / scale, 0, height))
            if xmax <= xmin or ymax <= ymin:
                continue
            detections.append({
                "class_id": str(class_idx),
                "score": round(float(confidence), 4),
                "bbox": [(xmin + xmax) / 2, (ymin + ymax) / 2, xmax - xmin, ymax - ymin],
                "source": "hailo8l_yolov8s",
            })
    return sorted(detections, key=lambda obj: obj["score"], reverse=True)[:60]


def serve(args: argparse.Namespace) -> None:
    capture = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open USB camera index {args.camera}")
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_FPS, args.fps)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    ok, first = capture.read()
    if not ok or first is None:
        raise RuntimeError("USB camera provided no frames")
    LOG.info("USB camera opened: %dx%d; MJPEG", first.shape[1], first.shape[0])

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", args.port))
    server.listen(1)
    server.settimeout(0.001)
    peer: socket.socket | None = None
    sequence = 0
    last_report = time.monotonic()
    next_frame_deadline = time.monotonic()
    frame_period = 1.0 / max(1, args.fps)
    hef = HEF(args.hef)
    input_info = hef.get_input_vstream_infos()[0]
    output_info = hef.get_output_vstream_infos()[0]
    input_height, input_width = input_info.shape[:2]
    LOG.info("NPU model: %s input=%s output=%s", args.hef, input_info.shape, output_info.name)

    try:
        with VDevice() as device:
            config = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
            group = device.configure(hef, config)[0]
            input_params = InputVStreamParams.make(group, format_type=FormatType.UINT8)
            output_params = OutputVStreamParams.make(group, format_type=FormatType.FLOAT32)
            with InferVStreams(group, input_params, output_params) as pipeline:
                with group.activate():
                    while True:
                        delay = next_frame_deadline - time.monotonic()
                        if delay > 0:
                            time.sleep(delay)
                        next_frame_deadline = time.monotonic() + frame_period
                        ok, frame = capture.read()
                        if not ok or frame is None:
                            raise RuntimeError("USB camera stream stopped")
                        captured_ns = time.monotonic_ns()
                        height, width = frame.shape[:2]
                        # Letterbox preserves geometry expected by YOLOv8 at 640x640.
                        scale = min(input_width / width, input_height / height)
                        resize_width = min(input_width, round(width * scale))
                        resize_height = min(input_height, round(height * scale))
                        pad_x = (input_width - resize_width) // 2
                        pad_y = (input_height - resize_height) // 2
                        model_image = np.full((input_height, input_width, 3),
                                              114, dtype=np.uint8)
                        model_image[pad_y:pad_y + resize_height,
                                    pad_x:pad_x + resize_width] = cv2.resize(
                                        frame, (resize_width, resize_height))
                        rgb = cv2.cvtColor(model_image, cv2.COLOR_BGR2RGB)
                        started = time.monotonic()
                        predictions = pipeline.infer({input_info.name: rgb[np.newaxis, ...]})
                        infer_ms = (time.monotonic() - started) * 1000
                        detections = decode_yolo(
                            predictions[output_info.name], width, height, args.min_score,
                            input_width, input_height, scale, pad_x, pad_y)
                        cones = cone_proposals(frame)
                        detections.extend(cones)
                        packet_info = {
                            "sequence": sequence, "capture_monotonic_ns": captured_ns,
                            "model": "yolov8s_h8l",
                            "npu": "HAILO8L", "npu_inference_ms": round(infer_ms, 2),
                            "width": width, "height": height,
                            "detections": detections,
                            "cone_method": "orange_shape_heuristic",
                        }
                        sequence += 1
                        now = time.monotonic()
                        if now - last_report > 5:
                            LOG.info("HAILO_NPU_RUNNING frames=%d npu_ms=%.1f detections=%d cones=%d",
                                     sequence, infer_ms, len(detections), len(cones))
                            last_report = now
                        if peer is None:
                            try:
                                peer, addr = server.accept()
                                peer.settimeout(1.5)
                                peer.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                                LOG.info("ROS camera bridge connected from %s", addr)
                            except socket.timeout:
                                continue
                        valid, encoded = cv2.imencode(".jpg", frame,
                                                      [cv2.IMWRITE_JPEG_QUALITY, 78])
                        if not valid:
                            raise RuntimeError("JPEG encoding failed")
                        header = json.dumps(packet_info, separators=(",", ":")).encode("utf-8")
                        jpeg = encoded.tobytes()
                        try:
                            peer.sendall(struct.pack("!II", len(header), len(jpeg)) + header + jpeg)
                        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                            LOG.warning("ROS bridge disconnected, waiting for reconnection")
                            peer.close()
                            peer = None
    finally:
        capture.release()
        if peer is not None:
            peer.close()
        server.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="RCauto Hailo-8L webcam inference")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--port", type=int, default=8878)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--min-score", type=float, default=0.35)
    parser.add_argument("--hef", default=DEFAULT_HEF)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        serve(args)
    except KeyboardInterrupt:
        LOG.info("Hailo camera inference stopped")


if __name__ == "__main__":
    main()
