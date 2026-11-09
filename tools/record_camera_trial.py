#!/usr/bin/env python3
"""Record camera-only RCauto trials: one combined MP4 plus synchronized CSV.

Odometry is subscribed exclusively by this recorder for evaluation and laps.
It is never published back to any controller, vision or actuator topic.
"""
import argparse
import csv
import math
import signal
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Bool, Float32, String


PROJECT = Path("/media/sirobo/HDD/RCauto")
RECORDINGS = PROJECT / "src/recordings/KSTI2026_DualControl"
CENTERLINE = PROJECT / "src/rcauto_cpp/sim/maps/ksti2026_centerline.csv"
ROBOT_WIDTH_M = 0.20
ROBOT_LENGTH_M = 0.35
ROAD_WIDTH_M = 0.50
SAFE_OFFSET_M = (ROAD_WIDTH_M - ROBOT_WIDTH_M) / 2


class TrialRecorder(Node):
    def __init__(self, args):
        super().__init__("rcauto_dual_control_recorder")
        self.mode = args.mode
        self.duration = args.duration
        self.stop_after_laps = args.stop_after_laps
        self.start = time.monotonic()
        self.end_requested = False
        self.arm = False
        self.armed_at = None
        self.stop_since = None
        self.bridge = CvBridge()
        self.image = {}
        self.last_image_time = {}
        self.image_count = {"front": 0, "world": 0}
        self.latest = {}
        self.status = "DISARMED"
        self.latest["status"] = self.status
        self.odom_last_time = None
        self.odom_x = 0.0
        self.odom_y = -2.919027
        self.odom_yaw = 3.134082
        self.measured_speed = 0.0
        self.lateral_offset = None
        self.last_s = None
        self.progress = 0.0
        self.off_track = 0
        self.odom_samples = 0
        self.previous_report = -1
        self.path = np.loadtxt(CENTERLINE, delimiter=",", skiprows=1)
        self.track_length = self.path[-1, 3] + float(
            np.linalg.norm(self.path[-1, :2] - self.path[0, :2]))
        self.directory = RECORDINGS / args.iteration
        # Never overwrite a prior experiment.
        self.directory.mkdir(parents=True, exist_ok=False)
        self.csv_path = self.directory / f"{args.mode}_telemetry.csv"
        self.video_path = self.directory / f"{args.mode}_camera_arena.mp4"
        self.columns = [
            "time_s", "mode", "status", "front_frames_received",
            "world_frames_received", "error", "raw_error", "near_error",
            "far_error", "curvature", "preview_visibility", "lane_confidence",
            "visual_risk", "speed_factor", "target_speed_mps",
            "steering_normalized", "steering_angle_rad", "speed_cmd_mps",
            "esc_pwm_ms", "steering_pwm_ms", "esc_pwm_us",
            "steering_pwm_us", "range_m", "hazard", "intersection", "signal",
            "odom_x_world_eval_only", "odom_y_world_eval_only",
            "odom_yaw_world_eval_only", "measured_speed_eval_only",
            "centerline_offset_eval_only", "progress_m_eval_only",
            "lap_count_eval_only", "off_track_eval_only",
            "offtrack_percent_eval_only",
        ]
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.columns)
        self.writer.writeheader()
        self.encoder = subprocess.Popen([
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-f", "rawvideo", "-pix_fmt", "bgr24", "-s:v", "960x540",
            "-r", "5", "-i", "pipe:0", "-an", "-c:v", "libx264",
            "-preset", "ultrafast", "-threads", "1", "-crf", "25",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(self.video_path)
        ], stdin=subprocess.PIPE)
        self.recorded_frames = 0
        self.csv_rows = 0
        self.command_pub = self.create_publisher(
            Bool, "/rcauto/operator/enable", 10)
        for topic, key in (
            ("/rcauto/camera/annotated", "front"),
            ("/rcauto/arena/overview", "world")
        ):
            self.create_subscription(
                Image, topic, lambda msg, label=key: self.on_image(label, msg),
                qos_profile_sensor_data)
        for topic, key in (
            ("/rcauto/lane_error", "error"),
            ("/rcauto/vision/lane_error_raw", "raw_error"),
            ("/rcauto/vision/near_error", "near_error"),
            ("/rcauto/vision/far_error", "far_error"),
            ("/rcauto/vision/curvature", "curvature"),
            ("/rcauto/vision/preview_visibility", "preview_visibility"),
            ("/rcauto/lane_confidence", "lane_confidence"),
            ("/rcauto/tuning/visual_risk", "visual_risk"),
            ("/rcauto/tuning/speed_factor", "speed_factor"),
            ("/rcauto/tuning/speed_command_mps", "target_speed_mps"),
            ("/rcauto/tuning/steer_norm", "steering_normalized"),
            ("/rcauto/actuator/esc_pwm_ms", "esc_pwm_ms"),
            ("/rcauto/actuator/steering_pwm_ms", "steering_pwm_ms"),
            ("/rcauto/actuator/esc_pwm_us", "esc_pwm_us"),
            ("/rcauto/actuator/steering_pwm_us", "steering_pwm_us")
        ):
            self.create_subscription(
                Float32, topic, lambda msg, key=key: self.receive_float(key, msg),
                10)
        self.create_subscription(
            String, "/rcauto/status", self.on_status, 10)
        self.create_subscription(
            AckermannDriveStamped, "/rcauto/drive", self.on_drive, 10)
        self.create_subscription(
            Odometry, "/rcauto/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            Range, "/rcauto/range", lambda m: self.latest.__setitem__(
                "range_m", m.range), 10)
        self.create_subscription(
            Bool, "/rcauto/hazard", lambda m: self.latest.__setitem__(
                "hazard", m.data), 10)
        self.create_subscription(
            Bool, "/rcauto/intersection", lambda m: self.latest.__setitem__(
                "intersection", m.data), 10)
        self.create_subscription(
            String, "/rcauto/signal", lambda m: self.latest.__setitem__(
                "signal", m.data), 10)
        self.create_timer(0.1, self.log_sample)
        self.create_timer(0.2, self.video_sample)
        self.create_timer(1.0, self.control_sample)
        print(f"TRIAL_BEGIN mode={self.mode} iteration={args.iteration} "
              f"target={self.stop_after_laps} laps "
              f"track={self.track_length:.3f}m duration={self.duration}s", flush=True)

    def receive_float(self, key, msg):
        self.latest[key] = float(msg.data)

    def on_image(self, key, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if image is not None and image.size:
                self.image[key] = image
                self.image_count[key] += 1
                self.last_image_time[key] = time.monotonic()
        except Exception as error:
            if self.image_count[key] == 0:
                print(f"IMAGE_READ_FAILURE {key}: {error}", flush=True)

    def on_status(self, msg):
        self.status = msg.data.split("|")[0].strip()
        self.latest["status"] = self.status

    def on_drive(self, msg):
        self.latest["steering_angle_rad"] = float(msg.drive.steering_angle)
        self.latest["speed_cmd_mps"] = float(msg.drive.speed)

    def on_odom(self, msg):
        # Scoring only. This node does not publish pose to the controller.
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if not (math.isfinite(p.x) and math.isfinite(p.y)):
            return
        yaw0 = 3.134082
        c, s = math.cos(yaw0), math.sin(yaw0)
        x = c*p.x - s*p.y
        y = -2.919027 + s*p.x + c*p.y
        yaw_local = math.atan2(
            2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.odom_x, self.odom_y = x, y
        self.odom_yaw = yaw0 + yaw_local
        vel = msg.twist.twist.linear
        self.measured_speed = math.hypot(vel.x, vel.y)
        t = time.monotonic()
        if self.odom_last_time is not None and t - self.odom_last_time < .08:
            return
        self.odom_last_time = t
        delta = np.hypot(self.path[:, 0]-x, self.path[:, 1]-y)
        index = int(np.argmin(delta))
        self.lateral_offset = float(delta[index])
        # Scoring is based on MOVING vehicle samples; long safety holds
        # outside the line must not artificially inflate the failure rate.
        if self.arm and (self.latest.get("speed_cmd_mps",0) > 0.02 or
                         self.measured_speed > 0.02):
            self.odom_samples += 1
            self.off_track += int(self.lateral_offset > SAFE_OFFSET_M)
        if self.lateral_offset > ROAD_WIDTH_M/2:
            return
        station = float(self.path[index, 3])
        if self.last_s is not None:
            ds = station - self.last_s
            if ds < -self.track_length/2:
                ds += self.track_length
            if ds > self.track_length/2:
                ds -= self.track_length
            if abs(ds) < 0.20:
                self.progress = max(0.0, self.progress + ds)
        self.last_s = station

    def log_sample(self):
        row = {key: self.latest.get(key, "") for key in self.columns}
        row.update({
            "time_s": round(time.monotonic()-self.start, 3),
            "mode": self.mode,
            "status": self.status,
            "front_frames_received": self.image_count["front"],
            "world_frames_received": self.image_count["world"],
            "odom_x_world_eval_only": round(self.odom_x, 4),
            "odom_y_world_eval_only": round(self.odom_y, 4),
            "odom_yaw_world_eval_only": round(self.odom_yaw, 4),
            "measured_speed_eval_only": round(self.measured_speed, 4),
            "centerline_offset_eval_only": (
                round(self.lateral_offset, 4)
                if self.lateral_offset is not None else ""),
            "progress_m_eval_only": round(self.progress, 3),
            "lap_count_eval_only": self.progress/self.track_length,
            "off_track_eval_only": (
                int(self.lateral_offset > SAFE_OFFSET_M)
                if self.lateral_offset is not None else ""),
            "offtrack_percent_eval_only": (
                round(100*self.off_track/self.odom_samples, 2)
                if self.odom_samples else ""),
        })
        self.writer.writerow(row)
        self.csv_rows += 1
        if self.csv_rows % 20 == 0:
            self.csv_file.flush()

    def render_world_map(self, width, height):
        """Top-down pose visualization for evaluation, not an autonomy sensor."""
        frame = np.full((height, width, 3), (25, 21, 24), dtype=np.uint8)
        xy = self.path[:, :2]
        center = (xy.max(axis=0) + xy.min(axis=0)) * .5
        span = np.maximum(xy.max(axis=0) - xy.min(axis=0), 0.5)
        factor = min((width-48)/(span[0]+.65), (height-65)/(span[1]+.65))

        def project(values):
            v = np.asarray(values)
            return np.round(np.column_stack((
                width*.5 + (v[:, 0]-center[0])*factor,
                height*.5 - (v[:, 1]-center[1])*factor
            ))).astype(np.int32)

        normal = np.stack((-np.sin(self.path[:, 2]),
                            np.cos(self.path[:, 2])), axis=1)
        for sign in (-1,1):
            boundary = project(xy + sign * ROAD_WIDTH_M*.5 * normal)
            cv2.polylines(frame, [boundary], True, (215, 215, 215),
                          2, cv2.LINE_AA)
        dashed = project(xy)
        for i in range(1, len(dashed)):
            if int(self.path[i, 3]/.22)%2==0:
                cv2.line(frame, tuple(dashed[i-1]), tuple(dashed[i]),
                         (160, 160, 165), 1, cv2.LINE_AA)
        x, y = self.odom_x, self.odom_y
        yaw = self.odom_yaw
        heading = np.array((math.cos(yaw), math.sin(yaw)))
        sideways = np.array((-math.sin(yaw), math.cos(yaw)))
        poly = np.array([
            np.array((x,y)) + f*ROBOT_LENGTH_M*.5*heading +
                              side*ROBOT_WIDTH_M*.5*sideways
            for f,side in ((1,1),(1,-1),(-1,-1),(-1,1))
        ])
        cv2.fillPoly(frame,[project(poly)],(70,200,240))
        cv2.polylines(frame,[project(poly)],True,(250,250,250),1)
        nose = project(np.array([[x,y],[x,y]]) +
                       np.array([[0,0],heading*.22]))
        cv2.arrowedLine(frame,tuple(nose[0]),tuple(nose[1]),
                        (50,255,120),2,tipLength=.28)
        cv2.putText(frame,"Gazebo pose (evaluation only)",(8,18),
                    cv2.FONT_HERSHEY_SIMPLEX,.42,(235,235,235),1)
        return frame

    def video_sample(self):
        canvas = np.full((540, 960, 3), (22, 20, 19), dtype=np.uint8)
        for key, left, top, width, height in (
            ("front", 0, 48, 600, 338),
            ("world", 600, 35, 360, 395)
        ):
            frame = self.image.get(key)
            # If Gazebo top-down sensor is unavailable, render evaluation pose.
            # Never feed this map back into steering or throttle controllers.
            if key == "world" and (
                    frame is None or time.monotonic() -
                    self.last_image_time.get(key, 0) > 2.0):
                frame = self.render_world_map(width, height)
            if frame is None:
                continue
            h, w = frame.shape[:2]
            ratio = min(width/w, height/h)
            new_w, new_h = int(w*ratio), int(h*ratio)
            view = cv2.resize(frame, (new_w, new_h))
            px = left+(width-new_w)//2
            py = top+(height-new_h)//2
            canvas[py:py+new_h, px:px+new_w] = view
            if (key == "front" and
                    time.monotonic()-self.last_image_time.get(key, 0) > 1.5):
                cv2.putText(canvas, "CAMERA STALE", (px+10, py+40),
                            cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 250), 2)
        def put(message, y, scale=.54):
            cv2.putText(canvas, message, (13, y),
                        cv2.FONT_HERSHEY_SIMPLEX, scale, (235, 240, 245),
                        1, cv2.LINE_AA)
        t = time.monotonic()-self.start
        put(f"RCauto CAMERA-ONLY | {self.mode.upper()} | {self.directory.name}", 28, .62)
        put(f"T={t:.1f}s | {self.status} | distance={self.progress:.2f}m "
            f"laps={self.progress/self.track_length:.2f}", 453)
        put("near={:+.2f} far={:+.2f} curve={:+.2f} risk={:.2f} preview={:.2f}".format(
            self.latest.get("near_error", 0), self.latest.get("far_error", 0),
            self.latest.get("curvature", 0), self.latest.get("visual_risk", 0),
            self.latest.get("preview_visibility", 0)), 474, .49)
        put("steer={:+.3f} rad | ESC={:.3f} m/s | factor={:.2f}".format(
            self.latest.get("steering_angle_rad", 0),
            self.latest.get("speed_cmd_mps", 0),
            self.latest.get("speed_factor", 0)), 496, .53)
        put("PWM servo={:.3f}ms ESC={:.3f}ms | camera error={:+.3f}".format(
            self.latest.get("steering_pwm_ms", 1.5),
            self.latest.get("esc_pwm_ms", 1.5),
            self.latest.get("error", 0)), 520, .53)
        try:
            self.encoder.stdin.write(canvas.tobytes())
            self.recorded_frames += 1
        except (BrokenPipeError, OSError) as error:
            print(f"ENCODING_FAILURE {error}", flush=True)
            self.end_requested = True

    def control_sample(self):
        elapsed = time.monotonic()-self.start
        if (not self.arm and elapsed > 5 and
                self.image_count["front"] >= 8):
            for _ in range(4):
                self.command_pub.publish(Bool(data=True))
            self.arm = True
            self.armed_at = elapsed
            print("ARMED_VIRTUAL_ONLY",flush=True)
        report_index = int(elapsed//12)
        if report_index > self.previous_report:
            self.previous_report = report_index
            print("TRIAL_PROGRESS time=%.1fs mode=%s state=%s "
                  "distance=%.2fm laps=%.2f offtrack=%.2f%% speed=%.3fmps "
                  "steer=%+.3frad curve=%+.2f risk=%.2f frames=%s" % (
                    elapsed, self.mode, self.status, self.progress,
                    self.progress/self.track_length,
                    100*self.off_track/max(1,self.odom_samples),
                    self.latest.get("speed_cmd_mps",0),
                    self.latest.get("steering_angle_rad",0),
                    self.latest.get("curvature",0),
                    self.latest.get("visual_risk",0),
                    self.image_count), flush=True)
        if self.stop_after_laps and self.progress/self.track_length >= self.stop_after_laps:
            print("LAP_GOAL_REACHED",flush=True)
            self.end_requested = True
        if elapsed >= self.duration:
            print("MAX_DURATION_REACHED",flush=True)
            self.end_requested = True
        if self.arm and self.status in ("SENSOR_STOP", "ESTOP"):
            if self.stop_since is None: self.stop_since = elapsed
            elif elapsed - self.stop_since > 15:
                print("PERSISTENT_SAFETY_STOP",flush=True)
                self.end_requested = True
        else:
            self.stop_since = None
        if not self.arm and elapsed > 40:
            print("CAMERA_NOT_READY",flush=True)
            self.end_requested = True

    def finish(self):
        self.command_pub.publish(Bool(data=False))
        self.csv_file.flush()
        self.csv_file.close()
        self.encoder.stdin.close()
        self.encoder.wait(timeout=40)
        print("TRIAL_FINISH mode=%s iteration=%s progress=%.3fm laps=%.3f "
              "offtrack=%.2f%% video_frames=%d csv_rows=%d "
              "video_rc=%d video=%s csv=%s" % (
                  self.mode, self.directory.name, self.progress,
                  self.progress/self.track_length,
                  100*self.off_track/max(1,self.odom_samples),
                  self.recorded_frames, self.csv_rows,
                  self.encoder.returncode, self.video_path, self.csv_path
              ), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("pid", "fuzzy", "ai"), required=True)
    parser.add_argument("--iteration", required=True)
    parser.add_argument("--duration", type=float, default=180)
    parser.add_argument("--stop-after-laps", type=int, default=2)
    arguments = parser.parse_args()
    if (not arguments.iteration or
            not all(ch.isalnum() or ch in "_-" for ch in arguments.iteration) or
            arguments.duration < 10):
        raise ValueError("Invalid iteration or duration")
    rclpy.init()
    recorder = TrialRecorder(arguments)
    try:
        while rclpy.ok() and not recorder.end_requested:
            rclpy.spin_once(recorder, timeout_sec=.06)
    finally:
        recorder.finish()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
