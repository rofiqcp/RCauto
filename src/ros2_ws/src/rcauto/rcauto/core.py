"""Dependency-free safety logic, controllers and RC driving state machine."""

from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path

import numpy as np


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, float(value)))


class ModeUnavailable(RuntimeError):
    pass


class PIDController:
    """Normalized lane error -> normalized steering; anti-windup and filtered derivative."""

    def __init__(self, kp=0.75, ki=0.06, kd=0.12, output_limit=0.85):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit = output_limit
        self.integral = 0.0
        self.previous_error = None
        self.derivative = 0.0

    def reset(self):
        self.integral = 0.0
        self.previous_error = None
        self.derivative = 0.0

    def step(self, error: float, dt: float) -> float:
        if not math.isfinite(error) or not 0.001 <= dt <= 0.25:
            raise ValueError("Unsafe controller input")
        raw = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        self.derivative = 0.8 * self.derivative + 0.2 * raw
        candidate = clamp(self.integral + error * dt, -2.0, 2.0)
        unsaturated = self.kp * error + self.ki * candidate + self.kd * self.derivative
        if abs(unsaturated) < self.limit or (unsaturated * error < 0):
            self.integral = candidate
        self.previous_error = error
        return clamp(self.kp * error + self.ki * self.integral + self.kd * self.derivative,
                     -self.limit, self.limit)


class FuzzyController:
    """25 Takagi-Sugeno rules over normalized lane offset and rate."""

    def __init__(self, output_limit=0.85):
        self.limit = output_limit
        self.previous_error = None
        self.centers = (-1.0, -0.5, 0.0, 0.5, 1.0)

    @staticmethod
    def memberships(value):
        x = clamp(value, -1, 1)
        centers = (-1, -0.5, 0, 0.5, 1)
        return [max(0.0, 1.0 - abs(x - c) / 0.5) for c in centers]

    def reset(self):
        self.previous_error = None

    def step(self, error: float, dt: float) -> float:
        if not math.isfinite(error) or not 0.001 <= dt <= 0.25:
            raise ValueError("Unsafe controller input")
        rate = 0 if self.previous_error is None else clamp((error - self.previous_error) / dt / 3, -1, 1)
        self.previous_error = error
        e_membership = self.memberships(error)
        de_membership = self.memberships(rate)
        value = total = 0.0
        for i, em in enumerate(e_membership):
            for j, dm in enumerate(de_membership):
                weight = em * dm
                value += weight * clamp(0.65 * self.centers[i] + 0.25 * self.centers[j], -1, 1)
                total += weight
        return clamp(value / max(total, 1e-9), -self.limit, self.limit)


class AIController:
    """Neural policy inference via ONNX, never falls back silently to PID."""

    def __init__(self, model_path, output_limit=0.85):
        if not model_path or not Path(model_path).is_file():
            raise ModeUnavailable("AI ONNX model is not present; steering prohibited")
        try:
            import onnxruntime as ort
            self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
            self.input_name = self.session.get_inputs()[0].name
        except Exception as exc:
            raise ModeUnavailable("Cannot initialize AI model") from exc
        self.previous_error = None
        self.limit = output_limit

    def reset(self):
        self.previous_error = None

    def step(self, error, dt):
        if not math.isfinite(error) or not 0.001 <= dt <= 0.25:
            raise ValueError("Unsafe controller input")
        rate = 0 if self.previous_error is None else clamp((error - self.previous_error) / dt / 3, -1, 1)
        self.previous_error = error
        output = self.session.run(None, {self.input_name: np.array([[error, rate]], dtype=np.float32)})
        value = float(np.asarray(output[0]).ravel()[0])
        if not math.isfinite(value):
            raise ModeUnavailable("AI model returned nonfinite output")
        return clamp(value, -self.limit, self.limit)


class DriveState(str, Enum):
    DISARMED = "DISARMED"
    FOLLOW = "FOLLOW"
    WAIT_SIGNAL = "WAIT_SIGNAL"
    CROSSING = "CROSSING"
    OBSTACLE_STOP = "OBSTACLE_STOP"
    SENSOR_STOP = "SENSOR_STOP"
    ESTOP = "ESTOP"


@dataclass
class Observation:
    lane_error: float = 0
    lane_confidence: float = 0
    lane_age: float = math.inf
    hazard: bool = True
    hazard_age: float = math.inf
    intersection: bool = False
    signal: str = "UNKNOWN"
    signal_age: float = math.inf


class SafetyFSM:
    """Conservative FSM: missing vision/hazard data -> stop; intersection always yields."""

    def __init__(self, max_sensor_age=0.6, min_lane_confidence=0.40,
                 signal_green_hold=0.6, intersection_release_timeout=3.0):
        self.max_sensor_age = max_sensor_age
        self.min_lane_confidence = min_lane_confidence
        self.green_hold = signal_green_hold
        self.release_timeout = intersection_release_timeout
        self.state = DriveState.DISARMED
        self.estop_latched = False
        self.green_since = None
        self.cross_since = None
        self.intersection_latched = False

    def reset_estop(self):
        self.estop_latched = False
        self.state = DriveState.DISARMED

    def evaluate(self, obs: Observation, now: float, armed: bool,
                 estop: bool = False):
        if estop:
            self.estop_latched = True
        if self.estop_latched:
            self.state = DriveState.ESTOP
            return self.state, 0.0
        if not armed:
            self.state = DriveState.DISARMED
            self.green_since = None
            self.cross_since = None
            self.intersection_latched = False
            return self.state, 0.0
        if (obs.lane_age > self.max_sensor_age or
                obs.hazard_age > self.max_sensor_age or
                obs.lane_confidence < self.min_lane_confidence or
                not math.isfinite(obs.lane_error)):
            self.state = DriveState.SENSOR_STOP
            return self.state, 0.0
        if obs.hazard:
            self.state = DriveState.OBSTACLE_STOP
            return self.state, 0.0
        if obs.intersection:
            self.intersection_latched = True
        if self.intersection_latched:
            if self.state != DriveState.CROSSING:
                self.state = DriveState.WAIT_SIGNAL
                if obs.signal_age > self.max_sensor_age or obs.signal != "GREEN":
                    self.green_since = None
                elif self.green_since is None:
                    self.green_since = now
                if self.green_since is not None and now - self.green_since >= self.green_hold:
                    self.state = DriveState.CROSSING
                    self.cross_since = now
                else:
                    return self.state, 0.0
            if self.cross_since is None or now - self.cross_since > self.release_timeout:
                self.state = DriveState.SENSOR_STOP
                return self.state, 0.0
            if not obs.intersection and now - self.cross_since >= 0.5:
                self.intersection_latched = False
                self.green_since = None
                self.cross_since = None
                self.state = DriveState.FOLLOW
                return self.state, 0.18
            return DriveState.CROSSING, 0.11
        self.state = DriveState.FOLLOW
        return self.state, 0.18
