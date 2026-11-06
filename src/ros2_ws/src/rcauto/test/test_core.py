import math
import pytest
from rcauto.core import (
    AIController, DriveState, FuzzyController, ModeUnavailable, Observation,
    PIDController, SafetyFSM,
)


def good(intersection=False, signal="UNKNOWN"):
    return Observation(lane_error=0.1, lane_confidence=0.9, lane_age=0.05,
                       hazard=False, hazard_age=0.05, intersection=intersection,
                       signal=signal, signal_age=0.05)


@pytest.mark.parametrize("controller", [PIDController(), FuzzyController()])
def test_control_sign_range(controller):
    assert 0 < controller.step(0.3, 0.05) <= 0.85
    controller.reset()
    assert -0.85 <= controller.step(-0.3, 0.05) < 0


def test_control_rejects_nan():
    with pytest.raises(ValueError):
        PIDController().step(math.nan, 0.05)


def test_untrained_ai_fails_closed():
    with pytest.raises(ModeUnavailable):
        AIController("/nonexistent/model.onnx")


def test_defaults_disarmed_and_stale_stop():
    s = SafetyFSM()
    assert s.evaluate(good(), 1.0, False)[0] == DriveState.DISARMED
    stale = good()
    stale.hazard_age = 10
    assert s.evaluate(stale, 1.1, True) == (DriveState.SENSOR_STOP, 0.0)


def test_hazard_preempts_signal():
    s = SafetyFSM()
    obs = good(True, "GREEN")
    obs.hazard = True
    assert s.evaluate(obs, 10, True) == (DriveState.OBSTACLE_STOP, 0.0)


def test_intersection_requires_stable_green_then_exits():
    s = SafetyFSM(signal_green_hold=0.6)
    assert s.evaluate(good(True, "RED"), 1, True)[0] == DriveState.WAIT_SIGNAL
    assert s.evaluate(good(True, "GREEN"), 2, True)[1] == 0
    assert s.evaluate(good(True, "GREEN"), 2.7, True)[0] == DriveState.CROSSING
    state, speed = s.evaluate(good(False, "UNKNOWN"), 3.3, True)
    assert state == DriveState.FOLLOW and speed > 0


def test_unknown_signal_never_passes_intersection():
    s = SafetyFSM()
    for i in range(20):
        assert s.evaluate(good(True, "UNKNOWN"), i * 0.1, True)[1] == 0


def test_estop_latched_until_manual_reset():
    s = SafetyFSM()
    assert s.evaluate(good(), 1, True, estop=True)[0] == DriveState.ESTOP
    assert s.evaluate(good(), 2, True)[1] == 0
    s.reset_estop()
    assert s.evaluate(good(), 3, False)[0] == DriveState.DISARMED


def test_crossing_timeout_stops():
    s = SafetyFSM()
    s.evaluate(good(True, "GREEN"), 1.0, True)
    s.evaluate(good(True, "GREEN"), 1.7, True)
    assert s.evaluate(good(True, "GREEN"), 5.0, True)[1] == 0
