#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace rcauto {
inline double clamp(double x, double lo, double hi) {
  if (!std::isfinite(x) || !std::isfinite(lo) || !std::isfinite(hi) || lo > hi) {
    throw std::invalid_argument("invalid finite clamp");
  }
  return std::max(lo, std::min(x, hi));
}
inline void validate_dt(double error, double dt) {
  if (!std::isfinite(error) || !std::isfinite(dt) || dt < 0.001 || dt > 0.25) {
    throw std::invalid_argument("control input/dt invalid");
  }
}
struct Gains { double kp{0.75}, ki{0.06}, kd{0.12}, max_output{0.85}; };
class Pid {
 public:
  explicit Pid(Gains g = {}) : gains_(g) {}
  void set_gains(Gains g) { gains_ = g; reset(); }
  void reset() { integral_ = derivative_ = last_error_ = 0; has_previous_ = false; }
  double calculate(double error, double dt) {
    validate_dt(error, dt);
    const double raw = has_previous_ ? (error - last_error_) / dt : 0.0;
    derivative_ = derivative_ * 0.8 + raw * 0.2;
    const double candidate = clamp(integral_ + error * dt, -2, 2);
    const double unsaturated = gains_.kp * error + gains_.ki * candidate +
                               gains_.kd * derivative_;
    if (std::abs(unsaturated) < gains_.max_output || unsaturated * error < 0) {
      integral_ = candidate;
    }
    last_error_ = error;
    has_previous_ = true;
    return clamp(gains_.kp * error + gains_.ki * integral_ +
                     gains_.kd * derivative_, -gains_.max_output, gains_.max_output);
  }
 private:
  Gains gains_;
  double integral_{0}, derivative_{0}, last_error_{0};
  bool has_previous_{false};
};
class Fuzzy {
 public:
  void set(double error_weight, double rate_weight, double limit) {
    if (error_weight < 0 || rate_weight < 0 || limit <= 0 || limit > 1.0 ||
        !std::isfinite(error_weight) || !std::isfinite(rate_weight)) {
      throw std::invalid_argument("invalid fuzzy weights");
    }
    ew_ = error_weight; rw_ = rate_weight; limit_ = limit;
    reset();
  }
  void reset() { previous_ = 0; has_previous_ = false; }
  double calculate(double error, double dt) {
    validate_dt(error, dt);
    const double rate = has_previous_ ? clamp((error - previous_) / dt / 3, -1, 1) : 0.0;
    previous_ = error; has_previous_ = true;
    const auto a = memberships(error);
    const auto b = memberships(rate);
    double result = 0, weights = 0;
    for (std::size_t i = 0; i < 5; ++i) for (std::size_t j = 0; j < 5; ++j) {
      const double w = a[i] * b[j];
      const double value = clamp(ew_ * centers_[i] + rw_ * centers_[j], -1, 1);
      result += w * value; weights += w;
    }
    if (weights < 1e-9) throw std::runtime_error("fuzzy membership sum zero");
    return clamp(result / weights, -limit_, limit_);
  }
 private:
  static std::array<double, 5> memberships(double value) {
    const double x = clamp(value, -1, 1);
    std::array<double, 5> v{};
    for (std::size_t i = 0; i < 5; ++i) {
      v[i] = std::max(0.0, 1.0 - std::abs(x - centers_[i]) / 0.5);
    }
    return v;
  }
  inline static constexpr std::array<double, 5> centers_{-1, -0.5, 0, 0.5, 1};
  double previous_{0}, ew_{0.65}, rw_{0.25}, limit_{0.85};
  bool has_previous_{false};
};

enum class State {
  Disarmed, Follow, WaitSignal, Crossing, ObstacleStop, SensorStop, Estop
};
inline const char * state_name(State s) {
  switch (s) {
    case State::Disarmed: return "DISARMED";
    case State::Follow: return "FOLLOW";
    case State::WaitSignal: return "WAIT_SIGNAL";
    case State::Crossing: return "CROSSING";
    case State::ObstacleStop: return "OBSTACLE_STOP";
    case State::SensorStop: return "SENSOR_STOP";
    case State::Estop: return "ESTOP";
  }
  return "UNKNOWN";
}
struct Observation {
  double error{0}, confidence{0}, lane_age{std::numeric_limits<double>::infinity()};
  bool hazard{true};
  double hazard_age{std::numeric_limits<double>::infinity()};
  bool intersection{false};
  std::string signal{"UNKNOWN"};
  double signal_age{std::numeric_limits<double>::infinity()};
};
struct Limits {
  double stale_s{0.6}, min_confidence{0.4}, green_hold_s{0.6},
    cross_timeout_s{3.0}, normal_speed{0.18}, crossing_speed{0.11};
};
class StateMachine {
 public:
  void reset_disarmed() {
    estop_ = false; fault_latched_ = false; state_ = State::Disarmed; green_since_ = -1;
    crossing_since_ = -1; latched_intersection_ = false;
  }
  void emergency_stop() { estop_ = true; state_ = State::Estop; }
  std::pair<State, double> update(const Observation& o, double now, bool armed, Limits lim) {
    if (estop_) return {state_ = State::Estop, 0};
    if (fault_latched_ && armed) return {state_ = State::SensorStop, 0};
    if (!armed) {
      fault_latched_ = false;
      state_ = State::Disarmed;
      green_since_ = crossing_since_ = -1;
      latched_intersection_ = false;
      return {state_, 0};
    }
    if (!std::isfinite(o.error) || !std::isfinite(o.confidence) ||
        o.confidence < lim.min_confidence || o.lane_age > lim.stale_s ||
        o.hazard_age > lim.stale_s) {
      return {state_ = State::SensorStop, 0};
    }
    if (o.hazard) return {state_ = State::ObstacleStop, 0};
    if (o.intersection) latched_intersection_ = true;
    if (latched_intersection_) {
      if (state_ != State::Crossing) {
        state_ = State::WaitSignal;
        if (o.signal_age > lim.stale_s || o.signal != "GREEN") green_since_ = -1;
        else if (green_since_ < 0) green_since_ = now;
        if (green_since_ < 0 || now - green_since_ < lim.green_hold_s)
          return {state_, 0};
        crossing_since_ = now;
        state_ = State::Crossing;
      }
      if (crossing_since_ < 0 || now - crossing_since_ > lim.cross_timeout_s) {
        fault_latched_ = true;
        return {state_ = State::SensorStop, 0};
      }
      if (!o.intersection && now - crossing_since_ >= 0.5) {
        latched_intersection_ = false; green_since_ = crossing_since_ = -1;
        return {state_ = State::Follow, lim.normal_speed};
      }
      return {state_ = State::Crossing, lim.crossing_speed};
    }
    return {state_ = State::Follow, lim.normal_speed};
  }
 private:
  State state_{State::Disarmed};
  bool estop_{false}, latched_intersection_{false}, fault_latched_{false};
  double green_since_{-1}, crossing_since_{-1};
};
}  // namespace rcauto
