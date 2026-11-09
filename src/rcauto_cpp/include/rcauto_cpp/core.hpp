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
// Reject malformed Range samples, including reversed bounds and NaN limits.
inline bool valid_range_sample(double value, double minimum, double maximum) {
  return std::isfinite(value) && std::isfinite(minimum) &&
         std::isfinite(maximum) && minimum >= 0.0 &&
         maximum >= minimum && value >= minimum && value <= maximum;
}
// The stop threshold must lie inside the sensor's declared measurement interval.
// This does not establish detection in the near-field blind zone below minimum.
inline bool range_covers_stop_zone(double minimum, double maximum, double stop_distance) {
  return std::isfinite(minimum) && std::isfinite(maximum) &&
         std::isfinite(stop_distance) && minimum >= 0.0 &&
         stop_distance > 0.0 && minimum <= stop_distance && maximum >= stop_distance;
}
// Reject missing, future-dated, or expired range measurements.
inline bool range_sample_healthy(double age_s, double timeout_s, bool sample_valid,
                                 double minimum, double maximum, double stop_distance) {
  return sample_valid && std::isfinite(age_s) && std::isfinite(timeout_s) &&
         age_s >= 0.0 && timeout_s > 0.0 && age_s <= timeout_s &&
         range_covers_stop_zone(minimum, maximum, stop_distance);
}
// Combine fresh vision and range observations without retaining an old range hazard.
inline bool combined_hazard(bool vision_hazard, bool range_obstacle) {
  return vision_hazard || range_obstacle;
}
struct Gains { double kp{0.75}, ki{0.06}, kd{0.12}, max_output{0.85}; };
class Pid {
 public:
  explicit Pid(Gains g = {}) : gains_(validated(g)) {}
  void set_gains(Gains g) { gains_ = validated(g); reset(); }
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
  static Gains validated(Gains g) {
    if (!std::isfinite(g.kp) || !std::isfinite(g.ki) ||
        !std::isfinite(g.kd) || !std::isfinite(g.max_output) ||
        g.kp < 0.0 || g.ki < 0.0 || g.kd < 0.0 ||
        g.max_output <= 0.0 || g.max_output > 1.0) {
      throw std::invalid_argument("invalid PID gains or output limit");
    }
    return g;
  }
  Gains gains_;
  double integral_{0}, derivative_{0}, last_error_{0};
  bool has_previous_{false};
};
class Fuzzy {
 public:
  void set(double error_weight, double rate_weight, double limit) {
    if (error_weight < 0 || rate_weight < 0 || limit <= 0 || limit > 1.0 ||
        !std::isfinite(error_weight) || !std::isfinite(rate_weight) ||
        !std::isfinite(limit)) {
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


// Dual-output speed controller. This PID regulates perceived visual corner
// risk, NOT actual vehicle speed (which requires a wheel/speed sensor).
// Risk is derived exclusively from the forward camera.
class VisualSpeedPid {
 public:
  void set_gains(Gains gains) { pid_.set_gains(gains); }
  void reset() { pid_.reset(); }
  double calculate(double visual_risk, double dt) {
    validate_dt(visual_risk, dt);
    const double error = 0.22 - clamp(visual_risk, 0.0, 1.0);
    return clamp(0.90 + pid_.calculate(error, dt), 0.25, 1.0);
  }
 private:
  Pid pid_{Gains{1.9, 0.0, 0.06, 1.0}};
};

// Mamdani-style triangular fuzzification with singleton defuzzification.
// Separate fuzzy rule surface for throttle, alongside the steering fuzzy.
class FuzzyThrottle {
 public:
  void set_aggressiveness(double value) {
    if (!std::isfinite(value) || value < 0.5 || value > 2.0)
      throw std::invalid_argument("fuzzy throttle aggression out of range");
    aggression_ = value;
  }
  void reset() { previous_ = 0.0; have_previous_ = false; }
  double calculate(double visual_risk, double dt) {
    validate_dt(visual_risk, dt);
    const double risk = clamp(visual_risk, 0.0, 1.0);
    const double trend = have_previous_
        ? clamp((risk - previous_) / dt / 3.0, -1.0, 1.0) : 0.0;
    previous_ = risk;
    have_previous_ = true;
    const auto risk_weights = memberships(risk * 2.0 - 1.0);
    const auto trend_weights = memberships(trend);
    double sum = 0.0, mass = 0.0;
    for (std::size_t i = 0; i < 5; ++i) {
      for (std::size_t j = 0; j < 5; ++j) {
        const double weight = risk_weights[i] * trend_weights[j];
        // Brake more for severe predicted curves and approaching curvature.
        const double slow = (0.14 * static_cast<double>(i) +
            0.045 * static_cast<double>(j > 2 ? j - 2 : 0)) * aggression_;
        sum += weight * clamp(1.0 - slow, 0.25, 1.0);
        mass += weight;
      }
    }
    if (mass <= 1e-8) throw std::runtime_error("empty throttle fuzzy memberships");
    return clamp(sum / mass, 0.25, 1.0);
  }
 private:
  static std::array<double, 5> memberships(double value) {
    const double x = clamp(value, -1.0, 1.0);
    constexpr std::array<double, 5> centers{-1.0,-0.5,0.0,0.5,1.0};
    std::array<double, 5> weights{};
    for (std::size_t i = 0; i < centers.size(); ++i)
      weights[i] = std::max(0.0, 1.0 - std::abs(x - centers[i]) / 0.5);
    return weights;
  }
  double aggression_{1.0}, previous_{0.0};
  bool have_previous_{false};
};

// Camera risk for coordinated steering and throttle. No map/pose inputs.
inline double camera_risk(double near_error, double far_error,
                          double signed_curvature, double confidence,
                          double preview_visibility) {
  const double risk = std::max({
      1.25 * std::abs(clamp(near_error, -1.0, 1.0)),
      0.90 * std::abs(clamp(far_error, -1.0, 1.0)),
      0.78 * std::abs(clamp(signed_curvature, -1.0, 1.0)),
      0.48 * (1.0 - clamp(confidence, 0.0, 1.0)),
      0.24 * (1.0 - clamp(preview_visibility, 0.0, 1.0))
  });
  return clamp(risk, 0.0, 1.0);
}

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
    crossing_since_ = -1; last_now_ = -1; latched_intersection_ = false;
  }
  void emergency_stop() { estop_ = true; state_ = State::Estop; }
  std::pair<State, double> update(const Observation& o, double now, bool armed, Limits lim) {
    if (estop_) return {state_ = State::Estop, 0};
    if (fault_latched_ && armed) return {state_ = State::SensorStop, 0};
    if (!armed) {
      fault_latched_ = false;
      state_ = State::Disarmed;
      green_since_ = crossing_since_ = last_now_ = -1;
      latched_intersection_ = false;
      return {state_, 0};
    }
    // Check clock monotonicity before sensor validation. Otherwise a stale
    // sample can hide a rollback, allowing a later valid sample to resume.
    if (!std::isfinite(now) || now < 0.0 ||
        (last_now_ >= 0.0 && now < last_now_)) {
      fault_latched_ = true;
      return {state_ = State::SensorStop, 0};
    }
    last_now_ = now;
    // Reject malformed clocks and sensor ages: negative/NaN ages must never
    // bypass the stale watchdog, especially after a ROS clock reset.
    const auto valid_age = [](double age, double maximum) {
      return std::isfinite(age) && age >= 0.0 && age <= maximum;
    };
    if (!std::isfinite(lim.stale_s) || lim.stale_s <= 0.0 ||
        !std::isfinite(lim.min_confidence) || lim.min_confidence < 0.0 ||
        lim.min_confidence > 1.0 ||
        !std::isfinite(lim.green_hold_s) || lim.green_hold_s <= 0.0 ||
        !std::isfinite(lim.cross_timeout_s) || lim.cross_timeout_s <= 0.0 ||
        !std::isfinite(lim.normal_speed) || lim.normal_speed < 0.0 ||
        !std::isfinite(lim.crossing_speed) || lim.crossing_speed < 0.0 ||
        !std::isfinite(o.error) || !std::isfinite(o.confidence) ||
        o.confidence < lim.min_confidence || o.confidence > 1.0 ||
        !valid_age(o.lane_age, lim.stale_s) ||
        !valid_age(o.hazard_age, lim.stale_s) ||
        ((o.intersection || latched_intersection_) &&
         !valid_age(o.signal_age, lim.stale_s))) {
      // Never automatically resume a crossing after sensor dropout.
      if (state_ == State::Crossing) fault_latched_ = true;
      green_since_ = -1;
      return {state_ = State::SensorStop, 0};
    }
    // Never resume a partially completed crossing automatically after a hazard.
    if (o.hazard) {
      if (state_ == State::Crossing) fault_latched_ = true;
      green_since_ = -1;
      return {state_ = State::ObstacleStop, 0};
    }
    // A red/unknown signal during an active crossing is a fail-closed fault.
    // The operator must disarm before any subsequent movement.
    if (state_ == State::Crossing && o.signal != "GREEN") {
      fault_latched_ = true;
      return {state_ = State::SensorStop, 0};
    }
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
  double green_since_{-1}, crossing_since_{-1}, last_now_{-1};
};
}  // namespace rcauto
