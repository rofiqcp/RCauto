#include "rcauto_cpp/core.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/dnn.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace std::chrono_literals;

namespace {
using Steady = std::chrono::steady_clock;
double age(const Steady::time_point& t) {
  return t == Steady::time_point{} ? std::numeric_limits<double>::infinity()
     : std::chrono::duration<double>(Steady::now() - t).count();
}
}

class ControllerNode final : public rclcpp::Node {
 public:
  ControllerNode() : Node("rcauto_cpp_controller") {
    declare_parameter("controller.mode", "pid");
    declare_parameter("controller.pid.kp", 0.75);
    declare_parameter("controller.pid.ki", 0.06);
    declare_parameter("controller.pid.kd", 0.12);
    declare_parameter("controller.pid.max_output", 0.85);
    declare_parameter("controller.fuzzy.error_weight", 0.65);
    declare_parameter("controller.fuzzy.rate_weight", 0.25);
    declare_parameter("controller.fuzzy.max_output", 0.85);
    declare_parameter("controller.ai.model_path", "");
    declare_parameter("controller.ai.output_gain", 1.0);
    declare_parameter("controller.speed_pid.kp", 1.9);
    declare_parameter("controller.speed_pid.ki", 0.0);
    declare_parameter("controller.speed_pid.kd", 0.06);
    declare_parameter("controller.fuzzy.speed_aggressiveness", 1.0);
    declare_parameter("drive.max_steering_rad", 0.35);
    declare_parameter("drive.steering_scale", 1.0);
    declare_parameter("drive.max_speed_mps", 0.18);
    declare_parameter("drive.follow_speed_mps", 0.18);
    declare_parameter("drive.corner_speed_scale", 1.0);
    declare_parameter("drive.cone_speed_scale", 0.50);
    declare_parameter("drive.steering_deadband_norm", 0.035);
    declare_parameter("drive.steering_slew_rad_s", 2.8);
    declare_parameter("drive.accel_mps2", 0.48);
    declare_parameter("drive.decel_mps2", 1.1);
    declare_parameter("drive.cross_speed_mps", 0.11);
    declare_parameter("safety.stale_s", 0.6);
    declare_parameter("safety.min_lane_confidence", 0.4);
    declare_parameter("safety.green_hold_s", 0.6);
    declare_parameter("safety.cross_timeout_s", 3.0);
    declare_parameter("safety.require_range", false);
    declare_parameter("safety.hardware_mode", false);
    declare_parameter("safety.range_stop_m", 0.50);
    declare_parameter("safety.range_stale_s", 0.4);
    declare_parameter("safety.preview_only", false);
    // Parameter overrides from YAML are declared before callbacks exist.
    // Reject unsafe startup configuration before creating any publishers.
    const auto startup_double = [this](const char* name) {
      const double value = get_parameter(name).as_double();
      if (!std::isfinite(value)) throw std::invalid_argument(std::string("Non-finite startup parameter: ") + name);
      return value;
    };
    const double startup_max = startup_double("drive.max_speed_mps");
    const double startup_follow = startup_double("drive.follow_speed_mps");
    const double startup_cross = startup_double("drive.cross_speed_mps");
    if (startup_max < 0.01 || startup_max > 0.5 ||
        startup_follow < 0.01 || startup_follow > startup_max ||
        startup_cross < 0.01 || startup_cross > startup_max) {
      throw std::invalid_argument("Unsafe startup speed configuration");
    }
    const rcauto::Gains startup_gains{
      startup_double("controller.pid.kp"), startup_double("controller.pid.ki"),
      startup_double("controller.pid.kd"), startup_double("controller.pid.max_output")};
    rcauto::Pid validated_startup_pid(startup_gains);
    rcauto::Pid validated_startup_speed_pid({startup_double("controller.speed_pid.kp"),
      startup_double("controller.speed_pid.ki"), startup_double("controller.speed_pid.kd"),1.0});
    (void)validated_startup_pid;
    (void)validated_startup_speed_pid;
    // YAML and CLI overrides bypass the runtime parameter callback at startup.
    const auto startup_range = [&startup_double](const char* name, double low, double high) {
      const double value = startup_double(name);
      if (value < low || value > high) {
        throw std::invalid_argument(std::string("Unsafe startup parameter: ") + name);
      }
    };
    startup_range("drive.max_steering_rad", 0.05, 0.75);
    startup_range("drive.steering_scale", 0.20, 1.50);
    startup_range("drive.corner_speed_scale", 0.4, 1.0);
    startup_range("drive.cone_speed_scale", 0.2, 1.0);
    startup_range("controller.fuzzy.error_weight", 0.0, 2.0);
    startup_range("controller.fuzzy.rate_weight", 0.0, 2.0);
    startup_range("controller.fuzzy.max_output", std::numeric_limits<double>::min(), 1.0);
    startup_range("controller.ai.output_gain", 0.5, 2.0);
    startup_range("controller.fuzzy.speed_aggressiveness", 0.5, 2.0);
    startup_range("drive.steering_deadband_norm", 0.0, 0.15);
    startup_range("drive.steering_slew_rad_s", 0.1, 8.0);
    startup_range("drive.accel_mps2", 0.05, 3.0);
    startup_range("drive.decel_mps2", 0.1, 5.0);
    startup_range("safety.stale_s", 0.1, 1.0);
    startup_range("safety.min_lane_confidence", 0.1, 1.0);
    startup_range("safety.green_hold_s", 0.2, 5.0);
    startup_range("safety.cross_timeout_s", 1.0, 10.0);
    startup_range("safety.range_stop_m", 0.1, 3.0);
    startup_range("safety.range_stale_s", 0.1, 1.0);
    const auto startup_mode = get_parameter("controller.mode").as_string();
    if (startup_mode != "pid" && startup_mode != "fuzzy" && startup_mode != "ai") {
      throw std::invalid_argument("Unsupported startup controller mode");
    }
    if (get_parameter("safety.preview_only").as_bool() &&
        get_parameter("safety.hardware_mode").as_bool())
      throw std::invalid_argument("Preview-only forbids hardware output");
    if (get_parameter("safety.hardware_mode").as_bool() &&
        !get_parameter("safety.require_range").as_bool()) {
      throw std::invalid_argument("Unsafe startup hardware safety configuration");
    }
    refresh();
    if (startup_mode == "ai" && !ai_ready_) {
      throw std::invalid_argument("AI startup requires a validated ONNX model");
    }
    cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) { return on_parameters(params); });
    const auto qos = rclcpp::QoS(10);
    sub_lane_ = create_subscription<std_msgs::msg::Float32>("/rcauto/lane_error", qos,
      [this](std_msgs::msg::Float32::ConstSharedPtr m) {
        observation_.error = m->data; stamp_lane_ = Steady::now();
      });
    sub_near_ = create_subscription<std_msgs::msg::Float32>("/rcauto/vision/near_error", qos,
      [this](std_msgs::msg::Float32::ConstSharedPtr m) {
        if (std::isfinite(m->data)) { near_error_ = m->data; stamp_preview_ = Steady::now(); }
      });
    sub_far_ = create_subscription<std_msgs::msg::Float32>("/rcauto/vision/far_error", qos,
      [this](std_msgs::msg::Float32::ConstSharedPtr m) {
        if (std::isfinite(m->data)) { far_error_ = m->data; stamp_preview_ = Steady::now(); }
      });
    sub_curvature_ = create_subscription<std_msgs::msg::Float32>("/rcauto/vision/curvature", qos,
      [this](std_msgs::msg::Float32::ConstSharedPtr m) {
        if (std::isfinite(m->data)) curvature_ = m->data;
      });
    sub_preview_ = create_subscription<std_msgs::msg::Float32>("/rcauto/vision/preview_visibility", qos,
      [this](std_msgs::msg::Float32::ConstSharedPtr m) {
        if (std::isfinite(m->data)) preview_visibility_ = m->data;
      });
    sub_avoid_active_ = create_subscription<std_msgs::msg::Bool>(
        "/rcauto/vision/avoidance_active", qos,
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          avoidance_active_=m->data;stamp_avoid_=Steady::now();
        });
    sub_avoid_blocked_ = create_subscription<std_msgs::msg::Bool>(
        "/rcauto/vision/avoidance_blocked", qos,
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          avoidance_blocked_=m->data;stamp_avoid_=Steady::now();
        });
    sub_conf_ = create_subscription<std_msgs::msg::Float32>("/rcauto/lane_confidence", qos,
      [this](std_msgs::msg::Float32::ConstSharedPtr m) {
        observation_.confidence = m->data; stamp_conf_ = Steady::now();
      });
    sub_cross_ = create_subscription<std_msgs::msg::Bool>("/rcauto/intersection", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        observation_.intersection = m->data; stamp_cross_ = Steady::now();
      });
    sub_hazard_ = create_subscription<std_msgs::msg::Bool>("/rcauto/hazard", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        last_vision_hazard_ = m->data; observation_.hazard = m->data; stamp_hazard_ = Steady::now();
      });
    sub_light_ = create_subscription<std_msgs::msg::String>("/rcauto/signal", qos,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        observation_.signal = m->data; stamp_signal_ = Steady::now();
      });
    sub_range_ = create_subscription<sensor_msgs::msg::Range>("/rcauto/range", qos,
      [this](sensor_msgs::msg::Range::ConstSharedPtr m) {
        range_m_ = m->range; range_valid_ = rcauto::valid_range_sample(range_m_, m->min_range, m->max_range) &&
          rcauto::range_covers_stop_zone(m->min_range, m->max_range, stop_range_m_);
        range_min_m_ = m->min_range;
        range_max_m_ = m->max_range;
        stamp_range_ = Steady::now();
      });
    sub_enable_ = create_subscription<std_msgs::msg::Bool>("/rcauto/operator/enable", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (m->data) {
          stamp_operator_ = Steady::now();
          armed_ = true;
        } else {
          disarm();
        }
      });
    sub_mode_ = create_subscription<std_msgs::msg::String>("/rcauto/operator/mode", qos,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        if (m->data != "pid" && m->data != "fuzzy" && m->data != "ai") return;
        if (m->data == "ai" && !ai_ready_) {
          RCLCPP_WARN(get_logger(), "AI ONNX not loaded; rejecting mode change");
          return;
        }
        set_parameter(rclcpp::Parameter("controller.mode", m->data));
      });
    pub_drive_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/rcauto/drive", qos);
    pub_state_ = create_publisher<std_msgs::msg::String>("/rcauto/status", qos);
    pub_mode_ = create_publisher<std_msgs::msg::String>(
        "/rcauto/control/mode", qos);
    pub_steer_ = create_publisher<std_msgs::msg::Float32>("/rcauto/tuning/steer_norm", qos);
    pub_speed_factor_ = create_publisher<std_msgs::msg::Float32>("/rcauto/tuning/speed_factor", qos);
    pub_risk_ = create_publisher<std_msgs::msg::Float32>("/rcauto/tuning/visual_risk", qos);
    pub_preview_speed_ = create_publisher<std_msgs::msg::Float32>("/rcauto/tuning/speed_command_mps", qos);
    timer_ = create_wall_timer(50ms, [this] { tick(); });
    last_tick_ = Steady::now();
    RCLCPP_INFO(get_logger(), "C++ autonomy initialized; DISARMED, mode=%s", mode_.c_str());
  }
 private:
  void disarm() {
    armed_ = false;
    pid_.reset();
    fuzzy_.reset();
    speed_pid_.reset();
    fuzzy_throttle_.reset();
    commanded_speed_ = last_steering_norm_ = 0.0;
  }

  rcl_interfaces::msg::SetParametersResult on_parameters(
      const std::vector<rclcpp::Parameter>& proposed) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    // Detect ambiguous atomic updates in linear time, before effective-value checks.
    std::unordered_set<std::string> seen_names;
    seen_names.reserve(proposed.size());
    for (const auto& parameter : proposed) {
      if (!seen_names.insert(parameter.get_name()).second) {
        result.successful = false;
        result.reason = "Duplicate parameter in atomic update: " + parameter.get_name();
        return result;
      }
    }
    for (const auto& p : proposed) {
      const auto& name = p.get_name();
      if (name.rfind("controller.", 0) != 0 &&
          name.rfind("drive.", 0) != 0 &&
          name.rfind("safety.", 0) != 0) continue;
      // Reject type changes before refresh() reads strongly typed parameters.
      // ROS 2 permits dynamic parameter typing in some configurations.
      if (has_parameter(name) && p.get_type() != get_parameter(name).get_type()) {
        result.successful = false;
        result.reason = "Parameter type change rejected: " + name;
        return result;
      }
      // Live tuning is bounded by the parameter validators below.
      if (name == "safety.hardware_mode" || name == "safety.preview_only" ||
          (hardware_mode_ && name == "safety.require_range")) {
        result.successful = false; result.reason = "Hardware safety mode is read-only";
        return result;
      }
      if (armed_ && name.rfind("safety.", 0) == 0) {
        result.successful = false;
        result.reason = "Sensor safety parameters require STOP";
        return result;
      }
      try {
        if (name == "controller.mode") {
          const auto value = p.as_string();
          if (value != "pid" && value != "fuzzy" && value != "ai")
            throw std::invalid_argument("invalid controller mode");
          if (value == "ai" && !ai_ready_) throw std::invalid_argument("AI model not validated");
        }
        else if (name == "controller.ai.model_path") {
          if (armed_) throw std::invalid_argument("disarm to reload AI");
        }
        else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          const double val = p.as_double();
          if (!std::isfinite(val)) throw std::invalid_argument("parameter must be finite");
          if (name == "controller.pid.kp" || name == "controller.pid.ki" ||
              name == "controller.pid.kd" ||
              name == "controller.speed_pid.kp" || name == "controller.speed_pid.ki" ||
              name == "controller.speed_pid.kd") {
            if (val < 0 || val > 8) throw std::invalid_argument("PID gain out of range [0,8]");
          } else if (name.find("max_output") != std::string::npos) {
            if (val <= 0 || val > 1) throw std::invalid_argument("output limit out of range");
          } else if (name.find("weight") != std::string::npos) {
            if (val < 0 || val > 2) throw std::invalid_argument("fuzzy weight out of range");
          } else if (name == "controller.fuzzy.speed_aggressiveness") {
            if (val < 0.5 || val > 2.0)
              throw std::invalid_argument("fuzzy throttle aggression outside [0.5,2]");
          } else if (name == "controller.ai.output_gain") {
            if (val < 0.5 || val > 2.0)
              throw std::invalid_argument("AI output gain out of range [0.5,2]");
          } else if (name == "drive.steering_deadband_norm") {
            if (val < 0 || val > 0.15) throw std::invalid_argument("invalid steering deadband");
          } else if (name == "drive.steering_slew_rad_s") {
            if (val < 0.1 || val > 8.0) throw std::invalid_argument("invalid steering slew");
          } else if (name == "drive.accel_mps2") {
            if (val < 0.05 || val > 3.0) throw std::invalid_argument("invalid acceleration");
          } else if (name == "drive.decel_mps2") {
            if (val < 0.1 || val > 5.0) throw std::invalid_argument("invalid deceleration");
          } else if (name == "drive.cone_speed_scale") {
            if (val < 0.2 || val > 1.0)
              throw std::invalid_argument("cone speed scale must be 0.2..1.0");
          } else if (name == "drive.corner_speed_scale") {
            if (val < 0.4 || val > 1.0)
              throw std::invalid_argument("corner speed scale outside [0.4,1]");
          } else if (name == "drive.steering_scale") {
            if(val<0.20 || val>1.50)
              throw std::invalid_argument("steering scale must be 0.2..1.5");
          } else if (name == "drive.max_steering_rad") {
            if (val < 0.05 || val > 0.75) throw std::invalid_argument("unsafe steering angle");
          } else if (name.find("speed_mps") != std::string::npos) {
            if (val < 0.01 || val > 0.5) throw std::invalid_argument("unsafe speed");
          } else if (name == "safety.min_lane_confidence") {
            if (val < 0.1 || val > 1) throw std::invalid_argument("confidence bounds [0.1,1]");
          } else if (name == "safety.stale_s" || name == "safety.range_stale_s") {
            if (val < 0.1 || val > 1) throw std::invalid_argument("watchdog bounds [0.1,1]");
          } else if (name == "safety.range_stop_m") {
            if (val < 0.1 || val > 3.0) throw std::invalid_argument("distance bounds");
          } else if (name == "safety.green_hold_s") {
            if (val < 0.2 || val > 5.0) throw std::invalid_argument("green hold bounds");
          } else if (name == "safety.cross_timeout_s") {
            if (val < 1.0 || val > 10.0) throw std::invalid_argument("cross timeout bounds");
          }
        }
      } catch (const std::exception& e) {
        result.successful = false; result.reason = e.what(); return result;
      }
    }
    // Validate the effective configuration, including atomic multi-parameter edits.
    auto effective = [this, &proposed](const char* key) {
      for (const auto& p : proposed)
        if (p.get_name() == key) return p.as_double();
      return get_parameter(key).as_double();
    };
    const double max_speed = effective("drive.max_speed_mps");
    const double follow_speed = effective("drive.follow_speed_mps");
    const double cross_speed = effective("drive.cross_speed_mps");
    if (follow_speed > max_speed || cross_speed > max_speed) {
      result.successful = false;
      result.reason = "Follow and crossing speeds must not exceed drive.max_speed_mps";
      return result;
    }
    // rclcpp invokes callbacks before updating parameter storage.
    // Apply values in post callback via pending flag, read on next tick.
    refresh_pending_ = true;
    return result;
  }

  void refresh() {
    const auto requested_mode = get_parameter("controller.mode").as_string();
    if (mode_ != requested_mode) {
      pid_.reset(); fuzzy_.reset(); speed_pid_.reset(); fuzzy_throttle_.reset();
      have_error_ = false;
    }
    mode_ = requested_mode;
    ai_gain_ = get_parameter("controller.ai.output_gain").as_double();
    steer_deadband_ = get_parameter("drive.steering_deadband_norm").as_double();
    steer_slew_ = get_parameter("drive.steering_slew_rad_s").as_double();
    acceleration_ = get_parameter("drive.accel_mps2").as_double();
    deceleration_ = get_parameter("drive.decel_mps2").as_double();
    max_steer_ = get_parameter("drive.max_steering_rad").as_double();
    steering_scale_=get_parameter("drive.steering_scale").as_double();
    max_speed_ = get_parameter("drive.max_speed_mps").as_double();
    corner_speed_scale_ = get_parameter("drive.corner_speed_scale").as_double();
    cone_speed_scale_ = get_parameter("drive.cone_speed_scale").as_double();
    limits_.normal_speed = get_parameter("drive.follow_speed_mps").as_double();
    limits_.crossing_speed = get_parameter("drive.cross_speed_mps").as_double();
    limits_.stale_s = get_parameter("safety.stale_s").as_double();
    limits_.min_confidence = get_parameter("safety.min_lane_confidence").as_double();
    limits_.green_hold_s = get_parameter("safety.green_hold_s").as_double();
    limits_.cross_timeout_s = get_parameter("safety.cross_timeout_s").as_double();
    hardware_mode_ = get_parameter("safety.hardware_mode").as_bool();
    preview_only_ = get_parameter("safety.preview_only").as_bool();
    require_range_ = get_parameter("safety.require_range").as_bool() || hardware_mode_;
    stop_range_m_ = get_parameter("safety.range_stop_m").as_double();
    range_stale_s_ = get_parameter("safety.range_stale_s").as_double();
    rcauto::Gains gains;
    gains.kp = get_parameter("controller.pid.kp").as_double();
    gains.ki = get_parameter("controller.pid.ki").as_double();
    gains.kd = get_parameter("controller.pid.kd").as_double();
    gains.max_output = get_parameter("controller.pid.max_output").as_double();
    pid_.set_gains(gains);
    speed_pid_.set_gains({get_parameter("controller.speed_pid.kp").as_double(),
       get_parameter("controller.speed_pid.ki").as_double(),
       get_parameter("controller.speed_pid.kd").as_double(), 1.0});
    fuzzy_throttle_.set_aggressiveness(
       get_parameter("controller.fuzzy.speed_aggressiveness").as_double());
    fuzzy_.set(get_parameter("controller.fuzzy.error_weight").as_double(),
               get_parameter("controller.fuzzy.rate_weight").as_double(),
               get_parameter("controller.fuzzy.max_output").as_double());
    const auto path = get_parameter("controller.ai.model_path").as_string();
    if (path != last_model_) {
      last_model_ = path;
      ai_ready_ = false;
      if (!path.empty()) {
        try {
          ai_net_ = cv::dnn::readNetFromONNX(path);
          float example[] = {0.0F, 0.0F, 0.0F, 0.0F, 0.24F};
          ai_net_.setInput(cv::Mat(1, 5, CV_32F, example).clone());
          cv::Mat out = ai_net_.forward();
          ai_ready_ = out.rows == 1 && out.cols == 2 &&
              std::isfinite(out.at<float>(0,0)) && std::isfinite(out.at<float>(0,1)) &&
              out.at<float>(0,1) >= 0.20F && out.at<float>(0,1) <= 1.05F;
          if (ai_ready_) RCLCPP_INFO(get_logger(), "ONNX policy loaded");
        } catch (const cv::Exception& e) {
          RCLCPP_ERROR(get_logger(), "Cannot load ONNX model: %s", e.what());
        }
      }
    }
  }

  std::pair<double, double> ai_step(double near_error, double far_error,
                                    double curvature, double visual_risk, double dt) {
    rcauto::validate_dt(near_error, dt);
    if (!ai_ready_) throw std::runtime_error("Dual-output AI model unavailable");
    const double error = rcauto::clamp(0.30 * near_error + 0.70 * far_error, -1, 1);
    const double rate = have_error_
        ? rcauto::clamp((error - prev_error_) / dt / 3.0, -1, 1) : 0;
    prev_error_ = error;
    have_error_ = true;
    float input[] = {static_cast<float>(near_error),
                           static_cast<float>(far_error),
                           static_cast<float>(curvature),
                           static_cast<float>(rate),
                           static_cast<float>(visual_risk)};
    ai_net_.setInput(cv::Mat(1, 5, CV_32F, input).clone());
    const cv::Mat out = ai_net_.forward();
    if (out.rows != 1 || out.cols != 2 ||
        !std::isfinite(out.at<float>(0, 0)) ||
        !std::isfinite(out.at<float>(0, 1)))
      throw std::runtime_error("AI must return finite steering and throttle");
    const double steering = rcauto::clamp(
        ai_gain_ * static_cast<double>(out.at<float>(0, 0)), -1, 1);
    const double throttle = rcauto::clamp(
        static_cast<double>(out.at<float>(0, 1)), 0.25, 1.0);
    return {steering, throttle};
  }

  void tick() {
    if (armed_ && age(stamp_operator_) > 1.25) {
      disarm();
      RCLCPP_WARN(get_logger(), "Operator heartbeat timeout: output STOP");
    }
    if (refresh_pending_) {
      refresh_pending_ = false;
      refresh();
      have_error_ = false;
    }
    const auto now = Steady::now();
    const double dt = rcauto::clamp(std::chrono::duration<double>(now - last_tick_).count(),
                                   0.001, 0.25);
    last_tick_ = now;
    observation_.lane_age = std::max({age(stamp_lane_), age(stamp_conf_), age(stamp_cross_)});
    observation_.hazard_age = age(stamp_hazard_);
    observation_.signal_age = age(stamp_signal_);
    if (preview_only_) {
      // Preview displays controller output for tuning, without launching
      // a motor actuator or falsely claiming obstacle-sensor coverage.
      observation_.hazard = false;
      observation_.hazard_age = 0.0;
      observation_.signal = "GREEN";
      observation_.signal_age = 0.0;
    }
    const bool range_stale = require_range_ &&
                !rcauto::range_sample_healthy(age(stamp_range_), range_stale_s_, range_valid_,
                                              range_min_m_, range_max_m_, stop_range_m_);
    const bool range_obstacle = require_range_ && !range_stale && range_m_ < stop_range_m_;
    // Recompute each cycle: a cleared Range obstacle must not persist as a phantom hazard.
    if (!preview_only_)
      observation_.hazard = rcauto::combined_hazard(last_vision_hazard_, range_obstacle);
    const bool avoidance_fresh=age(stamp_avoid_)<limits_.stale_s;
    const bool cone_avoiding=avoidance_fresh && avoidance_active_;
    const bool cone_blocked=avoidance_fresh && avoidance_blocked_;
    if(cone_blocked) {
      // No verified clearance: refuse motor movement even in preview.
      observation_.hazard=true;
      observation_.hazard_age=0.0;
    }
    if (range_stale) observation_.hazard_age = std::numeric_limits<double>::infinity();
    auto [state, speed] = fsm_.update(observation_,
      std::chrono::duration<double>(now.time_since_epoch()).count(), armed_, limits_);
    const bool have_preview = age(stamp_preview_) < limits_.stale_s;
    const double near_error = have_preview ? near_error_ : observation_.error;
    const double far_error = have_preview ? far_error_ : observation_.error;
    const double curvature = have_preview ? curvature_ : 0.0;
    const double visibility = have_preview ? preview_visibility_ : 0.0;
    const double risk = rcauto::camera_risk(
        near_error, far_error, curvature, observation_.confidence, visibility);
    // Near preview maintains lateral position; far preview anticipates
    // the upcoming bend. Far-only control drifts toward the road boundary.
    const double fused_error = rcauto::clamp(
        0.62 * near_error + 0.38 * far_error, -1, 1);
    double steering = 0.0;
    double throttle_factor = 0.0;
    if (speed > 0) {
      try {
        if (mode_ == "pid") {
          steering = pid_.calculate(fused_error, dt);
          throttle_factor = speed_pid_.calculate(risk, dt);
        } else if (mode_ == "fuzzy") {
          steering = fuzzy_.calculate(fused_error, dt);
          throttle_factor = fuzzy_throttle_.calculate(risk, dt);
        } else if (mode_ == "ai") {
          const auto command = ai_step(near_error, far_error, curvature, risk, dt);
          steering = command.first;
          throttle_factor = command.second;
        } else {
          throw std::runtime_error("Invalid controller mode");
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                              "Dual-output controller failure: %s", e.what());
        armed_ = false;
        speed = steering = throttle_factor = 0;
        state = rcauto::State::SensorStop;
      }
    }
    double target_speed = 0.0;
    if (speed > 0.0) {
      // The operator-selected ceiling is never overridden by a controller.
      // A shared camera-based envelope limits speed before sharp turns.
      const double camera_envelope = std::clamp(
          1.0 - 0.65 * std::max(0.0, risk - 0.12) / 0.88, 0.32, 1.0);
      throttle_factor = std::min(throttle_factor, camera_envelope);
      if (visibility < 0.12) throttle_factor = std::min(throttle_factor, 0.73);
      if (observation_.confidence < 0.65)
        throttle_factor = std::min(throttle_factor, 0.50);
      throttle_factor = std::clamp(throttle_factor, corner_speed_scale_ * 0.65, 1.0);
      target_speed = std::min(speed, max_speed_) * throttle_factor;
      if(cone_avoiding)target_speed*=cone_speed_scale_;
      // Tiny visual bends should not generate oscillating servo commands.
      if (std::abs(fused_error) < steer_deadband_ &&
          std::abs(curvature) < steer_deadband_ * 1.7)
        steering = 0.0;
      // Apply steering gain independently of controller choice,
      // then obey the configured physical angle and slew ceiling.
      steering=std::clamp(steering*steering_scale_,-1.0,1.0);
      steering = std::clamp(steering,
          last_steering_norm_ - steer_slew_ * dt / max_steer_,
          last_steering_norm_ + steer_slew_ * dt / max_steer_);
      last_steering_norm_ = steering;
      commanded_speed_ += std::clamp(target_speed - commanded_speed_,
                                     -deceleration_ * dt, acceleration_ * dt);
    } else {
      commanded_speed_ = 0;
      last_steering_norm_ = 0;
      speed_pid_.reset();
      fuzzy_throttle_.reset();
      have_error_ = false;
    }
    ackermann_msgs::msg::AckermannDriveStamped drive;
    drive.header.stamp = get_clock()->now();
    drive.header.frame_id = "base_link";
    drive.drive.steering_angle = static_cast<float>(steering * max_steer_);
    drive.drive.speed = static_cast<float>(commanded_speed_);
    pub_drive_->publish(drive);
    std_msgs::msg::Float32 steering_msg;
    steering_msg.data = static_cast<float>(steering);
    pub_steer_->publish(steering_msg);
    const auto emit = [](const auto& publisher, double value) {
      std_msgs::msg::Float32 m;
      m.data = static_cast<float>(value);
      publisher->publish(m);
    };
    emit(pub_speed_factor_, throttle_factor);
    emit(pub_risk_, risk);
    emit(pub_preview_speed_, commanded_speed_);
    std_msgs::msg::String status;
    std::ostringstream s;
    s << (preview_only_ ? "PREVIEW " : "") << rcauto::state_name(state)
      << " | mode=" << mode_ << " | armed=" << armed_
      << " | error=" << observation_.error << " | hazard=" << observation_.hazard
      << " | lane_age=" << observation_.lane_age
      << " | cone_avoid=" << cone_avoiding
      << " | cone_blocked=" << cone_blocked
      << " | range_m=" << range_m_;
    status.data = s.str();
    pub_state_->publish(status);
    std_msgs::msg::String mode_message;
    mode_message.data=mode_;
    pub_mode_->publish(mode_message);
    if (state != last_state_) {
      RCLCPP_INFO(get_logger(), "%s", status.data.c_str());
      last_state_ = state;
    }
  }

  rcauto::Pid pid_;
  rcauto::VisualSpeedPid speed_pid_;
  rcauto::Fuzzy fuzzy_;
  rcauto::FuzzyThrottle fuzzy_throttle_;
  rcauto::StateMachine fsm_;
  rcauto::Observation observation_;
  rcauto::Limits limits_;
  rcauto::State last_state_{rcauto::State::Disarmed};
  cv::dnn::Net ai_net_;
  bool ai_ready_{false}, armed_{false}, have_error_{false},
       refresh_pending_{false}, require_range_{false}, hardware_mode_{false},
       preview_only_{false},
       range_valid_{false}, last_vision_hazard_{true},
       avoidance_active_{false}, avoidance_blocked_{false};
  double prev_error_{0}, range_m_{std::numeric_limits<double>::quiet_NaN()},
    range_min_m_{std::numeric_limits<double>::quiet_NaN()},
    range_max_m_{std::numeric_limits<double>::quiet_NaN()},
    max_steer_{0.35}, steering_scale_{1.0}, max_speed_{0.18}, stop_range_m_{0.5}, range_stale_s_{0.4},
    ai_gain_{1.0}, corner_speed_scale_{1.0}, cone_speed_scale_{0.50},
    commanded_speed_{0.0},
    last_steering_norm_{0.0}, steer_deadband_{0.035}, steer_slew_{2.8},
    acceleration_{0.48}, deceleration_{1.1},
    near_error_{0.0}, far_error_{0.0}, curvature_{0.0}, preview_visibility_{0.0};
  std::string mode_{"pid"}, last_model_;
  Steady::time_point stamp_lane_{}, stamp_conf_{}, stamp_cross_{}, stamp_hazard_{},
                     stamp_signal_{}, stamp_range_{}, stamp_preview_{},
                     stamp_avoid_{}, stamp_operator_{}, last_tick_{};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_handle_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      sub_lane_, sub_conf_, sub_near_, sub_far_, sub_curvature_, sub_preview_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cross_, sub_hazard_,
    sub_enable_, sub_avoid_active_,sub_avoid_blocked_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_light_, sub_mode_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_range_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_,pub_mode_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      pub_steer_, pub_speed_factor_, pub_risk_, pub_preview_speed_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<ControllerNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("rcauto_cpp_controller"),
                 "Controller startup/runtime failure; outputs remain disabled: %s", error.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return exit_code;
}
