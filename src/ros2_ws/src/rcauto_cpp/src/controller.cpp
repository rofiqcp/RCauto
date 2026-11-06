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
    declare_parameter("drive.max_steering_rad", 0.35);
    declare_parameter("drive.max_speed_mps", 0.18);
    declare_parameter("drive.follow_speed_mps", 0.18);
    declare_parameter("drive.cross_speed_mps", 0.11);
    declare_parameter("safety.stale_s", 0.6);
    declare_parameter("safety.min_lane_confidence", 0.4);
    declare_parameter("safety.green_hold_s", 0.6);
    declare_parameter("safety.cross_timeout_s", 3.0);
    declare_parameter("safety.require_range", false);
    declare_parameter("safety.range_stop_m", 0.50);
    declare_parameter("safety.range_stale_s", 0.4);
    declare_parameter("safety.allow_live_tuning", false);
    refresh();
    cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) { return on_parameters(params); });
    const auto qos = rclcpp::QoS(10);
    sub_lane_ = create_subscription<std_msgs::msg::Float32>("/rcauto/lane_error", qos,
      [this](std_msgs::msg::Float32::ConstSharedPtr m) {
        observation_.error = m->data; stamp_lane_ = Steady::now();
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
        range_m_ = m->range; range_valid_ = std::isfinite(range_m_) &&
                         range_m_ >= m->min_range && range_m_ <= m->max_range;
        stamp_range_ = Steady::now();
      });
    sub_enable_ = create_subscription<std_msgs::msg::Bool>("/rcauto/operator/enable", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (m->data && !estop_) armed_ = true;
        else { armed_ = false; pid_.reset(); fuzzy_.reset(); }
      });
    sub_estop_ = create_subscription<std_msgs::msg::Bool>("/rcauto/operator/estop", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (m->data) { estop_ = true; armed_ = false; fsm_.emergency_stop(); }
      });
    sub_reset_ = create_subscription<std_msgs::msg::Bool>("/rcauto/operator/reset_estop", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (m->data && !armed_) { estop_ = false; fsm_.reset_disarmed(); }
      });
    sub_mode_ = create_subscription<std_msgs::msg::String>("/rcauto/operator/mode", qos,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        if (armed_) { RCLCPP_WARN(get_logger(), "Disarm before changing mode"); return; }
        if (m->data != "pid" && m->data != "fuzzy" && m->data != "ai") return;
        if (m->data == "ai" && !ai_ready_) {
          RCLCPP_WARN(get_logger(), "AI ONNX not loaded; rejecting mode change");
          return;
        }
        set_parameter(rclcpp::Parameter("controller.mode", m->data));
      });
    pub_drive_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/rcauto/drive", qos);
    pub_state_ = create_publisher<std_msgs::msg::String>("/rcauto/status", qos);
    pub_steer_ = create_publisher<std_msgs::msg::Float32>("/rcauto/tuning/steer_norm", qos);
    timer_ = create_wall_timer(50ms, [this] { tick(); });
    last_tick_ = Steady::now();
    RCLCPP_INFO(get_logger(), "C++ autonomy initialized; DISARMED, mode=%s", mode_.c_str());
  }
 private:
  rcl_interfaces::msg::SetParametersResult on_parameters(
      const std::vector<rclcpp::Parameter>& proposed) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& p : proposed) {
      const auto& name = p.get_name();
      if (name.rfind("controller.", 0) != 0 &&
          name.rfind("drive.", 0) != 0 &&
          name.rfind("safety.", 0) != 0) continue;
      // The UI displays all parameters; edits while armed require an explicit safe protocol.
      if (armed_ && !allow_live_tuning_) {
        result.successful = false; result.reason = "DISARM before tuning parameters";
        return result;
      }
      if (armed_ && (name.rfind("drive.", 0) == 0 ||
                    name.rfind("safety.", 0) == 0 ||
                    name == "controller.mode" ||
                    name == "controller.ai.model_path")) {
        result.successful = false;
        result.reason = "Safety, mode, motor limits and AI path are immutable while armed";
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
              name == "controller.pid.kd") {
            if (val < 0 || val > 8) throw std::invalid_argument("PID gain out of range [0,8]");
          } else if (name.find("max_output") != std::string::npos) {
            if (val <= 0 || val > 1) throw std::invalid_argument("output limit out of range");
          } else if (name.find("weight") != std::string::npos) {
            if (val < 0 || val > 2) throw std::invalid_argument("fuzzy weight out of range");
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
    // rclcpp invokes callbacks before updating parameter storage.
    // Apply values in post callback via pending flag, read on next tick.
    refresh_pending_ = true;
    return result;
  }

  void refresh() {
    mode_ = get_parameter("controller.mode").as_string();
    max_steer_ = get_parameter("drive.max_steering_rad").as_double();
    max_speed_ = get_parameter("drive.max_speed_mps").as_double();
    limits_.normal_speed = get_parameter("drive.follow_speed_mps").as_double();
    limits_.crossing_speed = get_parameter("drive.cross_speed_mps").as_double();
    limits_.stale_s = get_parameter("safety.stale_s").as_double();
    limits_.min_confidence = get_parameter("safety.min_lane_confidence").as_double();
    limits_.green_hold_s = get_parameter("safety.green_hold_s").as_double();
    limits_.cross_timeout_s = get_parameter("safety.cross_timeout_s").as_double();
    require_range_ = get_parameter("safety.require_range").as_bool();
    stop_range_m_ = get_parameter("safety.range_stop_m").as_double();
    range_stale_s_ = get_parameter("safety.range_stale_s").as_double();
    allow_live_tuning_ = get_parameter("safety.allow_live_tuning").as_bool();
    rcauto::Gains gains;
    gains.kp = get_parameter("controller.pid.kp").as_double();
    gains.ki = get_parameter("controller.pid.ki").as_double();
    gains.kd = get_parameter("controller.pid.kd").as_double();
    gains.max_output = get_parameter("controller.pid.max_output").as_double();
    pid_.set_gains(gains);
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
          float example[] = {0.0F, 0.0F};
          ai_net_.setInput(cv::Mat(1, 2, CV_32F, example).clone());
          cv::Mat out = ai_net_.forward();
          ai_ready_ = !out.empty() && std::isfinite(out.at<float>(0));
          if (ai_ready_) RCLCPP_INFO(get_logger(), "ONNX policy loaded");
        } catch (const cv::Exception& e) {
          RCLCPP_ERROR(get_logger(), "Cannot load ONNX model: %s", e.what());
        }
      }
    }
  }

  double ai_step(double error, double dt) {
    rcauto::validate_dt(error, dt);
    if (!ai_ready_) throw std::runtime_error("AI model unavailable");
    const double rate = have_error_ ? rcauto::clamp((error - prev_error_) / dt / 3, -1, 1) : 0;
    prev_error_ = error; have_error_ = true;
    float input[] = {static_cast<float>(error), static_cast<float>(rate)};
    ai_net_.setInput(cv::Mat(1, 2, CV_32F, input).clone());
    cv::Mat out = ai_net_.forward();
    if (out.empty() || !std::isfinite(out.at<float>(0)))
      throw std::runtime_error("AI returned invalid steering");
    return rcauto::clamp(static_cast<double>(out.at<float>(0)), -0.85, 0.85);
  }

  void tick() {
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
    const bool range_stale = require_range_ &&
                (age(stamp_range_) > range_stale_s_ || !range_valid_);
    const bool range_obstacle = require_range_ && range_valid_ && range_m_ < stop_range_m_;
    if (range_stale) observation_.hazard_age = std::numeric_limits<double>::infinity();
    if (range_obstacle) observation_.hazard = true;
    auto [state, speed] = fsm_.update(observation_,
      std::chrono::duration<double>(now.time_since_epoch()).count(), armed_, limits_);
    double steering = 0.0;
    if (speed > 0) {
      try {
        if (mode_ == "pid") steering = pid_.calculate(observation_.error, dt);
        else if (mode_ == "fuzzy") steering = fuzzy_.calculate(observation_.error, dt);
        else if (mode_ == "ai") steering = ai_step(observation_.error, dt);
        else throw std::runtime_error("invalid mode");
      } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Controller error: %s", e.what());
        armed_ = false; speed = 0; steering = 0; state = rcauto::State::SensorStop;
      }
    }
    ackermann_msgs::msg::AckermannDriveStamped drive;
    drive.header.stamp = get_clock()->now();
    drive.header.frame_id = "base_link";
    drive.drive.steering_angle = static_cast<float>(steering * max_steer_);
    drive.drive.speed = static_cast<float>(std::min(speed, max_speed_));
    pub_drive_->publish(drive);
    std_msgs::msg::Float32 steering_msg;
    steering_msg.data = static_cast<float>(steering);
    pub_steer_->publish(steering_msg);
    std_msgs::msg::String status;
    std::ostringstream s;
    s << rcauto::state_name(state) << " | mode=" << mode_ << " | armed=" << armed_
      << " | error=" << observation_.error << " | hazard=" << observation_.hazard
      << " | lane_age=" << observation_.lane_age << " | range_m=" << range_m_;
    status.data = s.str();
    pub_state_->publish(status);
    if (state != last_state_) {
      RCLCPP_INFO(get_logger(), "%s", status.data.c_str());
      last_state_ = state;
    }
    // Raw vision hazard is refreshed by perception node; do not latch on transient range
    // samples inside shared Observation.
    if (range_obstacle) observation_.hazard = last_vision_hazard_;
  }

  rcauto::Pid pid_;
  rcauto::Fuzzy fuzzy_;
  rcauto::StateMachine fsm_;
  rcauto::Observation observation_;
  rcauto::Limits limits_;
  rcauto::State last_state_{rcauto::State::Disarmed};
  cv::dnn::Net ai_net_;
  bool ai_ready_{false}, armed_{false}, estop_{false}, have_error_{false},
       refresh_pending_{false}, require_range_{false}, allow_live_tuning_{false},
       range_valid_{false}, last_vision_hazard_{true};
  double prev_error_{0}, range_m_{std::numeric_limits<double>::quiet_NaN()},
    max_steer_{0.35}, max_speed_{0.18}, stop_range_m_{0.5}, range_stale_s_{0.4};
  std::string mode_{"pid"}, last_model_;
  Steady::time_point stamp_lane_{}, stamp_conf_{}, stamp_cross_{}, stamp_hazard_{},
                     stamp_signal_{}, stamp_range_{}, last_tick_{};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_handle_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_lane_, sub_conf_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cross_, sub_hazard_,
    sub_enable_, sub_estop_, sub_reset_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_light_, sub_mode_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_range_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
