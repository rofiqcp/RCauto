#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace {
struct Waypoint { double x{}; double y{}; };
using Clock = std::chrono::steady_clock;

std::vector<Waypoint> read_waypoints(const std::string& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Cannot open F1TENTH centerline: " + path);
  std::vector<Waypoint> waypoints;
  std::string line;
  std::getline(file, line);
  while (std::getline(file, line)) {
    std::istringstream row(line);
    std::string sx, sy;
    if (!std::getline(row, sx, ',') || !std::getline(row, sy, ',')) continue;
    try {
      Waypoint point{std::stod(sx), std::stod(sy)};
      if (std::isfinite(point.x) && std::isfinite(point.y)) {
        waypoints.push_back(point);
      }
    } catch (const std::exception&) {
      throw std::runtime_error("Invalid centerline coordinate in " + path);
    }
  }
  if (waypoints.size() < 20) throw std::runtime_error("Centerline has too few points");
  return waypoints;
}
}

class F1TenthAdapter final : public rclcpp::Node {
 public:
  F1TenthAdapter() : Node("rcauto_f1tenth_adapter") {
    const auto path = declare_parameter<std::string>("centerline_path", "");
    if (path.empty()) throw std::invalid_argument("centerline_path must be set");
    path_ = read_waypoints(path);
    lookahead_ = declare_parameter<double>("lookahead_m", 0.24);
    lane_max_ = declare_parameter<double>("lane_max_offset_m", 0.23);
    timeout_s_ = declare_parameter<double>("command_timeout_s", 0.35);
    if (lookahead_ <= 0 || lane_max_ <= 0 || timeout_s_ <= 0) {
      throw std::invalid_argument("F1TENTH adapter parameters must be positive");
    }
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/ego_racecar/drive", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/rcauto/odom", 10);
    lane_pub_ = create_publisher<std_msgs::msg::Float32>("/rcauto/lane_error", 10);
    confidence_pub_ = create_publisher<std_msgs::msg::Float32>("/rcauto/lane_confidence", 10);
    intersection_pub_ = create_publisher<std_msgs::msg::Bool>("/rcauto/intersection", 10);
    hazard_pub_ = create_publisher<std_msgs::msg::Bool>("/rcauto/hazard", 10);
    signal_pub_ = create_publisher<std_msgs::msg::String>("/rcauto/signal", 10);
    range_pub_ = create_publisher<sensor_msgs::msg::Range>("/rcauto/range", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          const auto& p = m->pose.pose.position;
          const auto& q = m->pose.pose.orientation;
          if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
              !std::isfinite(q.x) || !std::isfinite(q.y) ||
              !std::isfinite(q.z) || !std::isfinite(q.w)) return;
          x_ = p.x;
          y_ = p.y;
          yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
          last_odom_ = Clock::now();
          received_odom_ = true;
          odom_pub_->publish(*m);
        });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/ego_racecar/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
          sensor_msgs::msg::Range value;
          value.header = scan->header;
          value.radiation_type = sensor_msgs::msg::Range::INFRARED;
          value.field_of_view = 0.30F;
          value.min_range = std::max(0.01F, scan->range_min);
          value.max_range = std::max(1.0F, scan->range_max);
          float nearest = value.max_range * 0.99F;
          for (size_t i = 0; i < scan->ranges.size(); ++i) {
            const double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
            if (std::abs(angle) > 0.15) continue;
            const float range = scan->ranges[i];
            if (std::isfinite(range) && range >= value.min_range &&
                range <= value.max_range) nearest = std::min(nearest, range);
          }
          value.range = nearest;
          range_pub_->publish(value);
        });
    controller_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/rcauto/drive", 10,
        [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr m) {
          const double speed = m->drive.speed;
          const double angle = m->drive.steering_angle;
          command_.drive.speed = static_cast<float>(
              std::isfinite(speed) ? std::clamp(speed, -0.25, 0.25) : 0.0);
          command_.drive.steering_angle = static_cast<float>(
              std::isfinite(angle) ? std::clamp(angle, -0.43, 0.43) : 0.0);
          received_command_ = true;
          last_command_ = Clock::now();
        });
    timer_ = create_wall_timer(50ms, [this] { publish_state(); });
    RCLCPP_INFO(get_logger(), "F1TENTH native physics adapter: %zu waypoints; physical PWM off",
                path_.size());
  }

 private:
  void publish_state() {
    const auto now = Clock::now();
    const auto fresh = [now](Clock::time_point stamp, double limit) {
      return std::chrono::duration<double>(now - stamp).count() < limit;
    };
    const bool valid = received_odom_ && fresh(last_odom_, 0.5);
    std_msgs::msg::Float32 error, confidence;
    if (valid) {
      size_t nearest = 0;
      double best = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < path_.size(); ++i) {
        const double dx = path_[i].x - x_;
        const double dy = path_[i].y - y_;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best) { best = d2; nearest = i; }
      }
      double travelled = 0.0;
      size_t target = nearest;
      while (travelled < lookahead_) {
        const size_t next = (target + 1) % path_.size();
        const double dx = path_[next].x - path_[target].x;
        const double dy = path_[next].y - path_[target].y;
        travelled += std::hypot(dx, dy);
        target = next;
        if (target == nearest) break;
      }
      const double dx = path_[target].x - x_;
      const double dy = path_[target].y - y_;
      const double forward = std::cos(yaw_) * dx + std::sin(yaw_) * dy;
      const double lateral = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;
      error.data = static_cast<float>(
          std::clamp(std::atan2(lateral, std::max(0.04, forward)) / 0.6, -1.0, 1.0));
      confidence.data = std::sqrt(best) < lane_max_ ? 0.90F : 0.0F;
    }
    lane_pub_->publish(error);
    confidence_pub_->publish(confidence);
    std_msgs::msg::Bool crossing, hazard;
    crossing.data = false;
    hazard.data = !valid;
    intersection_pub_->publish(crossing);
    hazard_pub_->publish(hazard);
    std_msgs::msg::String signal;
    signal.data = "GREEN";
    signal_pub_->publish(signal);
    auto drive = command_;
    if (!valid || !received_command_ || !fresh(last_command_, timeout_s_)) {
      drive.drive.speed = 0.0;
      drive.drive.steering_angle = 0.0;
    }
    drive.header.stamp = get_clock()->now();
    drive_pub_->publish(drive);
  }

  std::vector<Waypoint> path_;
  double lookahead_{0.24};
  double lane_max_{0.23};
  double timeout_s_{0.35};
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  bool received_odom_{false}, received_command_{false};
  Clock::time_point last_odom_{}, last_command_{};
  ackermann_msgs::msg::AckermannDriveStamped command_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lane_pub_, confidence_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr intersection_pub_, hazard_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr signal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr controller_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<F1TenthAdapter>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("rcauto_f1tenth_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
