#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

using namespace std::chrono_literals;

class GazeboAdapter final : public rclcpp::Node {
 public:
  GazeboAdapter() : Node("rcauto_gazebo_adapter") {
    wheelbase_ = declare_parameter<double>("wheelbase_m", 0.244);
    steering_limit_ = declare_parameter<double>("steering_limit_rad", 0.35);
    command_timeout_ = declare_parameter<double>("command_timeout_s", 0.35);
    front_cone_ = declare_parameter<double>("front_cone_rad", 0.45);
    if (wheelbase_ <= 0.0 || steering_limit_ <= 0.0 || command_timeout_ <= 0.0 ||
        front_cone_ <= 0.0) {
      throw std::invalid_argument("Gazebo adapter parameters must be positive");
    }
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/rcauto/gz/cmd_vel", 10);
    range_pub_ = create_publisher<sensor_msgs::msg::Range>("/rcauto/range", 10);
    // The virtual arena has no object classifier. Keep the camera-detection
    // stream fresh without inventing objects; LiDAR still provides stop range.
    // This adapter runs only in the Gazebo launch, never on the physical Pi.
    detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
        "/rcauto/detections", 10);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/rcauto/camera/image", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr image) {
          vision_msgs::msg::Detection2DArray detections;
          detections.header = image->header;
          detections_pub_->publish(detections);
        });
    drive_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/rcauto/drive", 10,
        [this](ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr message) {
          const double speed = message->drive.speed;
          const double steer = message->drive.steering_angle;
          if (!std::isfinite(speed) || !std::isfinite(steer)) {
            speed_ = 0.0;
            yaw_rate_ = 0.0;
          } else {
            speed_ = std::clamp(speed, -2.0, 2.0);
            yaw_rate_ = speed_ * std::tan(
                std::clamp(steer, -steering_limit_, steering_limit_)) / wheelbase_;
          }
          last_command_ = std::chrono::steady_clock::now();
          have_command_ = true;
        });
    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/rcauto/lidar/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
          auto range = sensor_msgs::msg::Range();
          range.header = scan->header;
          range.radiation_type = sensor_msgs::msg::Range::INFRARED;
          range.field_of_view = static_cast<float>(front_cone_);
          range.min_range = std::max(scan->range_min, 0.01F);
          range.max_range = scan->range_max;
          float closest = range.max_range * 0.99F;
          for (size_t i = 0; i < scan->ranges.size(); ++i) {
            const double angle = static_cast<double>(scan->angle_min) +
                                 static_cast<double>(i) * scan->angle_increment;
            if (std::abs(angle) > front_cone_ / 2.0) continue;
            const float value = scan->ranges[i];
            if (std::isfinite(value) && value >= range.min_range &&
                value <= range.max_range) {
              closest = std::min(closest, value);
            }
          }
          range.range = closest;
          range_pub_->publish(range);
        });
    // Gazebo odometry locates base_footprint; URDF places chassis above ground.
    odom_tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/rcauto/odom", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr odom) {
          const auto &pose = odom->pose.pose;
          if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
              !std::isfinite(pose.orientation.x) || !std::isfinite(pose.orientation.y) ||
              !std::isfinite(pose.orientation.z) || !std::isfinite(pose.orientation.w)) {
            return;
          }
          geometry_msgs::msg::TransformStamped tf;
          tf.header = odom->header;
          tf.header.frame_id = "odom";
          tf.child_frame_id = "base_footprint";
          tf.transform.translation.x = pose.position.x;
          tf.transform.translation.y = pose.position.y;
          tf.transform.translation.z = 0.0;
          tf.transform.rotation = pose.orientation;
          odom_tf_->sendTransform(tf);
        });
    timer_ = create_wall_timer(50ms, [this] {
      geometry_msgs::msg::Twist command;
      const bool fresh = have_command_ &&
          std::chrono::duration<double>(
              std::chrono::steady_clock::now() - last_command_).count() <= command_timeout_;
      if (fresh) {
        command.linear.x = speed_;
        command.angular.z = yaw_rate_;
      }
      cmd_pub_->publish(command);
    });
    RCLCPP_INFO(get_logger(), "Gazebo adapter ready; no physical actuator output");
  }

 private:
  double wheelbase_{0.30};
  double steering_limit_{0.45};
  double command_timeout_{0.35};
  double front_cone_{0.45};
  double speed_{0.0};
  double yaw_rate_{0.0};
  bool have_command_{false};
  std::chrono::steady_clock::time_point last_command_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GazeboAdapter>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("rcauto_gazebo_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
