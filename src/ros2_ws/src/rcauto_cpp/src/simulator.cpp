#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <memory>
using namespace std::chrono_literals;
class Simulator final : public rclcpp::Node {
 public:
  Simulator():Node("rcauto_cpp_simulator") {
    pub_image_=create_publisher<sensor_msgs::msg::Image>("/rcauto/camera/image",2);
    pub_detect_=create_publisher<vision_msgs::msg::Detection2DArray>("/rcauto/detections",10);
    pub_range_=create_publisher<sensor_msgs::msg::Range>("/rcauto/range",10);
    create_timer_=create_wall_timer(100ms,[this] { tick(); });
    RCLCPP_WARN(get_logger(),"SIMULATED input; never connect simulated sensors to live PWM");
  }
 private:
  void tick() {
    cv::Mat frame(240,320,CV_8UC3,cv::Scalar(55,55,55));
    cv::line(frame,cv::Point(145,239),cv::Point(157,120),cv::Scalar(0,220,255),12);
    sensor_msgs::msg::Image img;
    img.header.stamp=get_clock()->now();img.header.frame_id="sim_camera";
    img.width=static_cast<uint32_t>(frame.cols);img.height=static_cast<uint32_t>(frame.rows);
    img.encoding="bgr8";img.step=static_cast<uint32_t>(frame.step);
    img.data.assign(frame.datastart,frame.dataend);
    pub_image_->publish(img);
    vision_msgs::msg::Detection2DArray detections;
    detections.header=img.header;pub_detect_->publish(detections);
    sensor_msgs::msg::Range range;
    range.header=img.header;
    range.radiation_type=sensor_msgs::msg::Range::INFRARED;
    range.field_of_view=.4F;range.min_range=.08F;range.max_range=2.F;range.range=1.5F;
    pub_range_->publish(range);
  }
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_detect_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_range_;
  rclcpp::TimerBase::SharedPtr create_timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Simulator>());rclcpp::shutdown();return 0;
}
