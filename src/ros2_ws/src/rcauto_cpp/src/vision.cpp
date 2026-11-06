#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals;
class VisionNode final : public rclcpp::Node {
 public:
  VisionNode() : Node("rcauto_cpp_vision") {
    declare_parameter("vision.source", "ros");
    declare_parameter("vision.lane_style", "tape");
    declare_parameter("vision.line_color", "yellow");
    declare_parameter("vision.camera_index", 0);
    declare_parameter("vision.yellow_h_min", 15);
    declare_parameter("vision.yellow_h_max", 42);
    declare_parameter("vision.min_saturation", 70);
    declare_parameter("vision.min_brightness", 80);
    declare_parameter("vision.road_min_width_ratio", 0.22);
    declare_parameter("vision.road_max_width_ratio", 0.89);
    declare_parameter("vision.publish_debug", true);
    declare_parameter("vision.fps", 15.0);
    refresh();
    callback_handle_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& p) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto& item : p) {
        try {
          const auto& n = item.get_name();
          if (n == "vision.source" || n == "vision.camera_index" || n == "vision.fps") {
            result.successful = false; result.reason = "Camera source/fps requires restart"; break;
          }
          if (n == "vision.lane_style") {
            if (item.as_string() != "road" && item.as_string() != "tape")
              throw std::invalid_argument("lane_style must be road or tape");
          }
          if (n == "vision.line_color") {
            if (item.as_string() != "yellow" && item.as_string() != "white" &&
                item.as_string() != "black")
              throw std::invalid_argument("unsupported line color");
          }
          if (n.find("yellow_h_") != std::string::npos ||
              n.find("min_saturation") != std::string::npos ||
              n.find("min_brightness") != std::string::npos) {
            if (item.as_int() < 0 || item.as_int() > 255)
              throw std::invalid_argument("HSV limit invalid");
          }
          if (n.find("road_") != std::string::npos) {
            if (item.as_double() < 0.1 || item.as_double() > 0.99)
              throw std::invalid_argument("road width limit invalid");
          }
        } catch (const std::exception& e) {
          result.successful = false; result.reason = e.what(); break;
        }
      }
      if (result.successful) pending_ = true;
      return result;
    });
    pub_error_ = create_publisher<std_msgs::msg::Float32>("/rcauto/lane_error", 10);
    pub_conf_ = create_publisher<std_msgs::msg::Float32>("/rcauto/lane_confidence", 10);
    pub_cross_ = create_publisher<std_msgs::msg::Bool>("/rcauto/intersection", 10);
    pub_debug_ = create_publisher<sensor_msgs::msg::Image>("/rcauto/camera/annotated", 2);
    if (get_parameter("vision.source").as_string() == "usb") {
      camera_.open(static_cast<int>(get_parameter("vision.camera_index").as_int()));
      camera_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
      camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
      if (!camera_.isOpened()) RCLCPP_ERROR(get_logger(), "Camera missing, will stop by stale watchdog");
      const double fps = get_parameter("vision.fps").as_double();
      if (fps < 1 || fps > 60) throw std::invalid_argument("fps outside [1,60]");
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / fps), [this]() {
        cv::Mat frame;
        if (camera_.isOpened() && camera_.read(frame) && !frame.empty()) analyze(frame);
      });
    } else if (get_parameter("vision.source").as_string() == "ros") {
      sub_image_ = create_subscription<sensor_msgs::msg::Image>("/rcauto/camera/image", 3,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          try {
            auto image = cv_bridge::toCvShare(msg, "bgr8");
            analyze(image->image);
          } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Camera decode: %s", e.what());
          }
        });
    } else throw std::invalid_argument("vision.source must be usb or ros");
  }
 private:
  void refresh() {
    style_ = get_parameter("vision.lane_style").as_string();
    color_ = get_parameter("vision.line_color").as_string();
    hmin_ = static_cast<int>(get_parameter("vision.yellow_h_min").as_int());
    hmax_ = static_cast<int>(get_parameter("vision.yellow_h_max").as_int());
    smin_ = static_cast<int>(get_parameter("vision.min_saturation").as_int());
    vmin_ = static_cast<int>(get_parameter("vision.min_brightness").as_int());
    road_min_ = get_parameter("vision.road_min_width_ratio").as_double();
    road_max_ = get_parameter("vision.road_max_width_ratio").as_double();
    show_debug_ = get_parameter("vision.publish_debug").as_bool();
  }
  void analyze(const cv::Mat& frame) {
    if (frame.empty() || frame.cols < 30 || frame.rows < 30) return;
    if (pending_) { pending_ = false; refresh(); }
    const int w = frame.cols, h = frame.rows;
    const int y0 = static_cast<int>(h * (style_ == "road" ? 0.51 : 0.60));
    cv::Mat hsv; cv::cvtColor(frame(cv::Rect(0, y0, w, h-y0)), hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    if (style_ == "road") {
      cv::Mat white, yellow;
      cv::inRange(hsv, cv::Scalar(0,0,140), cv::Scalar(179,65,255), white);
      cv::inRange(hsv, cv::Scalar(hmin_,smin_,vmin_),
                  cv::Scalar(hmax_,255,255), yellow);
      cv::bitwise_or(white, yellow, mask);
    } else if (color_ == "black") {
      cv::inRange(hsv, cv::Scalar(0,0,0), cv::Scalar(179,255,85), mask);
    } else if (color_ == "white") {
      cv::inRange(hsv, cv::Scalar(0,0,150), cv::Scalar(179,75,255), mask);
    } else {
      cv::inRange(hsv, cv::Scalar(hmin_,smin_,vmin_),
                  cv::Scalar(hmax_,255,255), mask);
    }
    double error = 0, confidence = 0;
    const cv::Mat mid = mask(cv::Range(static_cast<int>(mask.rows*.15),
                                      static_cast<int>(mask.rows*.55)), cv::Range::all());
    int broad_rows = 0;
    for (int y=0; y<mid.rows; ++y) if (cv::countNonZero(mid.row(y)) > w*0.65) ++broad_rows;
    const bool intersection = broad_rows >= std::max(2, static_cast<int>(mid.rows*.12));
    if (style_ == "tape") {
      const cv::Mat near = mask(cv::Range(static_cast<int>(mask.rows*.55), mask.rows),
                               cv::Range::all());
      std::vector<int> columns;
      columns.reserve(static_cast<std::size_t>(cv::countNonZero(near)));
      for (int y=0;y<near.rows;++y) {
        const auto* pixels = near.ptr<uint8_t>(y);
        for (int x=0;x<w;++x) if (pixels[x]) columns.push_back(x);
      }
      if (static_cast<int>(columns.size()) >= std::max(30, static_cast<int>(static_cast<double>(near.total())*.002))) {
        const auto center_index=columns.size()/2;
        std::nth_element(columns.begin(), columns.begin()+static_cast<std::ptrdiff_t>(center_index),
                         columns.end());
        const double center=columns[center_index];
        error = std::clamp((center-w*.5)/(w*.5),-1.,1.);
        const auto minmax=std::minmax_element(columns.begin(),columns.end());
        const double spread=static_cast<double>(*minmax.second-*minmax.first);
        confidence = (spread >= w*.012 && spread < w*.55) ? .75 : .25;
      }
    } else {
      cv::Mat edges;
      cv::Canny(mask, edges, 70,140);
      std::vector<cv::Vec4i> lines;
      cv::HoughLinesP(edges,lines,1,CV_PI/180,22,std::max(14,w/15),std::max(10,w/20));
      std::vector<double> left,right;
      const double ref=mask.rows*.88;
      for(const auto& line:lines) {
        const double dy=line[3]-line[1], dx=line[2]-line[0];
        if(std::abs(dy) < mask.rows*.08) continue;
        const double slope=dx/dy;
        if(std::abs(slope)<.1 || std::abs(slope)>1.5) continue;
        const double xref=line[0]+(ref-line[1])*slope;
        if(xref<0 || xref>=w) continue;
        (xref<w/2.?left:right).push_back(xref);
      }
      if(!left.empty()&&!right.empty()) {
        auto median=[](std::vector<double>& a) {
          auto i=a.begin()+static_cast<std::ptrdiff_t>(a.size()/2);
          std::nth_element(a.begin(),i,a.end());return *i;
        };
        const double l=median(left), r=median(right), width=(r-l)/w;
        if(width>=road_min_&&width<=road_max_) {
          error=std::clamp(((l+r)*.5-w*.5)/(w*.5),-1.,1.);
          confidence=.78;
        }
      }
    }
    std_msgs::msg::Float32 err;err.data=static_cast<float>(error);pub_error_->publish(err);
    std_msgs::msg::Float32 conf;conf.data=static_cast<float>(confidence);pub_conf_->publish(conf);
    std_msgs::msg::Bool cross;cross.data=intersection;pub_cross_->publish(cross);
    if (show_debug_ && pub_debug_->get_subscription_count()>0) {
      cv::Mat debug=frame.clone();
      cv::line(debug,cv::Point(w/2,y0),cv::Point(w/2,h),cv::Scalar(200,200,200),1);
      cv::putText(debug,"lane="+std::to_string(error)+" conf="+std::to_string(confidence),
                  cv::Point(8,24),cv::FONT_HERSHEY_SIMPLEX,.5,cv::Scalar(0,255,0),1);
      std_msgs::msg::Header header;header.stamp=get_clock()->now();header.frame_id="camera_link";
      pub_debug_->publish(*cv_bridge::CvImage(header,"bgr8",debug).toImageMsg());
    }
  }
  cv::VideoCapture camera_;
  std::string style_,color_;
  bool pending_{false},show_debug_{true};
  int hmin_{15},hmax_{42},smin_{70},vmin_{80};
  double road_min_{.22},road_max_{.89};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_error_,pub_conf_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_cross_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<VisionNode>());rclcpp::shutdown();return 0;
}
