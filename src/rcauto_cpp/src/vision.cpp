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
    declare_parameter("vision.hardware_mode", false);
    declare_parameter("vision.lane_style", "tape");
    declare_parameter("vision.line_color", "yellow");
    declare_parameter("vision.camera_index", 0);
    declare_parameter("vision.yellow_h_min", 15);
    declare_parameter("vision.yellow_h_max", 42);
    declare_parameter("vision.min_saturation", 70);
    declare_parameter("vision.min_brightness", 80);
    declare_parameter("vision.lookahead_weight", 0.72);
    declare_parameter("vision.error_smoothing_alpha", 0.48);
    declare_parameter("vision.road_min_width_ratio", 0.22);
    declare_parameter("vision.road_max_width_ratio", 0.89);
    declare_parameter("vision.publish_debug", true);
    declare_parameter("vision.fps", 15.0);
    hardware_mode_ = get_parameter("vision.hardware_mode").as_bool();
    refresh();
    callback_handle_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& p) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      int candidate_hmin=hmin_,candidate_hmax=hmax_;
      double candidate_road_min=road_min_,candidate_road_max=road_max_;
      for (const auto& item : p) {
        try {
          const auto& n = item.get_name();
          if (n == "vision.hardware_mode") {
            result.successful=false;result.reason="Hardware mode is read-only";break;
          }
          if (n.rfind("vision.",0)==0 && hardware_mode_) {
            result.successful=false;
            result.reason="Vision tuning disabled in hardware mode; restart disarmed";
            break;
          }
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
            if (n == "vision.yellow_h_min") candidate_hmin=static_cast<int>(item.as_int());
            if (n == "vision.yellow_h_max") candidate_hmax=static_cast<int>(item.as_int());
          }
          if (n == "vision.lookahead_weight") {
            if (item.as_double() < 0.0 || item.as_double() > 1.0)
              throw std::invalid_argument("lookahead_weight must be in [0,1]");
          }
          if (n == "vision.error_smoothing_alpha") {
            if (item.as_double() < 0.05 || item.as_double() > 1.0)
              throw std::invalid_argument("error_smoothing_alpha must be in [0.05,1]");
          }
          if (n.find("road_") != std::string::npos) {
            if (item.as_double() < 0.1 || item.as_double() > 0.99)
              throw std::invalid_argument("road width limit invalid");
            if (n == "vision.road_min_width_ratio") candidate_road_min=item.as_double();
            if (n == "vision.road_max_width_ratio") candidate_road_max=item.as_double();
          }
        } catch (const std::exception& e) {
          result.successful = false; result.reason = e.what(); break;
        }
      }
      if (result.successful &&
          (candidate_hmin>candidate_hmax || candidate_road_min>=candidate_road_max)) {
        result.successful=false;
        result.reason="HSV min must not exceed max; road width min must be below max";
      }
      if (result.successful) pending_ = true;
      return result;
    });
    pub_error_ = create_publisher<std_msgs::msg::Float32>("/rcauto/lane_error", 10);
    pub_raw_error_ = create_publisher<std_msgs::msg::Float32>("/rcauto/vision/lane_error_raw",10);
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
    lookahead_weight_ = get_parameter("vision.lookahead_weight").as_double();
    smoothing_alpha_ = get_parameter("vision.error_smoothing_alpha").as_double();
    road_min_ = get_parameter("vision.road_min_width_ratio").as_double();
    road_max_ = get_parameter("vision.road_max_width_ratio").as_double();
    show_debug_ = get_parameter("vision.publish_debug").as_bool();
  }
  void analyze(const cv::Mat& frame) {
    if (frame.empty() || frame.cols < 30 || frame.rows < 30) return;
    if (pending_) { pending_ = false; refresh(); }
    const int w = frame.cols, h = frame.rows;
    const int y0 = static_cast<int>(h * (style_ == "road" ? 0.51 : 0.50));
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
    cv::Mat debug = frame.clone();
    const cv::Mat mid = mask(cv::Range(static_cast<int>(mask.rows*.15),
                                      static_cast<int>(mask.rows*.55)), cv::Range::all());
    int broad_rows = 0;
    for (int y=0; y<mid.rows; ++y) if (cv::countNonZero(mid.row(y)) > w*0.65) ++broad_rows;
    const bool intersection = broad_rows >= std::max(2, static_cast<int>(mid.rows*.12));
    if (style_ == "tape") {
      // Camera-only detection of the disconnected central guide markings.
      // Solid road boundaries form large connected components and are rejected.
      // No odometry, map, or simulator ground-truth contributes to steering.
      cv::Mat clean;
      cv::morphologyEx(mask, clean, cv::MORPH_OPEN,
                       cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(clean, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      struct Marker { cv::Point2f center; double area; cv::Rect rect; };
      std::vector<Marker> markers;
      markers.reserve(contours.size());
      const double max_area = w * h * 0.012;
      for (const auto& contour : contours) {
        const cv::Rect box = cv::boundingRect(contour);
        const double area = cv::contourArea(contour);
        if (area < 8 || area > max_area || box.width > w * 0.20 ||
            box.height > h * 0.30 || box.y + y0 < h * 0.57) continue;
        const auto moments = cv::moments(contour);
        if (moments.m00 <= 0.0) continue;
        const cv::Point2f center(
            static_cast<float>(moments.m10 / moments.m00),
            static_cast<float>(moments.m01 / moments.m00 + y0));
        markers.push_back({center, area, box});
      }
      // When central dashes are absent, infer a visual correction from
      // continuous road boundaries in the SAME camera frame.
      std::vector<float> left_edges, right_edges;
      for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        const cv::Rect box = cv::boundingRect(contour);
        if (area < w * h * 0.0007 ||
            (box.width < w * 0.23 && box.height < h * 0.28)) continue;
        const auto moments = cv::moments(contour);
        if (moments.m00 <= 0) continue;
        const float x = static_cast<float>(moments.m10 / moments.m00);
        if (x <= w * 0.50) left_edges.push_back(x);
        else right_edges.push_back(x);
      }
      const bool has_left = !left_edges.empty();
      const bool has_right = !right_edges.empty();
      double edge_target = 0;
      bool have_edge_target = false;
      if (has_left || has_right) {
        const double l = has_left ? *std::max_element(left_edges.begin(), left_edges.end()) : 0.0;
        const double r = has_right ? *std::min_element(right_edges.begin(), right_edges.end()) : 0.0;
        if (has_left && has_right && r - l > w * 0.28) {
          edge_half_width_px_ = std::clamp((r - l) * 0.5, w * 0.23, w * 0.44);
          edge_target = (r + l) * 0.5;
          have_edge_target = true;
        } else if (has_right && r > w * 0.46) {
          edge_target = r - edge_half_width_px_;
          have_edge_target = true;
        } else if (has_left && l < w * 0.54) {
          edge_target = l + edge_half_width_px_;
          have_edge_target = true;
        }
        if (have_edge_target) {
          cv::circle(debug, cv::Point(cvRound(edge_target), h * 3 / 4),
                     5, cv::Scalar(255, 150, 0), -1);
        }
      }
      std::sort(markers.begin(), markers.end(),
                [](const Marker& a, const Marker& b) {
                  return a.center.y > b.center.y;
                });
      std::vector<cv::Point2f> tracked;
      float previous_x = static_cast<float>(w) * 0.5F;
      for (const auto& marker : markers) {
        if (tracked.empty()) {
          if (marker.center.y < h * 0.60 ||
              std::abs(marker.center.x - last_mark_x_) > w * 0.37) continue;
        } else if (std::abs(marker.center.x - previous_x) > w * 0.28) {
          continue;
        }
        tracked.push_back(marker.center);
        previous_x = marker.center.x;
        if (tracked.size() >= 8) break;
      }
      if (!tracked.empty()) {
        const float near_x = tracked.front().x;
        const float far_x = tracked.size() >= 2 ? tracked.back().x : near_x;
        double target_x =
            near_x * (1.0 - lookahead_weight_) + far_x * lookahead_weight_;
        if (have_edge_target && std::abs(target_x - edge_target) < w * 0.24) {
          target_x = target_x * 0.82 + edge_target * 0.18;
        }
        error = std::clamp((target_x - w * 0.5) / (w * 0.5), -1.0, 1.0);
        confidence = tracked.size() >= 2 ? 0.87 : 0.52;
        last_mark_x_ = static_cast<float>(target_x);
        for (size_t i = 0; i < tracked.size(); ++i) {
          const cv::Point at(cvRound(tracked[i].x), cvRound(tracked[i].y));
          cv::circle(debug, at, 7, cv::Scalar(45, 240, 45), 2);
          if (i > 0) cv::line(debug, tracked[i - 1], tracked[i],
                              cv::Scalar(0, 220, 240), 2);
        }
        cv::line(debug, cv::Point(cvRound(target_x), h - 1),
                 cv::Point(cvRound(target_x), h * 3 / 4),
                 cv::Scalar(0, 200, 255), 2);
      } else if (have_edge_target) {
        // Temporal continuity from previous camera detection limits jumps
        // when the only visible feature is a solid boundary.
        edge_target = std::clamp(edge_target,
                                 static_cast<double>(last_mark_x_) - w * 0.26,
                                 static_cast<double>(last_mark_x_) + w * 0.26);
        last_mark_x_ = static_cast<float>(edge_target);
        error = std::clamp((edge_target - w * 0.5) / (w * 0.5), -1.0, 1.0);
        confidence = 0.58;
        cv::line(debug, cv::Point(cvRound(edge_target), h - 1),
                 cv::Point(cvRound(edge_target), h * 3 / 4),
                 cv::Scalar(255, 180, 0), 2);
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
    // Image +X points right, whereas ROS Ackermann +steering turns left.
    // Convert pixel displacement into vehicle-left-positive steering error.
    error = -error;
    const double raw_error = error;
    if (confidence > 0.0) {
      filtered_error_ = have_filtered_error_
          ? smoothing_alpha_ * error + (1.0 - smoothing_alpha_) * filtered_error_
          : error;
      have_filtered_error_ = true;
    } else {
      filtered_error_ = 0.0;
      have_filtered_error_ = false;
    }
    std_msgs::msg::Float32 raw;raw.data=static_cast<float>(raw_error);
    pub_raw_error_->publish(raw);
    error = filtered_error_;
    std_msgs::msg::Float32 err;err.data=static_cast<float>(error);pub_error_->publish(err);
    std_msgs::msg::Float32 conf;conf.data=static_cast<float>(confidence);pub_conf_->publish(conf);
    std_msgs::msg::Bool cross;cross.data=intersection;pub_cross_->publish(cross);
    if (show_debug_ && pub_debug_->get_subscription_count()>0) {
      cv::line(debug,cv::Point(w/2,y0),cv::Point(w/2,h),cv::Scalar(200,200,200),1);
      cv::putText(debug,"lane="+std::to_string(error)+" conf="+std::to_string(confidence),
                  cv::Point(8,24),cv::FONT_HERSHEY_SIMPLEX,.5,cv::Scalar(0,255,0),1);
      std_msgs::msg::Header header;header.stamp=get_clock()->now();header.frame_id="camera_link";
      pub_debug_->publish(*cv_bridge::CvImage(header,"bgr8",debug).toImageMsg());
    }
  }
  cv::VideoCapture camera_;
  std::string style_,color_;
  bool pending_{false},show_debug_{true},hardware_mode_{false};
  int hmin_{15},hmax_{42},smin_{70},vmin_{80};
  double road_min_{.22},road_max_{.89},lookahead_weight_{.72};
  double smoothing_alpha_{.48},filtered_error_{0.0};
  bool have_filtered_error_{false};
  float last_mark_x_{320.0F};
  double edge_half_width_px_{210.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_error_,pub_raw_error_,pub_conf_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_cross_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<VisionNode>());rclcpp::shutdown();return 0;
}
