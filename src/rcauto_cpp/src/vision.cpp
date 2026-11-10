#include "rcauto_cpp/road_vision.hpp"
#include "rcauto_cpp/cone_avoidance.hpp"
#include "rcauto_cpp/target_reference.hpp"
#include "rcauto_cpp/traffic_cone.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

using namespace std::chrono_literals;
class VisionNode final : public rclcpp::Node {
 public:
  VisionNode() : Node("rcauto_cpp_vision") {
    // Avoid OpenCV spawning one thread per Xeon core for small 640x480 frames.
    cv::setNumThreads(2);
    declare_parameter("vision.source", "ros");
    declare_parameter("vision.hardware_mode", false);
    declare_parameter("vision.lane_style", "tape");
    declare_parameter("vision.line_color", "yellow");
    declare_parameter("vision.camera_device", "auto");
    declare_parameter("vision.camera_capture_width", 1280);
    declare_parameter("vision.camera_capture_height", 720);
    declare_parameter("vision.camera_output_width", 1080);
    declare_parameter("vision.target_distance_ratio", 0.55);
    declare_parameter("vision.yellow_h_min", 15);
    declare_parameter("vision.yellow_h_max", 42);
    declare_parameter("vision.min_saturation", 70);
    declare_parameter("vision.min_brightness", 80);
    declare_parameter("vision.lookahead_weight", 0.72);
    declare_parameter("vision.error_smoothing_alpha", 0.48);
    declare_parameter("vision.road_min_width_ratio", 0.22);
    declare_parameter("vision.road_max_width_ratio", 0.89);
    declare_parameter("vision.publish_debug", true);
    declare_parameter("vision.detect_traffic_cones", true);
    declare_parameter("vision.cone_avoidance_enabled", true);
    declare_parameter("vision.cone_avoidance_offset_ratio", 0.50);
    declare_parameter("vision.jpeg_quality", 72);
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
          if (n == "vision.source" ||
              n == "vision.camera_device" || n == "vision.fps" ||
              n == "vision.camera_capture_width" ||
              n == "vision.camera_capture_height" ||
              n == "vision.camera_output_width") {
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
          if (n == "vision.cone_avoidance_offset_ratio") {
            if (item.as_double()<0.10 || item.as_double()>0.85)
              throw std::invalid_argument("cone offset must be 0.10..0.85");
          }
          if (n == "vision.target_distance_ratio") {
            if(item.as_double()<0.15 || item.as_double()>0.85)
              throw std::invalid_argument("Target distance ratio must be 0.15..0.85");
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
    pub_edge_conf_ = create_publisher<std_msgs::msg::Float32>(
        "/rcauto/vision/visible_edge_confidence",10);
    pub_edge_side_ = create_publisher<std_msgs::msg::Int32>(
        "/rcauto/vision/visible_edge_side",10);
    pub_edge_only_ = create_publisher<std_msgs::msg::Bool>(
        "/rcauto/vision/edge_only",10);
    pub_near_ = create_publisher<std_msgs::msg::Float32>("/rcauto/vision/near_error",10);
    pub_far_ = create_publisher<std_msgs::msg::Float32>("/rcauto/vision/far_error",10);
    pub_curvature_ = create_publisher<std_msgs::msg::Float32>("/rcauto/vision/curvature",10);
    pub_preview_ = create_publisher<std_msgs::msg::Float32>("/rcauto/vision/preview_visibility",10);
    pub_cross_ = create_publisher<std_msgs::msg::Bool>("/rcauto/intersection", 10);
    pub_debug_ = create_publisher<sensor_msgs::msg::Image>("/rcauto/camera/annotated", 1);
    pub_jpeg_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "/rcauto/camera/annotated/compressed",1);
    pub_eco_jpeg_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "/rcauto/camera/preview/compressed",1);
    pub_raw_jpeg_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "/rcauto/camera/raw/compressed",1);
    pub_avoid_active_ = create_publisher<std_msgs::msg::Bool>(
        "/rcauto/vision/avoidance_active",10);
    pub_avoid_blocked_ = create_publisher<std_msgs::msg::Bool>(
        "/rcauto/vision/avoidance_blocked",10);
    pub_avoid_offset_ = create_publisher<std_msgs::msg::Float32>(
        "/rcauto/vision/avoidance_offset_norm",10);
    pub_cone_ = create_publisher<std_msgs::msg::Bool>(
        "/rcauto/vision/traffic_cone/detected",10);
    pub_cone_boxes_ = create_publisher<vision_msgs::msg::Detection2DArray>(
        "/rcauto/vision/traffic_cone/detections",10);
    const auto capture_width=get_parameter("vision.camera_capture_width").as_int();
    const auto capture_height=get_parameter("vision.camera_capture_height").as_int();
    const auto output_width=get_parameter("vision.camera_output_width").as_int();
    if(capture_width<320 || capture_width>2560 ||
       capture_height<240 || capture_height>1440 ||
       output_width<320 || output_width>1080 ||
       output_width>capture_width)
      throw std::invalid_argument("Unsupported camera capture/output dimensions");
    capture_width_=static_cast<int>(capture_width);
    capture_height_=static_cast<int>(capture_height);
    output_width_=static_cast<int>(output_width);
    pub_camera_status_ = create_publisher<std_msgs::msg::String>(
        "/rcauto/camera/status",10);
    pub_camera_connected_ = create_publisher<std_msgs::msg::Bool>(
        "/rcauto/camera/connected",10);
    if (get_parameter("vision.source").as_string() == "usb") {
      camera_preference_ = get_parameter("vision.camera_device").as_string();
      if(camera_preference_.empty())camera_preference_="auto";
      const double fps = get_parameter("vision.fps").as_double();
      if (fps < 1 || fps > 60) throw std::invalid_argument("fps outside [1,60]");
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / fps),
                                 [this]() { poll_usb_camera(); });
      report_camera(false, "SEARCHING | USB camera auto-detection enabled");
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
  static bool is_capture_device(const std::string& device) {
    const int fd=::open(device.c_str(),O_RDONLY|O_NONBLOCK|O_CLOEXEC);
    if(fd<0)return false;
    v4l2_capability capabilities{};
    const bool query_ok=::ioctl(fd,VIDIOC_QUERYCAP,&capabilities)==0;
    ::close(fd);
    if(!query_ok)return false;
    const auto flags=capabilities.capabilities&V4L2_CAP_DEVICE_CAPS ?
        capabilities.device_caps:capabilities.capabilities;
    return (flags&V4L2_CAP_VIDEO_CAPTURE)!=0 &&
           (flags&V4L2_CAP_STREAMING)!=0;
  }

  std::vector<std::string> camera_candidates() const {
    if(camera_preference_!="auto")return {camera_preference_};
    namespace fs=std::filesystem;
    std::vector<std::string> devices;
    std::error_code error;
    const fs::path by_id("/dev/v4l/by-id");
    if(fs::is_directory(by_id,error)) {
      for(const auto& entry:fs::directory_iterator(
              by_id,fs::directory_options::skip_permission_denied,error)) {
        const auto name=entry.path().filename().string();
        if(name.find("-video-index0")==std::string::npos)continue;
        const std::string device=entry.path().string();
        if(fs::exists(entry.path(),error) &&
           is_capture_device(device))
          devices.push_back(device);
      }
    }
    std::sort(devices.begin(),devices.end());
    if(!devices.empty())return devices;
    for(int i=0;i<16;++i) {
      const std::string device="/dev/video"+std::to_string(i);
      if(fs::exists(device,error) && is_capture_device(device))
        devices.push_back(device);
    }
    return devices;
  }

  void report_camera(bool connected,const std::string& description) {
    if(connected!=camera_connected_ ||
       description!=camera_state_) {
      camera_state_=description;
      camera_connected_=connected;
      if(connected)RCLCPP_INFO(get_logger(),"%s",description.c_str());
      else RCLCPP_WARN(get_logger(),"%s",description.c_str());
    }
    std_msgs::msg::String state;
    state.data=camera_state_;
    pub_camera_status_->publish(state);
    std_msgs::msg::Bool present;
    present.data=connected;
    pub_camera_connected_->publish(present);
  }

  void disconnect_camera(const std::string& reason) {
    if(camera_.isOpened())camera_.release();
    active_device_.clear();
    failed_reads_=0;
    filtered_error_=0.0;
    have_filtered_error_=false;
    last_cone_time_={};
    // A previous frame must not remain valid after USB loss.
    std_msgs::msg::Float32 zero;
    zero.data=0.0F;
    pub_conf_->publish(zero);
    report_camera(false,reason);
  }

  void poll_usb_camera() {
    namespace fs=std::filesystem;
    const auto now=std::chrono::steady_clock::now();
    if(camera_.isOpened()) {
      std::error_code error;
      if(!fs::exists(active_device_,error)) {
        disconnect_camera("DISCONNECTED | USB camera removed, scanning");
      } else {
        cv::Mat frame;
        const bool received=camera_.read(frame) && !frame.empty();
        if(received) {
          failed_reads_=0;
          analyze_camera_frame(frame);
          report_camera(true,"CONNECTED | "+active_device_);
          return;
        }
        if(++failed_reads_<3)return;
        disconnect_camera("DISCONNECTED | camera frames lost, reconnecting");
      }
    }
    if(now-last_camera_attempt_<std::chrono::seconds(1)) {
      report_camera(false,"SEARCHING | camera USB hot-plug");
      return;
    }
    last_camera_attempt_=now;
    for(const auto& candidate:camera_candidates()) {
      if(!camera_.open(candidate,cv::CAP_V4L2))continue;
      camera_.set(cv::CAP_PROP_FOURCC,
                  cv::VideoWriter::fourcc('M','J','P','G'));
      camera_.set(cv::CAP_PROP_BUFFERSIZE,1);
      camera_.set(cv::CAP_PROP_FRAME_WIDTH,capture_width_);
      camera_.set(cv::CAP_PROP_FRAME_HEIGHT,capture_height_);
      camera_.set(cv::CAP_PROP_FPS,
                  get_parameter("vision.fps").as_double());
      active_device_=candidate;
      cv::Mat first;
      if(camera_.read(first) && !first.empty()) {
        failed_reads_=0;
        report_camera(true,"CONNECTED | "+candidate);
        analyze_camera_frame(first);
        return;
      }
      camera_.release();
      active_device_.clear();
    }
    report_camera(false,"SEARCHING | no readable USB capture camera");
  }

  void analyze_camera_frame(const cv::Mat& captured) {
    if(captured.empty())return;
    if(captured.cols>output_width_) {
      const int scaled_height=std::max(1,cvRound(
          static_cast<double>(captured.rows)*output_width_/captured.cols));
      cv::Mat output;
      cv::resize(captured,output,cv::Size(output_width_,scaled_height),
                 0,0,cv::INTER_AREA);
      analyze(output);
    } else {
      analyze(captured);
    }
  }

  void refresh() {
    style_ = get_parameter("vision.lane_style").as_string();
    color_ = get_parameter("vision.line_color").as_string();
    hmin_ = static_cast<int>(get_parameter("vision.yellow_h_min").as_int());
    hmax_ = static_cast<int>(get_parameter("vision.yellow_h_max").as_int());
    smin_ = static_cast<int>(get_parameter("vision.min_saturation").as_int());
    vmin_ = static_cast<int>(get_parameter("vision.min_brightness").as_int());
    lookahead_weight_ = get_parameter("vision.lookahead_weight").as_double();
    target_distance_ratio_ =
        get_parameter("vision.target_distance_ratio").as_double();
    smoothing_alpha_ = get_parameter("vision.error_smoothing_alpha").as_double();
    road_min_ = get_parameter("vision.road_min_width_ratio").as_double();
    road_max_ = get_parameter("vision.road_max_width_ratio").as_double();
    show_debug_ = get_parameter("vision.publish_debug").as_bool();
    detect_cones_ = get_parameter("vision.detect_traffic_cones").as_bool();
    avoid_enabled_ = get_parameter("vision.cone_avoidance_enabled").as_bool();
    avoid_offset_ratio_ =
        get_parameter("vision.cone_avoidance_offset_ratio").as_double();
    jpeg_quality_ = static_cast<int>(
        get_parameter("vision.jpeg_quality").as_int());
    if (jpeg_quality_<35 || jpeg_quality_>95)
      throw std::invalid_argument("JPEG quality must be 35..95");
  }
  void analyze(const cv::Mat& frame) {
    if (frame.empty() || frame.cols < 30 || frame.rows < 30) return;
    if (pending_) { pending_ = false; refresh(); }
    if(pub_raw_jpeg_->get_subscription_count()>0) {
      std::vector<unsigned char> encoded;
      if(cv::imencode(".jpg",frame,encoded,
                      {cv::IMWRITE_JPEG_QUALITY,60})) {
        sensor_msgs::msg::CompressedImage image;
        image.header.stamp=get_clock()->now();
        image.header.frame_id="camera_link";
        image.format="jpeg";
        image.data=std::move(encoded);
        pub_raw_jpeg_->publish(image);
      }
    }
    const int w = frame.cols, h = frame.rows;
    const int y0 = static_cast<int>(h*0.31);
    cv::Mat mask;
    if (style_ != "road") {
      cv::Mat hsv;
      cv::cvtColor(frame(cv::Rect(0,y0,w,h-y0)),hsv,cv::COLOR_BGR2HSV);
      if (color_ == "black") {
        cv::inRange(hsv,cv::Scalar(0,0,0),
                    cv::Scalar(179,255,85),mask);
      } else if (color_ == "white") {
        cv::inRange(hsv,cv::Scalar(0,0,150),
                    cv::Scalar(179,75,255),mask);
      } else {
        cv::inRange(hsv,cv::Scalar(hmin_,smin_,vmin_),
                    cv::Scalar(hmax_,255,255),mask);
      }
    }
    double error = 0, confidence = 0;
    double near_error = 0, far_error = 0, curvature = 0, preview_visibility = 0;
    double visible_edge_confidence=0;
    int visible_edge_side=0;
    bool edge_only=false;
    cv::Mat debug = frame.clone();
    bool intersection=false;
    std::optional<rcauto_cpp::LaneObservation> lane_measurement;
    std::optional<cv::Point> target_point;
    if (style_ == "tape") {
      const auto mid=mask(cv::Range(static_cast<int>(mask.rows*.15),
                                    static_cast<int>(mask.rows*.55)),
                          cv::Range::all());
      int broad_rows=0;
      for(int y=0;y<mid.rows;++y)
        if(cv::countNonZero(mid.row(y))>w*.65)++broad_rows;
      intersection=broad_rows>=std::max(2,static_cast<int>(mid.rows*.12));
    }
    if (style_ == "tape") {
      // Camera-only detection of the disconnected central guide markings.
      // Solid road boundaries form large connected components and are rejected.
      // No odometry, map, or simulator ground-truth contributes to steering.
      cv::Mat clean;
      cv::morphologyEx(mask, clean, cv::MORPH_OPEN,
                       cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(clean, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      // Vision-only preview: find the road midpoint at multiple image depths
      // by sampling both solid WHITE road boundaries. No pose or map.
      std::vector<cv::Point2f> road_centers;
      const int row_step = std::max(8, h / 28);
      for (int y = cvRound(h * 0.89); y >= cvRound(h * 0.36); y -= row_step) {
        const int ry = y - y0;
        if (ry < 0 || ry >= clean.rows) continue;
        const uchar* row = clean.ptr<uchar>(ry);
        std::vector<double> run_centers;
        int begin = -1;
        for (int x = 0; x <= w; ++x) {
          const bool white = x < w && row[x] != 0;
          if (white && begin < 0) begin = x;
          if (!white && begin >= 0) {
            if (x - begin >= 2) run_centers.push_back((begin + x - 1) * 0.5);
            begin = -1;
          }
        }
        if (run_centers.size() < 2) continue;
        const double span = run_centers.back() - run_centers.front();
        const double level = static_cast<double>(y) / h;
        const double minimum_span = w * (0.13 + 0.27 *
            std::clamp((level - 0.35) / 0.56, 0.0, 1.0));
        if (span < minimum_span) continue;
        const double x_center = (run_centers.front() + run_centers.back()) * 0.5;
        if (!road_centers.empty() &&
            std::abs(x_center - road_centers.back().x) > w * 0.25) continue;
        road_centers.emplace_back(static_cast<float>(x_center),
                                  static_cast<float>(y));
      }
      struct Marker { cv::Point2f center; double area; cv::Rect rect; };
      std::vector<Marker> markers;
      markers.reserve(contours.size());
      const double max_area = w * h * 0.005;
      for (const auto& contour : contours) {
        const cv::Rect box = cv::boundingRect(contour);
        const double area = cv::contourArea(contour);
        if (area < 8 || area > max_area || box.width > w * 0.12 ||
            box.height > h * 0.25 || box.y + y0 < h * 0.34) continue;
        const auto moments = cv::moments(contour);
        if (moments.m00 <= 0.0) continue;
        const cv::Point2f center(
            static_cast<float>(moments.m10 / moments.m00),
            static_cast<float>(moments.m01 / moments.m00 + y0));
        // Reject apparent "dashes" sitting on the outer road borders.
        // Both borders are extracted from this SAME camera scanline.
        const int scan_y = std::clamp(cvRound(center.y) - y0, 0, mask.rows-1);
        std::vector<cv::Point> white_points;
        cv::findNonZero(mask.row(scan_y), white_points);
        if (white_points.size() >= 4) {
          const int leftmost = white_points.front().x;
          const int rightmost = white_points.back().x;
          const double lane_span = rightmost - leftmost;
          if (lane_span > w * 0.24) {
            const double margin = std::max(9.0, lane_span * 0.19);
            if (center.x < leftmost + margin ||
                center.x > rightmost - margin) continue;
          }
        }
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
              std::abs(marker.center.x - last_mark_x_) > w * 0.19) continue;
        } else if (std::abs(marker.center.x - previous_x) > w * 0.21) {
          continue;
        }
        tracked.push_back(marker.center);
        previous_x = marker.center.x;
        if (tracked.size() >= 8) break;
      }
      if (!tracked.empty()) {
        const float near_x = tracked.front().x;
        const float far_x = tracked.size() >= 2 ? tracked.back().x : near_x;
        near_error = std::clamp(-(near_x - w * 0.5) / (w * 0.5), -1.0, 1.0);
        far_error = std::clamp(-(far_x - w * 0.5) / (w * 0.5), -1.0, 1.0);
        curvature = std::clamp((near_x - far_x) / (w * 0.67), -1.0, 1.0);
        preview_visibility = tracked.size() >= 2
            ? std::clamp((tracked.front().y - tracked.back().y) / (h * 0.48), 0.0, 1.0)
            : 0.0;
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
        near_error = far_error = -error;
        curvature = 0;
        preview_visibility = 0;
        confidence = 0.58;
        cv::line(debug, cv::Point(cvRound(edge_target), h - 1),
                 cv::Point(cvRound(edge_target), h * 3 / 4),
                 cv::Scalar(255, 180, 0), 2);
      }
      if (road_centers.size() >= 5) {
        // Solid road boundaries are only a secondary visual measurement.
        // Two bright lines in a row can be the center dash plus ONE border,
        // so never let such a center override a credible tracked guide line.
        // Aggregate three rows at both depths to avoid pixel-level servo jitter.
        const size_t samples = std::min<size_t>(3, road_centers.size());
        double near_x = 0.0, far_x = 0.0;
        for (size_t index = 0; index < samples; ++index) {
          near_x += road_centers[index].x;
          far_x += road_centers[road_centers.size() - 1 - index].x;
        }
        near_x /= static_cast<double>(samples);
        far_x /= static_cast<double>(samples);
        const double visual_target =
            near_x * (1.0 - lookahead_weight_) + far_x * lookahead_weight_;
        const bool agrees_with_markers = tracked.size() >= 2 &&
            std::abs(near_x - tracked.front().x) < w * 0.17 &&
            std::abs(far_x - tracked.back().x) < w * 0.22;
        const bool visually_continuous = tracked.empty() &&
            std::abs(visual_target - last_mark_x_) < w * 0.20;
        if (agrees_with_markers || visually_continuous) {
          const double blend = agrees_with_markers ? 0.25 : 1.0;
          const double previous_pixel_target = w * 0.5 + error * w * 0.5;
          const double accepted_target = std::clamp(
              (1.0 - blend) * previous_pixel_target + blend * visual_target,
              0.0, static_cast<double>(w));
          error = std::clamp((accepted_target - w * 0.5) / (w * 0.5), -1.0, 1.0);
          near_error = std::clamp(
              near_error * (1.0 - blend) -
              blend * (near_x - w * 0.5) / (w * 0.5), -1.0, 1.0);
          far_error = std::clamp(
              far_error * (1.0 - blend) -
              blend * (far_x - w * 0.5) / (w * 0.5), -1.0, 1.0);
          curvature = std::clamp(
              curvature * (1.0 - blend) +
              blend * (near_x - far_x) / (w * 0.67), -1.0, 1.0);
          preview_visibility = std::max(preview_visibility, std::clamp(
              (road_centers.front().y - road_centers.back().y) / (h * 0.53),
              0.0, 1.0));
          confidence = std::max(confidence, visually_continuous ? 0.70 : 0.87);
          last_mark_x_ = static_cast<float>(accepted_target);
          for (size_t index = 0; index < road_centers.size(); ++index) {
            const auto& point = road_centers[index];
            cv::circle(debug, point, 3, cv::Scalar(235, 230, 40), -1);
            if (index > 0)
              cv::line(debug, road_centers[index - 1], point,
                       cv::Scalar(220, 200, 50), 1);
          }
          cv::line(debug, cv::Point(cvRound(accepted_target), h - 1),
                   cv::Point(cvRound(accepted_target), cvRound(h * .65)),
                   cv::Scalar(250, 220, 70), 2);
        }
      }
    } else {
      const auto road=rcauto_cpp::detect_road_markings(
          frame,lookahead_weight_,road_min_,road_max_,smin_,vmin_);
      lane_measurement=road;
      target_point=rcauto_cpp::find_reference_target(
          road,w,h,target_distance_ratio_);
      error=road.error;
      confidence=road.confidence;
      near_error=road.near_error;
      far_error=road.far_error;
      curvature=road.curvature;
      preview_visibility=road.preview;
      visible_edge_confidence=road.visible_edge_confidence;
      visible_edge_side=road.visible_edge_side;
      edge_only=road.edge_only;
      if(target_point && confidence>=0.4) {
        // A farther observed reference anticipates bends sooner.
        // Keep existing near/far measurements so the controller retains
        // feedback when a distant segment is momentarily occluded.
        const double reference_error=std::clamp(
            (target_point->x-w*.5)/(w*.5),-1.0,1.0);
        error=std::clamp(.65*error+.35*reference_error,-1.0,1.0);
        near_error=std::clamp(
            .80*near_error-.20*reference_error,-1.0,1.0);
        far_error=std::clamp(
            .65*far_error-.35*reference_error,-1.0,1.0);
      }
      // Only actual observed paint pixels are marked. Do not draw a
      // fitted curve through gaps or outside the camera image.
      if (show_debug_) {
        const auto draw_edge=[&](const std::vector<cv::Point>& edge,
                                const cv::Scalar& color) {
          for(size_t i=1;i<edge.size();++i) {
            if(std::abs(edge[i].y-edge[i-1].y)>9 ||
               std::abs(edge[i].x-edge[i-1].x)>12)continue;
            cv::line(debug,edge[i-1],edge[i],color,2,cv::LINE_AA);
          }
        };
        draw_edge(road.left_edge,cv::Scalar(35,245,40));
        draw_edge(road.right_edge,cv::Scalar(255,90,5));
        draw_edge(road.unclassified_edge,cv::Scalar(160,165,165));
        cv::drawContours(debug,road.center_dashes,-1,
                         cv::Scalar(0,220,255),2,cv::LINE_AA);
        // The unique target is drawn after cone avoidance, so Foxglove
        // never shows conflicting primary and avoidance targets.
      }
    }
    vision_msgs::msg::Detection2DArray detections;
    detections.header.stamp=get_clock()->now();
    detections.header.frame_id="camera_link";
    const auto cone_candidates=detect_cones_ ?
        rcauto_cpp::detect_orange_cones(frame) :
        std::vector<rcauto_cpp::OrangeCone>{};
    std::vector<rcauto_cpp::OrangeCone> cones;
    const auto cone_now=std::chrono::steady_clock::now();
    const bool recent_cone=last_cone_time_ !=
        std::chrono::steady_clock::time_point{} &&
        std::chrono::duration<double>(cone_now-last_cone_time_).count()<0.65;
    for(const auto& candidate:cone_candidates) {
      const cv::Point2d delta(candidate.box.x+candidate.box.width*0.5 -
                                  (last_cone_box_.x+last_cone_box_.width*0.5),
                              candidate.box.y+candidate.box.height*0.5 -
                                  (last_cone_box_.y+last_cone_box_.height*0.5));
      const bool consistent=recent_cone &&
          std::hypot(delta.x/w,delta.y/h)<0.15 &&
          candidate.box.height>last_cone_box_.height*0.50 &&
          candidate.box.height<last_cone_box_.height*1.90;
      if(candidate.score>=0.59 ||
         (consistent && candidate.score>=0.51)) {
        cones.push_back(candidate);
      }
    }
    if(!cones.empty()) {
      last_cone_box_=cones.front().box;
      last_cone_time_=cone_now;
    }
    std_msgs::msg::Bool cone_flag;
    cone_flag.data=!cones.empty();
    pub_cone_->publish(cone_flag);
    for(const auto& cone:cones) {
      vision_msgs::msg::Detection2D item;
      item.header=detections.header;
      item.bbox.center.position.x=cone.box.x+cone.box.width*0.5;
      item.bbox.center.position.y=cone.box.y+cone.box.height*0.5;
      item.bbox.size_x=cone.box.width;
      item.bbox.size_y=cone.box.height;
      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id="traffic_cone";
      hypothesis.hypothesis.score=cone.score;
      item.results.push_back(hypothesis);
      detections.detections.push_back(item);
      if (show_debug_) {
        cv::rectangle(debug,cone.box,cv::Scalar(45,235,45),2,cv::LINE_AA);
        cv::putText(debug,"CONE",
            cv::Point(cone.box.x,std::max(17,cone.box.y-5)),
            cv::FONT_HERSHEY_SIMPLEX,0.6,
            cv::Scalar(45,235,45),2,cv::LINE_AA);
      }
    }
    const auto avoidance=(avoid_enabled_ && lane_measurement) ?
        rcauto_cpp::choose_cone_avoidance(
            *lane_measurement,cones,w,h,avoid_offset_ratio_) :
        rcauto_cpp::ConeAvoidance{};
    if(avoidance.active) {
      // The controller uses vehicle-left-positive errors, while this
      // visual measurement is still image-right-positive at this point.
      error=std::clamp(error+avoidance.offset_norm,-1.0,1.0);
      near_error=std::clamp(near_error-avoidance.offset_norm,-1.0,1.0);
      far_error=std::clamp(far_error-avoidance.offset_norm,-1.0,1.0);
      if(target_point) {
        // One steering target only. Avoidance overrides its horizontal
        // destination while the lookahead slider controls image depth.
        target_point->x=avoidance.target.x;
      }
    } else if(avoidance.blocked && show_debug_) {
      cv::putText(debug,"CONE: NO CLEAR PASSAGE",
                  {std::max(8,w/4),std::max(25,h/3)},
                  cv::FONT_HERSHEY_SIMPLEX,.58,
                  cv::Scalar(25,50,255),2,cv::LINE_AA);
    }
    if(show_debug_ && target_point) {
      // Exactly one purple target marker in road mode. No false
      // trajectory is drawn through an occluding cone.
      cv::circle(debug,*target_point,10,
                 cv::Scalar(225,30,230),-1,cv::LINE_AA);
      cv::circle(debug,*target_point,12,
                 cv::Scalar(255,255,255),1,cv::LINE_AA);

    }
    std_msgs::msg::Bool active_msg;
    active_msg.data=avoidance.active;
    pub_avoid_active_->publish(active_msg);
    std_msgs::msg::Bool blocked_msg;
    blocked_msg.data=avoidance.blocked;
    pub_avoid_blocked_->publish(blocked_msg);
    std_msgs::msg::Float32 offset_msg;
    offset_msg.data=static_cast<float>(avoidance.offset_norm);
    pub_avoid_offset_->publish(offset_msg);
    // Image +X points right, whereas ROS Ackermann +steering turns left.
    // Convert pixel displacement into vehicle-left-positive steering error.
    error = -error;
    // Near/far/curve telemetry uses vehicle-left-positive coordinates too.
    // Far preview is measured ONLY from visible perspective-projected image markers.
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
    const auto send = [](const auto& publisher, double value) {
      std_msgs::msg::Float32 m;
      m.data = static_cast<float>(value);
      publisher->publish(m);
    };
    send(pub_near_, near_error);
    send(pub_far_, far_error);
    send(pub_curvature_, curvature);
    send(pub_preview_, preview_visibility);
    error = filtered_error_;
    std_msgs::msg::Float32 err;err.data=static_cast<float>(error);pub_error_->publish(err);
    std_msgs::msg::Float32 conf;conf.data=static_cast<float>(confidence);pub_conf_->publish(conf);
    send(pub_edge_conf_,visible_edge_confidence);
    std_msgs::msg::Int32 boundary_side;
    boundary_side.data=visible_edge_side;
    pub_edge_side_->publish(boundary_side);
    std_msgs::msg::Bool recovery;
    recovery.data=edge_only;
    pub_edge_only_->publish(recovery);
    std_msgs::msg::Bool cross;cross.data=intersection;pub_cross_->publish(cross);
    pub_cone_boxes_->publish(detections);
    if(show_debug_ &&
       (pub_debug_->get_subscription_count()>0 ||
        pub_jpeg_->get_subscription_count()>0 ||
        pub_eco_jpeg_->get_subscription_count()>0)) {
      std_msgs::msg::Header header;
      header.stamp=detections.header.stamp;
      header.frame_id="camera_link";
      if(pub_debug_->get_subscription_count()>0)
        pub_debug_->publish(
            *cv_bridge::CvImage(header,"bgr8",debug).toImageMsg());
      if(pub_jpeg_->get_subscription_count()>0) {
        std::vector<unsigned char> encoded;
        if(cv::imencode(".jpg",debug,encoded,
                        {cv::IMWRITE_JPEG_QUALITY,jpeg_quality_})) {
          sensor_msgs::msg::CompressedImage image;
          image.header=header;
          image.format="jpeg";
          image.data=std::move(encoded);
          pub_jpeg_->publish(image);
        }
      }
      // Preserve image dimensions and frame rate on constrained links.
      // Only the JPEG encoding bitrate differs from the normal stream.
      if(pub_eco_jpeg_->get_subscription_count()>0) {
        constexpr int kBandwidthJpegQuality=48;
        std::vector<unsigned char> encoded;
        if(cv::imencode(".jpg",debug,encoded,
                        {cv::IMWRITE_JPEG_QUALITY,kBandwidthJpegQuality})) {
          sensor_msgs::msg::CompressedImage image;
          image.header=header;
          image.format="jpeg";
          image.data=std::move(encoded);
          pub_eco_jpeg_->publish(image);
        }
      }
    }
  }
  cv::VideoCapture camera_;
  std::string style_,color_,camera_preference_,active_device_,camera_state_;
  bool camera_connected_{false};
  int failed_reads_{0};
  std::chrono::steady_clock::time_point last_camera_attempt_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_camera_status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_camera_connected_;
  bool pending_{false},show_debug_{true},hardware_mode_{false},
       detect_cones_{true},avoid_enabled_{true};
  double avoid_offset_ratio_{0.50};
  int jpeg_quality_{72};
  int capture_width_{1280},capture_height_{720},output_width_{1080};
  int hmin_{15},hmax_{42},smin_{70},vmin_{80};
  double road_min_{.22},road_max_{.89},lookahead_weight_{.72},
      target_distance_ratio_{.55};
  double smoothing_alpha_{.48},filtered_error_{0.0};
  bool have_filtered_error_{false};
  float last_mark_x_{320.0F};
  cv::Rect last_cone_box_;
  std::chrono::steady_clock::time_point last_cone_time_{};
  double edge_half_width_px_{210.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_error_,pub_raw_error_,pub_conf_,
      pub_near_,pub_far_,pub_curvature_,pub_preview_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_edge_conf_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_edge_side_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_edge_only_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_cross_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      pub_jpeg_,pub_raw_jpeg_,pub_eco_jpeg_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      pub_avoid_active_,pub_avoid_blocked_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_avoid_offset_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_cone_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_cone_boxes_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<VisionNode>());rclcpp::shutdown();return 0;
}
