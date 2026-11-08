#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
std::string class_name(std::string s) {
  std::transform(s.begin(),s.end(),s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if(s=="0") return "person";
  if(s=="1") return "bicycle";
  if(s=="2") return "car";
  if(s=="3") return "motorcycle";
  if(s=="5") return "bus";
  if(s=="7") return "truck";
  if(s=="9") return "traffic_light";
  std::replace(s.begin(),s.end(),' ','_');
  return s;
}
std::string light_color(const cv::Mat& crop, double required_fraction) {
  if(crop.empty()||crop.rows<5||crop.cols<5)return "UNKNOWN";
  cv::Mat hsv;cv::cvtColor(crop,hsv,cv::COLOR_BGR2HSV);
  int red=0,yellow=0,green=0;
  for(int y=0;y<hsv.rows;++y) {
    const auto* row=hsv.ptr<cv::Vec3b>(y);
    for(int x=0;x<hsv.cols;++x) {
      const int h=row[x][0],s=row[x][1],v=row[x][2];
      if(v<95||s<110)continue;
      if(h<=10||h>=170)++red;
      else if(h>=18&&h<=37)++yellow;
      else if(h>=40&&h<=88)++green;
    }
  }
  const int n=crop.rows*crop.cols;
  const int best=std::max({red,yellow,green});
  if(best<std::max(4,static_cast<int>(n*required_fraction)))return "UNKNOWN";
  const int second=(red+yellow+green)-best;
  if(best<=second*2)return "UNKNOWN";
  if(best==red)return "RED";
  if(best==yellow)return "YELLOW";
  return "GREEN";
}
}

class Perception final : public rclcpp::Node {
 public:
  Perception():Node("rcauto_cpp_perception") {
    declare_parameter("detection.min_score",0.40);
    declare_parameter("detection.hardware_mode",false);
    declare_parameter("detection.hazard_corridor_left",0.23);
    declare_parameter("detection.hazard_corridor_right",0.77);
    declare_parameter("detection.hazard_bottom_min",0.42);
    declare_parameter("detection.green_min_fraction",0.06);
    declare_parameter("detection.max_image_age_s",0.3);
    hardware_mode_=get_parameter("detection.hardware_mode").as_bool();
    refresh();
    callback_handle_=add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& p) {
        rcl_interfaces::msg::SetParametersResult result;result.successful=true;
        double candidate_left=left_,candidate_right=right_;
        for(const auto& value:p) {
          const auto& name=value.get_name();
          if(name.rfind("detection.",0)!=0)continue;
          if(hardware_mode_ || name=="detection.hardware_mode") {
            result.successful=false;
            result.reason="Hardware or perception tuning is immutable in hardware mode";
            break;
          }
          try {
            const double x=value.as_double();
            if(!std::isfinite(x)||x<0||x>1)
              throw std::invalid_argument("detection parameter must be 0..1");
            if(name=="detection.hazard_corridor_left")candidate_left=x;
            if(name=="detection.hazard_corridor_right")candidate_right=x;
          }catch(const std::exception& e) {
            result.successful=false;result.reason=e.what();break;
          }
        }
        if(result.successful && candidate_left>=candidate_right) {
          result.successful=false;
          result.reason="Hazard corridor left must be smaller than right";
        }
        if(result.successful)pending_=true;
        return result;
      });
    pub_hazard_=create_publisher<std_msgs::msg::Bool>("/rcauto/hazard",10);
    pub_signal_=create_publisher<std_msgs::msg::String>("/rcauto/signal",10);
    sub_image_=create_subscription<sensor_msgs::msg::Image>("/rcauto/camera/image",3,
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
        try { frame_=cv_bridge::toCvCopy(m,"bgr8")->image;
              image_time_=Clock::now(); }
        catch(const cv_bridge::Exception& e) {
          RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Image: %s",e.what());
        }
      });
    sub_detect_=create_subscription<vision_msgs::msg::Detection2DArray>(
      "/rcauto/detections",10,
      [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr m){detect(*m);});
    RCLCPP_INFO(get_logger(),"C++ Hailo detection adapter ready; waiting on vision_msgs detections");
  }
 private:
  void refresh() {
    score_=get_parameter("detection.min_score").as_double();
    left_=get_parameter("detection.hazard_corridor_left").as_double();
    right_=get_parameter("detection.hazard_corridor_right").as_double();
    bottom_=get_parameter("detection.hazard_bottom_min").as_double();
    green_fraction_=get_parameter("detection.green_min_fraction").as_double();
    image_age_=get_parameter("detection.max_image_age_s").as_double();
    if(left_>=right_)throw std::invalid_argument("hazard corridor left must be < right");
  }
  void detect(const vision_msgs::msg::Detection2DArray& msg) {
    if(pending_) {pending_=false;refresh();}
    const bool image_fresh=!frame_.empty() &&
      std::chrono::duration<double>(Clock::now()-image_time_).count()<image_age_;
    // Missing image => stop: hazard=True to prevent false negatives without localization.
    bool hazard=!image_fresh;
    std::vector<std::string> lights;
    const double w=frame_.cols,h=frame_.rows;
    if(image_fresh)for(const auto& d:msg.detections) {
      if(d.results.empty())continue;
      const auto* top=&d.results.front();
      for(const auto& hyp:d.results)
        if(hyp.hypothesis.score>top->hypothesis.score)top=&hyp;
      if(top->hypothesis.score<score_)continue;
      const auto name=class_name(top->hypothesis.class_id);
      const double cx=d.bbox.center.position.x, cy=d.bbox.center.position.y;
      const double sx=d.bbox.size_x,sy=d.bbox.size_y;
      if(!std::isfinite(cx)||!std::isfinite(cy)||!std::isfinite(sx)||!std::isfinite(sy) ||
         sx<=0||sy<=0)continue;
      const double xa=cx-sx*.5,xb=cx+sx*.5,yb=cy+sy*.5;
      if(name=="person"||name=="car"||name=="truck"||name=="bus"||
         name=="motorcycle"||name=="bicycle") {
        if(xb>left_*w&&xa<right_*w&&yb>=bottom_*h)hazard=true;
      }else if(name=="traffic_light") {
        const auto clip=[](double v,int limit){return std::clamp(static_cast<int>(v),0,limit);};
        const int x1=clip(xa,frame_.cols),x2=clip(xb,frame_.cols);
        const int y1=clip(cy-sy*.5,frame_.rows),y2=clip(yb,frame_.rows);
        if(x2>x1&&y2>y1)lights.push_back(
            light_color(frame_(cv::Rect(x1,y1,x2-x1,y2-y1)),green_fraction_));
      }
    }
    std::string signal="UNKNOWN";
    if(lights.size()==1)signal=lights.front();
    else if(lights.size()>1&&std::all_of(lights.begin(),lights.end(),
               [&](const std::string& s){return s==lights[0];})) signal=lights[0];
    std_msgs::msg::Bool hmsg;hmsg.data=hazard;pub_hazard_->publish(hmsg);
    std_msgs::msg::String smsg;smsg.data=signal;pub_signal_->publish(smsg);
  }
  bool pending_{false},hardware_mode_{false};
  double score_{.4},left_{.23},right_{.77},bottom_{.42},green_fraction_{.06},image_age_{.3};
  cv::Mat frame_;
  Clock::time_point image_time_{};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_detect_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hazard_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_signal_;
};
int main(int argc,char**argv){
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Perception>());rclcpp::shutdown();return 0;
}
