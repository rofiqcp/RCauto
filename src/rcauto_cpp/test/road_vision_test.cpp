#include "rcauto_cpp/road_vision.hpp"
#include "rcauto_cpp/target_reference.hpp"

#include <opencv2/imgproc.hpp>

#include <cassert>
#include <cmath>
#include <iostream>

int main() {
  constexpr int width=800, height=450;
  cv::Mat road(height,width,CV_8UC3,cv::Scalar(88,88,88));
  // The left border is continuous, not a dashed centre reference.
  cv::line(road,{20,430},{585,145},cv::Scalar(245,245,245),14,cv::LINE_AA);
  // A tight turn brings the middle dashes against the frame boundary.
  for(const auto& dash:{
      cv::Rect(692,356,89,43),cv::Rect(725,278,67,35),
      cv::Rect(758,219,42,27),cv::Rect(764,167,34,22)}) {
    cv::rectangle(road,dash,cv::Scalar(246,246,246),cv::FILLED);
  }
  const auto check=[&](const cv::Mat& frame) {
    const auto observed=rcauto_cpp::detect_road_markings(
        frame,.78,.22,.89,70,88);
    const auto target=rcauto_cpp::find_reference_target(
        observed,width,height,.45);
    assert(observed.centers.size()>=3);
    assert(target.has_value());
    assert(target->x>=width*.72 && target->x<width);
    assert(target->y>=height*.30 && target->y<=height*.95);
    return observed;
  };
  const auto baseline=check(road);
  // A broad reflection may touch the nearest painted dash, so its
  // bright core must remain an independent centerline observation.
  cv::Mat reflective=road.clone();
  cv::ellipse(reflective,{630,380},{143,51},0,0,360,
              cv::Scalar(188,188,188),cv::FILLED);
  cv::rectangle(reflective,{692,356,89,43},
                cv::Scalar(246,246,246),cv::FILLED);
  check(reflective);
  cv::Mat dim;
  road.convertTo(dim,-1,.58);
  check(dim);
  cv::Mat bright;
  road.convertTo(bright,-1,1.36);
  check(bright);
  cv::Mat sensor_noise(road.size(),CV_16SC3);
  cv::RNG rng(12345);
  rng.fill(sensor_noise,cv::RNG::NORMAL,cv::Scalar::all(0),cv::Scalar::all(12));
  cv::Mat noisy,noise_source;
  road.convertTo(noise_source,CV_16SC3);
  cv::add(noise_source,sensor_noise,noise_source);
  noise_source.convertTo(noisy,CV_8UC3);
  check(noisy);
  cv::Mat empty(height,width,CV_8UC3,cv::Scalar(88,88,88));
  const auto none=rcauto_cpp::detect_road_markings(
      empty,.78,.22,.89,70,88);
  assert(none.centers.empty());
  assert(!rcauto_cpp::find_reference_target(none,width,height,.45));
  cv::ellipse(empty,{610,370},{130,48},0,0,360,
              cv::Scalar(188,188,188),cv::FILLED);
  const auto glare_only=rcauto_cpp::detect_road_markings(
      empty,.78,.22,.89,70,88);
  assert(glare_only.centers.empty());
  std::cout<<"PASS: edge-cropped yellow centre, noise, exposure, target, empty road; observed "
           <<baseline.centers.size()<<" markings\n";
}
