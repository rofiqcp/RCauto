#pragma once

#include "rcauto_cpp/road_vision.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace rcauto_cpp {

// A display reference can be provisional when only one painted boundary is
// visible. Controller safety and lane confidence remain independent.
inline std::optional<cv::Point> find_reference_target(
    const LaneObservation& lane, int width, int height,
    double distance_ratio) {
  if(width<64 || height<48 || !std::isfinite(distance_ratio) ||
     distance_ratio<0.15 || distance_ratio>0.85) return std::nullopt;

  const bool has_center=!lane.centers.empty();
  const bool has_left=lane.left_edge.size()>=15;
  const bool has_right=lane.right_edge.size()>=15;
  if(!lane.reference_valid && !has_center && !has_left && !has_right)
    return std::nullopt;

  const int requested_y=cvRound(height*(1.0-distance_ratio));
  int target_y=requested_y;
  int x=lane.reference_point.x;
  bool found=false;

  const auto nearest=[](const std::vector<cv::Point>& line,int y) {
    return std::min_element(line.begin(),line.end(),
        [y](const cv::Point& a,const cv::Point& b) {
          return std::abs(a.y-y)<std::abs(b.y-y);
        });
  };

  if(has_center) {
    auto marks=lane.centers;
    std::sort(marks.begin(),marks.end(),
        [](const cv::Point& a,const cv::Point& b){return a.y<b.y;});
    if(marks.size()==1) {
      x=marks.front().x;
      target_y=marks.front().y;
      found=true;
    } else {
      // Move the lookahead toward the farthest actual dash as the
      // observed curve becomes sharper. Never project into invisible road.
      const double bend=std::abs(marks.back().x-marks.front().x)/
                        static_cast<double>(width);
      const double anticipation=std::clamp((bend-.12)/.30,0.0,1.0);
      target_y=std::clamp(
          cvRound(requested_y*(1.0-anticipation)+
                  marks.front().y*anticipation),
          marks.front().y,marks.back().y);
      const int sample=target_y;
      const auto high=std::lower_bound(
          marks.begin(),marks.end(),sample,
          [](const cv::Point& a,int row){return a.y<row;});
      if(high==marks.begin())x=high->x;
      else if(high==marks.end())x=marks.back().x;
      else {
        const auto low=std::prev(high);
        const double t=static_cast<double>(sample-low->y)/
                       std::max(1,high->y-low->y);
        x=cvRound((1.0-t)*low->x+t*high->x);
      }
      found=true;
    }
  }

  if(!found && has_left && has_right) {
    const auto left=nearest(lane.left_edge,requested_y);
    const auto right=nearest(lane.right_edge,requested_y);
    if(std::abs(left->y-requested_y)<height*.16 &&
       std::abs(right->y-requested_y)<height*.16 &&
       right->x-left->x>width*.20) {
      x=(left->x+right->x)/2;
      target_y=(left->y+right->y)/2;
      found=true;
    }
  }

  if(!found) {
    // Maintain a visible, clearly provisional aim point with a single
    // real road edge. The perspective-based half-width is for display
    // only and never increases the lane-confidence score.
    const double half_width=width*std::clamp(
        .10+.31*static_cast<double>(requested_y)/height,.14,.40);
    for(const auto& candidate:{
        std::pair<const std::vector<cv::Point>*,int>{&lane.left_edge,1},
        {&lane.right_edge,-1}}) {
      if(candidate.first->size()<15)continue;
      const auto point=nearest(*candidate.first,requested_y);
      if(std::abs(point->y-requested_y)>height*.20)continue;
      x=cvRound(point->x+candidate.second*half_width);
      target_y=point->y;
      found=true;
      break;
    }
  }

  if(!found && lane.reference_valid) {
    x=lane.reference_point.x;
    target_y=lane.reference_point.y;
    found=true;
  }
  if(!found)return std::nullopt;
  return cv::Point(std::clamp(x,0,width-1),
                   std::clamp(target_y,0,height-1));
}

}  // namespace rcauto_cpp
