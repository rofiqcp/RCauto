#pragma once
#include "rcauto_cpp/road_vision.hpp"
#include "rcauto_cpp/traffic_cone.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace rcauto_cpp {

struct ConeAvoidance {
  bool active{false};
  bool blocked{false};
  int direction{0};  // -1: left in image, +1: right
  cv::Point target;
  double offset_norm{0.0};
  double available_half_width{0.0};
};

inline ConeAvoidance choose_cone_avoidance(
    const LaneObservation& lane,
    const std::vector<OrangeCone>& cones,
    int image_width,
    int image_height,
    double offset_ratio) {
  ConeAvoidance result;
  if(cones.empty() || image_width<100 || image_height<100 ||
     !std::isfinite(offset_ratio) || offset_ratio<0.0 ||
     offset_ratio>1.0)return result;

  const auto critical_object=[&](const OrangeCone& cone) {
    const double center_x=cone.box.x+cone.box.width*.5;
    return cone.box.y+cone.box.height>image_height*.25 &&
        cone.box.width>=image_width*.035 &&
        center_x>image_width*.12 && center_x<image_width*.88;
  };
  if(lane.confidence<0.4 || lane.centers.size()<3) {
    // A near-center cone with an uncertain lane is a STOP, not an
    // invitation to proceed straight through an untracked obstacle.
    for(const auto& cone:cones)
      if(critical_object(cone))result.blocked=true;
    return result;
  }

  for (const auto& cone:cones) {
    const int cone_y=std::clamp(
        cone.box.y+cone.box.height,0,image_height-1);
    if(cone_y<image_height*.22)continue;
    if(cone_y>image_height*.94) {
      if(critical_object(cone))result.blocked=true;
      continue;
    }
    const double cone_x=cone.box.x+cone.box.width*.5;
    const auto center=std::min_element(lane.centers.begin(),lane.centers.end(),
        [&](const cv::Point& a,const cv::Point& b){
          return std::abs(a.y-cone_y)<std::abs(b.y-cone_y);
        });
    if(center==lane.centers.end() ||
       std::abs(center->y-cone_y)>image_height*.30)continue;
    const double guide_x=center->x;
    const auto closest_boundary=[&](const std::vector<cv::Point>& points) {
      if(points.size()<12)return -1.0;
      const auto match=std::min_element(points.begin(),points.end(),
          [&](const cv::Point& a,const cv::Point& b) {
            return std::abs(a.y-cone_y)<std::abs(b.y-cone_y);
          });
      return std::abs(match->y-cone_y)<image_height*.16 ?
          static_cast<double>(match->x) : -1.0;
    };
    const double left_x=closest_boundary(lane.left_edge);
    const double right_x=closest_boundary(lane.right_edge);
    double available_left=left_x>=0?guide_x-left_x:0.0;
    double available_right=right_x>=0?right_x-guide_x:0.0;
    if(available_left<image_width*.10 && available_right<image_width*.10) {
      if(std::abs(cone_x-guide_x)<
         std::max(image_width*.20,cone.box.width*.8))
        result.blocked=true;
      continue;
    }

    // This object should interfere with the lane center path, not be
    // outside the roadway. A large nearby cone cannot safely be passed.
    const double maximum_half=std::max(available_left,available_right);
    if(std::abs(cone_x-guide_x)>maximum_half+cone.box.width*.4)
      continue;

    // Move away from the cone and prefer the side with larger visible
    // clearance when a cone is centered on the guide.
    const double center_tolerance=std::max(12.0,image_width*.045);
    int side=cone_x>guide_x+center_tolerance ? -1 :
             cone_x<guide_x-center_tolerance ? 1 :
             available_left>=available_right ? -1 : 1;
    double free_half=side<0?available_left:available_right;
    if(free_half<image_width*.10) {
      side=-side;
      free_half=side<0?available_left:available_right;
    }

    result.direction=side;
    result.available_half_width=free_half;
    result.target=cv::Point(cvRound(std::clamp(
        guide_x+side*offset_ratio*free_half,
        0.0,static_cast<double>(image_width-1))),center->y);

    // A target still inside the cone silhouette is not a valid path.
    const double clearance=std::abs(result.target.x-cone_x)-
                           cone.box.width*.5;
    const double safety_margin=std::max(14.0,image_width*.045);
    result.blocked=free_half<image_width*.10 ||
                   clearance<safety_margin;
    result.active=!result.blocked;
    if(result.active)
      result.offset_norm=side*offset_ratio*free_half/
                         (image_width*.5);
    return result;
  }
  return result;
}

}  // namespace rcauto_cpp
