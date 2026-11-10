#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace rcauto_cpp {

struct OrangeCone {
  cv::Rect box;
  double score;
};

inline std::vector<OrangeCone> detect_orange_cones(const cv::Mat& frame) {
  if(frame.empty()||frame.type()!=CV_8UC3)return {};
  cv::Mat hsv;
  cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
  std::vector<OrangeCone> result;
  const cv::Rect floor_sample(frame.cols/4,frame.rows*3/5,
                              frame.cols/2,frame.rows*2/5);
  const double ambient_value=cv::mean(hsv(floor_sample))[2];
  // Two complementary masks: permissive for distant/dim cones, strictly
  // saturated orange for nearby cones that visually merge into wood.
  for(const auto& threshold:std::array<std::array<int,3>,2>{{
          {{48,43,108}},{{90,125,120}}}}) {
    cv::Mat mask;
    cv::inRange(hsv,cv::Scalar(2,threshold[0],threshold[1]),
                cv::Scalar(28,255,255),mask);
  for(int y=0;y<frame.rows;++y) {
    const auto* pixels=frame.ptr<cv::Vec3b>(y);
    auto* row=mask.ptr<unsigned char>(y);
    for(int x=0;x<frame.cols;++x) {
      if(!row[x])continue;
      const auto& bgr=pixels[x];
      const int b=bgr[0],g=bgr[1],r=bgr[2];
      if(r*100<g*threshold[2] || g*100<b*103)row[x]=0;
    }
  }
  cv::morphologyEx(mask,mask,cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3)));
  cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5)));
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask.clone(),contours,cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  for(const auto& contour:contours) {
    const cv::Rect box=cv::boundingRect(contour);
    const double area=cv::contourArea(contour);
    const double aspect=static_cast<double>(box.height)/box.width;
    if(box.width<23 || box.height<42 || area<500 ||
       aspect<1.35 || aspect>4.0)continue;
    const double fill=area/box.area();
    if(fill<0.31 || fill>0.94)continue;

    constexpr std::array<double,5> levels{0.20,0.38,0.54,0.70,0.84};
    std::array<int,levels.size()> widths{};
    std::array<double,levels.size()> centers{};
    bool complete=true;
    for(size_t i=0;i<levels.size();++i) {
      const int y=box.y+cvRound((box.height-1)*levels[i]);
      const auto* row=mask.ptr<unsigned char>(y);
      int left=box.x+box.width,right=-1;
      for(int x=box.x;x<box.x+box.width;++x) {
        if(row[x]) {
          left=std::min(left,x);
          right=x;
        }
      }
      if(right-left<5) {
        complete=false;
        break;
      }
      widths[i]=right-left+1;
      centers[i]=(left+right)*0.5;
    }
    if(!complete)continue;
    const double taper=static_cast<double>(widths.back())/widths.front();
    if(taper<1.16 || taper>4.5)continue;
    const auto limits=std::minmax_element(centers.begin(),centers.end());
    const double drift=(*limits.second-*limits.first)/box.width;
    if(drift>0.41)continue;
    int widening=0;
    for(size_t i=1;i<widths.size();++i)
      if(widths[i]>=widths[i-1]-std::max(5,box.width/12))
        ++widening;
    if(widening<2)continue;

    double dark_base=0.0;
    const int base_y=box.y+box.height;
    const int base_height=std::min(std::max(4,box.height/9),
                                   frame.rows-base_y);
    if(base_height>0) {
      const int inset=box.width/10;
      const cv::Rect base(box.x+inset,base_y,box.width-2*inset,
                          base_height);
      cv::Mat dark;
      cv::inRange(hsv(base),cv::Scalar(0,0,0),
                  cv::Scalar(179,255,105),dark);
      dark_base=static_cast<double>(cv::countNonZero(dark))/static_cast<double>(dark.total());
    }
    const auto color=cv::mean(hsv(box),mask(box));
    if(color[2]<ambient_value+24.0)continue;
    const double score=
        0.24+0.18*std::clamp((taper-1.16)/0.84,0.0,1.0)
        +0.15*std::clamp(1.0-drift/0.41,0.0,1.0)
        +0.16*std::clamp((fill-0.31)/0.40,0.0,1.0)
        +0.16*std::clamp(color[1]/150.0,0.0,1.0)
        +0.11*std::clamp(dark_base*2.0,0.0,1.0);
    if(score>=0.50)result.push_back({box,score});
  }
  }
  std::sort(result.begin(),result.end(),
            [](const OrangeCone& a,const OrangeCone& b){
              return a.score>b.score;
            });
  std::vector<OrangeCone> unique;
  for(const auto& candidate:result) {
    bool duplicate=false;
    for(const auto& accepted:unique) {
      const cv::Rect intersection=candidate.box & accepted.box;
      const double overlap=static_cast<double>(intersection.area())/
          std::min(candidate.box.area(),accepted.box.area());
      if(overlap>0.55) {
        duplicate=true;
        break;
      }
    }
    if(!duplicate)unique.push_back(candidate);
  }
  return unique;
}

}  // namespace rcauto_cpp
