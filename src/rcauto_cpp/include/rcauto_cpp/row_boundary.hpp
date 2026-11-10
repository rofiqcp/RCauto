// Independent, conservative scanline recovery for white road boundaries
// partly merged with larger white connected components (feet, rolled mats).
// This module only returns real observed pixel coordinates.
#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

namespace rcauto_cpp {

inline std::vector<cv::Point> track_partial_road_edge(
    const cv::Mat& mask,
    const cv::Mat& gray,
    int side,
    double road_luminance,
    const std::vector<cv::Point>& center_markings={}) {
  if(mask.empty() || mask.type()!=CV_8UC1 ||
     gray.size()!=mask.size() || gray.type()!=CV_8UC1 ||
     (side!=-1 && side!=1))return {};
  const int w=mask.cols,h=mask.rows;
  if(w<100 || h<100)return {};
  struct Sample {
    cv::Point point;
    int predecessor{-1};
    int length{1};
    double score{1.0};
  };
  std::vector<Sample> samples;
  std::vector<std::vector<int>> row_ids;
  const int ystart=cvRound(h*.38);
  const int yend=std::min(h-2,cvRound(h*.97));
  for(int y=ystart;y<=yend;y+=3) {
    row_ids.emplace_back();
    const auto* row=mask.ptr<uchar>(y);
    const auto* luma=gray.ptr<uchar>(y);
    int start=-1;
    for(int x=0;x<=w;++x) {
      const bool white=x<w && row[x]!=0;
      if(white && start<0)start=x;
      if(!white && start>=0) {
        const int len=x-start;
        const int cx=(start+x-1)/2;
        const double t=static_cast<double>(y)/h;
        const int max_width=cvRound(w*(.035+.13*t*t));
        double guide=w*.50;
        if(center_markings.size()>=2) {
          const auto nearest=std::min_element(
              center_markings.begin(),center_markings.end(),
              [y](const cv::Point& a,const cv::Point& b){
                return std::abs(a.y-y)<std::abs(b.y-y);
              });
          // Do not extrapolate a road center into regions unseen by the
          // camera. The nearest observed dash is only a side reference.
          if(nearest!=center_markings.end() &&
             std::abs(nearest->y-y)<h*.24)
            guide=nearest->x;
        }
        if(len>=5 && len<=max_width &&
           (side<0 ? cx<guide-w*.055 : cx>guide+w*.055)) {
          const int road_x=side<0 ?
              std::min(w-5,x+std::max(14,len/5)):
              std::max(4,start-std::max(14,len/5));
          int road_sum=0,paint_sum=0;
          for(int dx=-4;dx<=4;++dx)
            road_sum+=luma[std::clamp(road_x+dx,0,w-1)];
          for(int px=start;px<x;++px)paint_sum+=luma[px];
          const double road=road_sum/9.0;
          const double paint=static_cast<double>(paint_sum)/len;
          const int rx=std::clamp(road_x-6,0,w-13);
          const int ry=std::clamp(y-5,0,h-11);
          cv::Scalar mean,stddev;
          cv::meanStdDev(gray(cv::Rect(rx,ry,13,11)),mean,stddev);
          // A black trouser leg is darker than the actual asphalt.
          // Require the supporting surface to fall within the observed
          // road luminance band rather than accepting any dark object.
          const double road_min=std::max(20.0,road_luminance*.48);
          if(road>=road_min && road<road_luminance+58 &&
             paint-road>=25 && stddev[0]>=1.05) {
            const int id=static_cast<int>(samples.size());
            const double score=1.0+
                std::min(0.5,(paint-road-25)/90.0);
            samples.push_back({{cx,y},-1,1,score});
            row_ids.back().push_back(id);
          }
        }
        start=-1;
      }
    }
  }
  for(size_t row=0;row<row_ids.size();++row) {
    for(int id:row_ids[row]) {
      auto& item=samples[id];
      for(size_t gap=1;gap<=3 && gap<=row;++gap) {
        for(int previous_id:row_ids[row-gap]) {
          const auto& prior=samples[previous_id];
          const double shift=std::abs(item.point.x-prior.point.x);
          const double max_shift=
              15.0+9.0*static_cast<double>(gap);
          if(shift>max_shift)continue;
          const double score=prior.score+1.0-
              .012*shift-.35*static_cast<double>(gap-1);
          if(score>item.score) {
            item.predecessor=previous_id;
            item.length=prior.length+1;
            item.score=score;
          }
        }
      }
    }
  }
  int best=-1;
  double best_score=0.0;
  for(size_t i=0;i<samples.size();++i) {
    const auto& item=samples[i];
    if(item.length<18)continue;
    if(item.score>best_score) {
      best=static_cast<int>(i);
      best_score=item.score;
    }
  }
  std::vector<cv::Point> track;
  while(best>=0) {
    const auto& item=samples[best];
    track.push_back(item.point);
    best=item.predecessor;
  }
  std::reverse(track.begin(),track.end());
  if(track.size()<18 ||
     track.back().y-track.front().y<h*.13)return {};

  // Retain observations only. Median comparison may REMOVE outliers,
  // but must never interpolate over missing or obstructed road paint.
  std::vector<cv::Point> stable;
  stable.reserve(track.size());
  for(size_t i=0;i<track.size();++i) {
    std::vector<int> neighborhood;
    for(size_t j=i>3?i-3:0;j<std::min(track.size(),i+4);++j)
      neighborhood.push_back(track[j].x);
    auto median=neighborhood.begin()+neighborhood.size()/2;
    std::nth_element(neighborhood.begin(),median,neighborhood.end());
    if(std::abs(track[i].x-*median)<20)
      stable.push_back(track[i]);
  }
  if(stable.size()<18 ||
     stable.back().y-stable.front().y<h*.13)return {};
  // A genuine close-range boundary reaches the lower camera field.
  // Isolated furniture edges above the road can otherwise form long,
  // narrow, high-contrast chains near the image sides.
  if(stable.back().y<h*.84 && stable.front().y<h*.60)return {};
  return stable;
}
} // namespace rcauto_cpp
