#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace rcauto_cpp {

struct SolidBoundaryTrack {
  std::vector<cv::Point> points;
  int side{0};  // -1 left, +1 right, 0 unclassified
  double confidence{0.0};
};

// Recover visible solid road paint when a curved camera view contains too
// few dashed-center observations for a polynomial lane fit. The white
// perimeter must be thin, continuous and next to actual dark road surface.
inline SolidBoundaryTrack detect_solid_boundary_fallback(
    const cv::Mat& white, const cv::Mat& gray,
    const std::vector<cv::Point>& center_candidates,
    double road_luminance) {
  SolidBoundaryTrack best;
  if(white.empty() || gray.empty() ||
     white.size()!=gray.size() || white.type()!=CV_8UC1 ||
     gray.type()!=CV_8UC1)return best;
  const int width=white.cols,height=white.rows;
  cv::Mat labels,stats,centroids;
  const int components=cv::connectedComponentsWithStats(
      white,labels,stats,centroids,8,CV_32S);
  double best_quality=0.0;

  for(int id=1;id<components;++id) {
    const int x=stats.at<int>(id,cv::CC_STAT_LEFT);
    const int y=stats.at<int>(id,cv::CC_STAT_TOP);
    const int w=stats.at<int>(id,cv::CC_STAT_WIDTH);
    const int h=stats.at<int>(id,cv::CC_STAT_HEIGHT);
    const int area=stats.at<int>(id,cv::CC_STAT_AREA);
    // Road borders may enter the image above the nominal ROI on a
    // strongly tilted camera. Only reject short upper-field blobs.
    if((y<height*.31 && y+h<height*.76) ||
       area<width*height*.0017)continue;
    const double density=static_cast<double>(area)/(w*h);
    const bool horizontal=w>=width*.25 && h>=height*.055 &&
                          w>h*1.3 && density<0.23;
    const bool vertical=h>=height*.23 && w>=width*.045 &&
                        h>w*1.3 && density<0.55 &&
                        (y+h>=height*.84 || y>=height*.65);
    // A steep diagonal stripe near the camera can have a square bounding
    // box, despite being a genuine narrow continuous road marking.
    // Require visible dark pavement on its side and a lower-field span;
    // a rolled white mat reaching the image boundary is not such a lane.
    const bool diagonal=x>width*.10 && x+w<width*.95 &&
        y>height*.45 && y+h>=height*.88 &&
        h>=height*.20 && w>=width*.07 &&
        h>w*.70 && h<w*2.7 && density<.45 &&
        area>=width*height*.006;
    if(!horizontal && !vertical && !diagonal)continue;
    const int step=3;
    std::vector<cv::Point> observed;
    int supported=0;
    int total=0;
    if(horizontal) {
      for(int px=x;px<x+w;px+=step) {
        int y_first=height, y_last=-1, count=0, sum_y=0;
        for(int py=y;py<y+h;++py) {
          if(labels.at<int>(py,px)!=id)continue;
          y_first=std::min(y_first,py);
          y_last=py;
          ++count;
          sum_y+=py;
        }
        const int max_thickness=std::max(7,cvRound(height*.055));
        if(count<3 || count>max_thickness ||
           y_last-y_first>max_thickness*1.35)continue;
        const int cy=sum_y/count;
        if(cy<height*.33 || cy>height*.95)continue;
        ++total;
        const int probe_y=std::min(height-2,cy+std::max(12,height/21));
        int nearby=0;
        for(int dx=-4;dx<=4;++dx)
          nearby+=gray.at<uchar>(probe_y,std::clamp(px+dx,0,width-1));
        const bool dark_road=nearby/9.0>=
            std::max(20.0,road_luminance*.48) &&
            nearby/9.0<road_luminance+58.0;
        const int patch_x=std::clamp(px-8,0,width-17);
        const int patch_y=std::clamp(probe_y-7,0,height-15);
        cv::Scalar patch_mean,patch_std;
        cv::meanStdDev(gray(cv::Rect(patch_x,patch_y,17,15)),
                       patch_mean,patch_std);
        // A smooth bright floor with an isolated white reflection is
        // not the textured dark road surface. Texture is exposure
        // tolerant and still works if the whole image is brightened.
        if(!dark_road || patch_std[0]<1.05)continue;
        ++supported;
        observed.emplace_back(px,cy);
      }
    }else{
      for(int py=y;py<y+h;py+=step) {
        int x_first=width,x_last=-1,count=0,sum_x=0;
        const int* row=labels.ptr<int>(py);
        for(int px=x;px<x+w;++px) {
          if(row[px]!=id)continue;
          x_first=std::min(x_first,px);
          x_last=px;
          ++count;
          sum_x+=px;
        }
        // Perspective widens white paint toward the bottom of image;
        // keep a dark-pavement check outside the actual stripe.
        const double row_fraction=static_cast<double>(py)/height;
        const int max_thickness=std::max(
            8,cvRound(width*((diagonal?.115:.065)+
                             .06*row_fraction*row_fraction)));
        if(count<3 || count>max_thickness ||
           x_last-x_first>max_thickness*1.35)continue;
        const int cx=sum_x/count;
        ++total;
        // The road is the large dark area beside the marking; the
        // white edge of a chair or wall has no such persistent neighbor.
        const int probe=std::max(
            {14,width/30,(x_last-x_first)/2+17});
        const int left=gray.at<uchar>(py,std::max(0,cx-probe));
        const int right=gray.at<uchar>(py,std::min(width-1,cx+probe));
        const double road_min=std::max(20.0,road_luminance*.48);
        const auto is_road=[&](int value) {
          return value>=road_min &&
                 value<road_luminance+58.0;
        };
        if(!is_road(left) && !is_road(right))continue;
        const bool choose_left=is_road(left) &&
            (!is_road(right) ||
             std::abs(left-road_luminance)<
             std::abs(right-road_luminance));
        const int px=choose_left ?
            std::max(0,cx-probe) : std::min(width-1,cx+probe);
        const int patch_x=std::clamp(px-8,0,width-17);
        const int patch_y=std::clamp(py-7,0,height-15);
        cv::Scalar patch_mean,patch_std;
        cv::meanStdDev(gray(cv::Rect(patch_x,patch_y,17,15)),
                       patch_mean,patch_std);
        // The diagonal case needs a strong observed road/paint contrast.
        // This excludes pale creases on furniture and shiny rolled mats.
        const int painted=gray.at<uchar>(py,cx);
        const int adjacent=choose_left?left:right;
        if(patch_std[0]<1.05 ||
           (diagonal && painted-adjacent<42))continue;
        ++supported;
        observed.emplace_back(cx,py);
      }
    }
    const int required=horizontal ?
        std::max(28,cvRound(width*.19/step)) :
        std::max(20,cvRound(height*(diagonal?.17:.13)/step));
    if(static_cast<int>(observed.size())<required ||
       supported<total*.65)continue;

    // Reject spurious blobs whose traced centers zig-zag across the
    // component. Keep only measurements that agree with local neighbors.
    std::vector<cv::Point> stable;
    for(size_t i=0;i<observed.size();++i) {
      const size_t start=i>3 ? i-3 : 0;
      const size_t end=std::min(observed.size(),i+4);
      std::vector<int> values;
      for(size_t j=start;j<end;++j)
        values.push_back(horizontal?observed[j].y:observed[j].x);
      auto median=values.begin()+values.size()/2;
      std::nth_element(values.begin(),median,values.end());
      if(std::abs((horizontal?observed[i].y:observed[i].x)-*median)<=
         (horizontal?13:19))
        stable.push_back(observed[i]);
    }
    if(static_cast<int>(stable.size())<required)continue;
    if(horizontal) {
      const auto vertical_extent=std::minmax_element(
          stable.begin(),stable.end(),
          [](const cv::Point& a,const cv::Point& b) {
            return a.y<b.y;
          });
      // The rolled edge of a mat and the floor-to-wall seam are long
      // nearly horizontal marks, not curved road boundaries. Real
      // outside stripes must show a meaningful perspective depth span.
      if(vertical_extent.second->y-vertical_extent.first->y<
         height*.14)continue;
    }

    // A short fragment is not sufficient to identify a road border.
    const int span=horizontal ?
        stable.back().x-stable.front().x :
        stable.back().y-stable.front().y;
    const int minimum_span=horizontal ?
        cvRound(width*.25) :
        cvRound(height*(diagonal?.19:.22));
    if(span<minimum_span)continue;

    // Classify left/right from the lateral position at comparable image
    // depth. A diagonal left border can cross the image centre at the
    // horizon, so screen-half labels or slope signs are unreliable.
    double side_vote=0.0;
    for(const auto& mark:center_candidates) {
      const auto nearest=std::min_element(
          stable.begin(),stable.end(),
          [&](const cv::Point& a,const cv::Point& b) {
            return std::abs(a.y-mark.y)<std::abs(b.y-mark.y);
          });
      if(nearest==stable.end() ||
         std::abs(nearest->y-mark.y)>height*.16)continue;
      const int lateral=mark.x-nearest->x;
      if(std::abs(lateral)<width*.07 ||
         std::abs(lateral)>width*.78)continue;
      side_vote+=lateral>0 ? -1.0 : 1.0;
    }
    int side=side_vote>0?1:(side_vote<0?-1:0);
    if(side==0 && !diagonal) {
      side=stable[stable.size()/2].x<width/2?-1:1;
    }
    // An isolated diagonal next to the cone is not necessarily the
    // right boundary just because it lies in the right image half.
    // Keep its side unknown until dashed paint corroborates it.
    const double coverage=static_cast<double>(span)/
        (horizontal?width:height);
    const double quality=std::clamp(coverage*.7+
        static_cast<double>(supported)/std::max(total,1)*.3,0.0,1.0);
    if(quality>best_quality) {
      best_quality=quality;
      best.points=std::move(stable);
      best.side=side;
      best.confidence=std::min(0.65,quality);
    }
  }
  return best;
}

}  // namespace rcauto_cpp
