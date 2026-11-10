#pragma once

#include "rcauto_cpp/solid_boundary.hpp"
#include "rcauto_cpp/row_boundary.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/geometry/2d.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace rcauto_cpp {

struct LaneObservation {
  std::vector<std::vector<cv::Point>> center_dashes;
  std::vector<cv::Point> centers;
  std::vector<cv::Point> left_edge;
  std::vector<cv::Point> right_edge;
  std::vector<cv::Point> unclassified_edge;
  double confidence{0.0};
  double error{0.0};
  double near_error{0.0};
  double far_error{0.0};
  double curvature{0.0};
  double preview{0.0};
  double visible_edge_confidence{0.0};
  int visible_edge_side{0};
  bool edge_only{false};
  bool reference_valid{false};
  bool reference_inferred{false};
  cv::Point reference_point{};
};

inline LaneObservation detect_road_markings(const cv::Mat& frame,
                                            double lookahead,
                                            double min_road_ratio,
                                            double max_road_ratio,
                                            int saturation_setting=70,
                                            int brightness_setting=80) {
  LaneObservation output;
  if (frame.empty() || frame.type()!=CV_8UC3 || frame.cols<100 ||
      frame.rows<100) return output;

  const int width=frame.cols, height=frame.rows;
  const int y0=cvRound(height*0.25);
  cv::Mat gray,hsv,background,contrast,white,local;
  cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
  // Remove isolated sensor noise before measuring local paint contrast.
  // The original colour image remains untouched for overlays/cones.
  cv::medianBlur(gray,gray,3);
  cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
  cv::GaussianBlur(gray,background,cv::Size(),16.0);
  cv::subtract(gray,background,contrast);
  // Use a local brightness threshold to remain sensitive in dim light,
  // while a sharp local residual rejects wide, smooth floor glare.
  const cv::Rect floor_roi(width/4, height*3/5, width/2, height*2/5);
  // Use the lower-middle quantile of actual camera luminance rather
  // than a mean that rises when sunlight, shoes or paint enter the ROI.
  // This reference is recomputed every frame; no exposure-specific
  // fixed thresholds or historical frames are required.
  cv::Mat floor_values=gray(floor_roi).clone().reshape(1,1);
  std::vector<uchar> luminances(floor_values.ptr<uchar>(),
      floor_values.ptr<uchar>()+floor_values.total());
  auto quantile=luminances.begin()+luminances.size()*42/100;
  std::nth_element(luminances.begin(),quantile,luminances.end());
  const cv::Scalar floor_luma(static_cast<double>(*quantile));
  const int minimum_value=std::clamp(
      cvRound(floor_luma[0]+22.0+
          std::clamp(brightness_setting-80,-80,175)*0.30),54,218);
  // A low-exposure road can be dark even when its paint stands out
  // locally. Accept either globally bright or sufficiently contrasted
  // low-saturation pixels, but never accept a uniform bright floor.
  const int max_saturation=std::clamp(
      cvRound(98.0-(saturation_setting-70)*0.75),25,155);
  // Recover the observed road horizon separately for each frame.
  // The lower-middle image window is compared against dark asphalt
  // statistics; bright walls, chairs and shoes cannot create a lane
  // above the continuous visible driving surface.
  int road_top=y0;
  int invalid_rows=0;
  const int sample_left=cvRound(width*.38);
  const int sample_right=cvRound(width*.70);
  for(int y=height-12;y>=cvRound(height*.12);y-=6) {
    int matching=0,total=0;
    for(int dy=-5;dy<=5;++dy) {
      const int py=std::clamp(y+dy,0,height-1);
      const auto* row_gray=gray.ptr<uchar>(py);
      const auto* row_hsv=hsv.ptr<cv::Vec3b>(py);
      for(int x=sample_left;x<sample_right;x+=3) {
        const int v=row_gray[x];
        const bool dark_road=
            v>std::max(20.0,floor_luma[0]*.55) &&
            v<std::min(245.0,floor_luma[0]*1.48+14.0) &&
            row_hsv[x][1]<85;
        matching+=static_cast<int>(dark_road);
        ++total;
      }
    }
    if(total==0)continue;
    invalid_rows=static_cast<double>(matching)/total<.59 ?
        invalid_rows+1 : 0;
    if(invalid_rows>=5) {
      road_top=std::clamp(y+28,y0,cvRound(height*.78));
      break;
    }
  }
  const int minimum_contrast=std::clamp(
      cvRound(floor_luma[0]*0.10),6,16);
  cv::inRange(hsv,
              cv::Scalar(0,0,std::max(40,cvRound(minimum_value*.67))),
              cv::Scalar(179,max_saturation,255),white);
  cv::Mat bright,locally_bright;
  cv::threshold(gray,bright,minimum_value-1,
                255,cv::THRESH_BINARY);
  cv::threshold(contrast,locally_bright,
                minimum_contrast+12,255,cv::THRESH_BINARY);
  cv::bitwise_or(bright,locally_bright,bright);
  cv::bitwise_and(white,bright,white);
  cv::threshold(contrast,local,minimum_contrast,
                255,cv::THRESH_BINARY);
  cv::bitwise_and(white,local,white);
  white.rowRange(0,std::max(y0,road_top)).setTo(0);
  cv::morphologyEx(white,white,cv::MORPH_CLOSE,
                   cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3)));

  struct Candidate {
    cv::Point center;
    std::vector<cv::Point> contour;
    double area;
  };
  std::vector<Candidate> candidates;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(white.clone(),contours,cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  for (const auto& contour:contours) {
    const auto box=cv::boundingRect(contour);
    const double area=cv::contourArea(contour);
    if(box.y<height*0.36 || box.width<11 || box.height<5 ||
       box.height>height*0.28 || box.width>width*0.36 ||
       box.width<box.height*0.55 || box.width>box.height*8.0 ||
       area<65.0 || area>width*height*0.023)
      continue;
    // The centre dashes can be cropped at either image boundary on
    // hairpins. Shape/contrast/trajectory reject clutter, not screen X.
    if(box.width>=width*.32 && (box.x==0 || box.br().x>=width))
      continue;
    const double fill=static_cast<double>(
        cv::countNonZero(white(box)))/box.area();
    if(fill<0.27)continue;
    // Mirror reflections form concave, branching patches; painted
    // centre dashes remain mostly convex even if partially clipped.
    std::vector<cv::Point> hull;
    cv::convexHull(contour,hull);
    const double hull_area=cv::contourArea(hull);
    if(hull_area<1.0 || area/hull_area<0.70)continue;
    const double local_contrast=cv::mean(contrast(box),white(box))[0];
    const bool clipped=(box.x<width*.04 ||
                        box.br().x>width*.96);
    const double minimum_local_contrast=std::clamp(
        floor_luma[0]*(clipped ? .21 : .30),
        clipped ? 12.0 : 15.0,clipped ? 29.0 : 38.0);
    if(local_contrast<minimum_local_contrast)continue;
    const auto moments=cv::moments(contour);
    if(moments.m00<1.0)continue;
    const int x=cvRound(moments.m10/moments.m00);
    const int y=cvRound(moments.m01/moments.m00);
    candidates.push_back({{x,y},contour,area});
  }

  // Bright paint cores remain distinct when local glare merges several
  // white patches into one irregular contour. This additional mask is
  // exposure-relative; low-light frames retain the original local detector.
  const int core_value=std::clamp(
      cvRound(floor_luma[0]*2.05+15.0),160,232);
  cv::Mat paint_cores;
  cv::inRange(hsv,cv::Scalar(0,0,core_value),
              cv::Scalar(179,std::min(80,max_saturation),255),paint_cores);
  paint_cores.rowRange(0,std::max(y0,road_top)).setTo(0);
  cv::morphologyEx(paint_cores,paint_cores,cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3)));

  std::vector<std::vector<cv::Point>> core_contours;
  cv::findContours(paint_cores,core_contours,cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  for(const auto& contour:core_contours) {
    const auto box=cv::boundingRect(contour);
    const double area=cv::contourArea(contour);
    if(box.y<height*.36 || box.width<11 || box.height<5 ||
       box.height>height*.29 || box.width>width*.36 ||
       box.width<box.height*.40 || box.width>box.height*6.0 ||
       area<75 || area>width*height*.020)continue;
    std::vector<cv::Point> hull;
    cv::convexHull(contour,hull);
    const double hull_area=cv::contourArea(hull);
    if(hull_area<1.0 || area/hull_area<.80)continue;

    const auto moments=cv::moments(contour);
    if(moments.m00<1.0)continue;
    const cv::Point center(cvRound(moments.m10/moments.m00),
                           cvRound(moments.m01/moments.m00));
    const bool duplicate=std::any_of(
        candidates.begin(),candidates.end(),
        [&](const Candidate& candidate) {
          const auto existing=cv::boundingRect(candidate.contour);
          const auto overlap=(box&existing).area();
          return overlap>0 &&
              static_cast<double>(overlap)/
                  std::min(box.area(),existing.area())>.52;
        });
    if(!duplicate)candidates.push_back({center,contour,area});
  }
  constexpr size_t max_candidates=22;
  if(candidates.size()>max_candidates) {
    std::partial_sort(candidates.begin(),candidates.begin()+max_candidates,
                      candidates.end(),[](const Candidate& a,const Candidate& b){
                        return a.area>b.area;
                      });
    candidates.resize(max_candidates);
  }

  // Fit the curve through multiple *separate* pieces of visible paint.
  // Never draw the interpolated curve over missing pavement.
  double best_score=-std::numeric_limits<double>::infinity();
  std::vector<size_t> best_indices;
  const double tolerance=std::max(19.0,width*0.035);
  for(size_t i=0;i<candidates.size();++i) {
    for(size_t j=i+1;j<candidates.size();++j) {
      for(size_t k=j+1;k<candidates.size();++k) {
        const double y1=static_cast<double>(candidates[i].center.y)/height;
        const double y2=static_cast<double>(candidates[j].center.y)/height;
        const double y3=static_cast<double>(candidates[k].center.y)/height;
        if(std::max({y1,y2,y3})-std::min({y1,y2,y3})<0.18)continue;
        const cv::Matx33d matrix(1,y1,y1*y1,1,y2,y2*y2,1,y3,y3*y3);
        if(std::abs(cv::determinant(matrix))<0.001)continue;
        cv::Vec3d coefficients;
        const cv::Vec3d x(candidates[i].center.x,
                          candidates[j].center.x,
                          candidates[k].center.x);
        if(!cv::solve(matrix,x,coefficients,cv::DECOMP_LU))continue;
        if(std::abs(coefficients[2])>width*8.0)continue;
        std::vector<size_t> inliers;
        double score=0.0;
        int top=height,bottom=0;
        for(size_t n=0;n<candidates.size();++n) {
          const double y=static_cast<double>(candidates[n].center.y)/height;
          const double fit=coefficients[0]+coefficients[1]*y+
                           coefficients[2]*y*y;
          if(std::abs(fit-candidates[n].center.x)>tolerance)continue;
          inliers.push_back(n);
          score+=std::log1p(candidates[n].area);
          top=std::min(top,candidates[n].center.y);
          bottom=std::max(bottom,candidates[n].center.y);
        }
        if(inliers.size()<3 || bottom-top<height*0.2 ||
           bottom<height*0.57)continue;
        score+=(bottom-top)*0.03;
        if(score>best_score) {
          best_score=score;
          best_indices=std::move(inliers);
        }
      }
    }
  }
  // Two or three disconnected dashes on an acute bend may not fit the
  // three-inlier quadratic model. Link only observed paint candidates in
  // increasing perspective depth; never interpolate missing markings.
  if(best_indices.size()<3 && candidates.size()>=2) {
    std::vector<size_t> order(candidates.size());
    for(size_t i=0;i<order.size();++i) order[i]=i;
    std::sort(order.begin(),order.end(),[&](size_t a,size_t b) {
      return candidates[a].center.y>candidates[b].center.y;
    });
    struct Chain {
      double quality{0.0};
      int parent{-1};
      int length{1};
    };
    std::vector<Chain> chains(order.size());
    for(size_t i=0;i<order.size();++i) {
      chains[i].quality=1.0+0.13*std::log1p(candidates[order[i]].area);
      for(size_t j=0;j<i;++j) {
        const auto& near=candidates[order[j]].center;
        const auto& far=candidates[order[i]].center;
        const double delta_y=near.y-far.y;
        const double delta_x=std::abs(near.x-far.x);
        if(delta_y<height*.035 || delta_y>height*.40 ||
           delta_x>std::min(width*.60,delta_y*2.5+width*.045))
          continue;
        const double slope=(far.x-near.x)/delta_y;
        double curve_penalty=0.0;
        if(chains[j].parent>=0) {
          const auto& previous=candidates[order[chains[j].parent]].center;
          const double prev_slope=(near.x-previous.x)/
                                  static_cast<double>(previous.y-near.y);
          const double turn=std::abs(slope-prev_slope);
          if(turn>2.0)continue;
          curve_penalty=turn*.24;
        }
        const double score=chains[j].quality+1.4-
            .12*delta_x/std::max(1.0,delta_y)-curve_penalty;
        if(score>chains[i].quality) {
          chains[i]={score,static_cast<int>(j),chains[j].length+1};
        }
      }
    }
    int best=-1;
    double quality=0.0;
    for(size_t i=0;i<chains.size();++i) {
      if(chains[i].length<2)continue;
      int start=static_cast<int>(i);
      while(chains[start].parent>=0)start=chains[start].parent;
      const double span=candidates[order[start]].center.y-
                        candidates[order[i]].center.y;
      if(span<height*.10)continue;
      const double score=chains[i].quality+span/height*1.5;
      if(score>quality) {
        quality=score;
        best=static_cast<int>(i);
      }
    }
    if(best>=0) {
      best_indices.clear();
      for(int n=best;n>=0;n=chains[n].parent)
        best_indices.push_back(order[n]);
    }
  }
  std::sort(best_indices.begin(),best_indices.end(),
            [&](size_t a,size_t b) {
              return candidates[a].center.y>candidates[b].center.y;
            });
  for(size_t n:best_indices) {
    output.centers.push_back(candidates[n].center);
    output.center_dashes.push_back(candidates[n].contour);
  }

  // The outer stripes are independent observations. On a curve, one
  // boundary can be fully visible while the opposite boundary leaves the
  // camera, so never require a symmetric left/right pair to draw them.
  // The center dashes constrain candidate identity but do not manufacture
  // white lines where none were observed.
  if(output.centers.size()>=3 &&
     output.centers.front().y-output.centers.back().y>=height*0.17) {
    const int count=static_cast<int>(output.centers.size());
    cv::Mat matrix(count,3,CV_64FC1),coordinates(count,1,CV_64FC1);
    for(int i=0;i<count;++i) {
      const double t=static_cast<double>(output.centers[i].y)/height;
      matrix.at<double>(i,0)=1.0;
      matrix.at<double>(i,1)=t;
      matrix.at<double>(i,2)=t*t;
      coordinates.at<double>(i)=output.centers[i].x;
    }
    cv::Mat polynomial;
    if(cv::solve(matrix,coordinates,polynomial,cv::DECOMP_SVD)) {
      struct Stripe {
        cv::Point center;
        double evidence;
        int side;
        int predecessor{-1};
        int length{1};
        double score{0.0};
      };
      std::vector<Stripe> stripes;
      std::vector<std::vector<int>> row_indices;
      const int top=std::max(cvRound(height*.45),
          output.centers.back().y-cvRound(height*0.075));
      const int bottom=std::min(height-2,
          output.centers.front().y+cvRound(height*0.23));
      // Use the brightness of the actually detected center paint as an
      // exposure reference. Glare can be brighter than asphalt, but is
      // typically much dimmer than the white painted markings themselves.
      std::vector<int> painted_values;
      for(const auto& point:output.centers) {
        if(point.x>=0 && point.x<width &&
           point.y>=0 && point.y<height)
          painted_values.push_back(
              hsv.at<cv::Vec3b>(point.y,point.x)[2]);
      }
      std::sort(painted_values.begin(),painted_values.end());
      const double center_paint_value=painted_values.empty() ?
          floor_luma[0]+45.0 :
          painted_values[painted_values.size()/2];
      const double min_white_value=std::max(
          {92.0,floor_luma[0]+28.0,center_paint_value*0.70});

      for(int y=top;y<=bottom;y+=3) {
        const double t=static_cast<double>(y)/height;
        const double x_guide=polynomial.at<double>(0)+
            polynomial.at<double>(1)*t+
            polynomial.at<double>(2)*t*t;
        row_indices.emplace_back();
        if(!std::isfinite(x_guide) || x_guide<-width*.25 ||
           x_guide>width*1.25)continue;

        const uchar* paint=white.ptr<uchar>(y);
        const auto* colors=hsv.ptr<cv::Vec3b>(y);
        const auto* luma=gray.ptr<uchar>(y);
        int first=-1;
        for(int x=0;x<=width;++x) {
          const bool has_paint=x<width && paint[x]!=0;
          if(has_paint && first<0)first=x;
          if(!has_paint && first>=0) {
            const int stripe_width=x-first;
            const int center_x=(first+x-1)/2;
            const double offset=center_x-x_guide;
            const int side=offset<0 ? -1 : 1;
            const double maximum_width=width*(
                0.025+(side>0 ? 0.32 : 0.17)*t*t);
            if(stripe_width<4 || stripe_width>maximum_width) {
              first=-1;
              continue;
            }
            int saturation_sum=0,brightness_sum=0;
            for(int px=first;px<x;++px) {
              saturation_sum+=colors[px][1];
              brightness_sum+=colors[px][2];
            }
            const double saturation=
                static_cast<double>(saturation_sum)/stripe_width;
            const double brightness=
                static_cast<double>(brightness_sum)/stripe_width;
            if(saturation>52.0 ||
               brightness<min_white_value) {
              first=-1;
              continue;
            }
            // Floor glare may merge with genuine road paint. Track the
            // contrasting stripe and validate continuity geometrically.
            const double separation=std::abs(offset);
            const double minimum_separation=width*std::clamp(
                min_road_ratio*.38,0.05,0.18);
            const double maximum_separation=width*std::clamp(
                max_road_ratio,0.45,0.99);
            if(separation<minimum_separation ||
               separation>maximum_separation) {
              first=-1;
              continue;
            }

            // White lane paint must have darker ROAD immediately inside.
            // This also rejects beige furniture, body parts, and large
            // bright objects even when an edge falls within the road ROI.
            const int inside_x=side<0 ?
                std::min(width-6,x+14) :
                std::max(5,first-14);
            int inside_sum=0;
            for(int dx=-4;dx<=4;++dx)
              inside_sum+=luma[std::clamp(inside_x+dx,0,width-1)];
            const double inside_luma=inside_sum/9.0;
            const double contrast_gap=brightness-inside_luma;
            if(contrast_gap<24.0) {
              first=-1;
              continue;
            }
            const double quality=
                1.15+std::min(0.70,(contrast_gap-24.0)/115.0)
                +0.20*(1.0-std::min(1.0,separation/width));
            const int id=static_cast<int>(stripes.size());
            stripes.push_back({{center_x,y},quality,
                               side,-1,1,quality});
            row_indices.back().push_back(id);
            first=-1;
          }
        }
      }

      // Dynamic programming links only adjacent observed white runs. It
      // tracks curves independently in each side and never draws across
      // occlusion gaps or extrapolates into the background.
      for(size_t row=0;row<row_indices.size();++row) {
        for(const int id:row_indices[row]) {
          auto& current=stripes[id];
          for(size_t gap=1;gap<=4 && gap<=row;++gap) {
            for(const int previous_id:row_indices[row-gap]) {
              const auto& previous=stripes[previous_id];
              if(previous.side!=current.side)continue;
              const double shift=std::abs(
                  current.center.x-previous.center.x);
              if(shift>55.0*static_cast<double>(gap))
                continue;
              // Predict from the previous two observed points, so a
              // sharp, smooth bend is permitted but a sudden jump from
              // the road marking to the rolled mat is penalized.
              double predicted_x=previous.center.x;
              if(previous.predecessor>=0) {
                const auto& ancestor=stripes[previous.predecessor];
                const int prior_span=
                    previous.center.y-ancestor.center.y;
                if(prior_span>0) {
                  const double slope=std::clamp(
                      static_cast<double>(
                          previous.center.x-ancestor.center.x)/prior_span,
                      -16.0,16.0);
                  predicted_x+=slope*(
                      current.center.y-previous.center.y);
                }
              }
              const double prediction_error=std::abs(
                  current.center.x-predicted_x);
              const double max_prediction_error=
                  previous.predecessor>=0 ?
                  40.0+3.0*static_cast<double>(gap) :
                  55.0;
              if(prediction_error>max_prediction_error)
                continue;
              const double next_score=previous.score+
                  current.evidence-0.009*shift-
                  0.022*prediction_error-
                  0.65*static_cast<double>(gap-1);
              if(next_score>current.score) {
                current.score=next_score;
                current.predecessor=previous_id;
                current.length=previous.length+1;
              }
            }
          }
        }
      }
      const auto select_track=[&](int side) {
        int best=-1;
        double quality=0.0;
        for(size_t i=0;i<stripes.size();++i) {
          const auto& candidate=stripes[i];
          if(candidate.side!=side || candidate.length<19)
            continue;
          // Long, uninterrupted evidence outranks a nearby glare patch.
          if(candidate.score>quality) {
            best=static_cast<int>(i);
            quality=candidate.score;
          }
        }
        std::vector<cv::Point> track;
        if(best<0)return track;
        while(best>=0) {
          const auto& selected=stripes[best];
          track.push_back(selected.center);
          best=selected.predecessor;
        }
        std::reverse(track.begin(),track.end());
        // Reject isolated glare pixels or fragmented bright furniture
        // without moving the observations off the actual white stripe.
        // Median filtering is only used to REMOVE outliers, never to
        // invent points in gaps or draw a smoothed artificial line.
        std::vector<cv::Point> stable;
        stable.reserve(track.size());
        for(size_t i=0;i<track.size();++i) {
          const size_t first=i>4 ? i-4 : 0;
          const size_t last=std::min(track.size(),i+5);
          std::vector<int> neighbors;
          for(size_t j=first;j<last;++j)
            neighbors.push_back(track[j].x);
          auto middle=neighbors.begin()+neighbors.size()/2;
          std::nth_element(neighbors.begin(),middle,neighbors.end());
          const double deviation=std::abs(track[i].x-*middle);
          if(deviation<=std::max(18.0,width*.028))
            stable.push_back(track[i]);
        }
        track=std::move(stable);
        if(track.size()<19 ||
           track.back().y-track.front().y<height*.13)
          track.clear();
        return track;
      };
      output.left_edge=select_track(-1);
      output.right_edge=select_track(1);

      // On overlapping scanlines, opposite borders must surround the
      // center markings with plausible widths. A rolled white mat can
      // form its own coherent stripe but sits much farther from the
      // guide than the real opposite boundary.
      if(output.left_edge.size()>=10 &&
         output.right_edge.size()>=10) {
        int checked=0,too_far_left=0,too_far_right=0;
        for(const auto& left:output.left_edge) {
          const auto it=std::min_element(
              output.right_edge.begin(),output.right_edge.end(),
              [&](const cv::Point& a,const cv::Point& b) {
                return std::abs(a.y-left.y)<std::abs(b.y-left.y);
              });
          if(it==output.right_edge.end() ||
             std::abs(it->y-left.y)>6)continue;
          const double t=static_cast<double>(left.y)/height;
          const double center_x=polynomial.at<double>(0)+
              polynomial.at<double>(1)*t+
              polynomial.at<double>(2)*t*t;
          const double left_half=center_x-left.x;
          const double right_half=it->x-center_x;
          if(left_half<=0 || right_half<=0)continue;
          ++checked;
          if(left_half>right_half*2.5)++too_far_left;
          if(right_half>left_half*2.5)++too_far_right;
        }
        if(checked>=6) {
          if(too_far_left*3>checked*2)output.left_edge.clear();
          if(too_far_right*3>checked*2)output.right_edge.clear();
        }
      }
    }
  }

  // Restrict independent edge recovery to the floor actually supported
  // by detected center paint. A white cabinet or wall above the known
  // track must not produce a blue/right boundary.
  cv::Mat boundary_mask=white.clone();
  if(output.centers.size()>=3) {
    int farthest=height;
    for(const auto& mark:output.centers)
      farthest=std::min(farthest,mark.y);
    const int top=std::clamp(
        farthest-cvRound(height*.11),cvRound(height*.20),height-1);
    boundary_mask.rowRange(0,top).setTo(0);
  }
  // A side border is valid evidence even when the dashed center is
  // completely outside the camera view. Recover observed white paint
  // independently; upper furniture is excluded when road markings exist.
  if(output.left_edge.empty() || output.right_edge.empty()) {
    std::vector<cv::Point> partial_markers;
    partial_markers.reserve(candidates.size());
    for(const auto& candidate:candidates)
      partial_markers.push_back(candidate.center);
    const auto solid=detect_solid_boundary_fallback(
        boundary_mask,gray,partial_markers,floor_luma[0]);
    if(!solid.points.empty()) {
      const bool accepted=(solid.side<0 && output.left_edge.empty()) ||
                          (solid.side>0 && output.right_edge.empty()) ||
                          solid.side==0;
      if(accepted) {
        if(solid.side<0)output.left_edge=solid.points;
        else if(solid.side>0)output.right_edge=solid.points;
        else output.unclassified_edge=solid.points;
        output.visible_edge_confidence=solid.confidence;
        output.visible_edge_side=solid.side;
      }
      // Even if dashed centers cannot form a stable navigational path,
      // show isolated painted dashes that lie on the road side of the
      // observed border. These visual dashes never feed steering.
      if(output.centers.empty()) {
        for(const auto& candidate:candidates) {
          if(candidate.center.y<height*.57)continue;
          const auto near=std::min_element(
              solid.points.begin(),solid.points.end(),
              [&](const cv::Point& a,const cv::Point& b) {
                return std::abs(a.x-candidate.center.x) <
                       std::abs(b.x-candidate.center.x);
              });
          if(near==solid.points.end() ||
             std::abs(near->x-candidate.center.x)>width*.14)continue;
          const double distance=static_cast<double>(
              candidate.center.y-near->y);
          if(solid.side>0 && distance>height*.045 &&
             distance<height*.55)
            output.center_dashes.push_back(candidate.contour);
        }
      }
    }
  }

  // Last-resort local scanline tracker, independent of whole-image
  // components. It recovers an observed short stripe partly occluded
  // by shoes or a cone, but does not extrapolate through the occlusion.
  if(output.left_edge.empty()) {
    auto track=track_partial_road_edge(
        boundary_mask,gray,-1,floor_luma[0],output.centers);
    if(!track.empty()) {
      output.left_edge=std::move(track);
      output.visible_edge_confidence=std::max(
          output.visible_edge_confidence,.44);
      output.visible_edge_side=-1;
    }
  }
  if(output.right_edge.empty()) {
    auto track=track_partial_road_edge(
        boundary_mask,gray,1,floor_luma[0],output.centers);
    if(!track.empty()) {
      output.right_edge=std::move(track);
      output.visible_edge_confidence=std::max(
          output.visible_edge_confidence,.44);
      output.visible_edge_side=1;
    }
  }

  const int matched_boundaries=
      static_cast<int>(std::min(output.left_edge.size(),
                                output.right_edge.size()));

  // Steering requires a near-field and far-field observation, rather
  // than a single white reflection or an extrapolated line.
  if(output.centers.size()>=3 &&
     output.centers.front().y>=height*0.54 &&
     output.centers.front().y-output.centers.back().y>=height*0.18) {
    const size_t samples=std::min<size_t>(3,output.centers.size());
    double near_x=0.0,far_x=0.0;
    for(size_t i=0;i<samples;++i) {
      near_x+=output.centers[i].x;
      far_x+=output.centers[output.centers.size()-1-i].x;
    }
    near_x/=static_cast<double>(samples);
    far_x/=static_cast<double>(samples);
    const double target=(1-lookahead)*near_x+lookahead*far_x;
    output.error=std::clamp((target-width*0.5)/(width*0.5),-1.0,1.0);
    output.near_error=std::clamp((width*0.5-near_x)/(width*0.5),-1.0,1.0);
    output.far_error=std::clamp((width*0.5-far_x)/(width*0.5),-1.0,1.0);
    output.curvature=std::clamp((near_x-far_x)/(width*.67),-1.0,1.0);
    output.preview=std::clamp(
        (output.centers.front().y-output.centers.back().y)/(height*.55),
        0.0,1.0);
    output.confidence=std::clamp(
        0.49+static_cast<double>(output.centers.size())*0.045+output.preview*0.13+
        (matched_boundaries>=5 ? 0.05 : 0.0),0.0,0.91);
  }
  // A reference is an observed center-line position if a coherent
  // dashed path is present. It does not require both outer edges.
  if(output.confidence>0.0 && !output.centers.empty()) {
    output.reference_point={
        cvRound(width*.5+output.error*width*.5),
        cvRound(height*.70)};
    output.reference_valid=true;
  }

  // A short observed center arc is sufficient to locate a provisional
  // display target, but not enough to grant controller lane confidence.
  if(!output.reference_valid && output.centers.size()>=2) {
    const auto& near=output.centers.front();
    const auto& far=output.centers.back();
    if(near.y-far.y>=height*.10) {
      output.reference_point=far;
      output.reference_valid=true;
      output.reference_inferred=true;
      output.confidence=std::max(output.confidence,.35);
    }
  }
  if(!output.reference_valid && output.centers.empty() &&
     candidates.size()==1 &&
     candidates.front().center.x>width*.18 &&
     candidates.front().center.x<width*.82 &&
     candidates.front().center.y>height*.43) {
    output.centers.push_back(candidates.front().center);
    output.center_dashes.push_back(candidates.front().contour);
    output.reference_point=candidates.front().center;
    output.reference_valid=true;
    output.reference_inferred=true;
  }

  // Recover partial dashed-center observations supported by ONE of the
  // solid road borders, even when a three-dash polynomial fit failed.
  // This is a geometric fallback: loose bright glare isn't sufficient.
  if(!output.reference_valid && !candidates.empty() &&
     (!output.left_edge.empty() || !output.right_edge.empty())) {
    std::vector<const Candidate*> paired;
    for(const auto& candidate:candidates) {
      if(candidate.center.y<height*.45 ||
         candidate.center.y>height*.97)continue;
      bool matches=false;
      for(const auto* border:{&output.left_edge,&output.right_edge}) {
        if(border->size()<18)continue;
        const auto nearest=std::min_element(
            border->begin(),border->end(),
            [&](const cv::Point& a,const cv::Point& b){
              return std::abs(a.y-candidate.center.y)<
                     std::abs(b.y-candidate.center.y);
            });
        const int delta_y=std::abs(
            nearest->y-candidate.center.y);
        const double separation=
            static_cast<double>(candidate.center.x-nearest->x);
        const bool correct_side=
            border==&output.left_edge ? separation>0 : separation<0;
        if(delta_y<height*.12 && correct_side &&
           std::abs(separation)>width*.085 &&
           std::abs(separation)<width*.55) {
          matches=true;
          break;
        }
      }
      if(matches)paired.push_back(&candidate);
    }
    std::sort(paired.begin(),paired.end(),
        [](const Candidate* a,const Candidate* b) {
          return a->center.y>b->center.y;
        });
    // A single observed dash plus a solid edge can mark a tentative
    // pixel target, but never generates a navigable lane confidence.
    if(!paired.empty()) {
      const auto& near=paired.front()->center;
      const auto& far=paired.back()->center;
      const double x=(1.0-lookahead)*near.x+lookahead*far.x;
      output.reference_point={cvRound(std::clamp(
          x,0.0,static_cast<double>(width-1))),
          cvRound((near.y+far.y)*.5)};
      output.reference_valid=true;
      output.reference_inferred=true;
      const bool enough=paired.size()>=2 &&
          near.y-far.y>=height*.13;
      if(enough) {
        output.error=std::clamp((x-width*.5)/(width*.5),-1.0,1.0);
        output.near_error=std::clamp(
            (width*.5-near.x)/(width*.5),-1.0,1.0);
        output.far_error=std::clamp(
            (width*.5-far.x)/(width*.5),-1.0,1.0);
        output.curvature=std::clamp(
            (near.x-far.x)/(width*.67),-1.0,1.0);
        output.preview=std::clamp(
            static_cast<double>(near.y-far.y)/(height*.55),0.0,1.0);
        // With only two dashes there is enough evidence to DISPLAY a
        // provisional target, not enough to authorize physical motion
        // with the default 0.40 lane-confidence gate.
        output.confidence=paired.size()>=3 &&
            near.y-far.y>=height*.19 ? .56 : .35;
      }
      for(const auto* marker:paired) {
        output.center_dashes.push_back(marker->contour);
        output.centers.push_back(marker->center);
      }
    }
  }

  // Two independent solid boundaries can supply a visible corridor
  // midpoint when all center dashes disappear. Never join missing
  // edge segments or claim a lane if their geometry is inconsistent.
  if(!output.reference_valid && output.left_edge.size()>=20 &&
     output.right_edge.size()>=20) {
    std::vector<cv::Point> midpoints;
    for(int y=cvRound(height*.52);y<=height*.90;y+=6) {
      const auto left=std::min_element(
          output.left_edge.begin(),output.left_edge.end(),
          [y](const cv::Point& a,const cv::Point& b){
            return std::abs(a.y-y)<std::abs(b.y-y);
          });
      const auto right=std::min_element(
          output.right_edge.begin(),output.right_edge.end(),
          [y](const cv::Point& a,const cv::Point& b){
            return std::abs(a.y-y)<std::abs(b.y-y);
          });
      if(std::abs(left->y-y)>9 || std::abs(right->y-y)>9)
        continue;
      const int span=right->x-left->x;
      if(span<width*.22 || span>width*.93)continue;
      midpoints.push_back({(left->x+right->x)/2,y});
    }
    if(midpoints.size()>=12 &&
       midpoints.back().y-midpoints.front().y>height*.13) {
      const double x=(1.0-lookahead)*midpoints.back().x+
                     lookahead*midpoints.front().x;
      output.reference_point={
          cvRound(std::clamp(x,0.0,static_cast<double>(width-1))),
          cvRound(height*.70)};
      output.reference_valid=true;
      output.reference_inferred=true;
      output.error=std::clamp((x-width*.5)/(width*.5),-1.0,1.0);
      output.near_error=std::clamp(
          (width*.5-midpoints.back().x)/(width*.5),-1.0,1.0);
      output.far_error=std::clamp(
          (width*.5-midpoints.front().x)/(width*.5),-1.0,1.0);
      output.preview=std::clamp(
          (midpoints.back().y-midpoints.front().y)/(height*.55),0.0,1.0);
      output.confidence=.52;
    }
  }
  output.edge_only=output.confidence==0.0 &&
      (!output.left_edge.empty() || !output.right_edge.empty() ||
       !output.unclassified_edge.empty());
  return output;
}

}  // namespace rcauto_cpp
