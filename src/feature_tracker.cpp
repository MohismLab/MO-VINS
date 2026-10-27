// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
// Author of upstream feature_tracker.cpp: Qin Tong (qintonguav@gmail.com)
// Modified 2026-10-28 for MOHISM: bounded OpenCV-only tracker, persistent IDs,
// stereo epipolar/disparity gates and metric sparse depth. See third_party notice.
#include "mohism.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <cmath>

namespace mohism {
static bool inBorder(const cv::Point2f& p,const cv::Size& size){
 return std::isfinite(p.x)&&std::isfinite(p.y)&&p.x>=2&&p.y>=2&&p.x<size.width-2&&p.y<size.height-2;
}
static void flow(const cv::Mat& a,const cv::Mat& b,const std::vector<cv::Point2f>& from,
                 std::vector<cv::Point2f>& to,std::vector<uchar>& valid){
 if(from.empty()){to.clear();valid.clear();return;}
 std::vector<float> error;std::vector<uchar> reverse_valid;std::vector<cv::Point2f> reverse;
 cv::calcOpticalFlowPyrLK(a,b,from,to,valid,error,{21,21},3);
 cv::calcOpticalFlowPyrLK(b,a,to,reverse,reverse_valid,error,{21,21},3);
 for(size_t i=0;i<from.size();++i)
  valid[i]=valid[i]&&reverse_valid[i]&&inBorder(to[i],b.size())&&inBorder(reverse[i],a.size())&&cv::norm(from[i]-reverse[i])<=.5;
}
const std::vector<Track>& FeatureTracker::process(const cv::Mat& left,const cv::Mat& right,double fx,double baseline){
 if(left.empty()||left.type()!=CV_8UC1||right.type()!=CV_8UC1||left.size()!=right.size()||
    !std::isfinite(fx)||!std::isfinite(baseline)||fx<=0||baseline<=0)
  throw std::invalid_argument("invalid feature tracker input");
 if(!previous_.empty()&&previous_.size()!=left.size())throw std::invalid_argument("feature tracker image size changed");
 std::vector<Track> surviving;
 if(!previous_.empty()&&!tracks_.empty()){
  std::vector<cv::Point2f> before,after;std::vector<uchar> valid;
  for(const auto& t:tracks_)before.push_back(t.point);
  flow(previous_,left,before,after,valid);
  for(size_t i=0;i<tracks_.size();++i)if(valid[i]){
   Track t=tracks_[i];t.previous=t.point;t.previous_right=t.right;t.previous_depth=t.depth;t.point=after[i];++t.age;t.depth=0;surviving.push_back(t);
  }
 }
 // VINS-Fusion setMask: retain long-lived tracks first, then replenish spaced corners.
 std::stable_sort(surviving.begin(),surviving.end(),[](const Track& a,const Track& b){return a.age>b.age;});
 cv::Mat mask(left.size(),CV_8UC1,cv::Scalar(255));tracks_.clear();
 for(const auto& t:surviving)if(mask.at<uchar>(cvRound(t.point.y),cvRound(t.point.x))){
  tracks_.push_back(t);cv::circle(mask,t.point,8,cv::Scalar(0),-1);
 }
 std::vector<cv::Point2f> corners;
 if(tracks_.size()<250)cv::goodFeaturesToTrack(left,corners,250-int(tracks_.size()),.015,8,mask);
 for(const auto& p:corners)if(inBorder(p,left.size()))tracks_.push_back({next_id_++,1,p,p,0,0});
 std::vector<cv::Point2f> points,right_points;std::vector<uchar> stereo_valid;
 for(const auto& t:tracks_)points.push_back(t.point);
 flow(left,right,points,right_points,stereo_valid);
 for(size_t i=0;i<tracks_.size();++i){
  double disparity=points[i].x-right_points[i].x;
  if(stereo_valid[i]&&std::abs(points[i].y-right_points[i].y)<=.75&&disparity>=.75){
   double z=fx*baseline/disparity;
   if(z>=.1&&z<=15){tracks_[i].depth=z;tracks_[i].right=right_points[i];}
  }
 }
 previous_=left.clone();return tracks_;
}
}
