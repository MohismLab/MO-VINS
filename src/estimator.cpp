// SPDX-License-Identifier: GPL-3.0-only
#include "mohism.hpp"
#include "preintegration.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace mohism {
bool Estimator::imuReadyFor(double t)const{
 return !std::isfinite(t)||!cfg_.calibrated||!enabled_||!filter_.ready()||last_imu_>=t;
}
void Estimator::reset(const std::string& reason){
 ++resets_;filter_=Filter{};vision_.reset();samples_.clear();stationary_.clear();
 have_endpoint_=false;last_imu_=last_frame_=-1;warmup_=0;
 pose_=Pose{};pose_.reset_counter=resets_;reason_=reason;
}
void Estimator::enable(bool enabled){
 if(enabled_!=enabled){enabled_=enabled;reset(enabled?"waiting for stationary IMU":"VIO disabled");}
}
void Estimator::addImu(const ImuSample& s){
 if(!enabled_||!cfg_.calibrated)return;
 if(!validImu(s)){reset("invalid or saturated IMU");return;}
 if(last_imu_>=0&&s.t<=last_imu_){reset("non-monotonic IMU timestamp");return;}
 if(last_imu_>=0&&s.t-last_imu_>.05)reset("IMU stream gap");
 if(samples_.size()>=4096)reset("IMU queue overflow");
 last_imu_=s.t;samples_.push_back(s);
 if(!filter_.ready()){
  stationary_.push_back(s);
  if(stationary_.size()>=208){
   try{
    filter_.initialize(stationary_);endpoint_=stationary_.back();have_endpoint_=true;
    samples_.clear();stationary_.clear();reason_="waiting for visual initialization";
   }catch(const std::runtime_error&){
    stationary_.erase(stationary_.begin(),stationary_.begin()+16);reason_="waiting for stationary IMU";
   }
  }
 }
}
Vision::Result Estimator::process(const Frame& l,const Frame& r){
 pose_.valid=false;Vision::Result output;
 // Always provide a preview, including during stationary initialization or recovery.
 for(const auto* f:{&l,&r})if(f->image.type()!=CV_8UC1||f->image.cols!=cfg_.capture_width||f->image.rows!=cfg_.capture_height){
  reset("invalid camera image");return output;
 }
 cv::resize(l.image,output.left,{cfg_.width,cfg_.height},0,0,cv::INTER_AREA);
 cv::resize(r.image,output.right,{cfg_.width,cfg_.height},0,0,cv::INTER_AREA);
 output.depth=cv::Mat::zeros(output.left.size(),CV_16UC1);
 double t=(l.t+r.t)*.5;
 if(!std::isfinite(l.t)||!std::isfinite(r.t)||l.t<0||r.t<0||std::abs(l.t-r.t)>cfg_.max_skew){
  reset("invalid stereo synchronization");return output;
 }
 if(!enabled_||!cfg_.calibrated)return output;
 if(last_frame_>=0&&(t<=last_frame_||t-last_frame_>.2)){reset("camera timestamp discontinuity");return output;}
 if(!filter_.ready()||!have_endpoint_){last_frame_=t;return output;}
 if(t<=filter_.pose().t){last_frame_=t;return output;} // queued camera frames from before initialization
 if(samples_.empty()||samples_.back().t<t){
  if(t-last_imu_>.1)reset("IMU did not reach camera timestamp");
  else reason_="waiting for IMU bracket";
  return output;
 }
 last_frame_=t;
 try{
  while(!samples_.empty()&&samples_.front().t<=t){
   auto s=samples_.front();samples_.pop_front();
   if(s.t>filter_.pose().t&&!filter_.predict(s))throw std::runtime_error("invalid IMU integration");
   endpoint_=s;
  }
  if(filter_.pose().t<t){
   // Interpolate between the actual samples straddling the image, never hold the future measurement backwards.
   const auto& next=samples_.front();double alpha=(t-endpoint_.t)/(next.t-endpoint_.t);
   if(!std::isfinite(alpha)||alpha<0||alpha>1)throw std::runtime_error("invalid IMU interpolation");
   ImuSample at{t,endpoint_.accel*(1-alpha)+next.accel*alpha,endpoint_.gyro*(1-alpha)+next.gyro*alpha};
   if(!filter_.predict(at))throw std::runtime_error("invalid frame IMU endpoint");
   endpoint_=at;
  }
  if(!vision_)vision_=std::make_unique<Vision>(cfg_,filter_.pose());
  auto result=vision_->process(l,r);
  if(!filter_.correct(t,result.visual.R,result.visual.p,result.visual.inliers,result.visual.measurement_covariance))throw std::runtime_error("visual innovation rejected");
  pose_=filter_.pose();pose_.inliers=result.visual.inliers;pose_.reset_counter=resets_;
  pose_.valid=pose_.valid&&++warmup_>=3;reason_=pose_.valid?"tracking":"visual warmup";
  return result;
 }catch(const std::exception& e){reset(e.what());return output;}
}
}
