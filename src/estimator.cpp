// SPDX-License-Identifier: GPL-3.0-only
#include "mohism.hpp"
#include "preintegration.hpp"
#include "tight_backend.hpp"
#include "loop_closure.hpp"
#include "initialization.hpp"
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace mohism {
Estimator::Estimator(const Config& cfg):cfg_(cfg){}
Estimator::~Estimator()=default;
void Estimator::setNoise(const std::array<double,4>& noise){const float low[]={.001f,.00001f,.000001f,.0000001f},high[]={10,1,.1f,.1f};for(int i=0;i<4;++i)if(!std::isfinite(noise[i])||noise[i]<low[i]||noise[i]>high[i])throw std::invalid_argument("invalid inertial noise density");noise_=noise;if(backend_)backend_->setNoise(noise);}
Estimator::Diagnostics Estimator::diagnostics()const{Diagnostics d;if(backend_)d=backend_->diagnostics();if(loops_){d.keyframes=loops_->keyframes();d.loops=loops_->closures();d.relocalizations=loops_->relocalizations();}return d;}

bool Estimator::imuReadyFor(double t)const{
 return !std::isfinite(t)||!cfg_.calibrated||!enabled_||!filter_.ready()||last_imu_>=t+(backend_?backend_->timeOffset():0);
}
void Estimator::reset(const std::string& reason){
 ++resets_;filter_=Filter{};backend_.reset();vision_.reset();initial_vision_.reset();initializer_.reset();if(loops_)loops_->newEpoch();samples_.clear();stationary_.clear();
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
    samples_.clear();stationary_.clear();initial_vision_.reset();initializer_.reset();reason_="waiting for visual initialization";
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
 if(!filter_.ready()||!have_endpoint_){
  if(!samples_.empty()&&samples_.back().t>=t){
   try{
    if(!initial_vision_)initial_vision_=std::make_unique<Vision>(cfg_);
    if(!initializer_)initializer_=std::make_unique<StereoInitializer>();
    auto visual=initial_vision_->process(l,r);
    if(auto seed=initializer_->add(visual.visual,samples_)){
     filter_.seed(seed->pose,seed->ba,seed->bg,seed->endpoint);endpoint_=seed->endpoint;have_endpoint_=true;
     while(!samples_.empty()&&samples_.front().t<=endpoint_.t)samples_.pop_front();
     stationary_.clear();initial_vision_.reset();initializer_.reset();reason_="metric stereo/IMU initialized";
    }
   }catch(const std::runtime_error&){initial_vision_.reset();initializer_.reset();}
  }
  last_frame_=t;return output;
 }
 if(t<=filter_.pose().t){last_frame_=t;return output;} // queued camera frames from before initialization
 if(samples_.empty()||samples_.back().t<t){
  if(t-last_imu_>.1)reset("IMU did not reach camera timestamp");
  else reason_="waiting for IMU bracket";
  return output;
 }
 try{
  double target=t+(backend_?backend_->timeOffset():0);
  if(target<=endpoint_.t)throw std::runtime_error("camera/IMU time correction moved backwards");
  if(samples_.empty()||samples_.back().t<target){reason_="waiting for corrected IMU bracket";return output;}
  last_frame_=t;
  std::vector<ImuSample> segment{endpoint_};
  while(!samples_.empty()&&samples_.front().t<=target){auto sample=samples_.front();samples_.pop_front();if(sample.t>segment.back().t)segment.push_back(sample);}
  if(segment.back().t<target){const auto& next=samples_.front();double alpha=(target-segment.back().t)/(next.t-segment.back().t);auto previous=segment.back();segment.push_back({target,previous.accel*(1-alpha)+next.accel*alpha,previous.gyro*(1-alpha)+next.gyro*alpha});}
  if(!vision_)vision_=std::make_unique<Vision>(cfg_,filter_.pose());
  if(!backend_)backend_=std::make_unique<TightBackend>(cfg_,vision_->geometry(),filter_.pose(),filter_.accelBias(),filter_.gyroBias(),endpoint_);
  backend_->setNoise(noise_);
  auto result=vision_->process(l,r,false);
  Pose local=backend_->process(target,result.observations,segment);endpoint_=segment.back();
  pose_=local;pose_.reset_counter=resets_;
  if(cfg_.loop_closure&&pose_.valid){
   if(!loops_){loops_=std::make_unique<LoopClosure>(cfg_,vision_->geometry());if(!cfg_.map_file.empty()){try{loops_->load(cfg_.map_file);}catch(const std::exception& e){std::cerr<<"map rejected: "<<e.what()<<'\n';}}}
   loops_->setRuntimeGeometry(backend_->geometry());
   auto revision=loops_->correctionVersion();auto corrected=loops_->update(local,result.left,result.right,resets_);if(loops_->correctionVersion()!=revision)++resets_;corrected.reset_counter=resets_;pose_=corrected;
  }
  reason_=pose_.valid?"tracking: tightly coupled stereo/IMU":"joint optimization warmup";
  result.visual=pose_;return result;
 }catch(const std::exception& e){reset(e.what());return output;}
}
}
