// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "mohism.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
namespace mohism {
// Runtime measurements use monotonic wall time. They never imply a sensor or USB
// host has received a frame; capture sequence progress and completed UVC buffers
// are reported separately from processed pairs.
class RuntimeStats {
 struct Camera {bool seen=false;uint32_t last=0;uint64_t progress=0,received=0,skipped=0,restarts=0;};
 Camera left_,right_;
 double start_,window_;uint64_t pairs_=0,valid_=0,depth_=0;
 std::vector<double> processing_,age_;
 static void camera(Camera& c,uint32_t seq){
  ++c.received;
  if(c.seen){uint32_t delta=seq-c.last;
   if(delta>0&&delta<1000000){c.progress+=delta;c.skipped+=delta-1;}
   else if(delta){++c.restarts;++c.progress;}
  }else{c.seen=true;++c.progress;}
  c.last=seq;
 }
 static double quantile(std::vector<double> values,double q){
  if(values.empty())return 0;
  std::sort(values.begin(),values.end());return values[std::min(values.size()-1,size_t(std::ceil(q*values.size())-1))];
 }
public:
 explicit RuntimeStats(double start):start_(start),window_(start){}
 void left(uint32_t seq){camera(left_,seq);}
 void right(uint32_t seq){camera(right_,seq);}
 void pair(double processing_ms,double age_ms,bool valid,bool depth){
  ++pairs_;valid_+=valid;depth_+=depth;
  if(std::isfinite(processing_ms)&&processing_ms>=0){if(processing_.size()==512)processing_.erase(processing_.begin());processing_.push_back(processing_ms);}
  if(std::isfinite(age_ms)&&age_ms>=0){if(age_.size()==512)age_.erase(age_.begin());age_.push_back(age_ms);}
 }
 bool due(double now)const{return now-window_>=5;}
 bool write(const std::string& path,double now,const Config& cfg,uint64_t uvc_completed,uint64_t uvc_repeated,bool streaming,Uvc::Stats depth={},Estimator::Diagnostics diagnostics={}){
  double elapsed=now-start_;if(!std::isfinite(elapsed)||elapsed<=0)return false;
  window_=now;
  std::string tmp=path+".tmp";std::ofstream out(tmp);
  out<<std::fixed<<std::setprecision(3)
   <<"{\n  \"uptime_s\": "<<elapsed
   <<",\n  \"requested_capture_fps\": "<<cfg.fps<<",\n  \"requested_usb_fps\": "<<cfg.usb_fps<<",\n  \"requested_depth_fps\": "<<cfg.depth_fps
   <<",\n  \"left_sequence_fps\": "<<left_.progress/elapsed<<",\n  \"right_sequence_fps\": "<<right_.progress/elapsed
   <<",\n  \"processed_fps\": "<<pairs_/elapsed<<",\n  \"valid_vio_fps\": "<<valid_/elapsed<<",\n  \"dense_depth_fps\": "<<depth_/elapsed
   <<",\n  \"left_skipped\": "<<left_.skipped<<",\n  \"right_skipped\": "<<right_.skipped
   <<",\n  \"camera_sequence_restarts\": "<<left_.restarts+right_.restarts
   <<",\n  \"processing_ms_p50\": "<<quantile(processing_,.5)<<",\n  \"processing_ms_p95\": "<<quantile(processing_,.95)
   <<",\n  \"frame_age_ms_p95\": "<<quantile(age_,.95)
   <<",\n  \"uvc_stereo_completed_buffers\": "<<uvc_completed<<",\n  \"uvc_stereo_repeated_buffers\": "<<uvc_repeated
   <<",\n  \"uvc_stereo_streaming\": "<<(streaming?"true":"false")
   <<",\n  \"uvc_depth_completed_buffers\": "<<depth.completed<<",\n  \"uvc_depth_repeated_buffers\": "<<depth.repeated
   <<",\n  \"uvc_depth_streaming\": "<<(depth.streaming?"true":"false")
   <<",\n  \"joint_window_states\": "<<diagnostics.window<<",\n  \"joint_landmarks\": "<<diagnostics.landmarks
   <<",\n  \"joint_solves\": "<<diagnostics.solves<<",\n  \"joint_marginalizations\": "<<diagnostics.marginalizations
   <<",\n  \"last_solve_ms\": "<<diagnostics.solve_ms<<",\n  \"last_joint_cost\": "<<diagnostics.cost
   <<",\n  \"loop_keyframes\": "<<diagnostics.keyframes<<",\n  \"loop_closures\": "<<diagnostics.loops
   <<",\n  \"relocalizations\": "<<diagnostics.relocalizations<<",\n  \"time_offset_s\": "<<std::setprecision(9)<<diagnostics.time_offset
   <<",\n  \"calibration_estimation_active\": "<<(diagnostics.calibration_active?"true":"false")<<"\n}\n";
  out.close();if(!out)return false;
  if(std::rename(tmp.c_str(),path.c_str()))return false;
  window_=now;return true;
 }
};
}
