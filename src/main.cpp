// SPDX-License-Identifier: GPL-3.0-only
#include "mohism.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <algorithm>
#include <memory>
using namespace mohism;
static volatile sig_atomic_t interrupted=0;
static void signal_handler(int){interrupted=1;}
int main(int argc,char** argv){
 if(argc!=2){std::cerr<<"usage: mohism-vins /etc/mohism/calibration.yml\n";return 2;}
 std::signal(SIGINT,signal_handler);std::signal(SIGTERM,signal_handler);
 try{
  Config cfg=Config::load(argv[1]);cv::setNumThreads(1);
  Mavlink link(cfg.serial,cfg.params);Uvc usb(cfg.uvc);Estimator estimator(cfg);
  std::unique_ptr<Capture> left,right;std::unique_ptr<Imu> imu;
  Frame l,r;bool have_l=false,have_r=false;
  double camera_retry=0,imu_retry=0,last_frame=monotonic(),last_imu=monotonic();
  std::string status;
  if(!cfg.calibrated)std::cerr<<"Calibration is not verified; raw stereo preview, invalid depth/VIO\n";
  while(!interrupted){
   double now=monotonic();
   link.poll(estimator.pose());usb.events();estimator.enable(link.enabled());estimator.setNoise(link.noise());
   if(!left&&now>=camera_retry){
    try{
     left=std::make_unique<Capture>(cfg.left,cfg.capture_width,cfg.capture_height,cfg.fps);
     right=std::make_unique<Capture>(cfg.right,cfg.capture_width,cfg.capture_height,cfg.fps);
     have_l=have_r=false;last_frame=now;
    }catch(const std::exception& e){
     left.reset();right.reset();camera_retry=now+1;estimator.reset(e.what());
    }
   }
   if(cfg.calibrated&&link.enabled()&&!imu&&now>=imu_retry){
    try{imu=std::make_unique<Imu>(cfg.imu);last_imu=now;}
    catch(const std::exception& e){imu_retry=now+1;estimator.reset(e.what());}
   }
   // The kernel queue absorbs image-processing latency; cap draining work so control remains responsive.
   if(imu){
    try{
     for(int n=0;n<512;++n){
      ImuSample s;if(!imu->read(s))break;
      s.t+=cfg.imu_offset;s.accel=M3(cfg.Rbi)*s.accel;s.gyro=M3(cfg.Rbi)*s.gyro;
      estimator.addImu(s);last_imu=now;
     }
     if(now-last_imu>.1)throw std::runtime_error("IMU stream timeout");
    }catch(const std::exception& e){imu.reset();imu_retry=now+1;estimator.reset(e.what());}
   }
   if(left&&right){
    try{
     // Keep the newest queued camera frame so slow processing cannot build unbounded latency.
     for(int n=0;n<4;++n){Frame next;if(!left->read(next,n==0&&!have_l?2:0))break;l=std::move(next);have_l=true;}
     for(int n=0;n<4;++n){Frame next;if(!right->read(next,n==0&&!have_r?2:0))break;r=std::move(next);have_r=true;}
     if(have_l&&have_r){
      double skew=std::min(cfg.max_skew,link.maxSkew());
      if(std::abs(l.t-r.t)>skew){if(l.t<r.t)have_l=false;else have_r=false;}
      else if(estimator.imuReadyFor((l.t+r.t)*.5)){
       auto result=estimator.process(l,r);
       if(!result.left.empty())usb.publish(result.left,result.right,result.depth);
       have_l=have_r=false;last_frame=now;
      }
     }
     if(now-last_frame>.2&&estimator.pose().valid)estimator.reset("camera stream or synchronization gap");
     if(now-last_frame>2)throw std::runtime_error("camera stream or synchronization timeout");
    }catch(const std::exception& e){
     left.reset();right.reset();have_l=have_r=false;camera_retry=now+1;estimator.reset(e.what());
    }
   }
   if(status!=estimator.status()){status=estimator.status();std::cerr<<"estimator: "<<status<<"\n";}
   link.poll(estimator.pose());usb.events();
   std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
 }catch(const std::exception& e){std::cerr<<"mohism-vins: "<<e.what()<<"\n";return 1;}
 return 0;
}
