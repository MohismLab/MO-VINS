#include "mohism.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <condition_variable>
#include <algorithm>
#include <memory>
#include <opencv2/imgproc.hpp>
using namespace mohism;
static volatile sig_atomic_t interrupted=0;
static void signal_handler(int){interrupted=1;}
int main(int argc,char** argv){
 if(argc!=2){std::cerr<<"usage: mohism-vins /etc/mohism/calibration.yml\n";return 2;}
 std::signal(SIGINT,signal_handler);std::signal(SIGTERM,signal_handler);
 try{
 Config cfg=Config::load(argv[1]);cv::setNumThreads(1);
 Mavlink link(cfg.serial,cfg.params);Uvc usb(cfg.uvc);
 if(!cfg.calibrated){
  std::cerr<<"Calibration is not verified; raw stereo preview, invalid depth/VIO\n";
  std::unique_ptr<Capture> left,right;
  try{left=std::make_unique<Capture>(cfg.left,cfg.capture_width,cfg.capture_height,cfg.fps);right=std::make_unique<Capture>(cfg.right,cfg.capture_width,cfg.capture_height,cfg.fps);}
  catch(const std::exception& e){std::cerr<<"raw preview unavailable: "<<e.what()<<"\n";left.reset();right.reset();}
  Frame a,b;bool have_a=false,have_b=false;cv::Mat zero=cv::Mat::zeros(200,320,CV_16UC1);
  while(!interrupted){
   link.poll(Pose{});usb.events();
   if(left&&right){
    try{
     if(!have_a)have_a=left->read(a,5);
     if(!have_b)have_b=right->read(b,5);
     if(have_a&&have_b){
      if(std::abs(a.t-b.t)<=cfg.max_skew){cv::Mat l,r;cv::resize(a.image,l,{320,200},0,0,cv::INTER_AREA);cv::resize(b.image,r,{320,200},0,0,cv::INTER_AREA);usb.publish(l,r,zero);have_a=have_b=false;}
      else if(a.t<b.t)have_a=false;else have_b=false;
     }
    }catch(const std::exception& e){std::cerr<<e.what()<<"\n";left.reset();right.reset();}
   }
   std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return 0;
 }
 Imu imu(cfg.imu);
 std::atomic<bool> run{true};std::mutex mutex;std::condition_variable cv;
 std::deque<ImuSample> samples;std::string error;
 std::thread reader([&]{try{while(run){ImuSample s;if(!imu.read(s)){std::this_thread::sleep_for(std::chrono::microseconds(200));continue;}
  s.t+=cfg.imu_offset;s.accel=M3(cfg.Rbi)*s.accel;s.gyro=M3(cfg.Rbi)*s.gyro;
  std::lock_guard<std::mutex> lock(mutex);if(samples.size()>=4096)throw std::runtime_error("IMU processing overrun");samples.push_back(s);cv.notify_one();
 }}catch(const std::exception& e){std::lock_guard<std::mutex> lock(mutex);error=e.what();run=false;cv.notify_one();}});
 auto stop=[&]{run=false;if(reader.joinable())reader.join();};
 try{
 Filter filter;std::vector<ImuSample> init;double deadline=monotonic()+5;
 while(init.size()<208&&!interrupted){
  {std::unique_lock<std::mutex> lock(mutex);cv.wait_for(lock,std::chrono::milliseconds(10),[&]{return !samples.empty()||!error.empty();});if(!error.empty())throw std::runtime_error(error);while(!samples.empty()){init.push_back(samples.front());samples.pop_front();}}
  link.poll(Pose{});usb.events();if(monotonic()>deadline)throw std::runtime_error("IMU initialization timed out");
 }
 if(interrupted){stop();return 0;}filter.initialize(init);
 Capture left(cfg.left,cfg.capture_width,cfg.capture_height,cfg.fps),right(cfg.right,cfg.capture_width,cfg.capture_height,cfg.fps);
 std::unique_ptr<Vision> vision;Frame l,r;bool have_l=false,have_r=false;Pose pose;
 double last_frame=monotonic();
 while(!interrupted){
  link.poll(pose);usb.events();
  {std::lock_guard<std::mutex> lock(mutex);if(!error.empty())throw std::runtime_error(error);}
  if(!have_l)have_l=left.read(l,10);
  if(!have_r)have_r=right.read(r,10);
  if(!have_l||!have_r){if(monotonic()-last_frame>2)throw std::runtime_error("camera stream timeout");continue;}
  double skew=std::min(cfg.max_skew,link.maxSkew());
  if(std::abs(l.t-r.t)>skew){if(l.t<r.t)have_l=false;else have_r=false;if(monotonic()-last_frame>2)throw std::runtime_error("stereo synchronization timeout");continue;}
  double t=(l.t+r.t)*.5;
  filter.setNoise(link.noise());
  if(t<=filter.pose().t){have_l=have_r=false;continue;}
  {std::unique_lock<std::mutex> lock(mutex);
   if(!cv.wait_for(lock,std::chrono::milliseconds(100),[&]{return (!samples.empty()&&samples.back().t>=t)||!error.empty();}))throw std::runtime_error("IMU did not reach camera timestamp");
   if(!error.empty())throw std::runtime_error(error);
   while(!samples.empty()&&samples.front().t<=t){if(!filter.predict(samples.front()))throw std::runtime_error("invalid IMU time gap");samples.pop_front();}
   if(filter.pose().t<t&&!samples.empty()){ImuSample endpoint=samples.front();endpoint.t=t;if(!filter.predict(endpoint))throw std::runtime_error("invalid frame/IMU timing");}
  }
  if(!vision)vision=std::make_unique<Vision>(cfg,filter.pose());
  auto result=vision->process(l,r);
  if(!filter.correct(t,result.visual.R,result.visual.p,result.visual.inliers))throw std::runtime_error("visual measurement rejected; reinitialization required");
  pose=filter.pose();pose.inliers=result.visual.inliers;
  usb.publish(result.left,result.right,result.depth);usb.events();link.poll(pose);
  have_l=have_r=false;last_frame=monotonic();
 }
 stop();
 }catch(...){stop();throw;}
 }catch(const std::exception& e){std::cerr<<"mohism-vins: "<<e.what()<<"\n";return 1;}
 return 0;
}
