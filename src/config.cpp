#include "mohism.hpp"
#include <stdexcept>
#include <time.h>
#include <cmath>
#include <algorithm>
namespace mohism {
double monotonic(){timespec ts{};clock_gettime(CLOCK_MONOTONIC,&ts);return ts.tv_sec+ts.tv_nsec*1e-9;}
Config Config::load(const std::string& path){
 cv::FileStorage fs(path,cv::FileStorage::READ);if(!fs.isOpened())throw std::runtime_error("cannot open config: "+path);
 Config c;
 #define READ(x) if(!fs[#x].empty())fs[#x]>>c.x
 READ(width);READ(height);READ(fps);READ(usb_fps);READ(depth_fps);READ(disparities);READ(capture_width);READ(capture_height);
 READ(window_size);READ(max_features);READ(solver_iterations);READ(loop_max_keyframes);READ(solver_time);READ(optimizer_fps);READ(pixel_noise);READ(estimate_extrinsics);READ(estimate_time_offset);READ(loop_closure);READ(map_file);
 READ(max_skew);READ(imu_offset);READ(left);READ(right);READ(imu);READ(serial);READ(uvc_stereo);READ(uvc_depth);READ(uvc_stereo_control);READ(uvc_stereo_stream);READ(uvc_depth_control);READ(uvc_depth_stream);READ(params);
 READ(K1);READ(D1);READ(K2);READ(D2);READ(R);READ(T);READ(Rbc);READ(tbc);READ(Rbi);READ(calibrated);
 #undef READ
 if(fs["usb_fps"].empty())c.usb_fps=std::min(c.fps,60);
 if(fs["depth_fps"].empty())c.depth_fps=std::min(c.fps,30);
 c.validateProfile();
 if(c.calibrated)c.validate();
 return c;
}
void Config::validateProfile()const{
 for(int n:{uvc_stereo_control,uvc_stereo_stream,uvc_depth_control,uvc_depth_stream})if(n<0||n>255)throw std::runtime_error("invalid UVC interface");
 if(!uvc_stereo.empty()&&uvc_stereo==uvc_depth)throw std::runtime_error("UVC streams require distinct output nodes");
 if(width!=320||height!=200)throw std::runtime_error("USB requires 320x200 stereo inputs and depth");
 if(fps!=15&&fps!=30&&fps!=60&&fps!=120)throw std::runtime_error("capture fps must be 15/30/60/120");
 if((usb_fps!=15&&usb_fps!=30&&usb_fps!=60)||usb_fps>fps)throw std::runtime_error("USB fps must be 15/30/60 and <= capture fps");
 if((depth_fps!=15&&depth_fps!=30&&depth_fps!=60&&depth_fps!=120)||depth_fps>fps)throw std::runtime_error("depth fps must be 15/30/60/120 and <= capture fps");
}
void Config::validate()const{
 validateProfile();
 if(window_size<4||window_size>12||max_features<30||max_features>250||solver_iterations<1||solver_iterations>30||loop_max_keyframes<30||loop_max_keyframes>1000)throw std::runtime_error("invalid bounded optimizer configuration");
 if(!std::isfinite(solver_time)||solver_time<.001||solver_time>2||!std::isfinite(optimizer_fps)||optimizer_fps<5||optimizer_fps>120||!std::isfinite(pixel_noise)||pixel_noise<.1||pixel_noise>5)throw std::runtime_error("invalid optimizer timing/noise");
 if(!calibrated)throw std::runtime_error("calibration not verified: set calibrated only after stereo/IMU calibration");
 if(disparities<=0||disparities%16||disparities>=width)throw std::runtime_error("invalid disparities");
 if(!std::isfinite(max_skew)||max_skew<=0||max_skew>0.02||!std::isfinite(imu_offset)||std::abs(imu_offset)>.5)throw std::runtime_error("invalid sensor time calibration");
 for(const auto* k:{&K1,&K2,&R,&Rbc,&Rbi})if(k->rows!=3||k->cols!=3||k->type()!=CV_64F||!cv::checkRange(*k))throw std::runtime_error("invalid calibration matrix");
 for(const auto* k:{&K1,&K2})if(k->at<double>(0,0)<=0||k->at<double>(1,1)<=0||std::abs(k->at<double>(2,2)-1)>1e-9||std::abs(k->at<double>(2,0))+std::abs(k->at<double>(2,1))>1e-9)throw std::runtime_error("invalid camera intrinsics");
 for(const auto* t:{&T,&tbc})if(t->total()!=3||t->type()!=CV_64F||!cv::checkRange(*t))throw std::runtime_error("invalid calibration translation");
 for(const auto* d:{&D1,&D2})if(d->total()!=5||d->type()!=CV_64F||!cv::checkRange(*d))throw std::runtime_error("requires 5-coefficient pinhole distortion");
 for(const auto* r:{&R,&Rbc,&Rbi})if(cv::norm((*r)*r->t()-cv::Mat::eye(3,3,CV_64F))>1e-5||std::abs(cv::determinant(*r)-1)>1e-5)throw std::runtime_error("extrinsic rotation is not SO(3)");
 if(cv::norm(T)<0.001||cv::norm(T)>1)throw std::runtime_error("baseline must be in metres");
 if(capture_width<=0||capture_height<=0||capture_width>4096||capture_height>4096)throw std::runtime_error("invalid capture dimensions");
}
}
