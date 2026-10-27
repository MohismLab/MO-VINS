#include "mohism.hpp"
#include <stdexcept>
#include <time.h>
#include <cmath>
namespace mohism {
double monotonic(){timespec ts{};clock_gettime(CLOCK_MONOTONIC,&ts);return ts.tv_sec+ts.tv_nsec*1e-9;}
Config Config::load(const std::string& path){
 cv::FileStorage fs(path,cv::FileStorage::READ);if(!fs.isOpened())throw std::runtime_error("cannot open config: "+path);
 Config c;
 #define READ(x) if(!fs[#x].empty())fs[#x]>>c.x
 READ(width);READ(height);READ(fps);READ(disparities);READ(capture_width);READ(capture_height);
 READ(max_skew);READ(imu_offset);READ(left);READ(right);READ(imu);READ(serial);READ(uvc);READ(params);
 READ(K1);READ(D1);READ(K2);READ(D2);READ(R);READ(T);READ(Rbc);READ(tbc);READ(Rbi);READ(calibrated);
 #undef READ
 if(c.calibrated)c.validate();
 return c;
}
void Config::validate()const{
 if(!calibrated)throw std::runtime_error("calibration not verified: set calibrated only after stereo/IMU calibration");
 if(width!=320||height!=200||fps!=15)throw std::runtime_error("USB profile is fixed at 960x200 YUYV / 15fps (three 320x200 tiles)");
 if(disparities<=0||disparities%16||disparities>=width)throw std::runtime_error("invalid disparities");
 if(max_skew<=0||max_skew>0.02)throw std::runtime_error("invalid sync threshold");
 for(const auto* k:{&K1,&K2,&R,&Rbc,&Rbi})if(k->rows!=3||k->cols!=3||k->type()!=CV_64F||!cv::checkRange(*k))throw std::runtime_error("invalid calibration matrix");
 for(const auto* t:{&T,&tbc})if(t->total()!=3||t->type()!=CV_64F||!cv::checkRange(*t))throw std::runtime_error("invalid calibration translation");
 for(const auto* d:{&D1,&D2})if(d->total()!=5||d->type()!=CV_64F||!cv::checkRange(*d))throw std::runtime_error("requires 5-coefficient pinhole distortion");
 for(const auto* r:{&R,&Rbc,&Rbi})if(cv::norm((*r)*r->t()-cv::Mat::eye(3,3,CV_64F))>1e-5||std::abs(cv::determinant(*r)-1)>1e-5)throw std::runtime_error("extrinsic rotation is not SO(3)");
 if(cv::norm(T)<0.001||cv::norm(T)>1)throw std::runtime_error("baseline must be in metres");
 if(capture_width<=0||capture_height<=0||capture_width>4096||capture_height>4096)throw std::runtime_error("invalid capture dimensions");
}
}
