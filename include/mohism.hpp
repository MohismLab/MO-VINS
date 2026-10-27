#pragma once
#include <opencv2/core.hpp>
#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>
#include <array>
namespace mohism {
using V3=cv::Vec3d; using M3=cv::Matx33d;
double monotonic();
M3 expSO3(const V3& w);
V3 logSO3(const M3& r);
struct Config {
 int width=320,height=200,fps=15,disparities=64;
 int capture_width=1280,capture_height=800;
 double max_skew=0.002,imu_offset=0;
 std::string left="/dev/video0",right="/dev/video4",imu="/dev/imu0";
 std::string serial="/dev/ttyGS0",uvc="",params="/userdata/mohism-params.yml";
 cv::Mat K1,D1,K2,D2,R,T,Rbc,tbc,Rbi;
 bool calibrated=false;
 static Config load(const std::string& file);
 void validate() const;
};
struct ImuSample {double t;V3 accel,gyro;};
struct Frame {double t=0;cv::Mat image;uint32_t sequence=0;};
struct Pose {
 double t=0;M3 R=M3::eye();V3 p={0,0,0},v={0,0,0};
 bool valid=false;int inliers=0;
};
class Filter {
 M3 R_=M3::eye();V3 p_={0,0,0},v_={0,0,0},ba_={0,0,0},bg_={0,0,0};
 V3 gravity_={0,0,9.80665}; cv::Mat P_;
 double t_=-1,visual_t_=-1;bool ready_=false;
 double acc_noise_=.1,gyro_noise_=.01,pos_noise_=.03,rot_noise_=.02;
public:
 Filter();
 void initialize(const std::vector<ImuSample>& stationary);
 bool predict(const ImuSample& s);
 bool correct(double t,const M3& R,const V3& p,int inliers);
 void setNoise(const std::array<double,4>& noise);
 Pose pose() const;
 bool ready() const{return ready_;}
};
class Vision {
 Config cfg_;cv::Mat map1x_,map1y_,map2x_,map2y_,K_,previous_,previous_depth_;
 M3 Rwc_=M3::eye();V3 pwc_={0,0,0};
 double fx_=0,fy_=0,cx_=0,cy_=0,baseline_=0;
 bool started_=false;Pose origin_;
public:
 explicit Vision(const Config& c,const Pose& origin=Pose{});
 struct Result {cv::Mat left,right,depth;Pose visual;};
 Result process(const Frame& left,const Frame& right);
};
class Capture {
 int fd_=-1;std::vector<void*> memory_;std::vector<size_t> lengths_;
 unsigned width_=0,height_=0,stride_=0;uint32_t format_=0,type_=0;
public:
 Capture(const std::string& path,int width,int height,int fps);
 ~Capture();
 Capture(const Capture&)=delete;Capture& operator=(const Capture&)=delete;
 bool read(Frame& f,int timeout_ms=1000);
};
class Imu {
 int fd_=-1;uint32_t sequence_=0;bool first_=true;
public:
 explicit Imu(const std::string& path);~Imu();
 bool read(ImuSample& s);
};
class Mavlink {
 struct Impl;Impl* impl_;
public:
 explicit Mavlink(const std::string& device,const std::string& params);
 ~Mavlink();void poll(const Pose& pose);
 bool enabled() const;double maxSkew() const;
 std::array<double,4> noise()const;
};
class Uvc {
 struct Impl;Impl* impl_;
public:
 explicit Uvc(const std::string& path);~Uvc();
 void publish(const cv::Mat& left,const cv::Mat& right,const cv::Mat& depth);
 void events();
};
}
