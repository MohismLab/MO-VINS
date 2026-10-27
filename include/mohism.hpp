#pragma once
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>
#include <array>
#include <memory>
namespace mohism {
using V3=cv::Vec3d; using M3=cv::Matx33d;
double monotonic();
M3 expSO3(const V3& w);
V3 logSO3(const M3& r);
struct Config {
 int width=320,height=200,fps=120,usb_fps=60,depth_fps=30,disparities=64;
 int capture_width=1280,capture_height=800;
 double max_skew=0.002,imu_offset=0;
 std::string left="/dev/video0",right="/dev/video4",imu="/dev/imu0";
 std::string serial="/dev/ttyGS0",uvc="",params="/userdata/mohism-params.yml";
 cv::Mat K1,D1,K2,D2,R,T,Rbc,tbc,Rbi;
 bool calibrated=false;
 static Config load(const std::string& file);
 void validateProfile() const;
 void validate() const;
};
struct ImuSample {double t;V3 accel,gyro;};
struct Frame {double t=0;cv::Mat image;uint32_t sequence=0;};
struct Pose {
 double t=0;M3 R=M3::eye();V3 p={0,0,0},v={0,0,0};
 bool valid=false;int inliers=0;
 uint8_t reset_counter=0;
 cv::Mat measurement_covariance; // approximate visual p(world), theta(body) covariance; not calibrated output uncertainty
};
class Filter {
 M3 R_=M3::eye();V3 p_={0,0,0},v_={0,0,0},ba_={0,0,0},bg_={0,0,0};
 V3 gravity_={0,0,9.80665}; cv::Mat P_;
 double t_=-1,visual_t_=-1;bool ready_=false;
 ImuSample previous_{};
 double acc_noise_=.1,gyro_noise_=.01,pos_noise_=.03,rot_noise_=.02;
public:
 Filter();
 void initialize(const std::vector<ImuSample>& stationary);
 bool predict(const ImuSample& s);
 bool correct(double t,const M3& R,const V3& p,int inliers,const cv::Mat& covariance=cv::Mat());
 void setNoise(const std::array<double,4>& noise);
 Pose pose() const;
 bool ready() const{return ready_;}
 cv::Mat covariance()const{return P_.clone();}
};
struct Track {
 uint64_t id=0;unsigned age=0;cv::Point2f point,previous;double depth=0,previous_depth=0;
};
class FeatureTracker {
 cv::Mat previous_;std::vector<Track> tracks_;uint64_t next_id_=0;
public:
 const std::vector<Track>& process(const cv::Mat& left,const cv::Mat& right,double fx,double baseline);
};
class Vision {
 Config cfg_;cv::Mat map1x_,map1y_,map2x_,map2y_,K_;
 M3 Rwc_=M3::eye();V3 pwc_={0,0,0};
 double fx_=0,fy_=0,cx_=0,cy_=0,baseline_=0;
 bool started_=false;Pose origin_;
 FeatureTracker tracker_;cv::Ptr<cv::StereoMatcher> matcher_;double previous_t_=-1,last_depth_t_=-1;
public:
 explicit Vision(const Config& c,const Pose& origin=Pose{});
 struct Result {cv::Mat left,right,depth;Pose visual;bool depth_updated=false;};
 Result process(const Frame& left,const Frame& right);
};
// A bounded, hardware-independent runtime. A reset starts a new local FRD epoch.
class Estimator {
 Config cfg_;Filter filter_;std::unique_ptr<Vision> vision_;
 std::deque<ImuSample> samples_;std::vector<ImuSample> stationary_;
 ImuSample endpoint_{};bool have_endpoint_=false,enabled_=true;
 double last_imu_=-1,last_frame_=-1;unsigned warmup_=0;uint8_t resets_=0;
 std::string reason_="waiting for stationary IMU";Pose pose_;
public:
 explicit Estimator(const Config& cfg):cfg_(cfg){}
 void reset(const std::string& reason);
 void enable(bool enabled);
 void addImu(const ImuSample& sample);
 bool imuReadyFor(double timestamp)const;
 Vision::Result process(const Frame& left,const Frame& right);
 Pose pose()const{return pose_;}
 const std::string& status()const{return reason_;}
 void setNoise(const std::array<double,4>& noise){filter_.setNoise(noise);}
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
 explicit Uvc(const std::string& path,int fps=60);~Uvc();
 void publish(const cv::Mat& left,const cv::Mat& right,const cv::Mat& depth);
 struct Stats {uint64_t completed=0,repeated=0;bool streaming=false;};
 Stats stats() const;
 void events();
};
}
