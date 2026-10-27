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
 int window_size=6,max_features=100,solver_iterations=8,loop_max_keyframes=200;
 double solver_time=.04,optimizer_fps=30,pixel_noise=.7;
 bool estimate_extrinsics=false,estimate_time_offset=false,loop_closure=true;
 std::string map_file="";
 int uvc_stereo_control=0,uvc_stereo_stream=1,uvc_depth_control=2,uvc_depth_stream=3;
 int capture_width=1280,capture_height=800;
 double max_skew=0.002,imu_offset=0;
 std::string left="/dev/video0",right="/dev/video4",imu="/dev/imu0";
 std::string serial="/dev/ttyGS0",uvc_stereo="",uvc_depth="",params="/userdata/mohism-params.yml";
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
 void seed(const Pose&,const V3& ba,const V3& bg,const ImuSample& endpoint);
 bool predict(const ImuSample& s);
 bool correct(double t,const M3& R,const V3& p,int inliers,const cv::Mat& covariance=cv::Mat());
 void setNoise(const std::array<double,4>& noise);
 Pose pose() const;
 bool ready() const{return ready_;}
 cv::Mat covariance()const{return P_.clone();}
 V3 accelBias()const{return ba_;}V3 gyroBias()const{return bg_;}
};
struct Track {
 uint64_t id=0;unsigned age=0;cv::Point2f point,previous;double depth=0,previous_depth=0;cv::Point2f right,previous_right;
};
class FeatureTracker {
 cv::Mat previous_;std::vector<Track> tracks_;uint64_t next_id_=0;
public:
 const std::vector<Track>& process(const cv::Mat& left,const cv::Mat& right,double fx,double baseline);
};
struct Observation {
 uint64_t id=0;cv::Point2d left,right,velocity,right_velocity;double depth=0;bool stereo=false;
};
struct CameraGeometry {cv::Mat K,Rbc,tbc;double baseline=0;};
class StereoInitializer;
class TightBackend;
class LoopClosure;
class Vision {
 Config cfg_;cv::Mat map1x_,map1y_,map2x_,map2y_,K_;
 M3 Rwc_=M3::eye();V3 pwc_={0,0,0};
 double fx_=0,fy_=0,cx_=0,cy_=0,baseline_=0;
 bool started_=false;Pose origin_;
 FeatureTracker tracker_;cv::Ptr<cv::StereoMatcher> matcher_;double previous_t_=-1,last_depth_t_=-1;
public:
 explicit Vision(const Config& c,const Pose& origin=Pose{});
 struct Result {cv::Mat left,right,depth;Pose visual;bool depth_updated=false;std::vector<Observation> observations;};
 CameraGeometry geometry() const{return {K_.clone(),cfg_.Rbc.clone(),cfg_.tbc.clone(),baseline_};}
 Result process(const Frame& left,const Frame& right,bool estimate_pose=true);
};
// A bounded, hardware-independent runtime. A reset starts a new local FRD epoch.
class Estimator {
 Config cfg_;Filter filter_;std::unique_ptr<Vision> vision_;
 std::unique_ptr<TightBackend> backend_;std::unique_ptr<LoopClosure> loops_;
 std::unique_ptr<Vision> initial_vision_;std::unique_ptr<StereoInitializer> initializer_;
 std::deque<ImuSample> samples_;std::vector<ImuSample> stationary_;
 ImuSample endpoint_{};bool have_endpoint_=false,enabled_=true;
 double last_imu_=-1,last_frame_=-1;unsigned warmup_=0;uint8_t resets_=0;
 std::string reason_="waiting for stationary IMU";Pose pose_;
 std::array<double,4> noise_{.1,.01,.0001,.00001};
public:
 explicit Estimator(const Config& cfg);
 ~Estimator();
 Estimator(const Estimator&)=delete;Estimator& operator=(const Estimator&)=delete;
 void reset(const std::string& reason);
 void enable(bool enabled);
 void addImu(const ImuSample& sample);
 bool imuReadyFor(double timestamp)const;
 Vision::Result process(const Frame& left,const Frame& right);
 Pose pose()const{return pose_;}
 const std::string& status()const{return reason_;}
 void setNoise(const std::array<double,4>& noise);
 struct Diagnostics {size_t window=0,landmarks=0,keyframes=0;uint64_t solves=0,marginalizations=0,loops=0,relocalizations=0;double solve_ms=0,cost=0,time_offset=0;bool calibration_active=false;V3 ba{},bg{};cv::Mat Rbc,tbc;};
 Diagnostics diagnostics() const;
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
 enum class Kind { Stereo, Depth };
 explicit Uvc(const std::string& path,Kind kind,int fps,int control_interface,int streaming_interface);~Uvc();
 Uvc(const Uvc&)=delete;Uvc& operator=(const Uvc&)=delete;
 static std::vector<uint8_t> pack(Kind kind,const cv::Mat& left,const cv::Mat& right,const cv::Mat& depth);
 void publish(const cv::Mat& left,const cv::Mat& right,const cv::Mat& depth);
 struct Stats {uint64_t completed=0,repeated=0;bool streaming=false;};
 Stats stats() const;
 void events();
};
}
