#include "mohism.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>
#include <cmath>
using namespace mohism;
static void require(bool p,const char* why){if(!p)throw std::runtime_error(why);}
int main(){try{
 Config bad;bool rejected=false;try{bad.validate();}catch(...){rejected=true;}require(rejected,"uncalibrated config accepted");
 Config c;c.calibrated=true;c.capture_width=c.width;c.capture_height=c.height;
 c.K1=(cv::Mat_<double>(3,3)<<200,0,160,0,200,100,0,0,1);c.K2=c.K1.clone();
 c.D1=cv::Mat::zeros(1,5,CV_64F);c.D2=c.D1.clone();c.R=cv::Mat::eye(3,3,CV_64F);c.Rbc=c.R.clone();c.Rbi=c.R.clone();
 c.T=(cv::Mat_<double>(3,1)<<-.06,0,0);c.tbc=cv::Mat::zeros(3,1,CV_64F);c.validate();
 cv::Mat texture(c.height,c.width,CV_8UC1);cv::RNG rng(42);rng.fill(texture,cv::RNG::UNIFORM,0,256);cv::GaussianBlur(texture,texture,{3,3},.7);
 cv::Mat right=cv::Mat::zeros(texture.size(),texture.type());texture(cv::Rect(8,0,312,200)).copyTo(right(cv::Rect(0,0,312,200)));
 Vision v(c);Frame l{1,texture,0},r{1,right,0};auto res=v.process(l,r);
 double sum=0;int count=0;for(int y=20;y<180;++y)for(int x=90;x<290;++x){auto d=res.depth.at<uint16_t>(y,x);if(d){sum+=d;++count;}}
 require(count>20000,"stereo depth too sparse");require(std::abs(sum/count-1500)<30,"wrong stereo metric scale");
 l.t=r.t=1.066666;auto second=v.process(l,r);require(second.visual.valid&&second.visual.inliers>=12,"static visual odometry rejected");require(cv::norm(second.visual.p)<.005,"static visual odometry drift");
 cv::Mat shifted=cv::Mat::zeros(texture.size(),texture.type()),shifted_r=shifted.clone();
 texture(cv::Rect(4,0,316,200)).copyTo(shifted(cv::Rect(0,0,316,200)));
 right(cv::Rect(4,0,316,200)).copyTo(shifted_r(cv::Rect(0,0,316,200)));
 l.t=r.t=1.133333;l.image=shifted;r.image=shifted_r;
 auto moving=v.process(l,r);require(moving.visual.valid,"translation tracking failed");
 require(std::abs(moving.visual.p[0]-.03)<.008,"translation scale or direction wrong");
 std::vector<ImuSample> stationary;for(int i=0;i<208;++i)stationary.push_back({i/416.,{0,0,-9.80665},{0,0,0}});
 Filter filter;filter.initialize(stationary);
 for(int i=208;i<624;++i)require(filter.predict({i/416.,{0,0,-9.80665},{0,0,0}}),"valid IMU rejected");
 require(cv::norm(filter.pose().p)<1e-8,"stationary inertial drift");
 require(filter.correct(filter.pose().t,M3::eye(),V3{},40),"consistent pose rejected");require(filter.pose().valid,"valid tracking not reported");
 require(!filter.correct(filter.pose().t,M3::eye(),V3{100,0,0},40),"outlier pose accepted");
 require(!filter.predict({3,{0,0,-9.80665},{0,0,0}})&&!filter.ready(),"IMU gap not detected");
 require(cv::norm(logSO3(expSO3({.1,.2,-.3}))-V3{.1,.2,-.3})<1e-10,"SO3 mismatch");
 std::cout<<"PASS calibration gates, metric stereo, static VO, IMU propagation, outlier and gap detection\n";
 return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
