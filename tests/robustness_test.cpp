// SPDX-License-Identifier: GPL-3.0-only
#include "mohism.hpp"
#include "preintegration.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
using namespace mohism;
static void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
template<class F>static void rejects(F f,const char* why){bool rejected=false;try{f();}catch(const std::exception&){rejected=true;}require(rejected,why);}
static Config calibration(){
 Config c;c.calibrated=true;c.capture_width=c.width;c.capture_height=c.height;
 c.K1=(cv::Mat_<double>(3,3)<<200,0,160,0,200,100,0,0,1);c.K2=c.K1.clone();
 c.D1=cv::Mat::zeros(1,5,CV_64F);c.D2=c.D1.clone();c.R=cv::Mat::eye(3,3,CV_64F);c.Rbc=c.R.clone();c.Rbi=c.R.clone();
 c.T=(cv::Mat_<double>(3,1)<<-.06,0,0);c.tbc=cv::Mat::zeros(3,1,CV_64F);return c;
}
static void psd(const cv::Mat& p){
 require(cv::checkRange(p)&&cv::norm(p-p.t())<1e-10,"nonfinite or asymmetric covariance");
 cv::Mat eigen;require(cv::eigen(p,eigen),"covariance eigensolve failed");
 require(eigen.at<double>(eigen.rows-1)>-1e-10,"covariance is not positive semidefinite");
}
int main(){try{
 cv::setNumThreads(1);const double nan=std::numeric_limits<double>::quiet_NaN();
 // Analytic acceleration ramp: midpoint must use both endpoints (right-endpoint integration is biased).
 Preintegration ramp({0,{0,0,0},{0,0,0}});
 for(int i=1;i<=416;++i){double t=i/416.;ramp.push({t,{t,0,0},{0,0,0}});}
 require(std::abs(ramp.dv[0]-.5)<1e-10&&std::abs(ramp.dp[0]-1./6)<1e-6,"midpoint acceleration ramp integration wrong");psd(ramp.covariance);
 Preintegration turn({0,{.3,.1,-9.80665},{.01,-.02,.3}});
 for(int i=1;i<=100;++i){double t=i/416.;turn.push({t,{.3+t,.1,-9.80665},{.01,-.02,.3+t*.1}});}
 cv::Mat jac=turn.jacobian.clone();V3 dp=turn.dp,dv=turn.dv;M3 dR=turn.dR;
 // Check all bias columns against actual replay, allowing the upstream first-order SO(3) Jacobian error.
 for(int axis=0;axis<6;++axis){V3 ba{},bg{};double eps=1e-5;if(axis<3)ba[axis]=eps;else bg[axis-3]=eps;
  turn.repropagate(ba,bg);V3 rp=(turn.dp-dp)/eps,rv=(turn.dv-dv)/eps,rr=logSO3(dR.t()*turn.dR)/eps;
  for(int j=0;j<3;++j){
   require(std::abs(rp[j]-jac.at<double>(j,axis+9))<.002,"preintegration position bias Jacobian wrong");
   require(std::abs(rv[j]-jac.at<double>(j+6,axis+9))<.015,"preintegration velocity bias Jacobian wrong");
   require(std::abs(rr[j]-jac.at<double>(j+3,axis+9))<.002,"preintegration rotation bias Jacobian wrong");
  }
 }
 turn.repropagate({},{});require(cv::norm(turn.dp-dp)<1e-12&&cv::norm(turn.dv-dv)<1e-12&&cv::norm(turn.dR-dR)<1e-12,"bias replay is not deterministic");
 psd(turn.covariance);
 rejects([&]{turn.push({nan,{0,0,-9.80665},{0,0,0}});},"NaN preintegration timestamp accepted");
 rejects([&]{turn.push({1,{0,0,-9.80665},{0,0,0}});},"preintegration gap accepted");
 rejects([&]{turn.repropagate({nan,0,0},{});},"NaN bias accepted");
 // Rotation while remaining stationary in world: endpoint accelerations must be rotated at their own attitudes.
 std::vector<ImuSample> init;for(int i=0;i<208;++i)init.push_back({i/416.,{0,0,-9.80665},{0,0,0}});
 Filter filter;filter.initialize(init);double start=init.back().t;
 M3 attitude=M3::eye();V3 old_w{};
 for(int i=1;i<=416;++i){V3 w{.6,0,0};attitude=attitude*expSO3((old_w+w)*(.5/416));
  require(filter.predict({start+i/416.,attitude.t()*V3{0,0,-9.80665},w}),"rotating IMU rejected");old_w=w;
 }
 require(cv::norm(filter.pose().p)<1e-8&&cv::norm(filter.pose().v)<1e-8,"rotation creates artificial acceleration");psd(filter.covariance());
 auto before=filter.pose();require(!filter.correct(before.t,M3::eye(),{nan,0,0},30),"NaN visual correction accepted");
 require(cv::norm(filter.pose().p-before.p)<1e-12,"rejected correction mutated state");
 require(!filter.predict({before.t+.001,{nan,0,0},{0,0,0}})&&!filter.ready(),"NaN IMU did not invalidate filter");
 filter.initialize(init);require(cv::norm(filter.pose().p)<1e-12&&!filter.pose().valid,"filter reuse retains stale state");
 rejects([&]{filter.setNoise({nan,.01,.03,.02});},"NaN noise accepted");
 filter.setNoise({.001f,.00001f,.001f,.0001f}); // MAVLink REAL32 boundary values
 auto duplicate=init;duplicate[20].t=duplicate[19].t;rejects([&]{filter.initialize(duplicate);},"duplicate initialization timestamp accepted");
 Config cfg=calibration();auto bad=cfg;bad.max_skew=nan;rejects([&]{bad.validate();},"NaN synchronization calibration accepted");
 bad=cfg;bad.K1=cfg.K1.clone();bad.K1.at<double>(0,0)=-1;rejects([&]{bad.validate();},"negative focal length accepted");
 cv::Mat texture(cfg.height,cfg.width,CV_8UC1);cv::RNG rng(72);rng.fill(texture,cv::RNG::UNIFORM,0,256);cv::GaussianBlur(texture,texture,{3,3},.7);
 cv::Mat right=cv::Mat::zeros(texture.size(),texture.type());texture(cv::Rect(8,0,312,200)).copyTo(right(cv::Rect(0,0,312,200)));
 FeatureTracker tracker;auto first=tracker.process(texture,right,200,.06);std::set<uint64_t> ids;for(const auto& t:first)ids.insert(t.id);
 auto second=tracker.process(texture,right,200,.06);int survived=0,depths=0;
 for(const auto& t:second){if(ids.count(t.id)&&t.age>=2)++survived;if(t.depth>0){++depths;require(std::abs(t.depth-1.5)<.06,"sparse stereo scale wrong");}}
 require(survived>150&&depths>150,"persistent stereo tracks lost");
 cv::Mat black=cv::Mat::zeros(texture.size(),texture.type());
 Vision blank(cfg);rejects([&]{blank.process({1,black,0},{1,black,0});},"textureless stereo initialized valid pose");
 Vision temporal(cfg);temporal.process({1,texture,0},{1,right,0});
 rejects([&]{temporal.process({1,texture,0},{1,right,0});},"duplicate frame timestamp accepted");
 Vision invalid(cfg);rejects([&]{invalid.process({nan,texture,0},{nan,right,0});},"NaN image timestamp accepted");
 // Full runtime: blackout, motion during restart, stationary recovery, disable, enable, timestamp fault.
 Estimator estimator(cfg);int imu_index=0;
 auto feed=[&](double until,bool moving=false){while(imu_index/416.<=until){double t=imu_index++/416.;estimator.addImu({t,{0,0,-9.80665},moving?V3{.3,0,0}:V3{}});}};
 auto frame=[&](double t,const cv::Mat& left,const cv::Mat& right_image,bool moving=false){feed(t+.005,moving);return estimator.process({t,left,0},{t,right_image,0});};
 feed(.53);for(int i=0;i<3;++i)frame(.55+i/15.,texture,right);
 require(estimator.pose().valid,"runtime did not reach tracking");uint8_t epoch=estimator.pose().reset_counter;
 auto blackout=frame(.75,black,black);require(!estimator.pose().valid&&estimator.pose().reset_counter!=epoch,"blackout did not reset output epoch");
 require(cv::countNonZero(blackout.depth)==0,"failure preview contains old depth");epoch=estimator.pose().reset_counter;
 for(int i=1;i<=10;++i)frame(.75+i/15.,texture,right,true);
 require(!estimator.pose().valid,"restart accepted moving initialization");
 for(int i=11;i<=25;++i)frame(.75+i/15.,texture,right);
 require(estimator.pose().valid&&estimator.pose().reset_counter==epoch,"stationary recovery failed");
 estimator.enable(false);require(!estimator.pose().valid,"disabled runtime still valid");
 frame(2.5,texture,right);require(!estimator.pose().valid,"disabled runtime emitted pose");
 estimator.enable(true);for(int i=1;i<=15;++i)frame(2.5+i/15.,texture,right);
 require(estimator.pose().valid,"enable did not reinitialize");
 epoch=estimator.pose().reset_counter;estimator.addImu({3,{0,0,-9.80665},{0,0,0}});
 require(!estimator.pose().valid&&estimator.pose().reset_counter!=epoch,"out-of-order IMU retained valid output");
 estimator.addImu({nan,{0,0,-9.80665},{0,0,0}});require(!estimator.pose().valid,"NaN runtime sample retained valid output");
 // A camera frame can arrive just before its upper IMU bracket: retrying it must not become a duplicate fault.
 Estimator delayed(cfg);for(int i=0;i<230;++i)delayed.addImu({i/416.,{0,0,-9.80665},{0,0,0}});
 require(!delayed.imuReadyFor(.56),"missing IMU upper bracket not detected");
 delayed.process({.56,texture,0},{.56,right,0});
 for(int i=230;i<236;++i)delayed.addImu({i/416.,{0,0,-9.80665},{0,0,0}});
 require(delayed.imuReadyFor(.56),"available IMU bracket not detected");
 auto retried=delayed.process({.56,texture,0},{.56,right,0});
 require(retried.visual.valid&&delayed.pose().reset_counter==0,"late IMU bracket prevents frame retry");
 std::cout<<"PASS midpoint/repropagation/Jacobians, PSD covariance, finite guards, persistent stereo, blackout/recovery, disable/enable, timestamp faults\n";
 return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
