// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
// Modified 2026-10-28 for MOHISM: OpenCV matrices, SO(3) exponential, bounded input,
// continuous noise densities, no ROS/Eigen/Ceres dependency. Upstream: integration_base.h.
#include "preintegration.hpp"
#include <cmath>
#include <stdexcept>

namespace mohism {
bool finite(const V3& v){return std::isfinite(v[0])&&std::isfinite(v[1])&&std::isfinite(v[2]);}
bool validImu(const ImuSample& s){
 return std::isfinite(s.t)&&s.t>=0&&finite(s.accel)&&finite(s.gyro)&&
        cv::norm(s.accel)<19.5&&cv::norm(s.gyro)<4.3;
}
bool rotation(const M3& r){
 return cv::checkRange(cv::Mat(r))&&cv::norm(r*r.t()-M3::eye())<1e-5&&std::abs(cv::determinant(r)-1)<1e-5;
}
static M3 skew(const V3& v){return M3(0,-v[2],v[1],v[2],0,-v[0],-v[1],v[0],0);}
static void block(cv::Mat& m,int row,int col,const M3& value){cv::Mat(value).copyTo(m(cv::Rect(col,row,3,3)));}

Preintegration::Preintegration(const ImuSample& first,const V3& ba,const V3& bg,double an,double gn)
 :first_(first),last_(first),ba_(ba),bg_(bg),acc_noise_(an),gyro_noise_(gn),
  jacobian(cv::Mat::eye(15,15,CV_64F)),covariance(cv::Mat::zeros(15,15,CV_64F)){
 if(!validImu(first)||!finite(ba)||!finite(bg)||!std::isfinite(an)||!std::isfinite(gn)||an<=0||gn<=0)
  throw std::invalid_argument("invalid IMU preintegration input");
}
void Preintegration::push(const ImuSample& s){
 double dt=s.t-last_.t;
 if(!validImu(s)||!std::isfinite(dt)||dt<=0||dt>.05||history_.size()>=4096)
  throw std::invalid_argument("invalid or unbounded IMU interval");
 integrate(dt,s);history_.push_back({dt,s});
}
void Preintegration::repropagate(const V3& ba,const V3& bg){
 if(!finite(ba)||!finite(bg))throw std::invalid_argument("invalid IMU bias");
 ba_=ba;bg_=bg;last_=first_;dp=V3{};dv=V3{};dR=M3::eye();duration=0;
 jacobian=cv::Mat::eye(15,15,CV_64F);covariance=cv::Mat::zeros(15,15,CV_64F);
 for(const auto& step:history_)integrate(step.dt,step.sample);
}
void Preintegration::integrate(double dt,const ImuSample& s){
 const M3 I=M3::eye(),r0=dR;
 const V3 a0=last_.accel-ba_,a1=s.accel-ba_,w=(last_.gyro+s.gyro)*.5-bg_;
 const M3 r1=r0*expSO3(w*dt),A0=skew(a0),A1=skew(a1),W=skew(w);
 const V3 acc=(r0*a0+r1*a1)*.5;
 // The transition and two-endpoint noise injection follow VINS-Fusion midpoint integration.
 cv::Mat F=cv::Mat::eye(15,15,CV_64F),V=cv::Mat::zeros(15,18,CV_64F);
 block(F,0,3,-.25*(r0*A0+r1*A1*(I-W*dt))*dt*dt);
 block(F,0,6,I*dt);block(F,0,9,-.25*(r0+r1)*dt*dt);
 block(F,0,12,.25*r1*A1*dt*dt*dt);
 block(F,3,3,I-W*dt);block(F,3,12,-I*dt);
 block(F,6,3,-.5*(r0*A0+r1*A1*(I-W*dt))*dt);
 block(F,6,9,-.5*(r0+r1)*dt);block(F,6,12,.5*r1*A1*dt*dt);
 block(V,0,0,.25*r0*dt*dt);block(V,0,6,.25*r1*dt*dt);
 block(V,0,3,-.125*r1*A1*dt*dt*dt);block(V,0,9,-.125*r1*A1*dt*dt*dt);
 block(V,3,3,.5*I*dt);block(V,3,9,.5*I*dt);
 block(V,6,0,.5*r0*dt);block(V,6,6,.5*r1*dt);
 block(V,6,3,-.25*r1*A1*dt*dt);block(V,6,9,-.25*r1*A1*dt*dt);
 block(V,9,12,I*dt);block(V,12,15,I*dt);
 cv::Mat N=cv::Mat::zeros(18,18,CV_64F);
 for(int i=0;i<3;++i){
  N.at<double>(i,i)=N.at<double>(i+6,i+6)=2*acc_noise_*acc_noise_/dt;
  N.at<double>(i+3,i+3)=N.at<double>(i+9,i+9)=2*gyro_noise_*gyro_noise_/dt;
  N.at<double>(i+12,i+12)=1e-6/dt;N.at<double>(i+15,i+15)=1e-8/dt;
 }
 jacobian=F*jacobian;covariance=F*covariance*F.t()+V*N*V.t();
 covariance=(covariance+covariance.t())*.5;
 dp+=dv*dt+acc*(.5*dt*dt);dv+=acc*dt;dR=r1;duration+=dt;last_=s;
}
}
