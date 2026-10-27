// SPDX-License-Identifier: GPL-3.0-only
// Metric specialization of VINS-Fusion initial_alignment.cpp. Scale is supplied
// by the calibrated stereo baseline; no monocular scale variable is introduced.
#include "initialization.hpp"
#include "preintegration.hpp"
#include <algorithm>
#include <stdexcept>
namespace mohism {
namespace {
ImuSample interpolate(const std::deque<ImuSample>& samples,double t){
 auto it=std::lower_bound(samples.begin(),samples.end(),t,[](const ImuSample& a,double stamp){return a.t<stamp;});
 if(it==samples.end())throw std::runtime_error("initializer lacks upper IMU bracket");
 if(std::abs(it->t-t)<1e-9)return *it;
 if(it==samples.begin())throw std::runtime_error("initializer lacks lower IMU bracket");
 auto previous=std::prev(it);double dt=it->t-previous->t;if(dt<=0||dt>.05)throw std::runtime_error("initializer IMU gap");double alpha=(t-previous->t)/dt;return {t,previous->accel*(1-alpha)+it->accel*alpha,previous->gyro*(1-alpha)+it->gyro*alpha};
}
void block(cv::Mat& A,int row,int col,const M3& value){cv::Mat(value).copyTo(A(cv::Rect(col,row,3,3)));}
void vector(cv::Mat& b,int row,const V3& value){for(int k=0;k<3;++k)b.at<double>(row+k)=value[k];}
V3 vector(const cv::Mat& a,int row){return {a.at<double>(row),a.at<double>(row+1),a.at<double>(row+2)};}
}
std::optional<InitialState> StereoInitializer::add(const Pose& visual,const std::deque<ImuSample>& imu){
 if(!visual.valid||!rotation(visual.R)||!finite(visual.p)||!std::isfinite(visual.t))throw std::runtime_error("invalid initialization pose");
 if(!frames_.empty()&&(visual.t<=frames_.back().t||visual.t-frames_.back().t>.2))throw std::runtime_error("initializer visual gap");
 // Keep a useful temporal baseline even if the camera delivers 120 Hz.
 if(!frames_.empty()&&visual.t-frames_.back().t<.045)return {};
 frames_.push_back(visual);while(frames_.size()>12)frames_.pop_front();
 if(frames_.size()<8||frames_.back().t-frames_.front().t<.4)return {};
 double motion=cv::norm(frames_.back().p-frames_.front().p),turn=cv::norm(logSO3(frames_.front().R.t()*frames_.back().R));if(motion<.05&&turn<.1)return {};
 std::vector<std::unique_ptr<Preintegration>> intervals;
 for(size_t i=1;i<frames_.size();++i){double a=frames_[i-1].t,b=frames_[i].t;auto first=interpolate(imu,a),last=interpolate(imu,b);auto pre=std::make_unique<Preintegration>(first);
  for(const auto& s:imu)if(s.t>a&&s.t<b)pre->push(s);
  pre->push(last);intervals.push_back(std::move(pre));
 }
 cv::Mat gyroA=cv::Mat::zeros(int(intervals.size()*3),3,CV_64F),gyrob=cv::Mat::zeros(gyroA.rows,1,CV_64F);
 for(size_t i=0;i<intervals.size();++i){auto& pre=*intervals[i];pre.jacobian(cv::Rect(12,3,3,3)).copyTo(gyroA(cv::Rect(0,int(i*3),3,3)));vector(gyrob,int(i*3),logSO3(pre.dR.t()*frames_[i].R.t()*frames_[i+1].R));}
 cv::Mat gyro;cv::solve(gyroA,gyrob,gyro,cv::DECOMP_SVD);V3 bg=vector(gyro,0);if(!finite(bg)||cv::norm(bg)>.3)return {};
 for(auto& p:intervals)p->repropagate({},bg);
 int count=int(frames_.size()),rows=(count-1)*6,columns=count*3+6;cv::Mat A=cv::Mat::zeros(rows+3,columns,CV_64F),b=cv::Mat::zeros(rows+3,1,CV_64F);
 for(int i=0;i<count-1;++i){auto& pre=*intervals[i];double dt=pre.duration;M3 Ri=frames_[i].R,Jp(pre.jacobian(cv::Rect(9,0,3,3))),Jv(pre.jacobian(cv::Rect(9,6,3,3)));
  block(A,i*6,i*3,M3::eye()*dt);block(A,i*6,count*3,M3::eye()*(.5*dt*dt));block(A,i*6,count*3+3,Ri*Jp);vector(b,i*6,frames_[i+1].p-frames_[i].p-Ri*pre.dp);
  block(A,i*6+3,i*3,-M3::eye());block(A,i*6+3,(i+1)*3,M3::eye());block(A,i*6+3,count*3,-M3::eye()*dt);block(A,i*6+3,count*3+3,-Ri*Jv);vector(b,i*6+3,Ri*pre.dv);
 }
 // A calibrated bias prior resolves gravity/bias ambiguity under limited rotation.
 block(A,rows,count*3+3,M3::eye()*.02);
 cv::Mat solution;if(!cv::solve(A,b,solution,cv::DECOMP_SVD)||!cv::checkRange(solution))return {};
 V3 gravity=vector(solution,count*3),ba=vector(solution,count*3+3);
 if(!finite(gravity)||std::abs(cv::norm(gravity)-9.80665)>.75||cv::norm(ba)>1)return {};
 gravity*=9.80665/cv::norm(gravity);
 cv::Mat reducedA=A.clone(),reducedb=b.clone();for(int row=0;row<A.rows;++row){for(int k=0;k<3;++k){reducedb.at<double>(row)-=A.at<double>(row,count*3+k)*gravity[k];reducedA.at<double>(row,count*3+k)=0;}}
 if(!cv::solve(reducedA,reducedb,solution,cv::DECOMP_SVD)||!cv::checkRange(solution))return {};
 cv::Mat residual=reducedA*solution-reducedb;for(int i=0;i<count-1;++i)if(cv::norm(residual.rowRange(i*6,i*6+3))>.04||cv::norm(residual.rowRange(i*6+3,i*6+6))>.5)return {};
 ba=vector(solution,count*3+3);V3 velocity=vector(solution,(count-1)*3);if(cv::norm(velocity)>15||cv::norm(ba)>1)return {};
 V3 z=gravity/9.80665,forward=frames_.front().R*V3{1,0,0},x=forward-z*forward.dot(z);if(cv::norm(x)<.1)return {};x/=cv::norm(x);V3 y=z.cross(x);M3 align(x[0],x[1],x[2],y[0],y[1],y[2],z[0],z[1],z[2]);
 InitialState result;result.pose=frames_.back();result.pose.R=align*result.pose.R;result.pose.p=align*(result.pose.p-frames_.front().p);result.pose.v=align*velocity;result.pose.valid=false;result.ba=ba;result.bg=bg;result.endpoint=interpolate(imu,result.pose.t);return result;
}
}
