// SPDX-License-Identifier: GPL-3.0-only
#include "mohism.hpp"
#include "preintegration.hpp"
#include <opencv2/calib3d.hpp>
#include <stdexcept>
#include <cmath>
namespace mohism {
static M3 skew(const V3& v){return M3(0,-v[2],v[1],v[2],0,-v[0],-v[1],v[0],0);}
M3 expSO3(const V3& w){M3 r;cv::Rodrigues(w,r);return r;}
V3 logSO3(const M3& r){
 V3 vee{r(2,1)-r(1,2),r(0,2)-r(2,0),r(1,0)-r(0,1)};
 // OpenCV's matrix Rodrigues branch rounds tiny rotations to zero; retain them for bias Jacobians and updates.
 if(cv::trace(cv::Mat(r))[0]>2.9999)return vee*(.5+vee.dot(vee)/48.);
 V3 w;cv::Rodrigues(r,w);return w;
}
Filter::Filter():P_(cv::Mat::eye(15,15,CV_64F)*0.01){}
void Filter::setNoise(const std::array<double,4>& n){
 const float low[]={.001f,.00001f,.001f,.0001f},high[]={10,1,10,1};
 for(int i=0;i<4;++i)if(!std::isfinite(n[i])||n[i]<low[i]||n[i]>high[i])throw std::invalid_argument("invalid estimator noise");
 acc_noise_=n[0];gyro_noise_=n[1];pos_noise_=n[2];rot_noise_=n[3];
}
void Filter::initialize(const std::vector<ImuSample>& s){
 ready_=false;
 if(s.size()<100)throw std::runtime_error("need >=100 stationary IMU samples");
 V3 a{},w{};double t=-1;
 for(const auto& x:s){
  if(!validImu(x)||x.t<=t||(t>=0&&x.t-t>.01))throw std::runtime_error("invalid initialization IMU timing or data");
  t=x.t;a+=x.accel;w+=x.gyro;
 }
 if(s.back().t-s.front().t<.24)throw std::runtime_error("stationary initialization interval too short");
 a*=1./s.size();w*=1./s.size();
 double av=0,wv=0;for(auto& x:s){av+=cv::norm(x.accel-a)*cv::norm(x.accel-a);wv+=cv::norm(x.gyro-w)*cv::norm(x.gyro-w);}
 if(std::abs(cv::norm(a)-9.80665)>0.5||cv::norm(w)>0.1||av/s.size()>0.08||wv/s.size()>0.0025)throw std::runtime_error("IMU initialization requires stationary device");
 V3 z=-a/cv::norm(a),x=V3{1,0,0}-z*z[0];
 if(cv::norm(x)<.1)throw std::runtime_error("initial body forward axis is vertical");
 x/=cv::norm(x);V3 y=z.cross(x);
 R_=M3(x[0],x[1],x[2],y[0],y[1],y[2],z[0],z[1],z[2]);
 p_=v_=V3{};P_=cv::Mat::eye(15,15,CV_64F)*.01;visual_t_=-1;
 gravity_=V3{0,0,9.80665};bg_=w;ba_=a*(1.-9.80665/cv::norm(a));t_=s.back().t;previous_=s.back();ready_=true;
}
void Filter::seed(const Pose& pose,const V3& ba,const V3& bg,const ImuSample& endpoint){
 if(!rotation(pose.R)||!finite(pose.p)||!finite(pose.v)||!finite(ba)||!finite(bg)||!validImu(endpoint)||std::abs(pose.t-endpoint.t)>1e-6)throw std::runtime_error("invalid inertial seed");
 R_=pose.R;p_=pose.p;v_=pose.v;ba_=ba;bg_=bg;t_=pose.t;previous_=endpoint;ready_=true;visual_t_=-1;P_=cv::Mat::eye(15,15,CV_64F)*.01;
}
bool Filter::predict(const ImuSample& s){
 if(!ready_)return false;
 double dt=s.t-t_;
 if(!validImu(s)||!std::isfinite(dt)||dt<=0||dt>.05){ready_=false;return false;}
 Preintegration step(previous_,ba_,bg_,acc_noise_,gyro_noise_);step.push(s);
 const M3 oldR=R_;
 p_+=v_*dt+oldR*step.dp+gravity_*(.5*dt*dt);
 v_+=oldR*step.dv+gravity_*dt;R_=oldR*step.dR;
 // Preintegration uses p,theta,v in the starting IMU frame; p/v errors are world-frame in this filter.
 cv::Mat A=cv::Mat::eye(15,15,CV_64F);
 cv::Mat(oldR).copyTo(A(cv::Rect(0,0,3,3)));cv::Mat(oldR).copyTo(A(cv::Rect(6,6,3,3)));
 cv::Mat F=A*step.jacobian*A.t();
 P_=F*P_*F.t()+A*step.covariance*A.t();P_=(P_+P_.t())*.5;
 previous_=s;t_=s.t;
 if(!finite(p_)||!finite(v_)||!rotation(R_)||!cv::checkRange(P_)||cv::norm(v_)>30){ready_=false;return false;}
 return true;
}
bool Filter::correct(double t,const M3& r,const V3& p,int inliers,const cv::Mat& covariance){
 if(!ready_||!std::isfinite(t)||!finite(p)||!rotation(r)||inliers<12||std::abs(t-t_)>1e-6||t<=visual_t_)return false;
 cv::Mat y=cv::Mat::zeros(6,1,CV_64F);V3 dp=p-p_,dr=logSO3(R_.t()*r);
 for(int i=0;i<3;++i){y.at<double>(i)=dp[i];y.at<double>(i+3)=dr[i];}
 cv::Mat H=cv::Mat::zeros(6,15,CV_64F);for(int i=0;i<6;++i)H.at<double>(i,i)=1;
 cv::Mat N=cv::Mat::eye(6,6,CV_64F)*(pos_noise_*pos_noise_);
 for(int i=3;i<6;++i)N.at<double>(i,i)=rot_noise_*rot_noise_;
 if(!covariance.empty()){
  if(covariance.rows!=6||covariance.cols!=6||covariance.type()!=CV_64F||!cv::checkRange(covariance)||cv::norm(covariance-covariance.t())>1e-8)return false;
  cv::Mat eigen;if(!cv::eigen(covariance,eigen)||eigen.at<double>(5)<=0)return false;
  N+=covariance;
 }
 cv::Mat S=H*P_*H.t()+N,solved;
 if(!cv::solve(S,H*P_,solved,cv::DECOMP_CHOLESKY))return false;
 cv::Mat whitened;if(!cv::solve(S,y,whitened,cv::DECOMP_CHOLESKY))return false;
 double distance=y.dot(whitened);if(!std::isfinite(distance)||distance>22.46)return false;
 cv::Mat K=solved.t(),dx=K*y;
 V3 np=p_,nv=v_,nba=ba_,nbg=bg_,dtheta;
 for(int i=0;i<3;++i){np[i]+=dx.at<double>(i);dtheta[i]=dx.at<double>(i+3);nv[i]+=dx.at<double>(i+6);nba[i]+=dx.at<double>(i+9);nbg[i]+=dx.at<double>(i+12);}
 if(!finite(np)||!finite(nv)||!finite(nba)||!finite(nbg)||cv::norm(nba)>2.5||cv::norm(nbg)>1||cv::norm(nv)>30)return false;
 M3 nr=R_*expSO3(dtheta);
 cv::Mat A=cv::Mat::eye(15,15,CV_64F)-K*H,newP=A*P_*A.t()+K*N*K.t();
 cv::Mat G=cv::Mat::eye(15,15,CV_64F);cv::Mat(M3::eye()-.5*skew(dtheta)).copyTo(G(cv::Rect(3,3,3,3)));
 newP=G*newP*G.t();newP=(newP+newP.t())*.5;
 if(!rotation(nr)||!cv::checkRange(newP))return false;
 p_=np;v_=nv;R_=nr;ba_=nba;bg_=nbg;P_=newP;visual_t_=t;return true;
}
Pose Filter::pose()const{Pose s;s.t=t_;s.R=R_;s.p=p_;s.v=v_;s.valid=ready_&&visual_t_>=0&&t_-visual_t_<.2;return s;}
}
