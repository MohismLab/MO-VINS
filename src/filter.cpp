#include "mohism.hpp"
#include <opencv2/calib3d.hpp>
#include <stdexcept>
#include <cmath>
namespace mohism {
static M3 skew(const V3& v){return M3(0,-v[2],v[1],v[2],0,-v[0],-v[1],v[0],0);}
M3 expSO3(const V3& w){M3 r;cv::Rodrigues(w,r);return r;}
V3 logSO3(const M3& r){V3 w;cv::Rodrigues(r,w);return w;}
Filter::Filter():P_(cv::Mat::eye(15,15,CV_64F)*0.01){}
void Filter::setNoise(const std::array<double,4>& n){acc_noise_=n[0];gyro_noise_=n[1];pos_noise_=n[2];rot_noise_=n[3];}
void Filter::initialize(const std::vector<ImuSample>& s){
 if(s.size()<100)throw std::runtime_error("need >=100 stationary IMU samples");
 V3 a{},w{};for(auto& x:s){a+=x.accel;w+=x.gyro;}a*=1./s.size();w*=1./s.size();
 double av=0,wv=0;for(auto& x:s){av+=cv::norm(x.accel-a)*cv::norm(x.accel-a);wv+=cv::norm(x.gyro-w)*cv::norm(x.gyro-w);}
 if(std::abs(cv::norm(a)-9.80665)>0.5||cv::norm(w)>0.1||av/s.size()>0.08||wv/s.size()>0.0025)throw std::runtime_error("IMU initialization requires stationary device");
 // Local FRD: down follows gravity, forward is initial body-X projected horizontally.
 V3 z=-a/cv::norm(a),x=V3{1,0,0}-z*z[0];
 if(cv::norm(x)<.1)throw std::runtime_error("initial body forward axis is vertical");
 x/=cv::norm(x);V3 y=z.cross(x);
 R_=M3(x[0],x[1],x[2],y[0],y[1],y[2],z[0],z[1],z[2]);
 gravity_=V3{0,0,9.80665};bg_=w;ba_=a*(1.-9.80665/cv::norm(a));t_=s.back().t;ready_=true;
}
bool Filter::predict(const ImuSample& s){
 if(!ready_)return false;
 double dt=s.t-t_;
 if(dt<=0)return false;
 if(dt>0.05){ready_=false;return false;}
 V3 omega=s.gyro-bg_,acc=s.accel-ba_;
 M3 mid=R_*expSO3(omega*(dt*.5));V3 aw=mid*acc+gravity_;
 p_+=v_*dt+aw*(.5*dt*dt);v_+=aw*dt;R_=R_*expSO3(omega*dt);
 cv::Mat F=cv::Mat::eye(15,15,CV_64F);
 cv::Mat(cv::Matx33d::eye()*dt).copyTo(F(cv::Rect(3,0,3,3)));
 cv::Mat(-mid*skew(acc)*dt).copyTo(F(cv::Rect(6,3,3,3)));
 cv::Mat(-mid*dt).copyTo(F(cv::Rect(9,3,3,3)));
 cv::Mat(M3::eye()-skew(omega)*dt).copyTo(F(cv::Rect(6,6,3,3)));
 cv::Mat(-M3::eye()*dt).copyTo(F(cv::Rect(12,6,3,3)));
 cv::Mat Q=cv::Mat::zeros(15,15,CV_64F);
 for(int i=0;i<3;++i){Q.at<double>(i+3,i+3)=acc_noise_*acc_noise_*dt;Q.at<double>(i+6,i+6)=gyro_noise_*gyro_noise_*dt;Q.at<double>(i+9,i+9)=1e-6*dt;Q.at<double>(i+12,i+12)=1e-8*dt;}
 P_=F*P_*F.t()+Q;t_=s.t;return true;
}
bool Filter::correct(double t,const M3& r,const V3& p,int inliers){
 if(!ready_||inliers<12||std::abs(t-t_)>.02)return false;
 cv::Mat y=cv::Mat::zeros(6,1,CV_64F);V3 dp=p-p_,dr=logSO3(R_.t()*r);
 for(int i=0;i<3;++i){y.at<double>(i)=dp[i];y.at<double>(i+3)=dr[i];}
 cv::Mat H=cv::Mat::zeros(6,15,CV_64F);for(int i=0;i<3;++i){H.at<double>(i,i)=1;H.at<double>(i+3,i+6)=1;}
 cv::Mat N=cv::Mat::eye(6,6,CV_64F)*(pos_noise_*pos_noise_);
 for(int i=3;i<6;++i)N.at<double>(i,i)=rot_noise_*rot_noise_;
 cv::Mat S=H*P_*H.t()+N,inv;cv::invert(S,inv,cv::DECOMP_CHOLESKY);
 cv::Mat distance=y.t()*inv*y;
 if(distance.at<double>(0)>22.46)return false;
 cv::Mat K=P_*H.t()*inv,dx=K*y;
 V3 dtheta;for(int i=0;i<3;++i){p_[i]+=dx.at<double>(i);v_[i]+=dx.at<double>(i+3);dtheta[i]=dx.at<double>(i+6);ba_[i]+=dx.at<double>(i+9);bg_[i]+=dx.at<double>(i+12);}
 R_=R_*expSO3(dtheta);
 cv::Mat A=cv::Mat::eye(15,15,CV_64F)-K*H;P_=A*P_*A.t()+K*N*K.t();
 cv::Mat G=cv::Mat::eye(15,15,CV_64F);cv::Mat(M3::eye()-.5*skew(dtheta)).copyTo(G(cv::Rect(6,6,3,3)));P_=G*P_*G.t();
 visual_t_=t;return true;
}
Pose Filter::pose()const{Pose s;s.t=t_;s.R=R_;s.p=p_;s.v=v_;s.valid=ready_&&visual_t_>=0&&t_-visual_t_<.2;return s;}
}
