// SPDX-License-Identifier: GPL-3.0-only
#include "initialization.hpp"
#include <iostream>
#include <stdexcept>
using namespace mohism;
static void require(bool b,const char* why){if(!b)throw std::runtime_error(why);}
int main(){try{
 StereoInitializer init;std::deque<ImuSample> imu;V3 bias{.01,-.015,.008},gyro{.1,.08,.15};
 // Constant world acceleration and a rotating body; initial velocity is nonzero.
 for(int i=0;i<=400;++i){double t=i/400.;M3 R=expSO3(gyro*t);imu.push_back({t,R.t()*V3{.4,.2,-9.80665}+bias,gyro+V3{.002,-.001,.003}});}
 std::optional<InitialState> result;
 for(int i=0;i<=10;++i){double t=i*.08;Pose visual;visual.t=t;visual.valid=true;visual.R=expSO3(gyro*t);visual.p={.3*t+.2*t*t,.1*t+.1*t*t,0};if(auto state=init.add(visual,imu))result=state;}
 require(result.has_value(),"moving stereo/IMU initializer failed");require(cv::norm(result->bg-V3{.002,-.001,.003})<.005,"initial gyro bias wrong");require(std::abs(cv::norm(result->pose.v)-cv::norm(V3{.3+.4*result->pose.t,.1+.2*result->pose.t,0}))<.08,"initial metric velocity wrong");
 require(cv::norm(result->pose.R.t()*V3{0,0,-9.80665}-(expSO3(gyro*result->pose.t).t()*V3{0,0,-9.80665}))<.1,"initial gravity alignment wrong");
 StereoInitializer static_init;for(int i=0;i<10;++i){Pose p;p.t=.05*i;p.valid=true;require(!static_init.add(p,imu),"unobservable stationary visual motion initialized dynamically");}
 std::cout<<"PASS moving metric stereo initialization, gyro bias, gravity/velocity alignment and unobservable-motion rejection\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
