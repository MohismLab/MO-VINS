// SPDX-License-Identifier: GPL-3.0-only
#include "tight_backend.hpp"
#include "marginal_prior.hpp"
#include "preintegration.hpp"
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projectionOneFrameTwoCamFactor.h"
#include "factor/projectionTwoFrameOneCamFactor.h"
#include "factor/projectionTwoFrameTwoCamFactor.h"
#include <iostream>
#include <stdexcept>
using namespace mohism;
static void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
static void derivatives(ceres::CostFunction& cost,std::vector<std::vector<double>> parameters,double tolerance){
 std::vector<double*> pointers;for(auto& p:parameters)pointers.push_back(p.data());
 int rows=cost.num_residuals();std::vector<double> residual(rows);std::vector<std::vector<double>> jac;std::vector<double*> jp;
 for(auto& p:parameters){jac.emplace_back(rows*p.size());jp.push_back(jac.back().data());}
 require(cost.Evaluate(pointers.data(),residual.data(),jp.data()),"factor evaluation failed");
 PoseLocalParameterization local;double maximum=0;
 for(size_t block=0;block<parameters.size();++block){auto original=parameters[block];int dim=original.size()==7?6:int(original.size());
  for(int c=0;c<dim;++c){const double eps=1e-6;std::vector<double> plus(rows),minus(rows);
   if(original.size()==7){double delta[6]{};delta[c]=eps;local.Plus(original.data(),delta,parameters[block].data());}else parameters[block][c]+=eps;
   require(cost.Evaluate(pointers.data(),plus.data(),nullptr),"factor positive perturbation failed");
   parameters[block]=original;pointers[block]=parameters[block].data();
   if(original.size()==7){double delta[6]{};delta[c]=-eps;local.Plus(original.data(),delta,parameters[block].data());}else parameters[block][c]-=eps;
   require(cost.Evaluate(pointers.data(),minus.data(),nullptr),"factor negative perturbation failed");
   parameters[block]=original;pointers[block]=parameters[block].data();
   for(int r=0;r<rows;++r){double numeric=(plus[r]-minus[r])/(2*eps),analytic=jac[block][r*original.size()+c];maximum=std::max(maximum,std::abs(numeric-analytic)/(1+std::abs(numeric)));}
  }
 }
 if(maximum>tolerance)std::cerr<<"Jacobian relative error="<<maximum<<'\n';require(maximum<tolerance,"factor local Jacobian mismatch");
}
struct Difference:ceres::SizedCostFunction<1,1,1>{bool Evaluate(double const*const* p,double* r,double** j)const override{r[0]=p[1][0]-p[0][0]-2;if(j){if(j[0])j[0][0]=-1;if(j[1])j[1][0]=1;}return true;}};
struct ScalarPrior:ceres::SizedCostFunction<1,1>{bool Evaluate(double const*const* p,double* r,double** j)const override{r[0]=p[0][0]-3;if(j&&j[0])j[0][0]=1;return true;}};
int main(){try{
 cv::setNumThreads(1);
 std::vector<double> pose0{.02,-.01,.03,0,0,0,1},pose1{.05,.02,.04,0,0,0,1};
 std::vector<double> ex0{.01,.02,-.01,0,0,0,1},ex1{.07,.02,-.01,0,0,0,1},inverse{.5},td{.001};
 Eigen::Vector3d a(.12,-.08,1),b(.08,-.06,1);Eigen::Vector2d v(.02,-.01),vr(-.013,.025);
 auto rotate=[](std::vector<double>& pose,Eigen::Vector3d axis,double angle){Eigen::Quaterniond q(Eigen::AngleAxisd(angle,axis.normalized()));for(int k=0;k<4;++k)pose[k+3]=q.coeffs()[k];};
 rotate(pose0,{1,2,3},.13);rotate(pose1,{-2,1,3},-.09);rotate(ex0,{1,-3,2},.08);rotate(ex1,{1,-3,2},.08);
 ProjectionOneFrameTwoCamFactor stereo(a,b,v,vr,.002,-.003);derivatives(stereo,{ex0,ex1,inverse,td},1e-4);
 ProjectionTwoFrameOneCamFactor temporal(a,b,v,vr,.002,-.003);derivatives(temporal,{pose0,pose1,ex0,inverse,td},1e-4);
 ProjectionTwoFrameTwoCamFactor cross(a,b,v,vr,.002,-.003);derivatives(cross,{pose0,pose1,ex0,ex1,inverse,td},1e-4);
 IntegrationBase pre({.2,.1,-9.80665},{.01,-.02,.03},{0,0,0},{0,0,0});for(int i=0;i<20;++i)pre.push_back(.0025,{.2,.1,-9.80665},{.01,-.02,.03});
 std::vector<double> sb(9,0),sb1(9,0);Eigen::Quaterniond q=pre.delta_q;pose1={pre.delta_p.x(),pre.delta_p.y(),pre.delta_p.z()+.5*9.80665*.05*.05,q.x(),q.y(),q.z(),q.w()};for(int k=0;k<3;++k)sb1[k]=pre.delta_v[k]+(k==2?9.80665*.05:0);pose0={0,0,0,0,0,0,1};IMUFactor imu(&pre);derivatives(imu,{pose0,sb,pose1,sb1},.015);
 double x=3,y=5;std::vector<FactorBlock> factors;factors.emplace_back(new ScalarPrior,std::vector<double*>{&x});factors.emplace_back(new Difference,std::vector<double*>{&x,&y});auto prior=MarginalPrior::build(factors,{&x});
 double r=0,J=0;double* jp=&J;const double* ptr=&y;require(prior->Evaluate(&ptr,&r,&jp)&&std::abs(r)<1e-10,"Schur prior has wrong center");require(std::abs(J*J-.5)<1e-8,"Schur information differs from full system");y=7;prior->Evaluate(&ptr,&r,nullptr);require(std::abs(r*r-2)<1e-8,"Schur prior residual differs from eliminated system");
 Config cfg;cfg.window_size=5;cfg.max_features=70;cfg.solver_time=.5;cfg.solver_iterations=8;cfg.optimizer_fps=20;
 CameraGeometry geometry;geometry.K=(cv::Mat_<double>(3,3)<<200,0,160,0,200,100,0,0,1);geometry.Rbc=cv::Mat::eye(3,3,CV_64F);geometry.tbc=cv::Mat::zeros(3,1,CV_64F);geometry.baseline=.06;
 Pose initial;initial.t=0;ImuSample first{0,{.1,0,-9.80665},{0,0,0}};TightBackend backend(cfg,geometry,initial,{},{},first);
 std::vector<V3> world;for(int i=0;i<70;++i)world.push_back({(i%10-4.5)*.16,(i/10-3.)*.12,2.+(i%3)*.7});
 double previous=0,max_error=0;Pose result;
 for(int frame=1;frame<=45;++frame){double t=frame*.05;V3 position{.05*t*t,0,0};std::vector<ImuSample> segment;for(int i=0;i<=20;++i){double stamp=previous+i*.0025;segment.push_back({stamp,{.1,0,-9.80665},{0,0,0}});}std::vector<Observation> observations;
  for(size_t i=0;i<world.size();++i){V3 point=world[i]-position;Observation o;o.id=i;o.left={point[0]/point[2],point[1]/point[2]};o.right={(point[0]-.06)/point[2],point[1]/point[2]};o.depth=point[2];o.stereo=true;o.velocity={-.1*t/point[2],0};observations.push_back(o);}
  result=backend.process(t,observations,segment);max_error=std::max(max_error,cv::norm(result.p-position));previous=t;
 }
 auto d=backend.diagnostics();require(result.valid&&d.solves==45&&d.window==5&&d.marginalizations==41,"joint window did not optimize/slide");require(max_error<.005&&std::abs(result.v[0]-.225)<.01,"joint stereo/IMU metric trajectory wrong");
 // A minority of gross image outliers must not pull the joint state away.
 for(int frame=46;frame<=47;++frame){double t=frame*.05;V3 position{.05*t*t,0,0};std::vector<ImuSample> segment;for(int k=0;k<=20;++k)segment.push_back({previous+k*.0025,{.1,0,-9.80665},{0,0,0}});std::vector<Observation> observations;
  for(size_t i=0;i<world.size();++i){V3 point=world[i]-position;Observation o;o.id=i;o.left={point[0]/point[2],point[1]/point[2]};o.right={(point[0]-.06)/point[2],point[1]/point[2]};o.depth=point[2];o.stereo=true;o.velocity={-.1*t/point[2],0};if(frame==46&&i<15){o.left.x+=.3;o.right.x+=.3;}observations.push_back(o);}
  result=backend.process(t,observations,segment);require(result.valid&&cv::norm(result.p-position)<.02,"minority visual outliers corrupted joint state");previous=t;
 }
 // Exercise both online calibration blocks through observable multi-axis motion.
 cfg.estimate_extrinsics=true;cfg.estimate_time_offset=true;cfg.solver_time=.5;
 auto orientation=[](double t){return expSO3({.28*std::sin(1.4*t),0,0})*expSO3({0,.25*std::sin(2.1*t),0})*expSO3({0,0,.22*std::sin(2.8*t)});};
 auto sample=[&](double t){M3 R=orientation(t);V3 gyro=logSO3(orientation(t-.00001).t()*orientation(t+.00001))/.00002;return ImuSample{t,R.t()*V3{0,0,-9.80665},gyro};};
 TightBackend calibrated(cfg,geometry,initial,{},{},sample(0));bool activated=false;
 for(int frame=1;frame<=90;++frame){double t=frame*.05;std::vector<ImuSample> segment;for(int k=0;k<=20;++k)segment.push_back(sample((frame-1)*.05+k*.0025));std::vector<Observation> observations;
  for(size_t i=0;i<world.size();++i){V3 point=orientation(t).t()*world[i];if(point[2]<.1)continue;Observation o;o.id=i;o.left={point[0]/point[2],point[1]/point[2]};o.right={(point[0]-.06)/point[2],point[1]/point[2]};o.depth=point[2];o.stereo=true;
   V3 before=orientation(t-.0001).t()*world[i],after=orientation(t+.0001).t()*world[i];o.velocity={(after[0]/after[2]-before[0]/before[2])/.0002,(after[1]/after[2]-before[1]/before[2])/.0002};o.right_velocity={((after[0]-.06)/after[2]-(before[0]-.06)/before[2])/.0002,o.velocity.y};observations.push_back(o);}
  result=calibrated.process(t,observations,segment);activated|=calibrated.diagnostics().calibration_active;
 }
 auto calibration=calibrated.diagnostics();require(activated&&result.valid,"online calibration never opened after excitation");require(std::abs(calibration.time_offset)<.005&&cv::norm(calibration.tbc)<.03&&cv::norm(result.p)<.03,"online calibration drifted on calibrated metric rotation");
 std::cout<<"PASS upstream stereo/temporal/IMU Jacobians, Schur equivalence, "<<d.marginalizations<<" marginalizations, metric motion error="<<max_error<<"m\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
