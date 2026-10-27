// SPDX-License-Identifier: GPL-3.0-only
// Window assembly follows the stereo/IMU factor graph in VINS-Fusion estimator.cpp.
#include "tight_backend.hpp"
#include "marginal_prior.hpp"
#include "preintegration.hpp"
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projectionOneFrameTwoCamFactor.h"
#include "factor/projectionTwoFrameOneCamFactor.h"
#include "factor/projectionTwoFrameTwoCamFactor.h"
#include <map>
#include <chrono>
#include <algorithm>
#include <stdexcept>
namespace mohism {
namespace {
Eigen::Vector3d eigen(const V3& v){return {v[0],v[1],v[2]};}
V3 cvvec(const Eigen::Vector3d& v){return {v.x(),v.y(),v.z()};}
Eigen::Matrix3d eigen(const M3& R){Eigen::Matrix3d out;for(int r=0;r<3;++r)for(int c=0;c<3;++c)out(r,c)=R(r,c);return out;}
M3 cvrot(const Eigen::Matrix3d& R){M3 out;for(int r=0;r<3;++r)for(int c=0;c<3;++c)out(r,c)=R(r,c);return out;}
struct State {
 uint64_t id=0;double t=0;std::array<double,7> x{0,0,0,0,0,0,1};std::array<double,9> sb{};
 std::vector<ImuSample> imu; // Segment from previous state, including both exact endpoints.
 Eigen::Map<Eigen::Vector3d> p(){return Eigen::Map<Eigen::Vector3d>(x.data());}
 Eigen::Quaterniond q()const{return Eigen::Map<const Eigen::Quaterniond>(x.data()+3);}
 Eigen::Map<Eigen::Vector3d> v(){return Eigen::Map<Eigen::Vector3d>(sb.data());}
 Eigen::Map<Eigen::Vector3d> ba(){return Eigen::Map<Eigen::Vector3d>(sb.data()+3);}
 Eigen::Map<Eigen::Vector3d> bg(){return Eigen::Map<Eigen::Vector3d>(sb.data()+6);}
 Pose pose(){Pose p;p.t=t;p.R=cvrot(q().toRotationMatrix());p.p=cvvec(this->p());p.v=cvvec(v());return p;}
};
struct Seen {uint64_t frame;Observation obs;double td=0;};
struct Landmark {double inverse=1;std::vector<Seen> seen;};
class SpeedPrior:public ceres::SizedCostFunction<9,9>{std::array<double,9> center_;
public:explicit SpeedPrior(const std::array<double,9>& center):center_(center){}
 bool Evaluate(double const*const* p,double* r,double** j)const override{for(int i=0;i<9;++i){double w=i<3?20:i<6?5:100;r[i]=(p[0][i]-center_[i])*w;if(j&&j[0]){for(int k=0;k<9;++k)j[0][i*9+k]=i==k?w:0;}}return true;}
};
class Anchor:public ceres::SizedCostFunction<6,7>{std::array<double,7> origin_;
public:explicit Anchor(const std::array<double,7>& x):origin_(x){}
 bool Evaluate(double const*const* p,double* r,double** j)const override{Eigen::Map<Eigen::Matrix<double,6,1>> out(r);out.head<3>()=(Eigen::Map<const Eigen::Vector3d>(p[0])-Eigen::Map<const Eigen::Vector3d>(origin_.data()))*1e4;Eigen::Quaterniond dq=Eigen::Map<const Eigen::Quaterniond>(origin_.data()+3).inverse()*Eigen::Map<const Eigen::Quaterniond>(p[0]+3);out.tail<3>()=2*(dq.w()<0?-1:1)*dq.vec()*1e4;if(j&&j[0]){Eigen::Map<Eigen::Matrix<double,6,7,Eigen::RowMajor>> J(j[0]);J.setZero();J.leftCols<6>().setIdentity();J*=1e4;}return out.allFinite();}
};
class PriorReference:public ceres::CostFunction{const MarginalPrior& prior_;
public:explicit PriorReference(const MarginalPrior& p):prior_(p){*mutable_parameter_block_sizes()=p.parameter_block_sizes();set_num_residuals(p.num_residuals());}
 bool Evaluate(double const*const* p,double* r,double** j)const override{return prior_.Evaluate(p,r,j);}
};
// Preserve the calibrated stereo baseline while the common camera/body transform is refined.
class RigResidual final:public ceres::SizedCostFunction<6,7,7> {
 double baseline_;
 void residual(const double* a,const double* b,double* r)const{
  Eigen::Map<const Eigen::Quaterniond> qa(a+3),qb(b+3);Eigen::Vector3d d=qa.conjugate()*(Eigen::Map<const Eigen::Vector3d>(b)-Eigen::Map<const Eigen::Vector3d>(a));Eigen::Quaterniond q=qa.conjugate()*qb;
  for(int k=0;k<3;++k){r[k]=(d[k]-(k==0?baseline_:0))*1e4;r[k+3]=q.vec()[k]*2e4;}
 }
public:
 explicit RigResidual(double baseline):baseline_(baseline){}
 bool Evaluate(double const*const* p,double* r,double** j)const override{
  residual(p[0],p[1],r);
  if(j){PoseLocalParameterization local;for(int block=0;block<2;++block)if(j[block]){
   std::fill(j[block],j[block]+42,0);for(int col=0;col<6;++col){double delta[6]{},plus[7],minus[7],rp[6],rm[6];delta[col]=1e-6;local.Plus(p[block],delta,plus);delta[col]=-1e-6;local.Plus(p[block],delta,minus);residual(block==0?plus:p[0],block==1?plus:p[1],rp);residual(block==0?minus:p[0],block==1?minus:p[1],rm);for(int row=0;row<6;++row)j[block][row*7+col]=(rp[row]-rm[row])*5e5;}
  }}return true;
 }
};
}
struct TightBackend::Impl {
 Config cfg;CameraGeometry geometry;
 std::deque<std::unique_ptr<State>> states;
 std::map<uint64_t,std::unique_ptr<Landmark>> points;
 std::unique_ptr<MarginalPrior> prior;
 std::array<double,7> extrinsic[2];std::array<double,7> seed_pose;std::array<double,9> seed_sb;
 Eigen::Matrix3d rotation_excitation=Eigen::Matrix3d::Zero();
 double td=0;std::array<double,4> noise{.1,.01,.0001,.00001};
 std::vector<ImuSample> pending;uint64_t next_id=1,solves=0,marginalizations=0;double last_cost=0,solve_ms=0,excitation=0;
 Pose latest;bool calibration_open=false;
 State* state(uint64_t id){for(auto& s:states)if(s->id==id)return s.get();throw std::runtime_error("landmark references removed state");}
 std::unique_ptr<IntegrationBase> integrate(const State& origin,const std::vector<ImuSample>& segment){
  if(segment.size()<2||segment.size()>512)throw std::runtime_error("invalid keyframe IMU segment size");
  auto p=std::make_unique<IntegrationBase>(eigen(segment.front().accel),eigen(segment.front().gyro),Eigen::Map<const Eigen::Vector3d>(origin.sb.data()+3),Eigen::Map<const Eigen::Vector3d>(origin.sb.data()+6),noise[0],noise[1],noise[2],noise[3]);
  for(size_t i=1;i<segment.size();++i){double dt=segment[i].t-segment[i-1].t;if(!validImu(segment[i])||dt<=0||dt>.05)throw std::runtime_error("invalid keyframe IMU data");p->push_back(dt,eigen(segment[i].accel),eigen(segment[i].gyro));}
  if(!p->covariance.allFinite()||p->sum_dt>.2+1e-6)throw std::runtime_error("invalid keyframe preintegration");
  return p;
 }
 std::unique_ptr<State> predict(const std::vector<ImuSample>& segment){
  auto& last=*states.back();auto pre=integrate(last,segment);auto next=std::make_unique<State>();next->id=next_id;next->t=segment.back().t;next->sb=last.sb;
  double dt=pre->sum_dt;Eigen::Vector3d gravity(0,0,9.80665);
  next->p()=last.p()+last.v()*dt+last.q()*pre->delta_p+gravity*(.5*dt*dt);
  next->v()=last.v()+last.q()*pre->delta_v+gravity*dt;
  Eigen::Map<Eigen::Quaterniond>(next->x.data()+3)=(last.q()*pre->delta_q).normalized();next->imu=segment;return next;
 }
 std::vector<FactorBlock> factors(bool marginal,std::vector<std::unique_ptr<IntegrationBase>>& integrations){
  std::vector<FactorBlock> out;uint64_t oldest=states.front()->id;
  if(prior)out.emplace_back(new PriorReference(*prior),prior->blocks);
  else{out.emplace_back(new Anchor(seed_pose),std::vector<double*>{states.front()->x.data()});out.emplace_back(new SpeedPrior(seed_sb),std::vector<double*>{states.front()->sb.data()});}
  for(size_t i=1;i<states.size();++i){if(marginal&&i!=1)continue;auto& a=*states[i-1];auto& b=*states[i];integrations.push_back(integrate(a,b.imu));out.emplace_back(new IMUFactor(integrations.back().get()),std::vector<double*>{a.x.data(),a.sb.data(),b.x.data(),b.sb.data()});}
  for(auto& [id,ptr]:points){auto& point=*ptr;if(point.seen.empty()||(marginal&&point.seen.front().frame!=oldest))continue;const auto& first=point.seen.front();auto* anchor=state(first.frame);
   Eigen::Vector3d a(first.obs.left.x,first.obs.left.y,1);Eigen::Vector2d va(first.obs.velocity.x,first.obs.velocity.y);
   for(const auto& seen:point.seen){
    Eigen::Vector2d velocity(seen.obs.velocity.x,seen.obs.velocity.y),right_velocity(seen.obs.right_velocity.x,seen.obs.right_velocity.y);auto* current=state(seen.frame);
    if(seen.frame!=first.frame){auto* cost=new ProjectionTwoFrameOneCamFactor(a,{seen.obs.left.x,seen.obs.left.y,1},va,velocity,first.td,seen.td);cost->sqrt_info=information();out.emplace_back(cost,std::vector<double*>{anchor->x.data(),current->x.data(),extrinsic[0].data(),&point.inverse,&td},true);}
    if(seen.obs.stereo){Eigen::Vector3d b(seen.obs.right.x,seen.obs.right.y,1);
     if(seen.frame==first.frame){auto* cost=new ProjectionOneFrameTwoCamFactor(a,b,va,right_velocity,first.td,seen.td);cost->sqrt_info=information();out.emplace_back(cost,std::vector<double*>{extrinsic[0].data(),extrinsic[1].data(),&point.inverse,&td},true);}
     else{auto* cost=new ProjectionTwoFrameTwoCamFactor(a,b,va,right_velocity,first.td,seen.td);cost->sqrt_info=information();out.emplace_back(cost,std::vector<double*>{anchor->x.data(),current->x.data(),extrinsic[0].data(),extrinsic[1].data(),&point.inverse,&td},true);}
    }
   }
  }
  if(!marginal&&calibration_open)out.emplace_back(new RigResidual(geometry.baseline),std::vector<double*>{extrinsic[0].data(),extrinsic[1].data()});
  return out;
 }
 Eigen::Matrix2d information()const{Eigen::Matrix2d info=Eigen::Matrix2d::Zero();info(0,0)=geometry.K.at<double>(0,0)/cfg.pixel_noise;info(1,1)=geometry.K.at<double>(1,1)/cfg.pixel_noise;return info;}
 void optimize(){
  std::vector<std::unique_ptr<IntegrationBase>> integrations;auto residuals=factors(false,integrations);
  ceres::Problem::Options ownership;ownership.cost_function_ownership=ceres::DO_NOT_TAKE_OWNERSHIP;ceres::Problem problem(ownership);
  auto* local=new PoseLocalParameterization;auto ordering=std::make_shared<ceres::ParameterBlockOrdering>();
  for(auto& s:states){problem.AddParameterBlock(s->x.data(),7,local);problem.AddParameterBlock(s->sb.data(),9);ordering->AddElementToGroup(s->x.data(),1);ordering->AddElementToGroup(s->sb.data(),1);for(int k=0;k<3;++k){problem.SetParameterLowerBound(s->sb.data(),k+3,-2.5);problem.SetParameterUpperBound(s->sb.data(),k+3,2.5);problem.SetParameterLowerBound(s->sb.data(),k+6,-1);problem.SetParameterUpperBound(s->sb.data(),k+6,1);}}
  for(auto& x:extrinsic){problem.AddParameterBlock(x.data(),7,local);ordering->AddElementToGroup(x.data(),2);if(!cfg.estimate_extrinsics||!calibration_open)problem.SetParameterBlockConstant(x.data());}
  problem.AddParameterBlock(&td,1);ordering->AddElementToGroup(&td,2);problem.SetParameterLowerBound(&td,0,-.02);problem.SetParameterUpperBound(&td,0,.02);if(!cfg.estimate_time_offset||!calibration_open)problem.SetParameterBlockConstant(&td);
  for(auto& [id,p]:points){problem.AddParameterBlock(&p->inverse,1);ordering->AddElementToGroup(&p->inverse,0);problem.SetParameterLowerBound(&p->inverse,0,1./30);problem.SetParameterUpperBound(&p->inverse,0,10);}
  for(auto& f:residuals)problem.AddResidualBlock(f.cost.get(),f.robust?new ceres::CauchyLoss(1):nullptr,f.blocks);
  ceres::Solver::Options options;options.linear_solver_type=ceres::DENSE_SCHUR;options.linear_solver_ordering=ordering;options.num_threads=1;options.max_num_iterations=cfg.solver_iterations;options.max_solver_time_in_seconds=cfg.solver_time;options.minimizer_progress_to_stdout=false;
  auto start=std::chrono::steady_clock::now();ceres::Solver::Summary summary;ceres::Solve(options,&problem,&summary);solve_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  if(!summary.IsSolutionUsable()||!std::isfinite(summary.final_cost))throw std::runtime_error("joint visual-inertial optimization failed");
  last_cost=summary.final_cost;++solves;
  for(auto& s:states)if(!Eigen::Map<Eigen::Matrix<double,7,1>>(s->x.data()).allFinite()||!Eigen::Map<Eigen::Matrix<double,9,1>>(s->sb.data()).allFinite()||std::abs(s->q().norm()-1)>1e-6||s->v().norm()>30)throw std::runtime_error("optimized state outside supported envelope");
 }
 void slide(){
  std::vector<std::unique_ptr<IntegrationBase>> integrations;auto residuals=factors(true,integrations);std::set<double*> drop{states.front()->x.data(),states.front()->sb.data()};uint64_t oldest=states.front()->id;
  for(auto& [id,p]:points)if(p->seen.front().frame==oldest)drop.insert(&p->inverse);
  auto next=MarginalPrior::build(residuals,drop);prior=std::move(next);
  for(auto it=points.begin();it!=points.end();)if(it->second->seen.front().frame==oldest)it=points.erase(it);else ++it;
  states.pop_front();states.front()->imu.clear();++marginalizations;
 }
};
TightBackend::TightBackend(const Config& cfg,const CameraGeometry& geometry,const Pose& pose,const V3& ba,const V3& bg,const ImuSample& endpoint):impl_(new Impl){auto& s=*impl_;s.cfg=cfg;s.geometry=geometry;auto first=std::make_unique<State>();first->t=pose.t;first->p()=eigen(pose.p);first->v()=eigen(pose.v);first->ba()=eigen(ba);first->bg()=eigen(bg);Eigen::Map<Eigen::Quaterniond>(first->x.data()+3)=Eigen::Quaterniond(eigen(pose.R));s.seed_pose=first->x;s.seed_sb=first->sb;s.states.push_back(std::move(first));
 auto R=eigen(M3(geometry.Rbc));Eigen::Vector3d t=eigen(V3(geometry.tbc));
 for(int i=0;i<2;++i){s.extrinsic[i]={0,0,0,0,0,0,1};Eigen::Map<Eigen::Vector3d>(s.extrinsic[i].data())=t+R*Eigen::Vector3d(i*geometry.baseline,0,0);Eigen::Map<Eigen::Quaterniond>(s.extrinsic[i].data()+3)=Eigen::Quaterniond(R);}
 s.pending.push_back(endpoint);s.latest=pose;
}
TightBackend::~TightBackend()=default;
void TightBackend::setNoise(const std::array<double,4>& n){impl_->noise=n;}
double TightBackend::timeOffset()const{return impl_->td;}
CameraGeometry TightBackend::geometry()const{auto out=impl_->geometry;Eigen::Map<const Eigen::Quaterniond> q(impl_->extrinsic[0].data()+3);out.Rbc=cv::Mat(cvrot(q.toRotationMatrix())).clone();out.tbc=cv::Mat(cvvec(Eigen::Map<const Eigen::Vector3d>(impl_->extrinsic[0].data()))).clone();return out;}
Pose TightBackend::process(double t,const std::vector<Observation>& observations,const std::vector<ImuSample>& segment){auto& s=*impl_;
 if(segment.size()<2||!std::isfinite(t)||std::abs(segment.back().t-t)>1e-6||std::abs(segment.front().t-s.pending.back().t)>1e-6)throw std::runtime_error("discontinuous backend IMU bracket");
 for(size_t i=1;i<segment.size();++i){if(segment[i].t<=s.pending.back().t)throw std::runtime_error("non-monotonic backend sample");s.pending.push_back(segment[i]);s.excitation+=cv::norm(segment[i].gyro)* (segment[i].t-segment[i-1].t);auto w=eigen(segment[i].gyro);s.rotation_excitation+=w*w.transpose()*(segment[i].t-segment[i-1].t);}
 if(s.pending.size()>512)throw std::runtime_error("backend pending IMU overflow");
 auto next=s.predict(s.pending);next->t=t;
 if(s.states.size()>1&&t-s.states.back()->t<1./s.cfg.optimizer_fps-1e-6){s.latest=next->pose();s.latest.valid=s.solves>=3;s.latest.inliers=int(observations.size());return s.latest;}
 // Freeze online calibration until rotational excitation and a populated window exist.
 Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> excited(s.rotation_excitation);s.calibration_open=s.excitation>.5&&excited.eigenvalues().minCoeff()>.002&&s.states.size()>=size_t(s.cfg.window_size);
 ++s.next_id;uint64_t frame=next->id;s.states.push_back(std::move(next));s.pending={segment.back()};
 for(const auto& o:observations){if(!std::isfinite(o.left.x)||!std::isfinite(o.left.y)||!std::isfinite(o.velocity.x)||!std::isfinite(o.velocity.y)||!std::isfinite(o.right_velocity.x)||!std::isfinite(o.right_velocity.y)||!std::isfinite(o.depth)||(o.stereo&&(!std::isfinite(o.right.x)||!std::isfinite(o.right.y))))throw std::runtime_error("nonfinite visual observation");auto it=s.points.find(o.id);
  if(it==s.points.end()){if(!o.stereo||o.depth<.1||o.depth>15||s.points.size()>=size_t(s.cfg.max_features))continue;auto p=std::make_unique<Landmark>();p->inverse=1/o.depth;it=s.points.emplace(o.id,std::move(p)).first;}
  it->second->seen.push_back({frame,o,s.td});
 }
 int visible=0;for(const auto& [id,p]:s.points)if(p->seen.back().frame==frame)++visible;
 if(visible<12)throw std::runtime_error("insufficient shared window features");
 s.optimize();
 int accepted=0;std::vector<uint64_t> remove;
 for(auto& [id,p]:s.points){if(p->seen.back().frame!=frame)continue;const auto& anchor=p->seen.front();const auto& seen=p->seen.back();auto* first=s.state(anchor.frame);auto& last=*s.states.back();
  Eigen::Vector3d ray(anchor.obs.left.x-(s.td-anchor.td)*anchor.obs.velocity.x,anchor.obs.left.y-(s.td-anchor.td)*anchor.obs.velocity.y,1);
  Eigen::Map<const Eigen::Quaterniond> qa(s.extrinsic[0].data()+3);Eigen::Map<const Eigen::Vector3d> ta(s.extrinsic[0].data());Eigen::Vector3d world=first->q()*(qa*(ray/p->inverse)+ta)+first->p();bool good=true;
  for(int camera=0;camera<(seen.obs.stereo?2:1);++camera){Eigen::Map<const Eigen::Quaterniond> qc(s.extrinsic[camera].data()+3);Eigen::Map<const Eigen::Vector3d> tc(s.extrinsic[camera].data());Eigen::Vector3d point=qc.conjugate()*(last.q().conjugate()*(world-last.p())-tc);auto observed=camera==0?seen.obs.left:seen.obs.right;auto velocity=camera==0?seen.obs.velocity:seen.obs.right_velocity;
   if(!point.allFinite()||point.z()<.1){good=false;break;}Eigen::Vector2d residual(point.x()/point.z()-observed.x+(s.td-seen.td)*velocity.x,point.y()/point.z()-observed.y+(s.td-seen.td)*velocity.y);if((s.information()*residual).norm()>4.5)good=false;
  }
  if(good)++accepted;else if(p->seen.size()>1)p->seen.pop_back();else remove.push_back(id);
 }
 for(auto id:remove)s.points.erase(id);
 if(accepted<12||accepted<visible*.5)throw std::runtime_error("joint reprojection consensus failed");
 s.latest=s.states.back()->pose();s.latest.valid=s.solves>=3;s.latest.inliers=accepted;
 if(s.states.size()>size_t(s.cfg.window_size))s.slide();
 return s.latest;
}
Estimator::Diagnostics TightBackend::diagnostics()const{const auto& s=*impl_;Estimator::Diagnostics d;d.window=s.states.size();d.landmarks=s.points.size();d.solves=s.solves;d.marginalizations=s.marginalizations;d.solve_ms=s.solve_ms;d.cost=s.last_cost;d.time_offset=s.td;d.calibration_active=s.calibration_open&&(s.cfg.estimate_extrinsics||s.cfg.estimate_time_offset);auto geometry=this->geometry();d.Rbc=geometry.Rbc;d.tbc=geometry.tbc;d.ba=cvvec(s.states.back()->ba());d.bg=cvvec(s.states.back()->bg());return d;}
}
