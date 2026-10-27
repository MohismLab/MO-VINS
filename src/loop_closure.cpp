// SPDX-License-Identifier: GPL-3.0-only
// Gravity-preserving four-DoF graph follows VINS-Fusion pose_graph. Candidate
// retrieval uses bounded ORB matching instead of its external BRIEF vocabulary.
#include "loop_closure.hpp"
#include "preintegration.hpp"
#include "marginal_prior.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <ceres/sized_cost_function.h>
#include <ceres/local_parameterization.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/loss_function.h>
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <map>
#include <algorithm>
#include <numeric>
#include <fcntl.h>
#include <unistd.h>
namespace mohism {
namespace {
double yaw(const M3& r){return std::atan2(r(1,0),r(0,0));}
double wrap(double x){return std::atan2(std::sin(x),std::cos(x));}
M3 rz(double x){return expSO3({0,0,x});}
struct Keyframe {uint64_t id=0,epoch=0;double t=0;Pose raw;std::array<double,4> map{};cv::Mat descriptors;std::vector<cv::Point2f> pixels;std::vector<cv::Point3f> body_points;};
M3 attitude(const Keyframe& f){return rz(wrap(f.map[3]-yaw(f.raw.R)))*f.raw.R;}
class GraphPriorReference final:public ceres::CostFunction {
 const MarginalPrior& prior_;
public:explicit GraphPriorReference(const MarginalPrior& p):prior_(p){*mutable_parameter_block_sizes()=p.parameter_block_sizes();set_num_residuals(p.num_residuals());}
 bool Evaluate(double const*const* p,double* r,double** j)const override{return prior_.Evaluate(p,r,j);}
};
class GraphAnchor final:public ceres::SizedCostFunction<4,4> {
 std::array<double,4> center_;
public:explicit GraphAnchor(const std::array<double,4>& p):center_(p){}
 bool Evaluate(double const*const* p,double* r,double** j)const override{for(int i=0;i<4;++i){r[i]=(p[0][i]-center_[i])*1e4;if(j&&j[0])for(int k=0;k<4;++k)j[0][i*4+k]=i==k?1e4:0;}return true;}
};
struct Edge {uint64_t a,b;V3 translation;double dyaw;bool loop;};
class GraphResidual final:public ceres::SizedCostFunction<4,4,4> {
 M3 tilt_;V3 translation_;double dyaw_,scale_;
public:
 GraphResidual(const M3& tilt,const V3& translation,double dyaw,double scale):tilt_(tilt),translation_(translation),dyaw_(dyaw),scale_(scale){}
 bool Evaluate(double const*const* p,double* residual,double** jacobians)const override{
  const auto* a=p[0];const auto* b=p[1];M3 R=rz(a[3])*tilt_;V3 delta{b[0]-a[0],b[1]-a[1],b[2]-a[2]},rel=R.t()*delta;
  for(int k=0;k<3;++k)residual[k]=(rel[k]-translation_[k])*scale_;
  residual[3]=wrap(b[3]-a[3]-dyaw_)*scale_*3;
  if(jacobians)for(int block=0;block<2;++block)if(jacobians[block]){
   auto* J=jacobians[block];std::fill(J,J+16,0);
   for(int row=0;row<3;++row)for(int col=0;col<3;++col)J[row*4+col]=(block==0?-1:1)*R(col,row)*scale_;
   if(block==0){V3 d=R.t()*V3{delta[1],-delta[0],0};for(int k=0;k<3;++k)J[k*4+3]=d[k]*scale_;}
   J[15]=(block==0?-1:1)*scale_*3;
  }return true;
 }
};
}
struct LoopClosure::Impl {
 Config cfg;CameraGeometry geometry,fingerprint;cv::Ptr<cv::ORB> orb=cv::ORB::create(350,1.2f,4,15,0,2,cv::ORB::HARRIS_SCORE,31,10);
 std::deque<Keyframe> frames;std::vector<Edge> edges;std::unique_ptr<MarginalPrior> prior;uint64_t next=0,epoch=0,loops=0,relocalized=0,revision=0;
 M3 correction=M3::eye();V3 shift{};double last_t=-1;uint64_t pending_id=UINT64_MAX,pending_epoch=UINT64_MAX;unsigned confirmations=0;Pose pending_pose;bool need_relocalization=false;
 Keyframe* find(uint64_t id){for(auto& f:frames)if(f.id==id)return &f;return nullptr;}
 Keyframe extract(const Pose& local,const cv::Mat& left,const cv::Mat& right){
  Keyframe frame;frame.id=next++;frame.epoch=epoch;frame.t=local.t;frame.raw=local;auto map=correct(local);frame.map={map.p[0],map.p[1],map.p[2],yaw(map.R)};
  std::vector<cv::KeyPoint> keys;cv::Mat descriptors;orb->detectAndCompute(left,cv::noArray(),keys,descriptors);
  if(keys.empty())return frame;
  std::vector<cv::Point2f> a,b,back;for(const auto& k:keys)a.push_back(k.pt);std::vector<uchar> valid,reverse;std::vector<float> error;
  cv::calcOpticalFlowPyrLK(left,right,a,b,valid,error,{21,21},3);cv::calcOpticalFlowPyrLK(right,left,b,back,reverse,error,{21,21},3);
  double fx=geometry.K.at<double>(0,0),fy=geometry.K.at<double>(1,1),cx=geometry.K.at<double>(0,2),cy=geometry.K.at<double>(1,2);M3 Rbc(geometry.Rbc);V3 tbc(geometry.tbc);
  for(size_t i=0;i<a.size();++i){double disparity=a[i].x-b[i].x;if(!valid[i]||!reverse[i]||cv::norm(a[i]-back[i])>.5||std::abs(a[i].y-b[i].y)>.75||disparity<.75)continue;double z=fx*geometry.baseline/disparity;if(z<.1||z>15)continue;V3 point=Rbc*V3{(a[i].x-cx)*z/fx,(a[i].y-cy)*z/fy,z}+tbc;
   frame.pixels.push_back(a[i]);frame.body_points.emplace_back(float(point[0]),float(point[1]),float(point[2]));frame.descriptors.push_back(descriptors.row(int(i)));
  }return frame;
 }
 Pose correct(const Pose& local)const{Pose out=local;out.R=correction*local.R;out.p=correction*local.p+shift;out.v=correction*local.v;return out;}
 bool verify(const Keyframe& old,const Keyframe& current,Pose& detected,int& count){
  if(old.descriptors.rows<30||current.descriptors.rows<30)return false;
  cv::BFMatcher matcher(cv::NORM_HAMMING);std::vector<std::vector<cv::DMatch>> forward;matcher.knnMatch(current.descriptors,old.descriptors,forward,2);
  std::vector<cv::DMatch> reverse;matcher.match(old.descriptors,current.descriptors,reverse);
  std::vector<cv::Point3f> world;std::vector<cv::Point2f> pixels;std::vector<int> query;M3 R=attitude(old);V3 p{old.map[0],old.map[1],old.map[2]};
  for(size_t i=0;i<forward.size();++i)if(forward[i].size()==2){const auto& a=forward[i][0];if(a.distance>50||a.distance>=.75f*forward[i][1].distance||reverse[a.trainIdx].trainIdx!=int(i))continue;auto bp=old.body_points[a.trainIdx];V3 x=R*V3{bp.x,bp.y,bp.z}+p;world.emplace_back(float(x[0]),float(x[1]),float(x[2]));pixels.push_back(current.pixels[i]);query.push_back(int(i));}
  if(world.size()<30)return false;
  cv::Mat rv,tv,inliers;if(!cv::solvePnPRansac(world,pixels,geometry.K,cv::noArray(),rv,tv,false,150,1.5,.999,inliers,cv::SOLVEPNP_EPNP)||inliers.rows<25||inliers.rows<int(world.size()*.6))return false;
  std::vector<cv::Point3f> fit3;std::vector<cv::Point2f> fit2;for(int i=0;i<inliers.rows;++i){int k=inliers.at<int>(i);fit3.push_back(world[k]);fit2.push_back(pixels[k]);}
  cv::solvePnPRefineLM(fit3,fit2,geometry.K,cv::noArray(),rv,tv);M3 Rcw;cv::Rodrigues(rv,Rcw);M3 Rbc(geometry.Rbc);V3 tbc(geometry.tbc);
  detected=current.raw;detected.R=Rcw.t()*Rbc.t();detected.p=-Rcw.t()*V3(tv)-detected.R*tbc;
  if(!rotation(detected.R)||!finite(detected.p))return false;
  M3 aligned=rz(yaw(detected.R)-yaw(current.raw.R))*current.raw.R;if(cv::norm(logSO3(aligned.t()*detected.R))>.10)return false;
  std::vector<cv::Point2f> hull;cv::convexHull(fit2,hull);if(std::abs(cv::contourArea(hull))<.05*320*200)return false;
  int consistent=0;for(int i=0;i<inliers.rows;++i){int k=inliers.at<int>(i);auto w=world[k],body=current.body_points[query[k]];V3 predicted=detected.R*V3{body.x,body.y,body.z}+detected.p;if(cv::norm(predicted-V3{w.x,w.y,w.z})<std::max(.08,.05*cv::norm(V3{body.x,body.y,body.z})))++consistent;}
  if(consistent<25||consistent<inliers.rows*.8)return false;
  count=consistent;return true;
 }
 void evict(){
  auto& oldest=frames.front();std::vector<FactorBlock> residuals;
  if(prior)residuals.emplace_back(new GraphPriorReference(*prior),prior->blocks);
  if(!prior)residuals.emplace_back(new GraphAnchor(oldest.map),std::vector<double*>{oldest.map.data()});
  for(const auto& edge:edges)if(edge.a==oldest.id||edge.b==oldest.id){auto* a=find(edge.a);auto* b=find(edge.b);if(!a||!b)throw std::runtime_error("graph edge lost node");residuals.emplace_back(new GraphResidual(rz(-yaw(a->raw.R))*a->raw.R,edge.translation,edge.dyaw,edge.loop?20.:50.),std::vector<double*>{a->map.data(),b->map.data()},edge.loop,edge.loop);}
  bool connected=false;for(const auto& factor:residuals)if(std::find(factor.blocks.begin(),factor.blocks.end(),oldest.map.data())!=factor.blocks.end())connected=true;
  if(connected&&residuals.size()>1)prior=MarginalPrior::build(residuals,{oldest.map.data()});
  uint64_t removed=oldest.id;edges.erase(std::remove_if(edges.begin(),edges.end(),[&](const Edge& e){return e.a==removed||e.b==removed;}),edges.end());frames.pop_front();
 }
 void optimize(){
  if(frames.empty())return;
  ceres::Problem problem;std::map<uint64_t,size_t> indices;std::vector<size_t> components(frames.size());std::iota(components.begin(),components.end(),0);
  auto root=[&](size_t index){while(components[index]!=index)index=components[index];return index;};
  for(size_t i=0;i<frames.size();++i){indices[frames[i].id]=i;problem.AddParameterBlock(frames[i].map.data(),4);}
  if(prior){problem.AddResidualBlock(new GraphPriorReference(*prior),nullptr,prior->blocks);size_t base=frames.size();for(auto* address:prior->blocks){for(size_t i=0;i<frames.size();++i)if(frames[i].map.data()==address){if(base==frames.size())base=i;else components[root(i)]=root(base);break;}}}
  for(const auto& edge:edges){auto ia=indices.find(edge.a),ib=indices.find(edge.b);if(ia==indices.end()||ib==indices.end())continue;auto& a=frames[ia->second];auto& b=frames[ib->second];components[root(ib->second)]=root(ia->second);auto* cost=new GraphResidual(rz(-yaw(a.raw.R))*a.raw.R,edge.translation,edge.dyaw,edge.loop?20.:50.);problem.AddResidualBlock(cost,edge.loop?new ceres::HuberLoss(1):nullptr,a.map.data(),b.map.data());}
  std::set<size_t> anchored;if(prior)for(auto* address:prior->blocks)for(size_t i=0;i<frames.size();++i)if(frames[i].map.data()==address)anchored.insert(root(i));
  for(size_t i=0;i<frames.size();++i)if(anchored.insert(root(i)).second)problem.SetParameterBlockConstant(frames[i].map.data());
  ceres::Solver::Options options;options.linear_solver_type=ceres::SPARSE_NORMAL_CHOLESKY;options.max_num_iterations=20;options.num_threads=1;options.max_solver_time_in_seconds=.1;ceres::Solver::Summary summary;ceres::Solve(options,&problem,&summary);
  if(!summary.IsSolutionUsable())throw std::runtime_error("loop pose graph solve failed");
  auto& current=frames.back();M3 updated=rz(wrap(current.map[3]-yaw(current.raw.R)));V3 updated_shift=V3{current.map[0],current.map[1],current.map[2]}-updated*current.raw.p;
  if(cv::norm(logSO3(correction.t()*updated))>.02||cv::norm(updated_shift-shift)>.05)++revision;
  correction=updated;shift=updated_shift;
 }
};
LoopClosure::LoopClosure(const Config& cfg,const CameraGeometry& geometry):impl_(new Impl){impl_->cfg=cfg;impl_->geometry=geometry;impl_->fingerprint=geometry;}
LoopClosure::~LoopClosure()=default;
void LoopClosure::setRuntimeGeometry(const CameraGeometry& geometry){if(!rotation(M3(geometry.Rbc))||!finite(V3(geometry.tbc)))throw std::runtime_error("invalid runtime extrinsics");impl_->geometry.Rbc=geometry.Rbc.clone();impl_->geometry.tbc=geometry.tbc.clone();}
void LoopClosure::newEpoch(){auto& s=*impl_;++s.epoch;s.correction=M3::eye();s.shift={};s.last_t=-1;s.confirmations=0;s.pending_id=UINT64_MAX;s.need_relocalization=true;}
Pose LoopClosure::update(const Pose& local,const cv::Mat& left,const cv::Mat& right,uint64_t){auto& s=*impl_;if(!local.valid)return local;
 if(s.last_t>=0&&local.t-s.last_t<.2)return s.correct(local);
 s.last_t=local.t;
 Keyframe current=s.extract(local,left,right);if(current.descriptors.rows<30)return s.correct(local);
 int best=0;uint64_t candidate=UINT64_MAX;Pose detected;
 for(const auto& old:s.frames){if(old.epoch==current.epoch&&current.id-old.id<20)continue;Pose pose;int count=0;if(s.verify(old,current,pose,count)&&count>best){best=count;candidate=old.id;detected=pose;}}
 bool confirmed=false;
 if(candidate!=UINT64_MAX){auto* reference=s.find(candidate);
  if(reference&&s.pending_epoch==reference->epoch&&std::abs(int64_t(s.pending_id)-int64_t(reference->id))<=5&&s.confirmations>0){
   auto expected=s.pending_pose.p+s.correction*(current.raw.p-s.frames.back().raw.p);
   if(cv::norm(detected.p-expected)<.3)++s.confirmations;else s.confirmations=1;
  }else s.confirmations=1;
  s.pending_id=candidate;s.pending_epoch=reference->epoch;s.pending_pose=detected;confirmed=s.confirmations>=2;
 }else{s.confirmations=0;s.pending_id=UINT64_MAX;}
 if(!s.frames.empty()&&s.frames.back().epoch==s.epoch){auto& previous=s.frames.back();s.edges.push_back({previous.id,current.id,previous.raw.R.t()*(current.raw.p-previous.raw.p),wrap(yaw(current.raw.R)-yaw(previous.raw.R)),false});}
 if(confirmed){auto& old=*s.find(candidate);
  if(old.epoch!=s.epoch&&s.need_relocalization){M3 align=rz(yaw(detected.R)-yaw(current.raw.R));V3 offset=detected.p-align*current.raw.p;
   for(auto& frame:s.frames)if(frame.epoch==s.epoch){V3 p=align*frame.raw.p+offset;frame.map={p[0],p[1],p[2],yaw(align*frame.raw.R)};}
   current.map={detected.p[0],detected.p[1],detected.p[2],yaw(detected.R)};
  }
 M3 R=attitude(old);V3 p{old.map[0],old.map[1],old.map[2]};s.edges.push_back({old.id,current.id,R.t()*(detected.p-p),wrap(yaw(detected.R)-old.map[3]),true});++s.loops;if(s.need_relocalization){++s.relocalized;s.need_relocalization=false;}s.confirmations=0;}
 s.frames.push_back(std::move(current));
 while(s.frames.size()>size_t(s.cfg.loop_max_keyframes))s.evict();
 if(confirmed)s.optimize();
 if(!s.cfg.map_file.empty()&&(confirmed||s.next%10==0)){try{save(s.cfg.map_file);}catch(const std::exception& e){std::cerr<<"map save: "<<e.what()<<'\n';}}
 return s.correct(local);
}
uint64_t LoopClosure::correctionVersion()const{return impl_->revision;}
size_t LoopClosure::keyframes()const{return impl_->frames.size();}
uint64_t LoopClosure::closures()const{return impl_->loops;}
uint64_t LoopClosure::relocalizations()const{return impl_->relocalized;}
void LoopClosure::save(const std::string& path)const{const auto& s=*impl_;std::string temporary=path+".tmp";cv::FileStorage file(temporary,cv::FileStorage::WRITE|cv::FileStorage::FORMAT_YAML);if(!file.isOpened())throw std::runtime_error("cannot write map");
 file<<"version"<<2<<"K"<<s.fingerprint.K<<"Rbc"<<s.fingerprint.Rbc<<"tbc"<<s.fingerprint.tbc<<"baseline"<<s.fingerprint.baseline<<"frames"<<"[";
 for(const auto& f:s.frames){file<<"{"<<"id"<<std::to_string(f.id)<<"epoch"<<std::to_string(f.epoch)<<"t"<<f.t<<"R"<<cv::Mat(f.raw.R)<<"p"<<cv::Mat(f.raw.p)<<"map"<<cv::Mat(std::vector<double>(f.map.begin(),f.map.end()))<<"descriptors"<<f.descriptors<<"pixels"<<f.pixels<<"body_points"<<f.body_points<<"}";}
 file<<"]"<<"edges"<<"[";for(const auto& e:s.edges)file<<"{"<<"a"<<std::to_string(e.a)<<"b"<<std::to_string(e.b)<<"translation"<<cv::Mat(e.translation)<<"dyaw"<<e.dyaw<<"loop"<<int(e.loop)<<"}";file<<"]";
 if(s.prior){cv::Mat A(int(s.prior->squareRoot().rows()),int(s.prior->squareRoot().cols()),CV_64F),b(int(s.prior->residualOrigin().size()),1,CV_64F);for(int row=0;row<A.rows;++row){b.at<double>(row)=s.prior->residualOrigin()[row];for(int col=0;col<A.cols;++col)A.at<double>(row,col)=s.prior->squareRoot()(row,col);}
  file<<"prior"<<"{"<<"A"<<A<<"b"<<b<<"blocks"<<"[";
  for(size_t i=0;i<s.prior->blocks.size();++i){uint64_t id=UINT64_MAX;for(const auto& f:s.frames)if(f.map.data()==s.prior->blocks[i])id=f.id;if(id==UINT64_MAX)throw std::runtime_error("cannot persist stale graph prior");cv::Mat origin(4,1,CV_64F);for(int k=0;k<4;++k)origin.at<double>(k)=s.prior->origins()[i][k];file<<"{"<<"id"<<std::to_string(id)<<"origin"<<origin<<"}";}
  file<<"]"<<"}";
 }file.release();
 int fd=open(temporary.c_str(),O_RDONLY|O_CLOEXEC);if(fd<0)throw std::runtime_error("map fsync open failed");int result=fsync(fd);close(fd);if(result||rename(temporary.c_str(),path.c_str()))throw std::runtime_error("map atomic replace failed");auto parent=std::filesystem::path(path).parent_path();if(parent.empty())parent=".";fd=open(parent.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC);if(fd>=0){fsync(fd);close(fd);}
}
void LoopClosure::load(const std::string& path){auto& s=*impl_;if(!std::filesystem::exists(path))return;if(std::filesystem::file_size(path)>32*1024*1024)throw std::runtime_error("map exceeds bounded input size");cv::FileStorage file(path,cv::FileStorage::READ);int version=0;file["version"]>>version;if(version!=2)throw std::runtime_error("unsupported map version");
 cv::Mat K,Rbc,tbc;double baseline=0;file["K"]>>K;file["Rbc"]>>Rbc;file["tbc"]>>tbc;file["baseline"]>>baseline;
 for(auto pair:{std::make_pair(K,s.fingerprint.K),std::make_pair(Rbc,s.fingerprint.Rbc),std::make_pair(tbc,s.fingerprint.tbc)})if(pair.first.size()!=pair.second.size()||pair.first.type()!=pair.second.type()||!cv::checkRange(pair.first)||cv::norm(pair.first-pair.second)>1e-9)throw std::runtime_error("map calibration mismatch");
 if(!std::isfinite(baseline)||std::abs(baseline-s.fingerprint.baseline)>1e-9)throw std::runtime_error("map baseline mismatch");
 auto records=file["frames"];if(records.type()!=cv::FileNode::SEQ||records.size()>size_t(s.cfg.loop_max_keyframes))throw std::runtime_error("map keyframe count invalid");std::deque<Keyframe> frames;std::set<uint64_t> ids;uint64_t next=0,epoch=0;
 for(const auto& item:records){Keyframe frame;std::string id,session;item["id"]>>id;item["epoch"]>>session;frame.id=std::stoull(id);frame.epoch=std::stoull(session);item["t"]>>frame.t;cv::Mat R,p,map;item["R"]>>R;item["p"]>>p;item["map"]>>map;item["descriptors"]>>frame.descriptors;item["pixels"]>>frame.pixels;item["body_points"]>>frame.body_points;
  if(R.size()!=cv::Size(3,3)||R.type()!=CV_64F||p.total()!=3||p.type()!=CV_64F||map.total()!=4||map.type()!=CV_64F||!cv::checkRange(R)||!cv::checkRange(p)||!cv::checkRange(map)||!std::isfinite(frame.t)||!ids.insert(frame.id).second||frame.id>=UINT64_MAX-1||frame.epoch>=UINT64_MAX-1)throw std::runtime_error("invalid map pose");
  frame.raw.R=M3(R);frame.raw.p=V3(p);if(!rotation(frame.raw.R))throw std::runtime_error("invalid map rotation");std::copy(map.ptr<double>(),map.ptr<double>()+4,frame.map.begin());
  if(frame.descriptors.type()!=CV_8UC1||frame.descriptors.cols!=32||frame.descriptors.rows<0||frame.descriptors.rows>350||frame.pixels.size()!=size_t(frame.descriptors.rows)||frame.body_points.size()!=frame.pixels.size())throw std::runtime_error("invalid map descriptors");
  for(size_t i=0;i<frame.pixels.size();++i){auto a=frame.pixels[i];auto b=frame.body_points[i];if(!std::isfinite(a.x)||!std::isfinite(a.y)||a.x<0||a.x>=320||a.y<0||a.y>=200||!std::isfinite(b.x)||!std::isfinite(b.y)||!std::isfinite(b.z))throw std::runtime_error("invalid map feature");}
  next=std::max(next,frame.id+1);epoch=std::max(epoch,frame.epoch+1);frames.push_back(std::move(frame));
 }
 std::vector<Edge> edges;auto links=file["edges"];if(links.type()!=cv::FileNode::SEQ||links.size()>records.size()*8)throw std::runtime_error("invalid map edge count");for(const auto& item:links){std::string a,b;Edge edge;int loop=0;cv::Mat t;item["a"]>>a;item["b"]>>b;edge.a=std::stoull(a);edge.b=std::stoull(b);item["translation"]>>t;item["dyaw"]>>edge.dyaw;item["loop"]>>loop;if(!ids.count(edge.a)||!ids.count(edge.b)||edge.a==edge.b||t.total()!=3||t.type()!=CV_64F||!cv::checkRange(t)||!std::isfinite(edge.dyaw)||(loop!=0&&loop!=1))throw std::runtime_error("invalid map edge");edge.translation=V3(t);edge.loop=loop;edges.push_back(edge);}
 std::unique_ptr<MarginalPrior> restored_prior;
 if(!file["prior"].empty()){
  cv::Mat A,b;file["prior"]["A"]>>A;file["prior"]["b"]>>b;
  if(A.type()!=CV_64F||b.type()!=CV_64F||A.rows!=A.cols||A.rows>int(records.size()*4)||b.total()!=size_t(A.rows)||!cv::checkRange(A)||!cv::checkRange(b))throw std::runtime_error("invalid map marginal prior");
  auto saved_blocks=file["prior"]["blocks"];if(saved_blocks.size()>records.size())throw std::runtime_error("map prior block count invalid");std::vector<double*> addresses;std::vector<Eigen::VectorXd> origins;
  for(const auto& item:saved_blocks){std::string id;cv::Mat origin;item["id"]>>id;item["origin"]>>origin;uint64_t value=std::stoull(id);auto found=std::find_if(frames.begin(),frames.end(),[&](const Keyframe& f){return f.id==value;});if(found==frames.end()||origin.total()!=4||origin.type()!=CV_64F||!cv::checkRange(origin))throw std::runtime_error("invalid map prior reference");addresses.push_back(found->map.data());Eigen::VectorXd center(4);for(int k=0;k<4;++k)center[k]=origin.at<double>(k);origins.push_back(center);}
  Eigen::MatrixXd matrix(A.rows,A.cols);Eigen::VectorXd vector(b.total());for(int row=0;row<A.rows;++row){vector[row]=b.at<double>(row);for(int col=0;col<A.cols;++col)matrix(row,col)=A.at<double>(row,col);}restored_prior=MarginalPrior::restore(matrix,vector,addresses,origins);
 }
 s.prior=std::move(restored_prior);
 s.frames=std::move(frames);s.edges=std::move(edges);s.next=next;s.epoch=epoch;s.need_relocalization=true;s.last_t=-1;s.pending_id=UINT64_MAX;s.confirmations=0;s.correction=M3::eye();s.shift={};
}
}
