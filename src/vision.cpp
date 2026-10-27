// SPDX-License-Identifier: GPL-3.0-only
#include "mohism.hpp"
#include "preintegration.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <cmath>
#include <algorithm>
namespace mohism {
static M3 skew(const V3& v){return M3(0,-v[2],v[1],v[2],0,-v[0],-v[1],v[0],0);}
Vision::Vision(const Config& c,const Pose& origin):cfg_(c),origin_(origin){
 c.validate();
 if(!finite(origin.p)||!rotation(origin.R))throw std::invalid_argument("invalid visual origin");
 cv::Mat a=c.K1.clone(),b=c.K2.clone();double sx=double(c.width)/c.capture_width,sy=double(c.height)/c.capture_height;
 a.row(0)*=sx;a.row(1)*=sy;b.row(0)*=sx;b.row(1)*=sy;
 cv::Mat r1,r2,p1,p2,q;cv::Size size(c.width,c.height);
 cv::stereoRectify(a,c.D1,b,c.D2,size,c.R,c.T,r1,r2,p1,p2,q,cv::CALIB_ZERO_DISPARITY,0,size);
 if(std::abs(p2.at<double>(1,3))>1e-6)throw std::runtime_error("only horizontal stereo is supported");
 cv::initUndistortRectifyMap(a,c.D1,r1,p1,size,CV_32FC1,map1x_,map1y_);
 cv::initUndistortRectifyMap(b,c.D2,r2,p2,size,CV_32FC1,map2x_,map2y_);
 K_=p1(cv::Rect(0,0,3,3)).clone();fx_=K_.at<double>(0,0);fy_=K_.at<double>(1,1);cx_=K_.at<double>(0,2);cy_=K_.at<double>(1,2);
 baseline_=-p2.at<double>(0,3)/p2.at<double>(0,0);
 if(!std::isfinite(baseline_)||baseline_<=0||!cv::checkRange(K_))throw std::runtime_error("invalid rectified stereo calibration");
 cfg_.Rbc=c.Rbc*r1.t();Rwc_=origin.R*M3(cfg_.Rbc);pwc_=origin.p+origin.R*V3(cfg_.tbc);
 matcher_=cv::StereoSGBM::create(0,cfg_.disparities,5,8*25,32*25,1,31,12,50,2,cv::StereoSGBM::MODE_SGBM);
}
Vision::Result Vision::process(const Frame& l,const Frame& r,bool estimate_pose){
 double t=(l.t+r.t)*.5,dt=t-previous_t_;
 if(!std::isfinite(l.t)||!std::isfinite(r.t)||l.t<0||r.t<0||std::abs(l.t-r.t)>cfg_.max_skew)
  throw std::runtime_error("invalid or unsynchronized stereo timestamps");
 if(previous_t_>=0&&(dt<=0||dt>.2))throw std::runtime_error("visual timestamp discontinuity");
 for(const auto* frame:{&l,&r})if(frame->image.type()!=CV_8UC1||frame->image.cols!=cfg_.capture_width||frame->image.rows!=cfg_.capture_height)
  throw std::runtime_error("invalid camera image dimensions or type");
 Result out;cv::Mat a,b;cv::resize(l.image,a,{cfg_.width,cfg_.height},0,0,cv::INTER_AREA);cv::resize(r.image,b,a.size(),0,0,cv::INTER_AREA);
 cv::remap(a,out.left,map1x_,map1y_,cv::INTER_LINEAR);cv::remap(b,out.right,map2x_,map2y_,cv::INTER_LINEAR);
 out.depth=cv::Mat::zeros(a.size(),CV_16UC1);
 // Sparse stereo VO runs every processed pair. Dense depth has its own budget.
 // Uncomputed depth is zero (invalid), never an old map paired with a new image.
 if(last_depth_t_<0||t-last_depth_t_>=1.0/cfg_.depth_fps-1e-6){
 cv::Mat disparity,reverse,flipped_l,flipped_r;
 matcher_->compute(out.left,out.right,disparity);
 cv::flip(out.left,flipped_l,1);cv::flip(out.right,flipped_r,1);
 matcher_->compute(flipped_r,flipped_l,reverse);cv::flip(reverse,reverse,1);
 for(int y=0;y<a.rows;++y)for(int x=0;x<a.cols;++x){
  double d=disparity.at<int16_t>(y,x)/16.;int rx=cvRound(x-d);
  if(d<=0||rx<0||rx>=a.cols)continue;
  double rd=reverse.at<int16_t>(y,rx)/16.;
  if(rd<=0||std::abs(d-rd)>1)continue;
  double mm=fx_*baseline_*1000/d;
  if(mm>=100&&mm<65535)out.depth.at<uint16_t>(y,x)=uint16_t(std::lround(mm));
 }
  last_depth_t_=t;out.depth_updated=true;
 }
 const auto& tracks=tracker_.process(out.left,out.right,fx_,baseline_);
 for(const auto& track:tracks){
  Observation o;o.id=track.id;o.left={(track.point.x-cx_)/fx_,(track.point.y-cy_)/fy_};
  o.stereo=track.depth>0;o.depth=track.depth;
  if(o.stereo)o.right={(track.right.x-cx_)/fx_,(track.right.y-cy_)/fy_};
  if(track.age>1&&dt>0)o.velocity={(track.point.x-track.previous.x)/(fx_*dt),(track.point.y-track.previous.y)/(fy_*dt)};
  if(track.age>1&&dt>0&&o.stereo&&track.previous_depth>0)o.right_velocity={(track.right.x-track.previous_right.x)/(fx_*dt),(track.right.y-track.previous_right.y)/(fy_*dt)};
  out.observations.push_back(o);
 }
 out.visual.t=t;
 if(!estimate_pose){
  out.visual.inliers=int(std::count_if(tracks.begin(),tracks.end(),[](const Track& track){return track.depth>0;}));
  if(out.visual.inliers<12)throw std::runtime_error("insufficient stereo observations");
  out.visual.valid=true;previous_t_=t;return out;
 }
 if(!started_){
  int depth_count=0;for(const auto& track:tracks)if(track.depth>0)++depth_count;
  if(depth_count<20)throw std::runtime_error("insufficient stereo features for visual initialization");
  out.visual.R=origin_.R;out.visual.p=origin_.p;out.visual.valid=true;out.visual.inliers=depth_count;started_=true;
 }else{
  std::vector<cv::Point3f> xyz;std::vector<cv::Point2f> uv;std::vector<double> observed_depth;
  for(const auto& track:tracks)if(track.age>1&&track.previous_depth>=.1&&track.previous_depth<=15&&track.depth>0){
   double z=track.previous_depth;
   xyz.emplace_back((track.previous.x-cx_)*z/fx_,(track.previous.y-cy_)*z/fy_,z);uv.push_back(track.point);observed_depth.push_back(track.depth);
  }
  if(xyz.size()<12)throw std::runtime_error("insufficient temporal stereo tracks");
  cv::Mat rv,tv,inliers;
  bool solved=cv::solvePnPRansac(xyz,uv,K_,cv::noArray(),rv,tv,false,100,1.5,.999,inliers,cv::SOLVEPNP_EPNP);
  if(!solved||inliers.rows<12||inliers.rows<int(xyz.size()*.5))throw std::runtime_error("visual RANSAC consensus failed");
  std::vector<cv::Point3f> fit_xyz;std::vector<cv::Point2f> fit_uv;
  for(int i=0;i<inliers.rows;++i){int index=inliers.at<int>(i);fit_xyz.push_back(xyz[index]);fit_uv.push_back(uv[index]);}
  cv::solvePnPRefineLM(fit_xyz,fit_uv,K_,cv::noArray(),rv,tv);
  M3 relative;cv::Rodrigues(rv,relative);V3 trans(tv);
  if(!rotation(relative)||!finite(trans)||cv::norm(trans)>8*dt||cv::norm(rv)>4*dt)
   throw std::runtime_error("visual motion exceeds supported envelope");
  std::vector<cv::Point2f> projected;cv::projectPoints(xyz,rv,tv,K_,cv::noArray(),projected);
  std::vector<cv::Point2f> consensus;std::vector<size_t> accepted;
  for(size_t i=0;i<xyz.size();++i){
   V3 camera=relative*V3(xyz[i].x,xyz[i].y,xyz[i].z)+trans;
   if(camera[2]>.1&&cv::norm(projected[i]-uv[i])<=1.5&&std::abs(camera[2]-observed_depth[i])<std::max(.10,.15*camera[2])){consensus.push_back(uv[i]);accepted.push_back(i);}
  }
  if(consensus.size()<12||consensus.size()<xyz.size()*.5)throw std::runtime_error("visual reprojection or stereo depth inconsistent");
  std::vector<cv::Point2f> hull;cv::convexHull(consensus,hull);
  if(std::abs(cv::contourArea(hull))<.02*a.total())throw std::runtime_error("visual features are spatially degenerate");
  M3 nextR=Rwc_*relative.t(),bodyR=nextR*M3(cfg_.Rbc).t();
  V3 nextP=pwc_-nextR*trans,bodyP=nextP-bodyR*V3(cfg_.tbc);
  // Geometric information in world-position / right body-rotation coordinates.
  // This is conditional on the previous stereo points, not a full correlated VO covariance.
  cv::Mat information=cv::Mat::zeros(6,6,CV_64F);
  for(size_t i:accepted){
   V3 world=Rwc_*V3(xyz[i].x,xyz[i].y,xyz[i].z)+pwc_;
   V3 body=bodyR.t()*(world-bodyP),camera=M3(cfg_.Rbc).t()*(body-V3(cfg_.tbc));
   double z=camera[2];cv::Matx<double,2,3> projection(fx_/z,0,-fx_*camera[0]/(z*z),0,fy_/z,-fy_*camera[1]/(z*z));
   auto jp=projection*(-nextR.t()),jr=projection*(M3(cfg_.Rbc).t()*skew(body));
   cv::Mat J(2,6,CV_64F);cv::Mat(jp).copyTo(J(cv::Rect(0,0,3,2)));cv::Mat(jr).copyTo(J(cv::Rect(3,0,3,2)));
   double error=cv::norm(projected[i]-uv[i]),weight=error>1?1/error:1;
   information+=(weight/(.7*.7))*(J.t()*J);
  }
  cv::Mat eigen;cv::eigen(information,eigen);
  if(eigen.at<double>(5)<=0||eigen.at<double>(0)/eigen.at<double>(5)>1e8||
     !cv::invert(information,out.visual.measurement_covariance,cv::DECOMP_CHOLESKY))
   throw std::runtime_error("visual pose information is ill-conditioned");
  out.visual.measurement_covariance=(out.visual.measurement_covariance+out.visual.measurement_covariance.t())*.5;
  pwc_=nextP;Rwc_=nextR;out.visual.R=bodyR;out.visual.p=bodyP;
  out.visual.valid=true;out.visual.inliers=int(consensus.size());
 }
 previous_t_=t;return out;
}
}
