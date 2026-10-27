#include "mohism.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <stdexcept>
#include <cmath>
namespace mohism {
Vision::Vision(const Config& c,const Pose& origin):cfg_(c),origin_(origin){
 cv::Mat a=c.K1.clone(),b=c.K2.clone();double sx=double(c.width)/c.capture_width,sy=double(c.height)/c.capture_height;
 a.row(0)*=sx;a.row(1)*=sy;b.row(0)*=sx;b.row(1)*=sy;
 cv::Mat r1,r2,p1,p2,q;cv::Size size(c.width,c.height);
 cv::stereoRectify(a,c.D1,b,c.D2,size,c.R,c.T,r1,r2,p1,p2,q,cv::CALIB_ZERO_DISPARITY,0,size);
 if(std::abs(p2.at<double>(1,3))>1e-6)throw std::runtime_error("only horizontal stereo is supported");
 cv::initUndistortRectifyMap(a,c.D1,r1,p1,size,CV_32FC1,map1x_,map1y_);
 cv::initUndistortRectifyMap(b,c.D2,r2,p2,size,CV_32FC1,map2x_,map2y_);
 K_=p1(cv::Rect(0,0,3,3)).clone();fx_=K_.at<double>(0,0);fy_=K_.at<double>(1,1);cx_=K_.at<double>(0,2);cy_=K_.at<double>(1,2);
 baseline_=-p2.at<double>(0,3)/p2.at<double>(0,0);
 if(baseline_<=0)throw std::runtime_error("stereo camera order must give positive disparity");
 // Convert rectified camera into IMU body frame (Rbc maps original left camera to body).
 cfg_.Rbc=c.Rbc*r1.t();Rwc_=origin.R*M3(cfg_.Rbc);pwc_=origin.p+origin.R*V3(cfg_.tbc);
}
Vision::Result Vision::process(const Frame& l,const Frame& r){
 if(std::abs(l.t-r.t)>cfg_.max_skew)throw std::runtime_error("unsynchronized stereo frames");
 Result out;cv::Mat a,b;cv::resize(l.image,a,{cfg_.width,cfg_.height},0,0,cv::INTER_AREA);cv::resize(r.image,b,a.size(),0,0,cv::INTER_AREA);
 cv::remap(a,out.left,map1x_,map1y_,cv::INTER_LINEAR);cv::remap(b,out.right,map2x_,map2y_,cv::INTER_LINEAR);
 auto matcher=cv::StereoSGBM::create(0,cfg_.disparities,5,8*25,32*25,1,31,12,50,2,cv::StereoSGBM::MODE_SGBM);
 cv::Mat disparity;matcher->compute(out.left,out.right,disparity);out.depth=cv::Mat::zeros(a.size(),CV_16UC1);
 for(int y=0;y<a.rows;++y)for(int x=0;x<a.cols;++x){double d=disparity.at<int16_t>(y,x)/16.;if(d>0){double mm=fx_*baseline_*1000/d;if(mm>=100&&mm<65535)out.depth.at<uint16_t>(y,x)=uint16_t(std::lround(mm));}}
 out.visual.t=(l.t+r.t)*.5;
 if(!previous_.empty()){
  std::vector<cv::Point2f> prev,next;cv::goodFeaturesToTrack(previous_,prev,250,.015,8);
  if(prev.size()>=12){
   std::vector<uchar> ok,back_ok;std::vector<float> errors;std::vector<cv::Point2f> back;
   cv::calcOpticalFlowPyrLK(previous_,out.left,prev,next,ok,errors);
   cv::calcOpticalFlowPyrLK(out.left,previous_,next,back,back_ok,errors);
   std::vector<cv::Point3f> xyz;std::vector<cv::Point2f> uv;
   for(size_t i=0;i<prev.size();++i){int x=cvRound(prev[i].x),y=cvRound(prev[i].y);
    if(!ok[i]||!back_ok[i]||cv::norm(prev[i]-back[i])>1||x<0||y<0||x>=a.cols||y>=a.rows)continue;
    double z=previous_depth_.at<uint16_t>(y,x)*.001;if(z<.1||z>15)continue;
    xyz.emplace_back((prev[i].x-cx_)*z/fx_,(prev[i].y-cy_)*z/fy_,z);uv.push_back(next[i]);
   }
   if(xyz.size()>=12){
    cv::Mat rv,tv,inliers;
    bool solved=cv::solvePnPRansac(xyz,uv,K_,cv::noArray(),rv,tv,false,80,1.5,.99,inliers,cv::SOLVEPNP_EPNP);
    if(solved&&inliers.rows>=12&&inliers.rows>=int(xyz.size()*.5)){
     M3 relative;cv::Rodrigues(rv,relative);V3 trans(tv);
     if(cv::norm(trans)<1&&cv::norm(rv)<.7){
      pwc_-=Rwc_*relative.t()*trans;Rwc_=Rwc_*relative.t();
      out.visual.R=Rwc_*M3(cfg_.Rbc).t();out.visual.p=pwc_-out.visual.R*V3(cfg_.tbc);
      out.visual.valid=true;out.visual.inliers=inliers.rows;
     }
    }
   }
  }
 }else{out.visual.R=origin_.R;out.visual.p=origin_.p;out.visual.valid=true;out.visual.inliers=100;started_=true;}
 // VO chain cannot silently resume across a missing transform. Caller must reinitialize.
 if(started_&&!out.visual.valid){previous_.release();previous_depth_.release();throw std::runtime_error("visual tracking lost; estimator restart required");}
 previous_=out.left.clone();previous_depth_=out.depth.clone();return out;
}
}
