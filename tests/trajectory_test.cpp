// SPDX-License-Identifier: GPL-3.0-only
#include "mohism.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>
#include <algorithm>
using namespace mohism;
static void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
struct Scene {
 std::vector<cv::Mat> textures;
 Scene(){cv::RNG random(20261028);for(int i=0;i<3;++i){cv::Mat texture(1024,1024,CV_8UC1);random.fill(texture,cv::RNG::UNIFORM,0,256);cv::GaussianBlur(texture,texture,{3,3},.6);textures.push_back(texture);}}
 cv::Mat render(const M3& R,const V3& p){
  std::vector<cv::Mat> mx(3),my(3),mask(3);for(int i=0;i<3;++i){mx[i]=cv::Mat(200,320,CV_32F,cv::Scalar(-1));my[i]=mx[i].clone();mask[i]=cv::Mat::zeros(200,320,CV_8U);}
  for(int y=0;y<200;++y)for(int x=0;x<320;++x){
   V3 ray=R*V3{(x-160.)/200,(y-100.)/200,1};
   for(int i=0;i<3;++i){double z=i==0?1.7:i==1?2.2:3.;double t=(z-p[2])/ray[2];V3 world=p+ray*t;
    bool inside=i==0?(world[0]>-.7&&world[0]<-.1&&std::abs(world[1])<.8):i==1?(world[0]>.2&&world[0]<.7&&std::abs(world[1])<.7):true;
    if(t>0&&inside){mx[i].at<float>(y,x)=float((world[0]+4)*128);my[i].at<float>(y,x)=float((world[1]+4)*128);mask[i].at<uchar>(y,x)=255;break;}
   }
  }
  cv::Mat image=cv::Mat::zeros(200,320,CV_8U);for(int i=0;i<3;++i){cv::Mat plane;cv::remap(textures[i],plane,mx[i],my[i],cv::INTER_LINEAR);plane.copyTo(image,mask[i]);}return image;
 }
};
int main(){try{
 cv::setNumThreads(1);Config c;c.calibrated=true;c.capture_width=320;c.capture_height=200;
 c.K1=(cv::Mat_<double>(3,3)<<200,0,160,0,200,100,0,0,1);c.K2=c.K1.clone();c.D1=cv::Mat::zeros(1,5,CV_64F);c.D2=c.D1.clone();
 c.R=cv::Mat::eye(3,3,CV_64F);c.Rbc=c.R.clone();c.Rbi=c.R.clone();c.T=(cv::Mat_<double>(3,1)<<-.06,0,0);c.tbc=cv::Mat::zeros(3,1,CV_64F);
 Scene scene;Vision vision(c);double max_position=0,max_angle=0;
 for(int i=0;i<30;++i){
  double t=i/15.;V3 p{.04*t,.01*std::sin(t),.01*(1-std::cos(t))};M3 R=expSO3({.015*std::sin(t),.02*std::sin(.5*t),.01*t});
  auto left=scene.render(R,p),right=scene.render(R,p+R*V3{.06,0,0});
  left.convertTo(left,CV_8U,1+.05*std::sin(t));right.convertTo(right,CV_8U,1.01+.05*std::sin(t));
  // A temporary foreground obstruction covers 20% of the view, independently of scene depth.
  if(i>=12&&i<=14){left(cv::Rect(0,0,64,200)).setTo(30);right(cv::Rect(0,0,64,200)).setTo(30);}
  auto result=vision.process({1+t,left,unsigned(i)},{1+t,right,unsigned(i)});
  require(result.visual.valid,"synthetic six-DoF tracking lost");
  if(i){require(!result.visual.measurement_covariance.empty(),"visual information covariance absent");}
  max_position=std::max(max_position,cv::norm(result.visual.p-p));max_angle=std::max(max_angle,cv::norm(logSO3(R.t()*result.visual.R)));
 }
 std::cout<<"synthetic 30-frame multi-depth six-DoF max_position_error_m="<<max_position<<" max_angle_error_rad="<<max_angle<<'\n';
 require(max_position<.02&&max_angle<.02,"six-DoF synthetic trajectory error exceeds regression limit");return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
