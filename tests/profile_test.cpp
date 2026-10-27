// SPDX-License-Identifier: GPL-3.0-only
#include "mohism.hpp"
#include "runtime_stats.hpp"
#include <fstream>
#include <iterator>
#include <unistd.h>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>
using namespace mohism;
static void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){try{
 cv::setNumThreads(1);
 Config c;c.calibrated=true;c.fps=120;c.usb_fps=60;c.depth_fps=30;
 c.capture_width=320;c.capture_height=200;
 c.K1=(cv::Mat_<double>(3,3)<<200,0,160,0,200,100,0,0,1);c.K2=c.K1.clone();
 c.D1=cv::Mat::zeros(1,5,CV_64F);c.D2=c.D1.clone();c.R=cv::Mat::eye(3,3,CV_64F);
 c.Rbc=c.R.clone();c.Rbi=c.R.clone();c.T=(cv::Mat_<double>(3,1)<<-.06,0,0);c.tbc=cv::Mat::zeros(3,1,CV_64F);
 c.validate();Config bad=c;bad.usb_fps=120;
 bool rejected=false;try{bad.validateProfile();}catch(...){rejected=true;}require(rejected,"unsupported USB 120fps accepted");
 bad=c;bad.fps=30;bad.usb_fps=60;rejected=false;try{bad.validateProfile();}catch(...){rejected=true;}require(rejected,"USB faster than capture accepted");
 cv::Mat left(200,320,CV_8UC1);cv::RNG rng(371);rng.fill(left,cv::RNG::UNIFORM,0,256);cv::GaussianBlur(left,left,{3,3},.7);
 cv::Mat right=cv::Mat::zeros(left.size(),left.type());left(cv::Rect(8,0,312,200)).copyTo(right(cv::Rect(0,0,312,200)));
 Vision v(c);int dense=0;
 for(unsigned i=0;i<16;++i){double t=1+i/120.;auto result=v.process({t,left,i},{t,right,i});
  require(result.visual.valid,"120Hz timestamps broke sparse VO");
  bool has_depth=cv::countNonZero(result.depth)>0;
  require(has_depth==(i%4==0),"dense depth cadence wrong or stale depth reused");dense+=has_depth;
 }
 require(dense==4,"wrong dense update count");
 char path[]="/tmp/mohism-stats-XXXXXX";int fd=mkstemp(path);require(fd>=0,"statistics temp file failed");close(fd);
 RuntimeStats stats(0);stats.left(100);stats.left(104);stats.left(0);
 stats.right(UINT32_MAX);stats.right(0);
 stats.pair(10,20,true,true);stats.pair(30,40,false,false);
 require(stats.write(path,1,c,60,4,true),"statistics write failed");
 std::ifstream input(path);std::string json((std::istreambuf_iterator<char>(input)),{});unlink(path);
 require(json.find("\"left_skipped\": 3")!=std::string::npos,"statistics lost skipped frames");
 require(json.find("\"camera_sequence_restarts\": 1")!=std::string::npos,"sequence wrap mistaken for restart");
 require(json.find("\"processing_ms_p95\": 30.000")!=std::string::npos,"statistics percentile wrong");
 require(json.find("\"valid_vio_fps\": 1.000")!=std::string::npos,"statistics valid throughput wrong");
 std::cout<<"PASS 120Hz input timestamps, independent 30Hz dense depth, no stale map and profile limits (synthetic software test)\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
