// SPDX-License-Identifier: GPL-3.0-only
#include "loop_closure.hpp"
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <unistd.h>
using namespace mohism;
static void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){try{
 cv::setNumThreads(1);Config cfg;cfg.loop_max_keyframes=30;CameraGeometry g;g.K=(cv::Mat_<double>(3,3)<<200,0,160,0,200,100,0,0,1);g.Rbc=cv::Mat::eye(3,3,CV_64F);g.tbc=cv::Mat::zeros(3,1,CV_64F);g.baseline=.06;
 cv::Mat left(200,320,CV_8UC1);cv::RNG rng(783);rng.fill(left,cv::RNG::UNIFORM,0,256);cv::GaussianBlur(left,left,{3,3},.5);cv::Mat right=cv::Mat::zeros(left.size(),left.type());left(cv::Rect(8,0,312,200)).copyTo(right(cv::Rect(0,0,312,200)));
 LoopClosure loop(cfg,g);Pose raw,corrected;raw.valid=true;
 for(int i=0;i<35;++i){raw.t=1+i*.25;raw.p={.006*i,0,0};corrected=loop.update(raw,left,right,0);}
 std::cout<<"loop closures="<<loop.closures()<<" raw="<<raw.p<<" corrected="<<corrected.p<<"\n";
 require(loop.keyframes()==30,"loop database is unbounded");require(loop.closures()>0,"geometrically verified revisit did not close loop");require(cv::norm(corrected.p)<cv::norm(raw.p)*.9,"pose graph did not reduce simulated odometry drift");
 auto closures=loop.closures();cv::Mat wrong=cv::Mat::zeros(left.size(),left.type());left(cv::Rect(4,0,316,200)).copyTo(wrong(cv::Rect(0,0,316,200)));
 for(int i=0;i<4;++i){raw.t+=.25;loop.update(raw,left,wrong,0);}require(loop.closures()==closures,"loop accepted inconsistent stereo geometry");
 char path[]="/tmp/mohism-map-XXXXXX";int fd=mkstemp(path);require(fd>=0,"map temporary file failed");close(fd);loop.save(path);
 LoopClosure restored(cfg,g);restored.load(path);require(restored.keyframes()==30,"map roundtrip lost keyframes");{std::ofstream corrupt(path);corrupt<<"%YAML:1.0\nversion: 2\nframes: [invalid";}
 bool malformed=false;try{restored.load(path);}catch(const std::exception&){malformed=true;}require(malformed&&restored.keyframes()==30,"malformed map replaced live database");loop.save(path);raw.p={2,0,0};
 for(int i=0;i<4;++i){raw.t+=.25;corrected=restored.update(raw,left,right,1);}
 require(restored.relocalizations()>0&&cv::norm(corrected.p)<1,"saved-map relocalization failed");
 CameraGeometry mismatch=g;mismatch.baseline=.07;LoopClosure rejected(cfg,mismatch);bool failed=false;try{rejected.load(path);}catch(const std::exception&){failed=true;}require(failed&&rejected.keyframes()==0,"calibration-mismatched map accepted");unlink(path);
 std::cout<<"PASS bounded ORB retrieval, geometric false-loop rejection, four-DoF drift correction, map persistence and cross-session relocalization\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
