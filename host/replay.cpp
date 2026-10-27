// SPDX-License-Identifier: GPL-3.0-only
// Stream EuRoC-format stereo/IMU CSVs through the same estimator as the device.
#include "mohism.hpp"
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <filesystem>
#include <cmath>
using namespace mohism;
namespace fs=std::filesystem;
struct Row {double t=0;std::vector<std::string> fields;};
static bool read(std::istream& input,Row& row){
 std::string line;
 while(std::getline(input,line)){
  if(line.empty()||line[0]=='#')continue;
  std::istringstream tokens(line);std::string cell;row.fields.clear();
  while(std::getline(tokens,cell,',')){if(!cell.empty()&&cell.back()=='\r')cell.pop_back();row.fields.push_back(cell);}
  if(row.fields.empty())continue;
  size_t end=0;unsigned long long ns=std::stoull(row.fields[0],&end);
  if(end!=row.fields[0].size())throw std::runtime_error("CSV timestamp must be integer nanoseconds");
  row.t=double(ns)*1e-9;return true;
 }
 return false;
}
static double number(const std::string& cell){
 size_t end=0;double x=std::stod(cell,&end);
 if(end!=cell.size()||!std::isfinite(x))throw std::runtime_error("invalid IMU CSV number");return x;
}
static ImuSample sample(const Row& row,const Config& cfg){
 if(row.fields.size()!=7)throw std::runtime_error("IMU CSV requires timestamp,wx,wy,wz,ax,ay,az");
 V3 gyro,accel;for(int i=0;i<3;++i){gyro[i]=number(row.fields[i+1]);accel[i]=number(row.fields[i+4]);}
 return {row.t+cfg.imu_offset,M3(cfg.Rbi)*accel,M3(cfg.Rbi)*gyro};
}
static std::string csv(const std::string& value){std::string out="\"";for(char c:value){out+=c;if(c=='\"')out+=c;}return out+'\"';}
int main(int argc,char** argv){
 if(argc!=4){std::cerr<<"usage: mohism-replay calibration.yml path/to/mav0 trajectory.csv\n";return 2;}
 try{
  Config cfg=Config::load(argv[1]);cfg.validate();cv::setNumThreads(1);Estimator estimator(cfg);fs::path root=argv[2];
  std::ifstream left(root/"cam0/data.csv"),right(root/"cam1/data.csv"),imu(root/"imu0/data.csv");std::ofstream out(argv[3]);
  if(!left||!right||!imu||!out)throw std::runtime_error("cannot open input CSVs or output trajectory");
  out<<"timestamp_s,valid,reset_counter,x,y,z,vx,vy,vz,rx,ry,rz,inliers,processing_ms,status,joint_solves,marginalizations,loop_closures\n"<<std::setprecision(16);
  Row l,r,s;bool hl=read(left,l),hr=read(right,r),hi=read(imu,s);unsigned frames=0,valid=0,skipped=0;double total_ms=0,last_imu=-1,last_l=-1,last_r=-1;
  while(hl&&hr){
   if(l.fields.size()!=2||r.fields.size()!=2)throw std::runtime_error("camera CSV requires timestamp,filename");
   if(std::abs(l.t-r.t)>cfg.max_skew){++skipped;if(l.t<r.t)hl=read(left,l);else hr=read(right,r);continue;}
   if(l.t<=last_l||r.t<=last_r)throw std::runtime_error("non-monotonic camera CSV");last_l=l.t;last_r=r.t;
   double t=(l.t+r.t)*.5;
   // Include the first sample after the camera for exact endpoint interpolation.
   while(hi){
    ImuSample measurement=sample(s,cfg);
    if(measurement.t<=last_imu)throw std::runtime_error("non-monotonic IMU CSV");
    estimator.addImu(measurement);last_imu=measurement.t;hi=read(imu,s);
    if(measurement.t>=t)break;
   }
   Frame a{l.t,cv::imread((root/"cam0/data"/l.fields[1]).string(),cv::IMREAD_GRAYSCALE),frames};
   Frame b{r.t,cv::imread((root/"cam1/data"/r.fields[1]).string(),cv::IMREAD_GRAYSCALE),frames};
   if(a.image.empty()||b.image.empty())throw std::runtime_error("cannot load stereo image");
   auto start=std::chrono::steady_clock::now();estimator.process(a,b);auto p=estimator.pose();
   double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();total_ms+=ms;
   V3 rotation=logSO3(p.R);out<<t<<','<<p.valid<<','<<unsigned(p.reset_counter);
   for(auto vector:{p.p,p.v,rotation})for(int j=0;j<3;++j)out<<','<<(p.valid?vector[j]:NAN);
   out<<','<<p.inliers<<','<<ms<<','<<csv(estimator.status());auto diagnostics=estimator.diagnostics();out<<','<<diagnostics.solves<<','<<diagnostics.marginalizations<<','<<diagnostics.loops<<'\n';++frames;if(p.valid)++valid;
   hl=read(left,l);hr=read(right,r);
  }
  if(!frames)throw std::runtime_error("no synchronized stereo frames");
  if(!out)throw std::runtime_error("trajectory write failed");
  std::cout<<"frames="<<frames<<" valid="<<valid<<" unpaired="<<skipped<<" mean_processing_ms="<<total_ms/frames<<'\n';
  // A run producing no usable estimates must not look like a passing benchmark.
  return valid?0:3;
 }catch(const std::exception& e){std::cerr<<"replay: "<<e.what()<<'\n';return 1;}
}
