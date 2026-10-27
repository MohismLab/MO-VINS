#include "mohism.hpp"
#include "mohism_imu.h"
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <cerrno>
namespace mohism {
Imu::Imu(const std::string& path){fd_=open(path.c_str(),O_RDONLY|O_NONBLOCK|O_CLOEXEC);if(fd_<0)throw std::runtime_error("cannot open kernel IMU stream: "+path);}
Imu::~Imu(){if(fd_>=0)close(fd_);}
bool Imu::read(ImuSample& s){
 static_assert(sizeof(mohism_imu_sample)==32,"IMU ABI mismatch");
 mohism_imu_sample raw{};ssize_t n=::read(fd_,&raw,sizeof raw);
 if(n<0&&(errno==EAGAIN||errno==EINTR))return false;
 if(n!=sizeof raw)throw std::runtime_error("kernel IMU stream read failed");
 if(raw.flags||(!first_&&raw.sequence!=sequence_+1))throw std::runtime_error("kernel IMU reported dropped samples or SPI error");
 first_=false;sequence_=raw.sequence;s.t=raw.timestamp_ns*1e-9;
 for(int i=0;i<3;++i){s.gyro[i]=raw.gyro[i]*.00875*CV_PI/180.;s.accel[i]=raw.accel[i]*.000061*9.80665;}
 return true;
}
}
