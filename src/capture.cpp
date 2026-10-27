#include "mohism.hpp"
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>
namespace mohism {
static void check(int r,const char* what){if(r<0)throw std::runtime_error(std::string(what)+": "+strerror(errno));}
Capture::Capture(const std::string& path,int width,int height,int fps){
 fd_=open(path.c_str(),O_RDWR|O_NONBLOCK|O_CLOEXEC);check(fd_,"open camera");
 try{
  v4l2_capability cap{};check(ioctl(fd_,VIDIOC_QUERYCAP,&cap),"QUERYCAP");
  unsigned caps=(cap.capabilities&V4L2_CAP_DEVICE_CAPS)?cap.device_caps:cap.capabilities;
  bool multi=caps&V4L2_CAP_VIDEO_CAPTURE_MPLANE;
  type_=multi?V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(!(caps&(V4L2_CAP_VIDEO_CAPTURE|V4L2_CAP_VIDEO_CAPTURE_MPLANE))||!(caps&V4L2_CAP_STREAMING))throw std::runtime_error("not a capture streaming node");
  v4l2_format f{};f.type=type_;
  if(multi){f.fmt.pix_mp.width=width;f.fmt.pix_mp.height=height;f.fmt.pix_mp.pixelformat=V4L2_PIX_FMT_Y10;f.fmt.pix_mp.field=V4L2_FIELD_NONE;f.fmt.pix_mp.num_planes=1;}
  else{f.fmt.pix.width=width;f.fmt.pix.height=height;f.fmt.pix.pixelformat=V4L2_PIX_FMT_Y10;f.fmt.pix.field=V4L2_FIELD_NONE;}
  check(ioctl(fd_,VIDIOC_S_FMT,&f),"S_FMT");
  if(multi){if(f.fmt.pix_mp.num_planes!=1)throw std::runtime_error("RAW10 must have one memory plane");format_=f.fmt.pix_mp.pixelformat;width_=f.fmt.pix_mp.width;height_=f.fmt.pix_mp.height;stride_=f.fmt.pix_mp.plane_fmt[0].bytesperline;}
  else{format_=f.fmt.pix.pixelformat;width_=f.fmt.pix.width;height_=f.fmt.pix.height;stride_=f.fmt.pix.bytesperline;}
  if(int(width_)!=width||int(height_)!=height)throw std::runtime_error("camera size differs from calibration");
  if(format_!=V4L2_PIX_FMT_GREY&&format_!=V4L2_PIX_FMT_Y10&&format_!=V4L2_PIX_FMT_SBGGR10)throw std::runtime_error("requires GREY or mono RAW10 CIF capture");
  if(stride_<width_*(format_==V4L2_PIX_FMT_GREY?1:2))throw std::runtime_error("packed RAW10 unsupported: disable CIF compact mode");
  v4l2_streamparm parm{};parm.type=type_;parm.parm.capture.timeperframe={1,unsigned(fps)};ioctl(fd_,VIDIOC_S_PARM,&parm);
  v4l2_requestbuffers req{};req.count=4;req.type=type_;req.memory=V4L2_MEMORY_MMAP;check(ioctl(fd_,VIDIOC_REQBUFS,&req),"REQBUFS");
  if(req.count<2)throw std::runtime_error("insufficient capture buffers");
  for(unsigned i=0;i<req.count;++i){v4l2_plane plane{};v4l2_buffer buf{};buf.type=type_;buf.memory=V4L2_MEMORY_MMAP;buf.index=i;
   if(multi){buf.m.planes=&plane;buf.length=1;}
   check(ioctl(fd_,VIDIOC_QUERYBUF,&buf),"QUERYBUF");
   size_t length=multi?plane.length:buf.length;off_t offset=multi?plane.m.mem_offset:buf.m.offset;
   void* p=mmap(nullptr,length,PROT_READ|PROT_WRITE,MAP_SHARED,fd_,offset);if(p==MAP_FAILED)throw std::runtime_error("mmap camera");
   memory_.push_back(p);lengths_.push_back(length);check(ioctl(fd_,VIDIOC_QBUF,&buf),"QBUF");
  }
  check(ioctl(fd_,VIDIOC_STREAMON,&type_),"STREAMON");
 }catch(...){for(size_t i=0;i<memory_.size();++i)munmap(memory_[i],lengths_[i]);close(fd_);fd_=-1;throw;}
}
Capture::~Capture(){if(fd_>=0){ioctl(fd_,VIDIOC_STREAMOFF,&type_);for(size_t i=0;i<memory_.size();++i)munmap(memory_[i],lengths_[i]);close(fd_);}}
bool Capture::read(Frame& f,int timeout){
 pollfd p{fd_,POLLIN,0};if(poll(&p,1,timeout)<=0)return false;
 bool multi=type_==V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;v4l2_plane plane{};v4l2_buffer b{};b.type=type_;b.memory=V4L2_MEMORY_MMAP;
 if(multi){b.m.planes=&plane;b.length=1;}
 if(ioctl(fd_,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN||errno==EINTR)return false;check(-1,"DQBUF");}
 if(b.index>=memory_.size())throw std::runtime_error("invalid camera buffer index");
 if((b.flags&V4L2_BUF_FLAG_TIMESTAMP_MASK)!=V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC){ioctl(fd_,VIDIOC_QBUF,&b);throw std::runtime_error("camera timestamps must use CLOCK_MONOTONIC");}
 unsigned bytes=multi?plane.bytesused:b.bytesused,offset=multi?plane.data_offset:0;
 if((b.flags&V4L2_BUF_FLAG_ERROR)||bytes<offset+stride_*height_||offset+stride_*height_>lengths_[b.index]){check(ioctl(fd_,VIDIOC_QBUF,&b),"requeue damaged camera buffer");return false;}
 f.t=b.timestamp.tv_sec+b.timestamp.tv_usec*1e-6;f.sequence=b.sequence;
 auto* data=static_cast<uint8_t*>(memory_[b.index])+offset;
 if(format_==V4L2_PIX_FMT_GREY)f.image=cv::Mat(height_,width_,CV_8UC1,data,stride_).clone();
 else cv::Mat(height_,width_,CV_16UC1,data,stride_).convertTo(f.image,CV_8U,.25);
 check(ioctl(fd_,VIDIOC_QBUF,&b),"QBUF");return true;
}
}
