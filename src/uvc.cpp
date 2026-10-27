#include "mohism.hpp"
#include "uvc_control.hpp"
#include <linux/videodev2.h>
#include <linux/usb/g_uvc.h>
#include <linux/usb/video.h>
#include <linux/usb/ch9.h>
#include <cerrno>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <endian.h>
#include <cstring>
#include <algorithm>
#include <stdexcept>
namespace mohism {
struct Uvc::Impl {
 int fd=-1;bool streaming=false;int control=0;
 Kind kind=Kind::Stereo;int width=640,control_interface=0,stream_interface=1;uint32_t pixel_format=V4L2_PIX_FMT_YUYV;
 std::vector<void*> buffers;std::vector<size_t> lengths;
 std::vector<uint64_t> generations;uint64_t generation=0,last_completed=0,completed=0,repeated=0;
 double published_at=0;
 uvc_streaming_control probe{},commit{};
 std::vector<uint8_t> frame;
 void stop(){if(!streaming)return;int type=V4L2_BUF_TYPE_VIDEO_OUTPUT;ioctl(fd,VIDIOC_STREAMOFF,&type);streaming=false;}
 void start(){
  if(streaming)return;
  try {
  v4l2_format fmt{};fmt.type=V4L2_BUF_TYPE_VIDEO_OUTPUT;fmt.fmt.pix.width=width;fmt.fmt.pix.height=200;fmt.fmt.pix.pixelformat=pixel_format;fmt.fmt.pix.field=V4L2_FIELD_NONE;fmt.fmt.pix.sizeimage=frame.size();
  if(ioctl(fd,VIDIOC_S_FMT,&fmt)<0)throw std::runtime_error("UVC output S_FMT failed");
  v4l2_requestbuffers req{};req.count=4;req.type=fmt.type;req.memory=V4L2_MEMORY_MMAP;
  if(ioctl(fd,VIDIOC_REQBUFS,&req)<0)throw std::runtime_error("UVC REQBUFS failed");
  for(unsigned i=0;i<req.count;++i){v4l2_buffer b{};b.index=i;b.type=req.type;b.memory=req.memory;
   if(ioctl(fd,VIDIOC_QUERYBUF,&b)<0)throw std::runtime_error("UVC QUERYBUF failed");
   void* p=mmap(nullptr,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,b.m.offset);
   if(p==MAP_FAILED)throw std::runtime_error("UVC mmap failed");
   buffers.push_back(p);lengths.push_back(b.length);generations.push_back(generation);
   if(b.length<frame.size())throw std::runtime_error("UVC buffer too small");
   memcpy(p,frame.data(),frame.size());b.bytesused=frame.size();if(ioctl(fd,VIDIOC_QBUF,&b)<0)throw std::runtime_error("UVC QBUF failed");
  }
  int type=V4L2_BUF_TYPE_VIDEO_OUTPUT;if(ioctl(fd,VIDIOC_STREAMON,&type)<0)throw std::runtime_error("UVC STREAMON failed");streaming=true;
  }catch(...){release();throw;}
 }
 void release(){stop();for(size_t i=0;i<buffers.size();++i)munmap(buffers[i],lengths[i]);buffers.clear();lengths.clear();generations.clear();v4l2_requestbuffers r{};r.type=V4L2_BUF_TYPE_VIDEO_OUTPUT;r.memory=V4L2_MEMORY_MMAP;ioctl(fd,VIDIOC_REQBUFS,&r);}
};
Uvc::Uvc(const std::string& path,Kind kind,int fps,int control_interface,int streaming_interface):impl_(new Impl){auto& s=*impl_;
 if((fps!=15&&fps!=30&&fps!=60)||control_interface<0||control_interface>255||streaming_interface<0||streaming_interface>255||control_interface==streaming_interface){delete impl_;throw std::invalid_argument("unsupported UVC profile");}
 s.kind=kind;s.width=kind==Kind::Stereo?640:320;s.pixel_format=kind==Kind::Stereo?V4L2_PIX_FMT_YUYV:V4L2_PIX_FMT_Z16;
 s.control_interface=control_interface;s.stream_interface=streaming_interface;s.frame.resize(s.width*200*2,0);
 if(kind==Kind::Stereo)for(size_t i=1;i<s.frame.size();i+=2)s.frame[i]=128;
 if(path.empty())return;
 s.fd=open(path.c_str(),O_RDWR|O_NONBLOCK|O_CLOEXEC);if(s.fd<0){delete impl_;throw std::runtime_error("cannot open UVC output");}
 v4l2_capability cap{};if(ioctl(s.fd,VIDIOC_QUERYCAP,&cap)<0||!((cap.device_caps?cap.device_caps:cap.capabilities)&V4L2_CAP_VIDEO_OUTPUT)){close(s.fd);delete impl_;throw std::runtime_error("configured UVC node is not VIDEO_OUTPUT");}
 for(auto type:{UVC_EVENT_SETUP,UVC_EVENT_DATA,UVC_EVENT_STREAMON,UVC_EVENT_STREAMOFF,UVC_EVENT_DISCONNECT}){v4l2_event_subscription e{};e.type=type;if(ioctl(s.fd,VIDIOC_SUBSCRIBE_EVENT,&e)<0){close(s.fd);delete impl_;throw std::runtime_error("UVC subscribe failed");}}
 s.probe.bmHint=htole16(1);s.probe.bFormatIndex=1;s.probe.bFrameIndex=1;s.probe.dwFrameInterval=htole32(10000000/fps);s.probe.dwMaxVideoFrameSize=htole32(s.frame.size());s.probe.dwMaxPayloadTransferSize=htole32(kind==Kind::Stereo?2048:1024);s.probe.dwClockFrequency=htole32(48000000);s.probe.bmFramingInfo=3;s.probe.bPreferedVersion=1;s.probe.bMaxVersion=1;s.commit=s.probe;
}
Uvc::~Uvc(){if(impl_->fd>=0){impl_->release();close(impl_->fd);}delete impl_;}
void Uvc::events(){auto& s=*impl_;if(s.fd<0)return;
 v4l2_event ev{};
 while(ioctl(s.fd,VIDIOC_DQEVENT,&ev)==0){auto* u=reinterpret_cast<uvc_event*>(ev.u.data);
  if(ev.type==UVC_EVENT_STREAMON){s.start();continue;}
  if(ev.type==UVC_EVENT_STREAMOFF||ev.type==UVC_EVENT_DISCONNECT){s.release();continue;}
  if(ev.type==UVC_EVENT_DATA){
   if(s.control&&u->data.length>=26){uvc_streaming_control candidate{};memcpy(&candidate,u->data.data,std::min(size_t(u->data.length),sizeof(candidate)));
    // Fixed profile: never accept a different buffer geometry or arbitrary payload size.
    if(candidate.bFormatIndex==1&&candidate.bFrameIndex==1){if(s.control==UVC_VS_COMMIT_CONTROL)s.commit=s.probe;}
   }s.control=0;continue;
  }
  if(ev.type!=UVC_EVENT_SETUP)continue;
  auto response=uvcSetup(u->req,s.control_interface,s.stream_interface,s.probe,s.commit,s.control);
  if(ioctl(s.fd,UVCIOC_SEND_RESPONSE,&response)<0)throw std::runtime_error("UVC control response failed");
 }
 if(!s.streaming)return;
 if(s.kind==Kind::Depth&&s.published_at>0&&monotonic()-s.published_at>.2){std::fill(s.frame.begin(),s.frame.end(),0);s.published_at=0;++s.generation;}
 v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_OUTPUT;b.memory=V4L2_MEMORY_MMAP;
 while(ioctl(s.fd,VIDIOC_DQBUF,&b)==0){if(b.index>=s.buffers.size())throw std::runtime_error("invalid UVC buffer");++s.completed;if(s.generations[b.index]==s.last_completed)++s.repeated;s.last_completed=s.generations[b.index];
 s.generations[b.index]=s.generation;memcpy(s.buffers[b.index],s.frame.data(),s.frame.size());b.bytesused=s.frame.size();if(ioctl(s.fd,VIDIOC_QBUF,&b)<0)throw std::runtime_error("UVC requeue failed");}
}
Uvc::Stats Uvc::stats()const{return {impl_->completed,impl_->repeated,impl_->streaming};}
std::vector<uint8_t> Uvc::pack(Kind kind,const cv::Mat& left,const cv::Mat& right,const cv::Mat& depth){
 std::vector<uint8_t> frame((kind==Kind::Stereo?640:320)*200*2);
 if(kind==Kind::Stereo){
  if(left.type()!=CV_8UC1||right.type()!=CV_8UC1||left.size()!=cv::Size(320,200)||right.size()!=left.size())throw std::runtime_error("invalid USB stereo dimensions");
  for(int y=0;y<200;++y)for(int x=0;x<320;++x){auto* row=frame.data()+y*1280;row[x*2]=left.at<uint8_t>(y,x);row[x*2+1]=128;row[640+x*2]=right.at<uint8_t>(y,x);row[641+x*2]=128;}
 }else{
  if(depth.type()!=CV_16UC1||depth.size()!=cv::Size(320,200))throw std::runtime_error("invalid USB depth dimensions");
  for(int y=0;y<200;++y)for(int x=0;x<320;++x){uint16_t d=depth.at<uint16_t>(y,x);frame[y*640+x*2]=d&255;frame[y*640+x*2+1]=d>>8;}
 }
 return frame;
}
void Uvc::publish(const cv::Mat& left,const cv::Mat& right,const cv::Mat& depth){
 auto& s=*impl_;s.frame=pack(s.kind,left,right,depth);s.published_at=monotonic();++s.generation;
}
}
