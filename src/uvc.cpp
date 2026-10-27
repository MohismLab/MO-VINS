#include "mohism.hpp"
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
 std::vector<void*> buffers;std::vector<size_t> lengths;
 uvc_streaming_control probe{},commit{};
 std::vector<uint8_t> frame=std::vector<uint8_t>(960*200*2,0);
 void stop(){if(!streaming)return;int type=V4L2_BUF_TYPE_VIDEO_OUTPUT;ioctl(fd,VIDIOC_STREAMOFF,&type);streaming=false;}
 void start(){
  if(streaming)return;
  v4l2_format fmt{};fmt.type=V4L2_BUF_TYPE_VIDEO_OUTPUT;fmt.fmt.pix.width=960;fmt.fmt.pix.height=200;fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_YUYV;fmt.fmt.pix.field=V4L2_FIELD_NONE;fmt.fmt.pix.sizeimage=frame.size();
  if(ioctl(fd,VIDIOC_S_FMT,&fmt)<0)throw std::runtime_error("UVC output S_FMT failed");
  v4l2_requestbuffers req{};req.count=4;req.type=fmt.type;req.memory=V4L2_MEMORY_MMAP;
  if(ioctl(fd,VIDIOC_REQBUFS,&req)<0)throw std::runtime_error("UVC REQBUFS failed");
  for(unsigned i=0;i<req.count;++i){v4l2_buffer b{};b.index=i;b.type=req.type;b.memory=req.memory;
   if(ioctl(fd,VIDIOC_QUERYBUF,&b)<0)throw std::runtime_error("UVC QUERYBUF failed");
   void* p=mmap(nullptr,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,b.m.offset);
   if(p==MAP_FAILED)throw std::runtime_error("UVC mmap failed");
   buffers.push_back(p);lengths.push_back(b.length);
   if(b.length<frame.size())throw std::runtime_error("UVC buffer too small");
   memcpy(p,frame.data(),frame.size());b.bytesused=frame.size();if(ioctl(fd,VIDIOC_QBUF,&b)<0)throw std::runtime_error("UVC QBUF failed");
  }
  int type=V4L2_BUF_TYPE_VIDEO_OUTPUT;if(ioctl(fd,VIDIOC_STREAMON,&type)<0)throw std::runtime_error("UVC STREAMON failed");streaming=true;
 }
 void release(){stop();for(size_t i=0;i<buffers.size();++i)munmap(buffers[i],lengths[i]);buffers.clear();lengths.clear();v4l2_requestbuffers r{};r.type=V4L2_BUF_TYPE_VIDEO_OUTPUT;r.memory=V4L2_MEMORY_MMAP;ioctl(fd,VIDIOC_REQBUFS,&r);}
};
Uvc::Uvc(const std::string& path):impl_(new Impl){auto& s=*impl_;
 for(int y=0;y<200;++y)for(int x=0;x<640;++x)s.frame[y*1920+x*2+1]=128;
 if(path.empty())return;
 s.fd=open(path.c_str(),O_RDWR|O_NONBLOCK|O_CLOEXEC);if(s.fd<0){delete impl_;throw std::runtime_error("cannot open UVC output");}
 v4l2_capability cap{};if(ioctl(s.fd,VIDIOC_QUERYCAP,&cap)<0||!((cap.device_caps?cap.device_caps:cap.capabilities)&V4L2_CAP_VIDEO_OUTPUT)){close(s.fd);delete impl_;throw std::runtime_error("configured UVC node is not VIDEO_OUTPUT");}
 for(auto type:{UVC_EVENT_SETUP,UVC_EVENT_DATA,UVC_EVENT_STREAMON,UVC_EVENT_STREAMOFF,UVC_EVENT_DISCONNECT}){v4l2_event_subscription e{};e.type=type;if(ioctl(s.fd,VIDIOC_SUBSCRIBE_EVENT,&e)<0){close(s.fd);delete impl_;throw std::runtime_error("UVC subscribe failed");}}
 s.probe.bmHint=htole16(1);s.probe.bFormatIndex=1;s.probe.bFrameIndex=1;s.probe.dwFrameInterval=htole32(666666);s.probe.dwMaxVideoFrameSize=htole32(s.frame.size());s.probe.dwMaxPayloadTransferSize=htole32(3072);s.probe.dwClockFrequency=htole32(48000000);s.probe.bmFramingInfo=3;s.probe.bPreferedVersion=1;s.probe.bMaxVersion=1;s.commit=s.probe;
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
  auto& req=u->req;uvc_request_data response{};response.length=-EL2HLT;
  const int selector=le16toh(req.wValue)>>8;
  if((req.bRequestType&USB_TYPE_MASK)==USB_TYPE_CLASS&&(le16toh(req.wIndex)&255)==0&&selector==UVC_VC_REQUEST_ERROR_CODE_CONTROL&&req.bRequest==UVC_GET_CUR){response.length=1;response.data[0]=0;}
  if((req.bRequestType&USB_TYPE_MASK)==USB_TYPE_CLASS&&(selector==UVC_VS_PROBE_CONTROL||selector==UVC_VS_COMMIT_CONTROL)&&(le16toh(req.wIndex)&255)==1){
   if(req.bRequest==UVC_SET_CUR){s.control=selector;response.length=std::min<int>(le16toh(req.wLength),sizeof(s.probe));}
   else if(req.bRequest==UVC_GET_LEN){response.length=2;response.data[0]=sizeof(s.probe);response.data[1]=0;}
   else if(req.bRequest==UVC_GET_INFO){response.length=1;response.data[0]=3;}
   else if(req.bRequest==UVC_GET_CUR||req.bRequest==UVC_GET_MIN||req.bRequest==UVC_GET_MAX||req.bRequest==UVC_GET_DEF||req.bRequest==UVC_GET_RES){
    response.length=std::min<int>(le16toh(req.wLength),sizeof(s.probe));memcpy(response.data,selector==UVC_VS_COMMIT_CONTROL?&s.commit:&s.probe,response.length);
   }
  }
  if(ioctl(s.fd,UVCIOC_SEND_RESPONSE,&response)<0)throw std::runtime_error("UVC control response failed");
 }
 if(!s.streaming)return;
 v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_OUTPUT;b.memory=V4L2_MEMORY_MMAP;
 while(ioctl(s.fd,VIDIOC_DQBUF,&b)==0){if(b.index>=s.buffers.size())throw std::runtime_error("invalid UVC buffer");memcpy(s.buffers[b.index],s.frame.data(),s.frame.size());b.bytesused=s.frame.size();if(ioctl(s.fd,VIDIOC_QBUF,&b)<0)throw std::runtime_error("UVC requeue failed");}
}
void Uvc::publish(const cv::Mat& left,const cv::Mat& right,const cv::Mat& depth){auto& s=*impl_;
 if(left.type()!=CV_8UC1||right.type()!=CV_8UC1||depth.type()!=CV_16UC1||left.size()!=cv::Size(320,200)||right.size()!=left.size()||depth.size()!=left.size())throw std::runtime_error("invalid USB tile dimensions");
 for(int y=0;y<200;++y){uint8_t* row=s.frame.data()+y*1920;for(int x=0;x<320;++x){row[x*2]=left.at<uint8_t>(y,x);row[x*2+1]=128;row[640+x*2]=right.at<uint8_t>(y,x);row[641+x*2]=128;uint16_t d=depth.at<uint16_t>(y,x);row[1280+x*2]=d&255;row[1281+x*2]=d>>8;}}
}
}
