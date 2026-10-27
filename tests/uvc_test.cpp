#include "mohism.hpp"
#include "uvc_control.hpp"
#include <iostream>
#include <stdexcept>
using namespace mohism;
static void check(bool ok){if(!ok)throw std::runtime_error("UVC transport check failed");}
int main(){
 uvc_streaming_control probe{},commit{};probe.dwMaxVideoFrameSize=htole32(128000);
 for(int vs:{1,3,7}){
  usb_ctrlrequest req{};req.bRequestType=USB_DIR_IN|USB_TYPE_CLASS|USB_RECIP_INTERFACE;req.bRequest=UVC_GET_CUR;req.wIndex=htole16(vs);req.wValue=htole16(UVC_VS_PROBE_CONTROL<<8);req.wLength=htole16(sizeof(probe));int pending=0;
  auto response=uvcSetup(req,vs-1,vs,probe,commit,pending);check(response.length==sizeof(probe));check(!std::memcmp(response.data,&probe,sizeof(probe)));
  check(uvcSetup(req,vs+1,vs+2,probe,commit,pending).length<0);
  req.bRequest=UVC_SET_CUR;check(uvcSetup(req,vs-1,vs,probe,commit,pending).length==sizeof(probe)&&pending==UVC_VS_PROBE_CONTROL);
  req.bRequest=UVC_GET_LEN;req.wLength=htole16(1);check(uvcSetup(req,vs-1,vs,probe,commit,pending).length==1);
  req.wIndex=htole16(0x100|vs);check(uvcSetup(req,vs-1,vs,probe,commit,pending).length<0);
 }
 // Exercise non-contiguous inputs and every uint16 depth value across a frame.
 cv::Mat l(202,322,CV_8UC1),r(202,322,CV_8UC1),d(202,322,CV_16UC1);
 auto left=l(cv::Rect(1,1,320,200)),right=r(cv::Rect(1,1,320,200)),depth=d(cv::Rect(1,1,320,200));
 for(int y=0;y<200;++y)for(int x=0;x<320;++x){left.at<uint8_t>(y,x)=(x+y)%256;right.at<uint8_t>(y,x)=(x-y)&255;depth.at<uint16_t>(y,x)=(y*320+x+1536)&65535;}
 auto stereo=Uvc::pack(Uvc::Kind::Stereo,left,right,{});auto z16=Uvc::pack(Uvc::Kind::Depth,{},{},depth);
 check(stereo.size()==256000&&z16.size()==128000);
 for(int y=0;y<200;++y)for(int x=0;x<320;++x){size_t i=y*1280+x*2,j=y*640+x*2;check(stereo[i]==left.at<uint8_t>(y,x)&&stereo[i+1]==128&&stereo[i+640]==right.at<uint8_t>(y,x)&&stereo[i+641]==128);check((z16[j]|(z16[j+1]<<8))==depth.at<uint16_t>(y,x));}
 Uvc a("",Uvc::Kind::Stereo,60,4,5),b("",Uvc::Kind::Depth,30,6,7);
 a.publish(left,right,{});b.publish({},{},depth);a.events();b.events();check(!a.stats().streaming&&!b.stats().streaming);
 bool rejected=false;try{Uvc::pack(Uvc::Kind::Depth,{},{},left);}catch(const std::exception&){rejected=true;}check(rejected);
 std::cout<<"PASS independent stereo/depth packing and disabled endpoints\n";
}
