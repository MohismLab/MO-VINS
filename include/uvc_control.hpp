#pragma once
#include <linux/usb/g_uvc.h>
#include <linux/usb/video.h>
#include <linux/usb/ch9.h>
#include <endian.h>
#include <algorithm>
#include <cerrno>
#include <cstring>
namespace mohism {
// Interface IDs belong to a function instance, not to a fixed composite layout.
inline uvc_request_data uvcSetup(const usb_ctrlrequest& req,int vc,int vs,
 const uvc_streaming_control& probe,const uvc_streaming_control& commit,int& pending){
 uvc_request_data out{};out.length=-EL2HLT;pending=0;
 if((req.bRequestType&(USB_TYPE_MASK|USB_RECIP_MASK))!=(USB_TYPE_CLASS|USB_RECIP_INTERFACE)||
    (le16toh(req.wIndex)>>8)!=0)return out;
 int interface=le16toh(req.wIndex)&255,selector=le16toh(req.wValue)>>8;
 if(interface==vc&&selector==UVC_VC_REQUEST_ERROR_CODE_CONTROL&&req.bRequest==UVC_GET_CUR){out.length=1;out.data[0]=0;}
 if(interface==vs&&(selector==UVC_VS_PROBE_CONTROL||selector==UVC_VS_COMMIT_CONTROL)){
  if(req.bRequest==UVC_SET_CUR){pending=selector;out.length=std::min<int>(le16toh(req.wLength),sizeof(probe));}
  else if(req.bRequest==UVC_GET_LEN){out.length=2;out.data[0]=sizeof(probe);}
  else if(req.bRequest==UVC_GET_INFO){out.length=1;out.data[0]=3;}
  else if(req.bRequest==UVC_GET_CUR||req.bRequest==UVC_GET_MIN||req.bRequest==UVC_GET_MAX||req.bRequest==UVC_GET_DEF||req.bRequest==UVC_GET_RES){
   out.length=std::min<int>(le16toh(req.wLength),sizeof(probe));std::memcpy(out.data,selector==UVC_VS_COMMIT_CONTROL?&commit:&probe,out.length);
  }
 }
 if(out.length>=0)out.length=std::min<int>(out.length,le16toh(req.wLength));
 return out;
}
}
