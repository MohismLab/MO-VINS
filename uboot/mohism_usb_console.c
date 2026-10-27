// SPDX-License-Identifier: GPL-2.0-or-later
/* MOHISM post-relocation CDC ACM console for the pinned Rockchip U-Boot.
 * No host-dependent wait in the output path; UART remains available.
 */
#include <common.h>
#include <malloc.h>
#include <stdio_dev.h>
#include <usb.h>
#include <g_dnl.h>
#include <linux/usb/cdc.h>
#include <watchdog.h>

#define TX_SIZE 32768
#define RX_SIZE 1024
#define PACKET 512
static struct {
 struct usb_function function;
 struct usb_ep *in, *out, *notify;
 struct usb_request *tx, *rx, *notification;
 struct usb_cdc_line_coding coding;
 u8 output[TX_SIZE], input[RX_SIZE];
 unsigned tx_head, tx_tail, rx_head, rx_tail;
 bool registered, busy, online, data_on, notify_on, dtr, servicing, notification_busy, notify_pending, set_line;
 int control_id, data_id;
} port;
static struct usb_interface_descriptor control = {
 .bLength=sizeof(control), .bDescriptorType=USB_DT_INTERFACE, .bNumEndpoints=1,
 .bInterfaceClass=USB_CLASS_COMM, .bInterfaceSubClass=USB_CDC_SUBCLASS_ACM,
 .bInterfaceProtocol=USB_CDC_ACM_PROTO_AT_V25TER,
};
static struct usb_cdc_header_desc header = {
 .bLength=sizeof(header), .bDescriptorType=USB_DT_CS_INTERFACE,
 .bDescriptorSubType=USB_CDC_HEADER_TYPE, .bcdCDC=__constant_cpu_to_le16(0x0110),
};
static struct usb_cdc_call_mgmt_descriptor call = {
 .bLength=sizeof(call), .bDescriptorType=USB_DT_CS_INTERFACE, .bDescriptorSubType=USB_CDC_CALL_MANAGEMENT_TYPE,
};
static struct usb_cdc_acm_descriptor acm = {
 .bLength=sizeof(acm), .bDescriptorType=USB_DT_CS_INTERFACE,
 .bDescriptorSubType=USB_CDC_ACM_TYPE, .bmCapabilities=USB_CDC_CAP_LINE,
};
static struct usb_cdc_union_desc association = {
 .bLength=sizeof(association), .bDescriptorType=USB_DT_CS_INTERFACE,
 .bDescriptorSubType=USB_CDC_UNION_TYPE,
};
static struct usb_interface_descriptor data = {
 .bLength=sizeof(data), .bDescriptorType=USB_DT_INTERFACE, .bNumEndpoints=2,
 .bInterfaceClass=USB_CLASS_CDC_DATA,
};
static struct usb_endpoint_descriptor fs_notify = {
 .bLength=USB_DT_ENDPOINT_SIZE, .bDescriptorType=USB_DT_ENDPOINT, .bEndpointAddress=USB_DIR_IN,
 .bmAttributes=USB_ENDPOINT_XFER_INT, .wMaxPacketSize=__constant_cpu_to_le16(16), .bInterval=16,
}, hs_notify = {
 .bLength=USB_DT_ENDPOINT_SIZE, .bDescriptorType=USB_DT_ENDPOINT, .bEndpointAddress=USB_DIR_IN,
 .bmAttributes=USB_ENDPOINT_XFER_INT, .wMaxPacketSize=__constant_cpu_to_le16(16), .bInterval=9,
}, fs_in = {
 .bLength=USB_DT_ENDPOINT_SIZE, .bDescriptorType=USB_DT_ENDPOINT, .bEndpointAddress=USB_DIR_IN,
 .bmAttributes=USB_ENDPOINT_XFER_BULK, .wMaxPacketSize=__constant_cpu_to_le16(64),
}, hs_in = {
 .bLength=USB_DT_ENDPOINT_SIZE, .bDescriptorType=USB_DT_ENDPOINT, .bEndpointAddress=USB_DIR_IN,
 .bmAttributes=USB_ENDPOINT_XFER_BULK, .wMaxPacketSize=__constant_cpu_to_le16(512),
}, fs_out = {
 .bLength=USB_DT_ENDPOINT_SIZE, .bDescriptorType=USB_DT_ENDPOINT, .bEndpointAddress=USB_DIR_OUT,
 .bmAttributes=USB_ENDPOINT_XFER_BULK, .wMaxPacketSize=__constant_cpu_to_le16(64),
}, hs_out = {
 .bLength=USB_DT_ENDPOINT_SIZE, .bDescriptorType=USB_DT_ENDPOINT, .bEndpointAddress=USB_DIR_OUT,
 .bmAttributes=USB_ENDPOINT_XFER_BULK, .wMaxPacketSize=__constant_cpu_to_le16(512),
};
#define DESCRIPTORS(n,i,o) (struct usb_descriptor_header *)&control, \
 (struct usb_descriptor_header *)&header, (struct usb_descriptor_header *)&call, \
 (struct usb_descriptor_header *)&acm, (struct usb_descriptor_header *)&association, \
 (struct usb_descriptor_header *)&n, (struct usb_descriptor_header *)&data, \
 (struct usb_descriptor_header *)&i, (struct usb_descriptor_header *)&o, NULL
static struct usb_descriptor_header *fs_desc[] = {DESCRIPTORS(fs_notify,fs_in,fs_out)};
static struct usb_descriptor_header *hs_desc[] = {DESCRIPTORS(hs_notify,hs_in,hs_out)};
static void tx_done(struct usb_ep *ep,struct usb_request *req){port.busy=false;}
static void notify_done(struct usb_ep *ep,struct usb_request *req){port.notification_busy=false;}
static void rx_done(struct usb_ep *ep,struct usb_request *req){
 unsigned i;
 if(req->status||!port.online)return;
 for(i=0;i<req->actual;++i){unsigned next=(port.rx_head+1)%RX_SIZE;if(next==port.rx_tail)break;
  port.input[port.rx_head]=((u8 *)req->buf)[i];port.rx_head=next;
 }
 if(port.online)usb_ep_queue(ep,req,0);
}
static struct usb_request *request(struct usb_ep *ep,unsigned size,void (*done)(struct usb_ep *,struct usb_request *)){
 struct usb_request *req=usb_ep_alloc_request(ep,0);
 if(!req)return NULL;
 req->buf=memalign(ARCH_DMA_MINALIGN,ALIGN(size,ARCH_DMA_MINALIGN));
 if(!req->buf){usb_ep_free_request(ep,req);return NULL;}
 req->complete=done;req->length=size;return req;
}
static void free_request(struct usb_ep *ep,struct usb_request **req){
 if(*req){free((*req)->buf);usb_ep_free_request(ep,*req);*req=NULL;}
}
static void disable(struct usb_function *f){
 port.online=false;port.dtr=false;
 if(port.data_on){usb_ep_disable(port.out);usb_ep_disable(port.in);port.data_on=false;}
 if(port.notify_on){usb_ep_disable(port.notify);port.notify_on=false;}
 port.busy=port.notification_busy=false;port.rx_head=port.rx_tail=0;
}
static int set_alt(struct usb_function *f,unsigned interface,unsigned alt){
 struct usb_gadget *g=f->config->cdev->gadget;int ret;
 if(alt)return -EINVAL;
 if(interface==(unsigned)port.control_id){
  if(port.notify_on)usb_ep_disable(port.notify);
  ret=usb_ep_enable(port.notify,ep_choose(g,&hs_notify,&fs_notify));
  port.notify_on=!ret;return ret;
 }
 if(interface!=(unsigned)port.data_id)return -EINVAL;
 port.online=false;
 if(port.data_on){usb_ep_disable(port.out);usb_ep_disable(port.in);port.data_on=false;}
 ret=usb_ep_enable(port.in,ep_choose(g,&hs_in,&fs_in));if(ret)return ret;
 ret=usb_ep_enable(port.out,ep_choose(g,&hs_out,&fs_out));if(ret){usb_ep_disable(port.in);return ret;}
 port.data_on=true;port.online=true;port.busy=false;port.rx_head=port.rx_tail=0;
 port.rx->length=PACKET;ret=usb_ep_queue(port.out,port.rx,0);
 if(ret)disable(f);
 return ret;
}
static void control_done(struct usb_ep *ep,struct usb_request *req){
 if(port.set_line&&!req->status&&req->actual==sizeof(port.coding))memcpy(&port.coding,req->buf,sizeof(port.coding));
 port.set_line=false;
}
static int setup(struct usb_function *f,const struct usb_ctrlrequest *ctrl){
 struct usb_composite_dev *cdev=f->config->cdev;struct usb_request *req=cdev->req;
 u16 length=le16_to_cpu(ctrl->wLength),value=le16_to_cpu(ctrl->wValue);int answer=-EOPNOTSUPP;
 if(le16_to_cpu(ctrl->wIndex)!=port.control_id)return -EOPNOTSUPP;
 port.set_line=false;req->complete=control_done;
 if(ctrl->bRequestType==(USB_DIR_OUT|USB_TYPE_CLASS|USB_RECIP_INTERFACE)){
  if(ctrl->bRequest==USB_CDC_REQ_SET_LINE_CODING&&length==sizeof(port.coding)){port.set_line=true;answer=length;}
  if(ctrl->bRequest==USB_CDC_REQ_SET_CONTROL_LINE_STATE&&!length){port.dtr=!!(value&1);port.notify_pending=true;answer=0;}
  if(ctrl->bRequest==USB_CDC_REQ_SEND_BREAK&&!length)answer=0;
 }
 if(ctrl->bRequestType==(USB_DIR_IN|USB_TYPE_CLASS|USB_RECIP_INTERFACE)&&ctrl->bRequest==USB_CDC_REQ_GET_LINE_CODING){
  answer=min_t(unsigned,length,sizeof(port.coding));memcpy(req->buf,&port.coding,answer);
 }
 if(answer<0)return answer;
 req->length=answer;req->zero=0;return usb_ep_queue(cdev->gadget->ep0,req,0);
}
static int bind(struct usb_configuration *c,struct usb_function *f){
 struct usb_gadget *g=c->cdev->gadget;
 port.control_id=usb_interface_id(c,f);if(port.control_id<0)return port.control_id;
 port.data_id=usb_interface_id(c,f);if(port.data_id<0)return port.data_id;
 control.bInterfaceNumber=port.control_id;data.bInterfaceNumber=port.data_id;
 association.bMasterInterface0=port.control_id;association.bSlaveInterface0=port.data_id;call.bDataInterface=port.data_id;
 port.notify=usb_ep_autoconfig(g,&fs_notify);port.in=usb_ep_autoconfig(g,&fs_in);port.out=usb_ep_autoconfig(g,&fs_out);
 if(!port.notify||!port.in||!port.out)return -ENODEV;
 hs_notify.bEndpointAddress=fs_notify.bEndpointAddress;hs_in.bEndpointAddress=fs_in.bEndpointAddress;hs_out.bEndpointAddress=fs_out.bEndpointAddress;
 port.tx=request(port.in,PACKET,tx_done);port.rx=request(port.out,PACKET,rx_done);port.notification=request(port.notify,16,notify_done);
 if(!port.tx||!port.rx||!port.notification){
  free_request(port.in,&port.tx);free_request(port.out,&port.rx);free_request(port.notify,&port.notification);return -ENOMEM;
 }
 f->descriptors=fs_desc;f->hs_descriptors=hs_desc;return 0;
}
static void unbind(struct usb_configuration *c,struct usb_function *f){
 disable(f);free_request(port.in,&port.tx);free_request(port.out,&port.rx);free_request(port.notify,&port.notification);
}
static int configure(struct usb_configuration *c){
 port.function.name="MOHISM U-Boot CDC debug";port.function.bind=bind;port.function.unbind=unbind;
 port.function.set_alt=set_alt;port.function.disable=disable;port.function.setup=setup;
 return usb_add_function(c,&port.function);
}
DECLARE_GADGET_BIND_CALLBACK(usb_dnl_mohism_debug,configure);
static void service(void){
 unsigned n=0;
 if(!port.registered||port.servicing)return;
 port.servicing=true;usb_gadget_handle_interrupts(0);
 if(port.notify_on&&port.notify_pending&&!port.notification_busy){
  u8 *b=port.notification->buf;
  b[0]=USB_DIR_IN|USB_TYPE_CLASS|USB_RECIP_INTERFACE;b[1]=USB_CDC_NOTIFY_SERIAL_STATE;b[2]=b[3]=0;
  b[4]=port.control_id;b[5]=0;b[6]=2;b[7]=0;b[8]=port.dtr?3:0;b[9]=0;
  port.notification->length=10;port.notification_busy=true;
  if(usb_ep_queue(port.notify,port.notification,0))port.notification_busy=false;else port.notify_pending=false;
 }
 if(port.online&&port.dtr&&!port.busy&&port.tx_head!=port.tx_tail){
  while(n<PACKET&&port.tx_head!=port.tx_tail){((u8 *)port.tx->buf)[n++]=port.output[port.tx_tail];port.tx_tail=(port.tx_tail+1)%TX_SIZE;}
  port.tx->length=n;port.tx->zero=!(n%port.in->maxpacket);port.busy=true;
  if(usb_ep_queue(port.in,port.tx,0))port.busy=false;
 }
 port.servicing=false;
}
static void push(u8 c){
 unsigned next=(port.tx_head+1)%TX_SIZE;
 if(next==port.tx_tail)port.tx_tail=(port.tx_tail+1)%TX_SIZE;
 port.output[port.tx_head]=c;port.tx_head=next;
}
static void putc_console(struct stdio_dev *dev,const char c){if(c=='\n')push('\r');push(c);service();}
static void puts_console(struct stdio_dev *dev,const char *s){while(*s)putc_console(dev,*s++);}
static int tstc_console(struct stdio_dev *dev){service();return port.rx_head!=port.rx_tail;}
static int getc_console(struct stdio_dev *dev){
 unsigned c;while(!tstc_console(dev)){WATCHDOG_RESET();udelay(100);}
 c=port.input[port.rx_tail];port.rx_tail=(port.rx_tail+1)%RX_SIZE;return c;
}
int mohism_usb_console_init(void){
 struct stdio_dev dev;int ret;
 ret=usb_gadget_initialize(0);if(ret)return ret;
 port.coding.dwDTERate=cpu_to_le32(115200);port.coding.bDataBits=8;
 g_dnl_set_serialnumber("MOHISM-UBOOT-DEBUG");ret=g_dnl_register("usb_dnl_mohism_debug");
 if(ret){usb_gadget_release(0);return ret;}
 port.registered=true;memset(&dev,0,sizeof(dev));strcpy(dev.name,"usbdbg");
 dev.flags=DEV_FLAGS_INPUT|DEV_FLAGS_OUTPUT;dev.putc=putc_console;dev.puts=puts_console;dev.tstc=tstc_console;dev.getc=getc_console;
 ret=stdio_register(&dev);if(ret){g_dnl_unregister();usb_gadget_release(0);port.registered=false;return ret;}
 env_set("stdin","serial,usbdbg");env_set("stdout","serial,usbdbg");env_set("stderr","serial,usbdbg");
 return 0;
}
void mohism_usb_console_stop(void){
 if(!port.registered)return;
 env_set("stdin","serial");env_set("stdout","serial");env_set("stderr","serial");
 port.registered=false;g_dnl_unregister();usb_gadget_release(0);
}
