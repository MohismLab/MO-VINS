// SPDX-License-Identifier: GPL-3.0-only
#include "mohism.hpp"
#include <common/mavlink.h>
#include <pty.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <iostream>
#include <limits>
using namespace mohism;
static void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
int main(){int master=-1,slave=-1;char path[]="/tmp/mohism-odom-XXXXXX";try{
 char name[128];require(openpty(&master,&slave,name,nullptr,nullptr)==0,"openpty failed");
 fcntl(master,F_SETFL,O_NONBLOCK);int temp=mkstemp(path);require(temp>=0,"temp file failed");close(temp);unlink(path);
 Mavlink link(name,path);mavlink_message_t rx{};mavlink_status_t parser{};
 auto receive=[&]{std::vector<mavlink_odometry_t> out;uint8_t buffer[4096];ssize_t n;
  while((n=read(master,buffer,sizeof buffer))>0)for(int i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_1,buffer[i],&rx,&parser)&&rx.msgid==MAVLINK_MSG_ID_ODOMETRY){mavlink_odometry_t value;mavlink_msg_odometry_decode(&rx,&value);out.push_back(value);}return out;
 };
 auto poll=[&](Pose p){std::this_thread::sleep_for(std::chrono::milliseconds(60));if(p.t==0)p.t=monotonic();link.poll(p);return receive();};
 auto command=[&](float enabled){mavlink_message_t message;char id[16]="VIO_ENABLE";
  mavlink_msg_param_set_pack(255,190,&message,1,MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,id,enabled,MAV_PARAM_TYPE_REAL32);
  uint8_t bytes[MAVLINK_MAX_PACKET_LEN];size_t n=mavlink_msg_to_send_buffer(bytes,&message);require(write(master,bytes,n)==ssize_t(n),"PTY command write failed");
 };
 Pose pose;pose.valid=true;pose.reset_counter=7;pose.p={1,2,3};auto first=poll(pose);
 require(first.size()==1&&first[0].reset_counter==7&&first[0].x==1,"ODOMETRY reset epoch or position lost on wire");
 require(first[0].frame_id==MAV_FRAME_LOCAL_FRD&&first[0].child_frame_id==MAV_FRAME_BODY_FRD,"wrong odometry frames");
 command(0);require(poll(pose).empty()&&!link.enabled(),"disabled estimator still emits odometry");
 command(1);pose.reset_counter=8;auto resumed=poll(pose);require(link.enabled()&&resumed.size()==1&&resumed[0].reset_counter==8,"new epoch not transmitted after enable");
 pose.t=monotonic()-1;require(poll(pose).empty(),"stale pose transmitted");
 pose.t=monotonic()+10;require(poll(pose).empty(),"future pose transmitted");
 pose.t=0;pose.p[0]=std::numeric_limits<double>::quiet_NaN();require(poll(pose).empty(),"NaN position transmitted");
 pose.p={};pose.R=M3::zeros();require(poll(pose).empty(),"invalid rotation transmitted");
 close(master);close(slave);unlink(path);std::cout<<"PASS ODOMETRY epochs/frames, disable, stale/future/NaN/rotation guards over PTY\n";return 0;
 }catch(const std::exception& e){if(master>=0)close(master);if(slave>=0)close(slave);unlink(path);std::cerr<<e.what()<<'\n';return 1;}}
