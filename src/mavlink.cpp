#include "mohism.hpp"
#include "preintegration.hpp"
#include <common/mavlink.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <cstdio>
#include <cerrno>
namespace mohism {
struct Mavlink::Impl {
 int fd=-1;std::string path;bool enabled=true;float skew=2;double heartbeat=-1,last_pose_t=-1;uint8_t last_pose_epoch=0;
 float noise[4]={.1f,.01f,.03f,.02f};
 struct Pending {std::vector<uint8_t> bytes;uint32_t id;double queued;uint8_t epoch;};
 mavlink_status_t status{};mavlink_message_t rx{};std::deque<Pending> tx;size_t offset=0;int listing=-1;uint8_t epoch=0;
 const char* names[8]={"VIO_ENABLE","SYNC_MAX_MS","VIO_VALID","FW_VERSION","ACC_NOISE","GYRO_NOISE","POSE_POS_SD","POSE_ROT_SD"};
 float value(int i,const Pose& p)const{return i>=4?noise[i-4]:i==0?enabled:i==1?skew:i==2?(p.valid&&enabled):400.f;}
 void queue(const mavlink_message_t& m){
  if(m.msgid==MAVLINK_MSG_ID_ODOMETRY){
   for(auto it=tx.begin();it!=tx.end();){
    if(it->id==MAVLINK_MSG_ID_ODOMETRY&&!(it==tx.begin()&&offset))it=tx.erase(it);else ++it;
   }
  }
  if(tx.size()>=64)return;
  uint8_t b[MAVLINK_MAX_PACKET_LEN];unsigned n=mavlink_msg_to_send_buffer(b,&m);tx.push_back({std::vector<uint8_t>(b,b+n),m.msgid,monotonic(),epoch});}
 void discardOldOdometry(const Pose& p){
  for(auto it=tx.begin();it!=tx.end();){
   // Finish a partly written MAVLink packet; unsent stale epochs must never resume after recovery.
   bool partial=it==tx.begin()&&offset!=0;
   if(!partial&&it->id==MAVLINK_MSG_ID_ODOMETRY&&(!p.valid||!enabled||it->epoch!=p.reset_counter||monotonic()-it->queued>.2))it=tx.erase(it);
   else ++it;
  }
 }
 bool target(int sys,int comp)const{return (sys==0||sys==1)&&(comp==0||comp==MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY);}
 void parameter(int i,const Pose& p){mavlink_message_t m;char name[16]{};std::memcpy(name,names[i],std::min(sizeof(name),std::strlen(names[i])));mavlink_msg_param_value_pack(1,MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,&m,name,value(i,p),MAV_PARAM_TYPE_REAL32,8,i);queue(m);}
 bool save(){
  std::string tmp=path+".tmp";FILE* f=fopen(tmp.c_str(),"w");if(!f)return false;
  fprintf(f,"%%YAML:1.0\nVIO_ENABLE: %d\nSYNC_MAX_MS: %.9g\n",enabled?1:0,skew);
  for(int i=0;i<4;++i)fprintf(f,"%s: %.9g\n",names[i+4],noise[i]);
  bool ok=fflush(f)==0&&fsync(fileno(f))==0;if(fclose(f)!=0)ok=false;
  if(!ok||rename(tmp.c_str(),path.c_str())!=0){unlink(tmp.c_str());return false;}return true;
 }
 void message(const Pose& p){
  if(rx.msgid==MAVLINK_MSG_ID_PARAM_REQUEST_LIST){mavlink_param_request_list_t q;mavlink_msg_param_request_list_decode(&rx,&q);if(target(q.target_system,q.target_component))listing=0;}
  if(rx.msgid==MAVLINK_MSG_ID_PARAM_REQUEST_READ){mavlink_param_request_read_t q;mavlink_msg_param_request_read_decode(&rx,&q);if(!target(q.target_system,q.target_component))return;
   int i=q.param_index;if(i<0)for(int k=0;k<8;++k)if(std::strncmp(q.param_id,names[k],16)==0)i=k;if(i>=0&&i<8)parameter(i,p);
  }
  if(rx.msgid==MAVLINK_MSG_ID_PARAM_SET){mavlink_param_set_t q;mavlink_msg_param_set_decode(&rx,&q);if(!target(q.target_system,q.target_component))return;
   for(int i=0;i<8;++i)if(std::strncmp(q.param_id,names[i],16)==0){bool old=enabled;float oldskew=skew;float oldnoise=i>=4?noise[i-4]:0;
    if(q.param_type==MAV_PARAM_TYPE_REAL32&&std::isfinite(q.param_value)){
     if(i==0&&(q.param_value==0||q.param_value==1))enabled=q.param_value!=0;
     if(i==1&&q.param_value>=.1&&q.param_value<=20)skew=q.param_value;
     const float low[4]={.001f,.00001f,.001f,.0001f},high[4]={10.f,1.f,10.f,1.f};
     if(i>=4&&q.param_value>=low[i-4]&&q.param_value<=high[i-4])noise[i-4]=q.param_value;
    }
    if((enabled!=old||skew!=oldskew||(i>=4&&noise[i-4]!=oldnoise))&&!save()){enabled=old;skew=oldskew;if(i>=4)noise[i-4]=oldnoise;}
    parameter(i,p);
   }
  }
 }
};
Mavlink::Mavlink(const std::string& device,const std::string& path):impl_(new Impl){
 impl_->path=path;impl_->fd=open(device.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
 if(impl_->fd<0){delete impl_;throw std::runtime_error("cannot open USB ACM serial");}
 termios t{};if(tcgetattr(impl_->fd,&t)==0){cfmakeraw(&t);t.c_cflag|=CLOCAL|CREAD;cfsetispeed(&t,B115200);cfsetospeed(&t,B115200);tcsetattr(impl_->fd,TCSANOW,&t);}
 if(access(path.c_str(),F_OK)==0){cv::FileStorage fs(path,cv::FileStorage::READ);if(fs.isOpened()){int en=1;float skew=2;fs["VIO_ENABLE"]>>en;fs["SYNC_MAX_MS"]>>skew;impl_->enabled=en!=0;if(std::isfinite(skew)&&skew>=.1&&skew<=20)impl_->skew=skew;
  const float low[4]={.001f,.00001f,.001f,.0001f},high[4]={10.f,1.f,10.f,1.f};
  for(int i=0;i<4;++i)if(!fs[impl_->names[i+4]].empty()){float v;fs[impl_->names[i+4]]>>v;if(std::isfinite(v)&&v>=low[i]&&v<=high[i])impl_->noise[i]=v;}
 }}
}
Mavlink::~Mavlink(){close(impl_->fd);delete impl_;}
bool Mavlink::enabled()const{return impl_->enabled;}
double Mavlink::maxSkew()const{return impl_->skew*.001;}
std::array<double,4> Mavlink::noise()const{return {impl_->noise[0],impl_->noise[1],impl_->noise[2],impl_->noise[3]};}
void Mavlink::poll(const Pose& input){
 Pose p=input;double age=monotonic()-p.t;p.valid=p.valid&&std::isfinite(age)&&age>=0&&age<.2&&finite(p.p)&&finite(p.v)&&rotation(p.R);
 auto& s=*impl_;uint8_t bytes[512];ssize_t n;
 for(int batch=0;batch<4;++batch){
  n=read(s.fd,bytes,sizeof bytes);if(n<=0)break;
  for(int i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,bytes[i],&s.rx,&s.status))s.message(p);
 }
 p.valid=p.valid&&s.enabled;s.epoch=p.reset_counter;s.discardOldOdometry(p);
 double now=monotonic();mavlink_message_t m;
 if(now-s.heartbeat>=1){mavlink_msg_heartbeat_pack(1,MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,&m,MAV_TYPE_ONBOARD_CONTROLLER,MAV_AUTOPILOT_INVALID,0,0,p.valid?MAV_STATE_ACTIVE:MAV_STATE_STANDBY);s.queue(m);s.heartbeat=now;}
 if(s.listing>=0&&s.tx.size()<8){s.parameter(s.listing++,p);if(s.listing==8)s.listing=-1;}
 if(p.valid&&now-p.t<.2&&s.enabled&&(p.reset_counter!=s.last_pose_epoch||p.t>s.last_pose_t)){
  // OpenCV Rodrigues -> quaternion. Largest diagonal branch handles rotations near pi.
  auto& R=p.R;double qw,qx,qy,qz,tr=R(0,0)+R(1,1)+R(2,2);
  if(tr>0){double a=std::sqrt(tr+1)*2;qw=.25*a;qx=(R(2,1)-R(1,2))/a;qy=(R(0,2)-R(2,0))/a;qz=(R(1,0)-R(0,1))/a;}
  else{int i=R(1,1)>R(0,0)?1:0;if(R(2,2)>R(i,i))i=2;int j=(i+1)%3,k=(i+2)%3;double a=std::sqrt(1+R(i,i)-R(j,j)-R(k,k))*2;double v[3]{};v[i]=.25*a;v[j]=(R(j,i)+R(i,j))/a;v[k]=(R(k,i)+R(i,k))/a;qw=(R(k,j)-R(j,k))/a;qx=v[0];qy=v[1];qz=v[2];}
  float q[4]={float(qw),float(qx),float(qy),float(qz)},cov[21]{};cov[0]=NAN;
  V3 vb=p.R.t()*p.v;
  mavlink_msg_odometry_pack(1,MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,&m,uint64_t(p.t*1e6),MAV_FRAME_LOCAL_FRD,MAV_FRAME_BODY_FRD,p.p[0],p.p[1],p.p[2],q,vb[0],vb[1],vb[2],NAN,NAN,NAN,cov,cov,p.reset_counter,MAV_ESTIMATOR_TYPE_VIO,-1);
  s.queue(m);s.last_pose_t=p.t;s.last_pose_epoch=p.reset_counter;
 }
 while(!s.tx.empty()){auto& b=s.tx.front().bytes;ssize_t written=write(s.fd,b.data()+s.offset,b.size()-s.offset);if(written<=0){if(written<0&&errno!=EAGAIN&&errno!=EINTR){s.tx.clear();s.offset=0;}break;}s.offset+=written;if(s.offset==b.size()){s.tx.pop_front();s.offset=0;}}
}
}
