#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include "basic_control/footprint.h"
#include "basic_control/navigation.h"

// Run only against the isolated dry-run master created by validate-release.sh.
int main(int argc,char** argv) {
  ros::init(argc,argv,"lab3_integration_test");ros::NodeHandle nh;
  auto odom=nh.advertise<nav_msgs::Odometry>("/Odometry",1);
  auto lidar=nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points",1);
  double command_v=0,command_w=0,x=0,y=0,yaw=0,v=0,w=0;
  auto sub=nh.subscribe<geometry_msgs::Twist>("/lab26/command",1,[&](const geometry_msgs::Twist::ConstPtr& m){command_v=m->linear.x;command_w=m->angular.z;});
  int fd=socket(AF_INET,SOCK_STREAM,0);sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(18768);a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  if(connect(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a))<0){std::cerr<<"isolated console unavailable\n";return 1;}
  fcntl(fd,F_SETFL,O_NONBLOCK);
  auto send=[&](const std::string& text){const auto data=text+"\n";::send(fd,data.data(),data.size(),MSG_NOSIGNAL);};
  auto fail=[&](const char* text){send("stop");close(fd);std::cerr<<text<<std::endl;return 1;};
  const double begin=ros::WallTime::now().toSec();double previous=begin;int ticks=0;
  bool start=false,pause=false,resume=false,done=false;double paused_x=0,paused_y=0;
  std::string status,buffer;ros::WallRate rate(100);
  while(ros::ok()){
    const double now=ros::WallTime::now().toSec(),t=now-begin,dt=std::min(.05,now-previous);previous=now;
    ros::spinOnce();v+=(command_v-v)*.12;w+=(command_w-w)*.12;
    x+=v*std::cos(yaw)*dt;y+=v*std::sin(yaw)*dt;yaw+=w*dt;
    if(command_v>1.001||command_v<-.201||std::abs(command_w)>.801)return fail("Speed limit violation");
    nav_msgs::Odometry p;p.header.stamp=ros::Time::now();p.header.frame_id="camera_init";p.pose.pose.position.x=x;p.pose.pose.position.y=y;p.pose.pose.orientation.z=std::sin(yaw/2);p.pose.pose.orientation.w=std::cos(yaw/2);odom.publish(p);
    if(ticks%10==0){
      sensor_msgs::PointCloud2 cloud;cloud.header=p.header;cloud.header.frame_id="velodyne";
      sensor_msgs::PointCloud2Modifier mod(cloud);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(256);
      sensor_msgs::PointCloud2Iterator<float> px(cloud,"x"),py(cloud,"y"),pz(cloud,"z");
      for(int i=0;i<256;++i,++px,++py,++pz){double a=2*3.141592653589793*i/256,dx=2+.2*std::cos(a)-x,dy=.2*std::sin(a)-y;
        *px=dx*std::cos(yaw)+dy*std::sin(yaw);*py=-dx*std::sin(yaw)+dy*std::cos(yaw);*pz=.1;
        if(std::abs(*px)<lab26::body_half_length&&std::abs(*py)<lab26::body_half_width)return fail("Simulated chassis collided");
      }lidar.publish(cloud);
    }
    // Deliberately lose the heartbeat near the beginning, then explicitly resume.
    if(ticks%10==0&&!(t>=4&&t<5))send("heartbeat");
    if(t>2&&!start){send("start3");start=true;}
    if(t>=4.7&&!pause){if(std::abs(command_v)>.001||std::abs(command_w)>.001)return fail("Heartbeat loss did not stop output");paused_x=x;paused_y=y;pause=true;}
    if(t>5&&!resume){if(std::hypot(x-paused_x,y-paused_y)>.03)return fail("Robot moved while heartbeat was absent");send("heartbeat");send("resume");resume=true;}
    char data[8192];ssize_t n;while((n=recv(fd,data,sizeof(data),MSG_DONTWAIT))>0)buffer.append(data,n);
    size_t pos;while((pos=buffer.find('\n'))!=std::string::npos){auto line=buffer.substr(0,pos);buffer.erase(0,pos+1);if(line.find("\"stage\"")!=std::string::npos)status=line;}
    if(status.find("\"stage\":\"FAULT\"")!=std::string::npos)return fail(status.c_str());
    if(status.find("\"stage\":\"DONE\"")!=std::string::npos){done=true;break;}
    if(t>90)return fail("Integration run timed out");
    ++ticks;rate.sleep();
  }
  send("stop");close(fd);
  if(!done||std::hypot(x-10,y)>.17||std::abs(std::atan2(std::sin(yaw),std::cos(yaw)))>8.0*std::acos(-1.0)/180.0||std::abs(command_v)>.001||std::abs(command_w)>.001){std::cerr<<"Goal/heading/stop verification failed\n";return 1;}
  std::cout<<"PASS: real ROS controller + TCP start3 + heartbeat loss + resume + bypass + latched goal stop; x="<<x<<" y="<<y<<std::endl;
  return 0;
}
