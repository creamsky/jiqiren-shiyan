// Run only against an isolated ROS master and a dry-run controller on port 25892.
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <string>

int main(int argc,char** argv){
  if(!getenv("ROS_MASTER_URI")||std::string(getenv("ROS_MASTER_URI"))!="http://127.0.0.1:25891")return 2;
  ros::init(argc,argv,"disconnect_fixture");ros::NodeHandle nh;
  bool dry=false;int port=0;
  if(!nh.getParam("/disconnect_vehicle/dry_run",dry)||!dry||!nh.getParam("/disconnect_vehicle/console_port",port)||port!=25892)return 2;
  auto odom=nh.advertise<nav_msgs::Odometry>("/Odometry",1);
  auto laser=nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points",1);
  double v=0.;auto sub=nh.subscribe<geometry_msgs::Twist>("/lab26/command",1,[&](const geometry_msgs::Twist::ConstPtr& m){v=m->linear.x;});
  auto tick=[&](){
    nav_msgs::Odometry p;p.header.stamp=ros::Time::now();p.header.frame_id="camera_init";p.pose.pose.orientation.w=1.;odom.publish(p);
    sensor_msgs::PointCloud2 c;c.header=p.header;c.header.frame_id="velodyne";
    sensor_msgs::PointCloud2Modifier mod(c);mod.setPointCloud2FieldsByString(1,"xyz");mod.resize(360);
    sensor_msgs::PointCloud2Iterator<float> x(c,"x"),y(c,"y"),z(c,"z");
    for(int i=0;i<360;++i,++x,++y,++z){*x=4*std::cos(i*M_PI/180);*y=4*std::sin(i*M_PI/180);*z=0.;}
    laser.publish(c);ros::spinOnce();ros::WallDuration(.03).sleep();
  };
  auto openPeer=[&](){
    int fd=::socket(AF_INET,SOCK_STREAM,0);sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(port);a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    if(fd<0||::connect(fd,reinterpret_cast<sockaddr*>(&a),sizeof(a))!=0){if(fd>=0)::close(fd);return -1;}return fd;
  };
  int fd=openPeer();if(fd<0)return 3;
  for(int i=0;i<90;i++)tick();
  const std::string id="12345678-1234-1234-1234-123456789abc";
  const std::string start="heartbeat\nonboard "+id+" startroam 0.18\n";
  if(::send(fd,start.data(),start.size(),MSG_NOSIGNAL)!=ssize_t(start.size()))return 4;
  for(int i=0;i<150&&v<=.02;i++)tick();
  if(v<=.02){std::cerr<<"Dry-run task did not move\n";return 5;}
  ::close(fd);
  const double startTime=ros::WallTime::now().toSec();double moving=0.;
  while(ros::WallTime::now().toSec()-startTime<13.){tick();if(v>.02)moving=ros::WallTime::now().toSec()-startTime;}
  if(moving<12.5||(fd=openPeer())<0){std::cerr<<"Task or listener lost during disconnect\n";return 6;}
  std::string feedback;char buf[8192];
  for(int i=0;i<100&&feedback.find(id)==std::string::npos;i++){tick();auto n=recv(fd,buf,sizeof(buf),MSG_DONTWAIT);if(n>0)feedback.append(buf,n);}
  if(feedback.find(id)==std::string::npos)return 7;
  ::send(fd,"stop\n",5,MSG_NOSIGNAL);
  for(int i=0;i<35;i++)tick();
  ::close(fd);if(std::abs(v)>.001)return 8;
  std::cout<<"PASS: 13 s disconnected, positive dry-run motion, same task on reconnect, explicit stop confirmed\n";
}
