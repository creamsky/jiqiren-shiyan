// Compare clearance decisions on identical live frames without publishing commands.
#include "basic_control/footprint.h"
#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <iostream>

int main(int argc,char** argv){
  ros::init(argc,argv,"observed_clearance_check",ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  unsigned old_ok[2]={},new_ok[2]={},near_hits[2]={};
  ros::Time last;
  for(int i=0;i<20;){
    auto msg=ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/velodyne_points",nh,ros::Duration(4));
    if(!msg)return 1;
    if(msg->header.stamp<=last)continue;
    last=msg->header.stamp;
    std::vector<lab26::ObstaclePoint> obstacles;
    lab26::NearPlaneRays rays;
    sensor_msgs::PointCloud2ConstIterator<float> x(*msg,"x"),y(*msg,"y"),z(*msg,"z");
    for(;x!=x.end();++x,++y,++z){
      if(!std::isfinite(*x)||!std::isfinite(*y)||!std::isfinite(*z))continue;
      if(std::abs(*x)<=lab26::body_half_length && std::abs(*y)<=lab26::body_half_width)continue;
      rays.add(*x,*y,*z);
      if(std::hypot(*x,*y)<6. && std::abs(*z)<=.35)obstacles.push_back({*x,*y});
    }
    for(int d=0;d<2;++d){
      const double angle=d?std::acos(-1.):0.,travel=d?.165:.65;
      old_ok[d]+=lab26::observedCorridor(obstacles,angle,travel);
      new_ok[d]+=lab26::observedCorridor(obstacles,angle,travel,{&rays,0,0,0});
      near_hits[d]+=lab26::bumperClearance(obstacles,angle)<=travel;
    }
    ++i;
  }
  for(int d=0;d<2;++d)std::cout<<(d?"rear":"front")<<" frames=20 endpoints_only="<<old_ok[d]
    <<" with_ray_segments="<<new_ok[d]<<" near_obstacle="<<near_hits[d]<<"\n";
}
