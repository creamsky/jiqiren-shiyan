#include "basic_control/navigation.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <chrono>
#include <iostream>

// Read-only live perception check: no command publisher, socket or CAN access.
int main(int argc,char**argv){
  ros::init(argc,argv,"lab3_readonly_shadow");ros::NodeHandle nh;
  int samples=0;double maximum=0,total=0;size_t count=0;
  std::string state,reason;
  auto sub=nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points",1,[&](const sensor_msgs::PointCloud2::ConstPtr& cloud){
    if((ros::Time::now()-cloud->header.stamp).toSec()>.5)return;
    std::vector<lab26::ObstaclePoint> points;
    sensor_msgs::PointCloud2ConstIterator<float> x(*cloud,"x"),y(*cloud,"y"),z(*cloud,"z");
    for(;x!=x.end();++x,++y,++z)if(std::isfinite(*x)&&std::isfinite(*y)&&std::isfinite(*z)&&std::abs(*z)<=.35&&
      (std::abs(*x)>lab26::body_half_length||std::abs(*y)>lab26::body_half_width)&&std::hypot(*x,*y)<6)points.push_back({*x,*y});
    lab3::Navigator planner;
    const auto begin=std::chrono::steady_clock::now();planner.step({},points,{},{},0);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    maximum=std::max(maximum,ms);total+=ms;count=points.size();++samples;state=planner.state();reason=planner.reason();
  });
  const auto end=ros::WallTime::now()+ros::WallDuration(8);ros::WallRate rate(100);
  while(ros::ok()&&samples<20&&ros::WallTime::now()<end){ros::spinOnce();rate.sleep();}
  std::cout<<"samples="<<samples<<" obstacle_points="<<count<<" mean_ms="<<(samples?total/samples:0)<<" max_ms="<<maximum<<" state="<<state<<" reason="<<reason<<std::endl;
  return samples>=10&&maximum<20?0:1;
}
