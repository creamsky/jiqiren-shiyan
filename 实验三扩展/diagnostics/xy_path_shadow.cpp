// Sample the production path service, tracking logic and measured lidar.
// This process has no CAN socket, action client, or velocity publisher.
#include "basic_control/global_route_client.h"
#include "basic_control/ground_returns.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <iostream>
#include <map>

int main(int argc,char** argv){
  ros::init(argc,argv,"xy_path_shadow",ros::init_options::AnonymousName);
  if(argc!=3)return 2;
  const double dx=std::stod(argv[1]),dy=std::stod(argv[2]);
  ros::NodeHandle nh;GlobalRouteClient routes(nh,false);
  nav_msgs::OdometryConstPtr odom;
  sensor_msgs::PointCloud2ConstPtr raw;
  auto pose_sub=nh.subscribe<nav_msgs::Odometry>("/Odometry",1,[&](const nav_msgs::OdometryConstPtr& m){odom=m;});
  auto scan_sub=nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points",1,[&](const sensor_msgs::PointCloud2ConstPtr& m){raw=m;});
  lab26::ObstaclePoint goal{};lab26::PathTracker tracker;std::map<std::string,int> states;
  ros::Time last;
  int frames=0,nonempty=0,low=0;
  bool fixed_goal=false;
  double first_v=0.,first_w=0.,max_ms=0.,min_front=10.;
  ros::WallRate rate(50);
  const double timeout=ros::WallTime::now().toSec()+10.;
  while(ros::ok() && frames<30 && ros::WallTime::now().toSec()<timeout){
    ros::spinOnce();rate.sleep();
    if(!raw||!odom||raw->header.stamp==last || (ros::Time::now()-raw->header.stamp).toSec()>.35 ||
       std::abs((raw->header.stamp-odom->header.stamp).toSec())>.22)continue;
    const auto& pos=odom->pose.pose.position;const auto& q=odom->pose.pose.orientation;
    lab26::Pose p{pos.x,pos.y,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};
    if(!fixed_goal){
      goal={p.x+dx*std::cos(p.yaw)-dy*std::sin(p.yaw),p.y+dx*std::sin(p.yaw)+dy*std::cos(p.yaw)};
      fixed_goal=true;
    }
    auto frame=odom->header.frame_id;if(!frame.empty() && frame.front()=='/')frame.erase(0,1);
    const double now=ros::WallTime::now().toSec();
    const auto* path=routes.update(p,goal,frame,odom->header.stamp.toSec(),now);
    if(!path)continue;
    last=raw->header.stamp;
    lab26::GroundReturns floor;lab26::NearPlaneRays rays;std::vector<lab26::ObstaclePoint> scan;
    for(int pass=0;pass<2;++pass){
      sensor_msgs::PointCloud2ConstIterator<float> x(*raw,"x"),y(*raw,"y"),z(*raw,"z");
      for(;x!=x.end();++x,++y,++z){
        if(!std::isfinite(*x)||!std::isfinite(*y)||!std::isfinite(*z))continue;
        if(!pass){floor.observe(*x,*y,*z);continue;}
        if(std::abs(*x)<=lab26::body_half_length && std::abs(*y)<=lab26::body_half_width)continue;
        rays.add(*x,*y,*z);
        if(std::hypot(*x,*y)<6. && floor.obstacle(*x,*y,*z)){scan.push_back({*x,*y});if(*z<-.35)++low;}
      }
      if(!pass)floor.finish();
    }
    const auto begin=ros::WallTime::now();
    const auto cmd=tracker.step(*path,p,goal,scan,{&rays,0,0,0},{},{},now,.02,.2,.6);
    max_ms=std::max(max_ms,(ros::WallTime::now()-begin).toSec()*1000);
    if(!frames){first_v=cmd.v;first_w=cmd.w;}
    ++states[tracker.state];++frames;nonempty+=!path->empty();
    min_front=std::min(min_front,lab26::bumperClearance(scan,0.));
  }
  std::cout<<"frames="<<frames<<" nonempty_global_path="<<nonempty<<" received_plans="<<routes.replans
           <<" min_front="<<min_front<<" low_returns="<<low<<" first_v="<<first_v<<" first_w="<<first_w
           <<" max_tracker_ms="<<max_ms<<" states:";
  for(const auto& entry:states)std::cout<<" "<<entry.first<<"="<<entry.second;
  std::cout<<"\n";return frames==30?0:1;
}
