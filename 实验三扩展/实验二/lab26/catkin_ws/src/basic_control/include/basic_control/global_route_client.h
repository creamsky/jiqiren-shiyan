#pragma once
#include "basic_control/path_tracker.h"
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <future>

// Only the path service is called. No move_base action or velocity command
// is submitted. A slow global search never blocks the CAN watchdog.
class GlobalRouteClient {
 public:
  explicit GlobalRouteClient(ros::NodeHandle& nh,bool publish_odometry=true){
    service_=nh.serviceClient<nav_msgs::GetPlan>("/lab3_dwa/move_base/NavfnROS/make_plan");
    health_=nh.subscribe("/lab3_dwa/health",1,&GlobalRouteClient::health,this);
    if(publish_odometry)odometry_=nh.advertise<nav_msgs::Odometry>("/lab26/chassis_odom",1);
  }
  void feedback(lab26::Pose p,double stamp,double v,double w){
    if(stamp==last_pose_)return;
    last_pose_=stamp;
    nav_msgs::Odometry msg;msg.header.stamp.fromSec(stamp);msg.header.frame_id="camera_init";msg.child_frame_id="body";
    msg.pose.pose.position.x=p.x;msg.pose.pose.position.y=p.y;
    msg.pose.pose.orientation.z=std::sin(p.yaw/2);msg.pose.pose.orientation.w=std::cos(p.yaw/2);
    msg.twist.twist.linear.x=v;msg.twist.twist.angular.z=w;odometry_.publish(msg);
  }
  bool ready()const{return healthy_ && (ros::WallTime::now()-health_at_).toSec()<.5;}
  void reset(){++generation_;route_.clear();received_=-1.;next_=0.;replans=0;}
  unsigned replans=0;
  const lab26::Waypoints* update(lab26::Pose p,lab26::ObstaclePoint goal,const std::string& frame,double stamp,double now){
    if(pending_.valid() && pending_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
      Result result=pending_.get();
      if(result.generation==generation_){
        route_=std::move(result.path);received_=result.ok?result.requested:-1.;++replans;
      }
    }
    if(!ready())return nullptr;
    if(!pending_.valid() && now>=next_){
      next_=now+.45;
      const auto generation=generation_;auto client=service_;
      pending_=std::async(std::launch::async,[client,generation,p,goal,frame,stamp,now]()mutable{
        Result result{generation,now,false,{}};
        nav_msgs::GetPlan srv;
        auto& start=srv.request.start;start.header.frame_id=frame;start.header.stamp.fromSec(stamp);
        start.pose.position.x=p.x;start.pose.position.y=p.y;
        start.pose.orientation.z=std::sin(p.yaw*.5);start.pose.orientation.w=std::cos(p.yaw*.5);
        srv.request.goal=start;srv.request.goal.pose.position.x=goal.x;srv.request.goal.pose.position.y=goal.y;
        if(!client.call(srv))return result;
        const auto& plan=srv.response.plan;
        if(plan.poses.size()>10000 || (!plan.poses.empty() && plan.header.frame_id!=frame))return result;
        for(const auto& point:plan.poses){
          const auto& xy=point.pose.position;
          if(!std::isfinite(xy.x)||!std::isfinite(xy.y)||(!point.header.frame_id.empty()&&point.header.frame_id!=frame))return result;
          result.path.push_back({xy.x,xy.y});
        }
        if(!result.path.empty() && std::hypot(goal.x-result.path.back().x,goal.y-result.path.back().y)>.21)return result;
        result.ok=true;return result;
      });
    }
    return received_>=0. && now-received_<1.2?&route_:nullptr;
  }
 private:
  struct Result{unsigned generation;double requested;bool ok;lab26::Waypoints path;};
  void health(const std_msgs::String::ConstPtr& status){healthy_=status->data=="ready";health_at_=ros::WallTime::now();}
  ros::ServiceClient service_;ros::Publisher odometry_;ros::Subscriber health_;ros::WallTime health_at_;
  bool healthy_=false;unsigned generation_=0;double received_=-1.,next_=0.,last_pose_=-1.;
  lab26::Waypoints route_;std::future<Result> pending_;
};
