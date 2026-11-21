#pragma once
#include "basic_control/motion.h"
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iomanip>

class DwaClient {
 public:
  explicit DwaClient(ros::NodeHandle& nh){
    request_=nh.advertise<std_msgs::String>("/lab3_dwa/request",5);
    odom_=nh.advertise<nav_msgs::Odometry>("/lab26/chassis_odom",1);
    velocity_=nh.subscribe("/lab3_dwa/command",1,&DwaClient::velocity,this);
    health_=nh.subscribe("/lab3_dwa/health",1,&DwaClient::health,this);
    status_=nh.subscribe("/lab3_dwa/state",1,&DwaClient::status,this);
  }
  bool ready()const{return healthy_&&ros::WallTime::now().toSec()-health_time_<.5;}
  void feedback(lab26::Pose p,double stamp,double v,double w){
    if(stamp==last_pose_stamp_)return;
    last_pose_stamp_=stamp;
    nav_msgs::Odometry msg;msg.header.stamp.fromSec(stamp);msg.header.frame_id="camera_init";msg.child_frame_id="body";
    msg.pose.pose.position.x=p.x;msg.pose.pose.position.y=p.y;
    msg.pose.pose.orientation.z=std::sin(p.yaw/2);msg.pose.pose.orientation.w=std::cos(p.yaw/2);
    msg.twist.twist.linear.x=v;msg.twist.twist.angular.z=w;odom_.publish(msg);
  }
  void stop(){if(!token_.empty())send("stop "+token_);token_.clear();proposal_={};command_time_=0;}
  lab26::Command step(lab26::Motion& motion,lab26::Pose pose,double feedback_v){
    motion.external_navigation=true;
    motion.external_remaining=std::hypot(motion.cfg.goal_x-pose.x,motion.cfg.goal_y-pose.y);
    const double dx=pose.x-motion.origin.x,dy=pose.y-motion.origin.y;
    motion.displacement=std::hypot(dx,dy);
    motion.progress=std::max(0.0,motion.cfg.distance-motion.external_remaining);
    motion.cross_track=-dx*std::sin(motion.origin.yaw)+dy*std::cos(motion.origin.yaw);
    if(motion.external_remaining<=.20){arrived_=true;}
    if(arrived_){motion.external_state="settling_goal";stop();return {};}
    if(token_.empty()){
      token_="xy_"+std::to_string(ros::WallTime::now().toNSec());
      std::ostringstream out;out<<std::setprecision(12)<<"goal "<<token_<<' '<<motion.cfg.goal_x<<' '<<motion.cfg.goal_y<<' '<<pose.yaw<<" 0 camera_init";
      send(out.str());previous_pose_=pose;reverse_distance_=forward_distance_=0;reverse_disabled_=false;
      state_="PLANNING";command_time_=0;
    }
    const double along=(pose.x-previous_pose_.x)*std::cos(previous_pose_.yaw)+(pose.y-previous_pose_.y)*std::sin(previous_pose_.yaw);
    previous_pose_=pose;
    if(feedback_v<-.01)reverse_distance_+=std::max(0.0,-along);
    if(feedback_v>.01)forward_distance_+=std::max(0.0,along);
    if(reverse_distance_>=.20&&!reverse_disabled_){send("forward "+token_);reverse_disabled_=true;forward_distance_=0;}
    if(reverse_disabled_&&forward_distance_>=.15){send("reverse "+token_);reverse_disabled_=false;reverse_distance_=0;}
    motion.external_state=state_=="ACTIVE"?"dwa_tracking":"dwa_replanning";
    motion.external_reason=state_=="BLOCKED"?"DWA is replanning around obstacles":"";
    if(stalled()){motion.external_state="waiting_handover";return {};}
    if(state_!="ACTIVE"||!ready()||ros::WallTime::now().toSec()-command_time_>.30)return {};
    auto command=proposal_;
    if(reverse_disabled_&&command.v<0)return {};
    if(command.v<-.01)motion.external_state="recover_reverse";
    else if(std::abs(command.v)<.02&&std::abs(command.w)>.03)motion.external_state="pivot_to_path";
    return command;
  }
  bool arrived()const{return arrived_;}
  bool stalled()const{return !token_.empty()&&state_=="STALLED"&&ready();}
  void reset(){stop();arrived_=false;}
 private:
  void send(const std::string& value){std_msgs::String msg;msg.data=value;request_.publish(msg);}
  void health(const std_msgs::String::ConstPtr& msg){healthy_=msg->data=="ready";health_time_=ros::WallTime::now().toSec();}
  void status(const std_msgs::String::ConstPtr& msg){std::istringstream in(msg->data);std::string token,state;in>>token>>state;if(token==token_)state_=state;}
  void velocity(const geometry_msgs::TwistStamped::ConstPtr& msg){
    if(msg->header.frame_id!=token_||token_.empty()||!std::isfinite(msg->twist.linear.x)||!std::isfinite(msg->twist.angular.z))return;
    const double age=(ros::Time::now()-msg->header.stamp).toSec();
    if(age<-.1||age>.3||msg->twist.linear.x<-.1001||msg->twist.linear.x>.5001||std::abs(msg->twist.angular.z)>.7001)return;
    proposal_={msg->twist.linear.x,msg->twist.angular.z};command_time_=ros::WallTime::now().toSec();
  }
  ros::Publisher request_,odom_;ros::Subscriber velocity_,health_,status_;
  std::string token_,state_;lab26::Command proposal_;lab26::Pose previous_pose_;
  double command_time_=0,health_time_=0,reverse_distance_=0,forward_distance_=0,last_pose_stamp_=-1;
  bool healthy_=false,arrived_=false,reverse_disabled_=false;
};
