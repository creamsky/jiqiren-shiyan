#include "basic_control/motion.h"
#include "basic_control/console.h"
#include "basic_control/rate_limit.h"
#include "basic_control/manual_input.h"
#include "basic_control/footprint.h"
#include <ros/ros.h>
#include <ros/master.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>
#include <memory>
#include <chrono>
#include <iomanip>

using Clock=std::chrono::steady_clock;
double now() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
class Experiment {
 public:
  Experiment(): nh("~") {
    lab26::Settings cfg;
    nh.param("distance",cfg.distance,5.0); nh.param("speed",cfg.speed,0.50);
    default_distance=cfg.distance;
    double deg; nh.param("turn_degrees",deg,45.0); cfg.turn=deg*lab26::pi/180;
    nh.param("max_angular_speed",cfg.max_w,0.30);
    nh.param("heading_kp",cfg.heading_gain,0.8);
    nh.param("heading_kd",cfg.heading_d,0.08);
    nh.param("derivative_filter",cfg.derivative_filter,0.15);
    motion.reset(new lab26::Motion(cfg));
    nh.param("dry_run",dry,true); nh.param("run_timeout",run_timeout,180.0);
    nh.param("console",console_mode,false);
    nh.param("linear_acceleration",linear_rate,0.20);
    nh.param("manual_linear_acceleration",manual_linear_rate,0.60);
    if(!std::isfinite(manual_linear_rate)||manual_linear_rate<=0||manual_linear_rate>0.60)throw std::runtime_error("Invalid manual acceleration");
    nh.param("angular_acceleration",angular_rate,0.60);
    nh.param("manual_speed",manual_speed,1.50);
    nh.param("manual_angular_speed",manual_angular_speed,0.50);
    nh.param("rear_obstacle_pause",rear_obstacle_pause,false);
    if(!std::isfinite(manual_speed)||manual_speed<=0||manual_speed>1.5||
       !std::isfinite(manual_angular_speed)||manual_angular_speed<=0||manual_angular_speed>0.50)
      throw std::runtime_error("Invalid manual speed limits");
    if(!std::isfinite(linear_rate)||linear_rate<=0||!std::isfinite(angular_rate)||angular_rate<=0)
      throw std::runtime_error("Invalid acceleration limits");
    if(console_mode) { int port;nh.param("console_port",port,8766);console.reset(new Console(port)); operator_pause=true; }
    std::string log_path; nh.param<std::string>("csv",log_path,"/tmp/lab26_control.csv");
    log.open(log_path); if (!log) throw std::runtime_error("Cannot create CSV log");
    log<<std::fixed<<std::setprecision(6);
    log<<"wall_time,stage,x,y,yaw,progress,cross_track,displacement,command_v,command_w,feedback_v,feedback_w,mode,obstacle_turn,obstacle_drive,avoidance_state,replans,goal_distance,avoidance_reason\n";
    if (!dry) {
      lockfd=open("/tmp/lab26_can.lock",O_CREAT|O_RDWR,0600);
      if (lockfd<0 || flock(lockfd,LOCK_EX|LOCK_NB)<0) throw std::runtime_error("Another lab26 controller owns CAN");
      canfd=socket(PF_CAN,SOCK_RAW|SOCK_NONBLOCK,CAN_RAW);
      if(canfd<0) throw std::runtime_error("Cannot open CAN socket");
      ifreq ifr{}; std::strncpy(ifr.ifr_name,"can0",IFNAMSIZ-1);
      if(ioctl(canfd,SIOCGIFINDEX,&ifr)<0) throw std::runtime_error("Missing can0");
      sockaddr_can addr{}; addr.can_family=AF_CAN; addr.can_ifindex=ifr.ifr_ifindex;
      if(bind(canfd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0) throw std::runtime_error("Cannot bind can0");
    }
    pose_sub=nh.subscribe("/Odometry",1,&Experiment::onPose,this);
    cloud_sub=nh.subscribe("/velodyne_points",1,&Experiment::onCloud,this);
    world_cloud_sub=nh.subscribe("/cloud_registered",1,&Experiment::onWorldCloud,this);
    cmd_pub=nh.advertise<geometry_msgs::Twist>("/lab26/command",1);
    status_pub=nh.advertise<std_msgs::String>("/lab26/status",1,true);
    created=now();
    ROS_INFO("Lab 2.6 ready: dry_run=%s, angle=%.1f deg, distance=%.2f m, v<=%.2f, w<=%.2f",dry?"true":"false",deg,cfg.distance,cfg.speed,cfg.max_w);
  }
  ~Experiment() {
    if(canfd>=0) { for(int i=0;i<20;++i) { send({}); usleep(20000); } close(canfd); }
    if(lockfd>=0) close(lockfd);
  }
  int run() {
    ros::WallRate rate(50);
    while(ros::ok()) {
      ros::spinOnce(); readCan(); const double t=now(); lab26::Command cmd;
      const double dt=last_tick>0?t-last_tick:0; last_tick=t;
      if(console) {
        console->poll([&](const std::string& c) { command(c,t); });
        if(!console->connected || t-last_heartbeat>0.4) { operator_pause=true; manual_cmd={}; }
        if(console->connected) last_console_seen=t;
        else if((last_console_seen>0&&t-last_console_seen>10)||
                (last_console_seen<0&&t-console_started>60)) return 0;
      }
      const bool terminal=motion->stage==lab26::Stage::Done||motion->stage==lab26::Stage::Fault;
      // A sensor launch may own and restart roscore during experiment switching.
      // Re-register only while stopped; never clear a fault or command movement.
      if(console_mode&&operator_pause&&(t-pose_time>1||t-cloud_time>1)&&t-last_rebind>2) {
        last_rebind=t;
        if(ros::master::check()) {
          pose_sub.shutdown();cloud_sub.shutdown();world_cloud_sub.shutdown();
          pose_sub=nh.subscribe("/Odometry",1,&Experiment::onPose,this);
          cloud_sub=nh.subscribe("/velodyne_points",1,&Experiment::onCloud,this);
          world_cloud_sub=nh.subscribe("/cloud_registered",1,&Experiment::onWorldCloud,this);
          stable_count=0;clear_since=-1;
        }
      }
      if(!terminal) {
        if(operator_pause||obstacle_pause||manual) { motion->hold(dt); created+=dt; }
        if(!motion->cfg.avoidance&&t-created>run_timeout) motion->stop("Run timeout");
        if(motion->stage==lab26::Stage::Waiting&&!manual) {
          const bool fresh=pose_time>0 && t-pose_time<0.5 && cloud_time>0 && t-cloud_time<0.5;
          const bool can_ok=dry||(system_time>0 && t-system_time<0.8 && t-motion_time<0.5 && vehicle_state==0 && errors==0 && rc_swb!=1 && rc_swb!=3 && std::abs(feedback_v)<0.02 && std::abs(feedback_w)<0.02);
          // Lab 3 must enter its planner even with an obstacle ahead, rather
          // than requiring the old straight-line corridor to be clear first.
          const bool start_blocked=!motion->cfg.avoidance&&turn_obstacle;
          if(fresh&&!start_blocked) { if(clear_since<0) clear_since=t; } else clear_since=-1;
          if(!operator_pause&&!manual&&fresh&&can_ok&&stable_count>=20&&clear_since>=0&&t-clear_since>=1.0) {
            // CAN mode 1 is required. Never override remote-control/standby mode.
            if(!dry && mode!=1) { ROS_WARN_THROTTLE(2,"Waiting for CAN control mode (feedback mode=%d). No mode-switch command is sent.",mode); }
            else { motion->start(pose,t); ROS_INFO("Origin %.3f %.3f yaw %.3f; target yaw %.3f",pose.x,pose.y,pose.yaw,motion->target_yaw); }
          }
          ROS_INFO_THROTTLE(2,"WAIT: pose=%d cloud=%d stable=%d turn_obstacle=%d can_mode=%d",pose_time>0,cloud_time>0,stable_count,turn_obstacle,mode);
        } else {
          if(t-pose_time>0.5) motion->stop("Odometry timeout");
          if(t-cloud_time>0.5) motion->stop("Lidar timeout");
          if(!dry&&(t-system_time>0.8||t-motion_time>0.5)) motion->stop("CAN feedback timeout");
          if(!dry&&(vehicle_state!=0||errors!=0||mode!=1||rc_swb==1||rc_swb==3)) motion->stop("Chassis fault or control-mode change");
          // Predict without advancing the task: driving may also rotate in place.
          const bool autonomous_avoidance=motion->cfg.avoidance&&!manual;
          if(autonomous_avoidance) {
            obstacle_pause=false;obstacle_clear_since=-1;
            if(!operator_pause) {
              motion->avoidance_points=lab3::reframe(avoid_points,{avoid_cloud_pose.x,avoid_cloud_pose.y,avoid_cloud_pose.yaw},{pose.x,pose.y,pose.yaw});
              motion->avoidance_previous=previous_cmd;
              motion->avoidance_feedback={feedback_v,feedback_w};
              cmd=motion->step(pose,t);
            }
          } else {
          auto prediction=*motion;
          auto proposed=manual?manual_cmd:prediction.step(pose,t);
          const bool turning=(manual&&(std::abs(proposed.w)>0||std::abs(previous_cmd.w)>0))||motion->stage==lab26::Stage::Turning||
            (std::abs(proposed.v)<1e-6&&std::abs(proposed.w)>0)||
            (std::abs(previous_cmd.w)>1e-6&&
             (std::abs(previous_cmd.v)<1e-6||std::abs(proposed.w)<1e-6));
          const bool forward=proposed.v>0||previous_cmd.v>0;
          const bool reverse=rear_obstacle_pause&&(proposed.v<0||previous_cmd.v<0);
          const bool blocked=(turning&&turn_obstacle)||(forward&&drive_obstacle)||(reverse&&back_obstacle);
          const bool release=(!turning||turn_clear)&&(!forward||drive_clear)&&(!reverse||back_clear);
          if(blocked) { obstacle_pause=true; obstacle_clear_since=-1; }
          if(obstacle_pause) {
            if(release) { if(obstacle_clear_since<0) obstacle_clear_since=t; }
            else obstacle_clear_since=-1;
            if(obstacle_clear_since>=0&&t-obstacle_clear_since>=2.0) obstacle_pause=false;
          }
          if(!operator_pause&&!obstacle_pause) {
            if(manual) { if(t-last_manual<0.3) cmd=manual_cmd; }
            else cmd=motion->step(pose,t);
          }
          }
          if(motion->stage==lab26::Stage::Fault) cmd={};
          if(!dry && (std::abs(cmd.v)>0.04||std::abs(cmd.w)>0.04) && std::abs(feedback_v)<0.01 && std::abs(feedback_w)<0.01) {
            if(no_response_since<0) no_response_since=t;
            if(t-no_response_since>2.0) { motion->stop("No chassis motion response"); cmd={}; }
          } else no_response_since=-1;
        }
      }
      // Zero commands are deliberate stops: never smooth a safety or release stop.
      if(manual) {
        previous_cmd=lab26::manualRampOrigin(cmd,previous_cmd);
      }
      if((!motion->cfg.avoidance||manual)&&(cmd.v!=0||cmd.w!=0)&&!operator_pause&&!obstacle_pause&&
         motion->stage!=lab26::Stage::Fault&&motion->stage!=lab26::Stage::Done)
        cmd=lab26::rateLimit(cmd,previous_cmd,dt,manual?manual_linear_rate:linear_rate,angular_rate);
      previous_cmd=cmd;
      geometry_msgs::Twist out; out.linear.x=cmd.v; out.angular.z=cmd.w; cmd_pub.publish(out);
      if(!dry&&!send(cmd)) motion->stop("CAN transmit error");
      if(motion->stage!=last_stage) { ROS_INFO("STATE %s %s",motion->name(),motion->fault.c_str()); last_stage=motion->stage; }
      std_msgs::String status; status.data=std::string(motion->name())+" "+motion->fault; status_pub.publish(status);
      if(++ticks%5==0 || motion->stage==lab26::Stage::Done || motion->stage==lab26::Stage::Fault) {
        log<<ros::WallTime::now().toSec()<<','<<motion->name()<<','<<pose.x<<','<<pose.y<<','<<pose.yaw<<','<<motion->progress<<','<<motion->cross_track<<','<<motion->displacement<<','<<cmd.v<<','<<cmd.w<<','<<feedback_v<<','<<feedback_w<<','<<mode<<','<<turn_obstacle<<','<<drive_obstacle<<','<<(motion->cfg.avoidance?motion->avoidanceState():"")<<','<<motion->avoidanceReplans()<<','<<motion->avoidanceRemaining()<<','<<motion->avoidanceReason()<<std::endl;
        if(console) {
          std::ostringstream s; s<<"{\"stage\":\""<<motion->name()<<"\",\"mode\":\""<<(manual?"manual":"auto")<<"\",\"paused\":"<<(operator_pause?"true":"false")<<",\"obstacle\":"<<(obstacle_pause?"true":"false")<<",\"progress\":"<<motion->progress<<",\"v\":"<<cmd.v<<",\"w\":"<<cmd.w<<",\"chassis_mode\":"<<mode<<",\"feedback_v\":"<<feedback_v<<",\"feedback_w\":"<<feedback_w<<",\"reason\":\""<<motion->fault<<"\"}";
          auto status_json=s.str();status_json.pop_back();
          status_json+=",\"total_distance\":"+std::to_string(motion->cfg.figure_eight?4*lab26::pi*motion->cfg.radius:motion->cfg.distance);
          const bool ready=pose_time>0&&t-pose_time<0.5&&cloud_time>0&&t-cloud_time<0.5;
          const bool feedback_fresh=dry||(system_time>0&&t-system_time<0.8&&t-motion_time<0.5);
          status_json+=std::string(",\"sensors_ready\":")+(ready?"true":"false")+",\"feedback_fresh\":"+(feedback_fresh?"true":"false")+"}";
          status_json.pop_back();const auto blocked=manualBlocked(t);
          status_json+=std::string(",\"manual_ready\":")+(blocked?"false":"true")+",\"manual_block_reason\":\""+(blocked?blocked:"")+"\",\"remote_switch\":"+std::to_string(rc_swb)+",\"chassis_errors\":"+std::to_string(errors)+"}";
          status_json.pop_back();status_json+=",\"manual_vector\":true}";
          status_json.pop_back();
          status_json+=std::string(",\"avoidance_state\":\"")+(motion->cfg.avoidance?motion->avoidanceState():"")+"\",\"front_left_points\":"+std::to_string(avoid_left)+",\"front_right_points\":"+std::to_string(avoid_right)+"}";
          status_json.pop_back();status_json+=",\"avoidance_reason\":\""+motion->avoidanceReason()+"\",\"avoidance_replans\":"+std::to_string(motion->avoidanceReplans())+",\"goal_distance\":"+std::to_string(motion->avoidanceRemaining())+",\"avoidance_version\":5}";
          console->status(status_json);
        }
      }
      if(!console_mode&&(motion->stage==lab26::Stage::Done||motion->stage==lab26::Stage::Fault)) {
        ROS_INFO("RESULT %s: distance %.3f, along %.3f, cross %.3f; %s",motion->name(),motion->displacement,motion->progress,motion->cross_track,motion->fault.c_str());
        if(!console_mode) return motion->stage==lab26::Stage::Done?0:2;
      }
      rate.sleep();
    }
    return 0;
  }
 private:
  ros::NodeHandle nh; ros::Subscriber pose_sub,cloud_sub,world_cloud_sub; ros::Publisher cmd_pub,status_pub;
  std::unique_ptr<lab26::Motion> motion; lab26::Pose pose,stable_ref,previous;
  std::unique_ptr<Console> console;
  bool console_mode=false, operator_pause=false, manual=false, obstacle_pause=false;
  bool back_obstacle=true,turn_clear=false,drive_clear=false,back_clear=false;
  double last_heartbeat=-1,last_manual=-1,last_tick=0,obstacle_clear_since=-1;
  double last_console_seen=-1,console_started=now();
  double last_cloud_preview=-1;
  double last_world_cloud=-1,last_pose_preview=-1;
  lab26::Command manual_cmd;
  lab26::Command previous_cmd;
  double linear_rate=0.20,angular_rate=0.60;
  double default_distance=5.0;
  double manual_linear_rate=0.60;
  double manual_speed=1.50,manual_angular_speed=0.50;
  double pose_stamp=0;
  double last_rebind=0;
  bool rear_obstacle_pause=false;
  std::vector<lab26::ObstaclePoint> avoid_points;
  lab26::Pose avoid_cloud_pose;
  int avoid_left=0,avoid_right=0;
  const char* manualBlocked(double t) const {
    if(pose_time<=0||t-pose_time>=0.5)return "odometry_unavailable";
    if(cloud_time<=0||t-cloud_time>=0.5)return "lidar_unavailable";
    if(!dry){
      if(system_time<=0||t-system_time>=0.8||t-motion_time>=0.5)return "feedback_unavailable";
      if(vehicle_state!=0||errors!=0)return "chassis_fault";
      if(mode!=1)return "chassis_mode";
      if(rc_swb==1||rc_swb==3)return "remote_takeover";
      if((motion->stage==lab26::Stage::Fault||motion->stage==lab26::Stage::Done)&&
         (std::abs(feedback_v)>=0.02||std::abs(feedback_w)>=0.02))return "not_stopped";
    }
    return nullptr;
  }
  void manualReply(bool ok,const char* why="") {
    if(console)console->status(std::string("{\"type\":\"ack\",\"command\":\"manual\",\"ok\":")+(ok?"true":"false")+",\"reason\":\""+why+"\"}");
  }
  void command(const std::string& c,double t) {
    if(c=="heartbeat") { last_heartbeat=t; return; }
    if(c=="stop") { motion->stop("Operator stop");manual_cmd={};operator_pause=true;return; }
    if(c=="zero") { manual_cmd={};return; }
    if(c=="pause") { operator_pause=true;manual_cmd={};return; }
    if(t-last_heartbeat>0.4) { if(c=="manual")manualReply(false,"heartbeat_missing");return; }
    if(c=="start"||c=="start3"||c.rfind("start8 ",0)==0) {
      if(motion->stage!=lab26::Stage::Waiting&&motion->stage!=lab26::Stage::Done&&motion->stage!=lab26::Stage::Fault) return;
      auto cfg=motion->cfg;cfg.figure_eight=c.rfind("start8 ",0)==0;cfg.avoidance=c=="start3";
      cfg.distance=cfg.avoidance?10.0:default_distance;
      if(cfg.figure_eight){std::istringstream input(c);std::string verb,point,extra;double radius;
        if(!(input>>verb>>radius>>point)||(input>>extra)||!std::isfinite(radius)||radius<0.5||radius>10||(point!="cross"&&point!="outer"))return;
        cfg.radius=radius;cfg.outer_start=point=="outer";
      }
      run_timeout=cfg.figure_eight?4*lab26::pi*cfg.radius/std::min(cfg.speed,0.8*cfg.max_w*cfg.radius)*2+90:180;
      motion.reset(new lab26::Motion(cfg));
      manual=false;manual_cmd={};operator_pause=false;obstacle_pause=false;created=t;clear_since=-1;no_response_since=-1;
    } else if(c=="resume") { operator_pause=false; }
    else if(c=="manual") {
      if(const auto blocked=manualBlocked(t)){manualReply(false,blocked);return;}
      if(motion->stage==lab26::Stage::Fault||motion->stage==lab26::Stage::Done) {
        auto cfg=motion->cfg;motion.reset(new lab26::Motion(cfg));created=t;
      }
      manual=true;operator_pause=true;manual_cmd={};
      manualReply(true);
    }
    else if(c=="auto") { manual=false;operator_pause=true;manual_cmd={}; }
    else if(manual) {
      if(!lab26::manualInput(c,manual_speed,manual_angular_speed,manual_cmd))return;
      last_manual=t;operator_pause=false;
    }
  }
  lab26::Stage last_stage=lab26::Stage::Waiting;
  bool dry=true,turn_obstacle=true,drive_obstacle=true; int stable_count=0,ticks=0;
  int canfd=-1,lockfd=-1,mode=-1,vehicle_state=-1,errors=-1,rc_swb=0;
  double created=0,pose_time=0,cloud_time=0,system_time=0,motion_time=0,run_timeout=180;
  double feedback_v=0,feedback_w=0; std::ofstream log;
  double no_response_since=-1;
  double clear_since=-1;
  void onPose(const nav_msgs::Odometry::ConstPtr& msg) {
    const auto &p=msg->pose.pose.position; const auto &q=msg->pose.pose.orientation;
    const double age=(ros::Time::now()-msg->header.stamp).toSec();
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    lab26::Pose next{p.x,p.y,std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))};
    if(age>0.5||age< -0.1||!std::isfinite(norm)||std::abs(norm-1)>0.05||!std::isfinite(next.x)||!std::isfinite(next.y)||!std::isfinite(next.yaw)) {
      if(manual||motion->stage!=lab26::Stage::Waiting) motion->stop("Invalid/stale odometry");
      return;
    }
    const double t=now();
    const double plausible_speed=manual?std::max(0.7,manual_speed+0.2):0.7;
    if(pose_time>0&&(manual||motion->stage!=lab26::Stage::Waiting) && (std::hypot(next.x-previous.x,next.y-previous.y)>0.15+plausible_speed*(t-pose_time)||std::abs(lab26::wrap(next.yaw-previous.yaw))>0.1+0.8*(t-pose_time))) motion->stop("Odometry jump");
    pose=next; previous=next; pose_time=t; pose_stamp=msg->header.stamp.toSec();
    if(console&&console->connected&&t-last_pose_preview>=0.05&&std::isfinite(p.z)) {
      std::ostringstream out;out<<std::setprecision(12)<<"{\"type\":\"pose\",\"stamp\":"<<pose_stamp<<",\"frame\":\""<<safeFrame(msg->header.frame_id)<<"\",\"pose\":["<<pose.x<<','<<pose.y<<','<<pose.yaw<<"],\"position\":["<<p.x<<','<<p.y<<','<<p.z<<"],\"quaternion\":["<<q.x<<','<<q.y<<','<<q.z<<','<<q.w<<"]}";
      console->status(out.str());last_pose_preview=t;
    }
    if(stable_count==0||std::hypot(next.x-stable_ref.x,next.y-stable_ref.y)>0.06||std::abs(lab26::wrap(next.yaw-stable_ref.yaw))>0.04) { stable_ref=next; stable_count=1; }
    else ++stable_count;
  }
  void onCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    const double age=(ros::Time::now()-msg->header.stamp).toSec();
    if(age>0.5||age< -0.1||msg->width*msg->height<100) return;
    int turn_count=0,drive_count=0,back_count=0,turn_release=0,drive_release=0,back_release=0;
    avoid_points.clear();
    const bool preview=console&&console->connected&&now()-last_world_cloud>2.0&&now()-last_cloud_preview>=0.5;
    std::ostringstream cloud_json;
    if(preview) cloud_json<<std::fixed<<std::setprecision(3)<<"{\"type\":\"cloud\",\"registered\":false,\"frame\":\""<<safeFrame(msg->header.frame_id)<<"\",\"stamp\":"<<std::setprecision(6)<<msg->header.stamp.toSec()<<std::setprecision(3)<<",\"points\":[";
    size_t point_index=0,preview_count=0;
    const size_t stride=std::max<size_t>(1,(size_t(msg->width)*msg->height+7999)/8000);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg,"x"),y(*msg,"y"),z(*msg,"z");
      for(;x!=x.end();++x,++y,++z) {
        const bool sampled=(point_index++%stride)==0;
        if(preview&&sampled&&std::isfinite(*x)&&std::isfinite(*y)&&std::isfinite(*z)&&std::abs(*x)<50&&std::abs(*y)<50&&std::abs(*z)<20) {
          if(preview_count++) cloud_json<<',';
          cloud_json<<'['<<*x<<','<<*y<<','<<*z<<']';
        }
        if(!std::isfinite(*x)||!std::isfinite(*y)||!std::isfinite(*z)||std::abs(*z)>0.35) continue;
        const double r=std::hypot(*x,*y);
        // Centered lidar; use the assignment's 650 x 550 mm rectangle.
        const bool outside_body=std::abs(*x)>lab26::body_half_length||std::abs(*y)>lab26::body_half_width;
        if(outside_body) {
          const double range=6.0;
          if(r<range)avoid_points.push_back({*x,*y});
        }
        if(r>0.45&&r<1.0) ++turn_count;
        if(r>0.45&&r<1.2) ++turn_release;
        if(std::abs(*y)<0.5) {
          if(*x>0.45&&*x<1.0) ++drive_count;
          if(*x>0.45&&*x<1.2) ++drive_release;
          if(*x< -0.45&&*x> -1.0) ++back_count;
          if(*x< -0.45&&*x> -1.2) ++back_release;
        }
      }
      turn_obstacle=turn_count>=3; drive_obstacle=drive_count>=3; cloud_time=now();
      avoid_cloud_pose=pose;
      // Explore only when even slow forward motion is blocked. Higher speed
      // feasibility is checked on the actual command in the control loop.
      const auto forward_guard=lab26::sweptFootprint(avoid_points,0.15,0);
      avoid_left=forward_guard.left;avoid_right=forward_guard.right;
      back_obstacle=back_count>=3;turn_clear=turn_release==0;drive_clear=drive_release==0;back_clear=back_release==0;
      if(preview) { cloud_json<<"]}";console->status(cloud_json.str());last_cloud_preview=now(); }
    } catch(const std::runtime_error& e) { ROS_ERROR_THROTTLE(2,"Invalid cloud: %s",e.what()); }
  }
  static double decode(const uint8_t *p) { return static_cast<int16_t>((uint16_t(p[0])<<8)|p[1])/1000.0; }
  static std::string safeFrame(const std::string& frame) {
    std::string result;for(char c:frame)if((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'||c=='/')result+=c;
    return result;
  }
  void onWorldCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    const double age=(ros::Time::now()-msg->header.stamp).toSec();
    if(age>0.5||age< -0.1||msg->width*msg->height==0)return;
    last_world_cloud=now();
    if(!console||!console->connected||now()-last_cloud_preview<0.5)return;
    std::ostringstream out;out<<std::fixed<<std::setprecision(6)<<"{\"type\":\"cloud\",\"registered\":true,\"frame\":\""<<safeFrame(msg->header.frame_id)<<"\",\"stamp\":"<<msg->header.stamp.toSec()<<std::setprecision(3)<<",\"points\":[";
    const size_t stride=std::max<size_t>(1,(size_t(msg->width)*msg->height+7999)/8000);size_t index=0,count=0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg,"x"),y(*msg,"y"),z(*msg,"z");
      for(;x!=x.end();++x,++y,++z){if(index++%stride||!std::isfinite(*x)||!std::isfinite(*y)||!std::isfinite(*z))continue;if(count++)out<<',';out<<'['<<*x<<','<<*y<<','<<*z<<']';}
      out<<"]}";console->status(out.str());last_cloud_preview=now();
    }catch(const std::runtime_error& e){ROS_WARN_THROTTLE(2,"Invalid registered cloud: %s",e.what());}
  }
  void readCan() {
    if(canfd<0) return;
    can_frame f{};
    for(int i=0;i<500;++i) {
      ssize_t n=read(canfd,&f,sizeof(f)); if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)) break;
      if(n!=sizeof(f)) { motion->stop("CAN receive error"); break; }
      if(f.can_id==0x211&&f.can_dlc==8) { vehicle_state=f.data[0]; mode=f.data[1]; errors=(uint16_t(f.data[4])<<8)|f.data[5]; system_time=now(); }
      if(f.can_id==0x221&&f.can_dlc==8) { feedback_v=decode(f.data); feedback_w=decode(f.data+2); motion_time=now(); }
      if(f.can_id==0x241&&f.can_dlc==8) rc_swb=(f.data[0]>>2)&3;
    }
  }
  bool send(lab26::Command c) {
    if(canfd<0) return true;
    can_frame f{}; f.can_id=0x111; f.can_dlc=8;
    auto encode=[](double value,uint8_t* b) { const uint16_t raw=static_cast<uint16_t>(static_cast<int16_t>(std::lround(value*1000))); b[0]=raw>>8; b[1]=raw&255; };
    encode(c.v,f.data); encode(c.w,f.data+2);
    return write(canfd,&f,sizeof(f))==sizeof(f);
  }
};
int main(int argc,char**argv) {
  ros::init(argc,argv,"basicControl");
  try { Experiment experiment; return experiment.run(); }
  catch(const std::exception& e) { ROS_FATAL("%s",e.what()); return 1; }
}
