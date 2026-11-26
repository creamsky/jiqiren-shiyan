#include "basic_control/motion.h"
#include "basic_control/console.h"
#include "basic_control/rate_limit.h"
#include "basic_control/manual_input.h"
#include "basic_control/footprint.h"
#include "basic_control/can_output.h"
#include "basic_control/global_route_client.h"
#include "basic_control/ground_returns.h"
#include "basic_control/low_obstacles.h"
#include "basic_control/clearing_rays.h"
#include "basic_control/maze_guard.h"
#include "basic_control/gap_roam.h"
#include "basic_control/scan_alignment.h"
#include <ros/ros.h>
#include <ros/master.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <set>
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
    nh.param("xy_use_dwa",use_global_route,false);
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
    log<<"wall_time,stage,x,y,yaw,progress,cross_track,displacement,command_v,command_w,feedback_v,feedback_w,mode,obstacle_turn,obstacle_drive,avoidance_state,replans,goal_distance,avoidance_reason,fault_reason,last_can_tx_errno,can_tx_retries,front_gap_m,rear_gap_m\n";
    if (!dry) {
      lockfd=open("/tmp/lab26_can.lock",O_CREAT|O_RDWR,0600);
      if (lockfd<0 || flock(lockfd,LOCK_EX|LOCK_NB)<0) throw std::runtime_error("Another lab26 controller owns CAN");
      if(!bindCan()&&!console_mode)throw std::runtime_error("Cannot bind can0 feedback socket");
    }
    bindRosIO();
    nh.getParam("/run_id",master_id);
    nh.getParam("/lab26/map_generation",map_generation);
    created=now();
    ROS_INFO("Lab 2.6 ready: dry_run=%s, angle=%.1f deg, distance=%.2f m, v<=%.2f, w<=%.2f",dry?"true":"false",deg,cfg.distance,cfg.speed,cfg.max_w);
  }
  ~Experiment() {
    if(canfd>=0) { for(int i=0;i<20&&networkSelected();++i) { send({}); usleep(20000); } close(canfd); }
    if(lockfd>=0) close(lockfd);
  }
  int run() {
    ros::WallRate rate(50);
    while(ros::ok()) {
      ros::spinOnce(); readCan(); const double t=now(); lab26::Command cmd;
      motion->guard_state.clear();motion->guard_reason.clear();
      const double dt=last_tick>0?t-last_tick:0; last_tick=t;
      if(pose_time>0&&t-pose_time<.5&&(dry||(t-system_time<.8&&t-motion_time<.5)))global_route->feedback(pose,pose_stamp,feedback_v,feedback_w);
      if(!dry&&(operator_pause||motion->stage==lab26::Stage::Fault)&&t-last_can_check>1){
        last_can_check=t;
        const unsigned index=if_nametoindex("can0");
        if(index&&index!=can_index&&bindCan()){
          operator_pause=true;previous_cmd={};manual_cmd={};
          ROS_WARN("USB-CAN reconnected; socket rebound, navigation remains stopped");
        }
      }
      if(console) {
        console->poll([&](const std::string& c) { command(c,t); });
        if(!dry&&!controlPermitted(t)){
          if(!operator_pause||manual)motion->stop("Chassis fault or control-mode change");
          operator_pause=true;manual=false;manual_cmd={};previous_cmd={};
        }
        if(onboard_id.empty()&&(!console->connected || t-last_heartbeat>0.4)) { operator_pause=true; manual_cmd={}; }
        if(!onboard_id.empty()&&motion->stage==lab26::Stage::Waiting){
          if(t-pose_time>.5)motion->stop("Odometry timeout");
          else if(t-cloud_time>.5)motion->stop("Lidar timeout");
        }
      }
      if(!maze_leg.empty()&&!operator_pause&&!manual&&motion->stage!=lab26::Stage::Done&&motion->stage!=lab26::Stage::Fault){
        std::string current_generation;
        if(!nh.getParamCached("/lab26/map_generation",current_generation)||current_generation!=map_generation){
          motion->stop("Maze map generation changed");operator_pause=true;
        }
      }
      const bool terminal=motion->stage==lab26::Stage::Done||motion->stage==lab26::Stage::Fault;
      // A sensor launch may own and restart roscore during experiment switching.
      // Re-register only while stopped; never clear a fault or command movement.
      if(console_mode&&operator_pause&&t-last_rebind>1) {
        last_rebind=t;
        std::string current,generation;
        if(ros::master::check() && nh.getParam("/run_id",current)){
          if(current!=master_id){bindRosIO();master_id=current;}
          if(nh.getParam("/lab26/map_generation",generation) && generation!=map_generation){
            map_generation=generation;pose_time=cloud_time=0;stable_count=0;clear_since=-1;
            pending_scans.clear();scan_alignment.clear();low_obstacles.clear();avoid_points.clear();global_route->reset();path_tracker=lab26::PathTracker();
          }
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
          if(!operator_pause&&!manual&&fresh&&can_ok&&(!use_global_route||!motion->cfg.xy_goal||global_route->ready())&&stable_count>=20&&clear_since>=0&&t-clear_since>=1.0) {
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
              if(gap_roaming&&motion->stage!=lab26::Stage::Fault){
                motion->stage=lab26::Stage::Driving;
                low_obstacles.append(pose,motion->avoidance_points,true);
                const double dx=ray_pose.x-pose.x,dy=ray_pose.y-pose.y;
                const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
                const lab26::RayView rays{&near_plane_rays,c*dx+s*dy,-s*dx+c*dy,ray_pose.yaw-pose.yaw};
                cmd=gap_roam.step(pose,motion->avoidance_points,rays,{feedback_v,feedback_w},previous_cmd,t,dt,maze_extension);
                motion->external_navigation=true;motion->external_state=gap_roam.state;
                motion->external_reason.clear();
                if(cmd.v==0.&&cmd.w==0.&&(ticks+1)%5==0){
                  const auto& hits=motion->avoidance_points;
                  motion->external_reason="fused_front_m="+std::to_string(lab26::bumperClearance(hits,0.))+
                      " fused_rear_m="+std::to_string(lab26::bumperClearance(hits,lab26::pi))+
                      " forward_20cm="+std::to_string(lab26::observedCorridor(hits,0.,.20,rays))+
                      " rear_20cm="+std::to_string(lab26::observedCorridor(hits,lab26::pi,.20,rays));
                }
              }else if(use_global_route&&motion->cfg.xy_goal&&motion->stage==lab26::Stage::Driving){
                low_obstacles.append(pose,motion->avoidance_points);
                front_gap=lab26::bumperClearance(motion->avoidance_points,0.);
                rear_gap=lab26::bumperClearance(motion->avoidance_points,lab26::pi);
                const double dx=ray_pose.x-pose.x,dy=ray_pose.y-pose.y;
                const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
                const lab26::RayView rays{&near_plane_rays,c*dx+s*dy,-s*dx+c*dy,ray_pose.yaw-pose.yaw};
                const lab26::ObstaclePoint goal{motion->cfg.goal_x,motion->cfg.goal_y};
                const auto* path=global_route->update(pose,goal,pose_frame,pose_stamp,t);
                motion->external_navigation=true;
                motion->external_remaining=std::hypot(goal.x-pose.x,goal.y-pose.y);
                motion->displacement=std::hypot(pose.x-motion->origin.x,pose.y-motion->origin.y);
                motion->cross_track=-(pose.x-motion->origin.x)*std::sin(motion->origin.yaw)+(pose.y-motion->origin.y)*std::cos(motion->origin.yaw);
                motion->progress=std::max(0.,motion->cfg.distance-motion->external_remaining);
                if(path || global_route->ready() || motion->external_remaining<=.20 || motion->external_state=="settling_goal"){
                  static const lab26::Waypoints empty;
                  cmd=path_tracker.step(path?*path:empty,pose,goal,motion->avoidance_points,rays,
                    previous_cmd,{feedback_v,feedback_w},t,dt,linear_rate,angular_rate);
                  motion->external_state=path_tracker.state;motion->external_reason=path_tracker.reason;
                  if(!maze_leg.empty() && (cmd.v!=0. || cmd.w!=0.)){
                    cmd=lab26::mazeSafeCommand(motion->avoidance_points,{feedback_v,feedback_w},cmd,maze_extension);
                    if(cmd.v==0.&&cmd.w==0.){
                      motion->external_state="maze_blocked";motion->external_reason="Cone base clearance";
                    }
                  }
                  if(path_tracker.finished)motion->stage=lab26::Stage::Done;
                }else{
                  motion->external_state="waiting_route";motion->external_reason="Waiting for current global path";
                }
              }else cmd=motion->step(pose,t);
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
      if(operator_pause||manual||motion->stage==lab26::Stage::Fault||motion->stage==lab26::Stage::Done){
        if(!navigation_idle){global_route->reset();path_tracker=lab26::PathTracker();navigation_idle=true;}
      }
      // Zero commands are deliberate stops: never smooth a safety or release stop.
      if(manual) {
        previous_cmd=lab26::manualRampOrigin(cmd,previous_cmd);
      }
      if((!motion->cfg.avoidance||manual)&&(cmd.v!=0||cmd.w!=0)&&!operator_pause&&!obstacle_pause&&
         motion->stage!=lab26::Stage::Fault&&motion->stage!=lab26::Stage::Done)
        cmd=lab26::rateLimit(cmd,previous_cmd,dt,manual?manual_linear_rate:linear_rate,angular_rate);
      if(!dry&&(!console_mode||networkSelected())){
        const auto sent=send(cmd);can_tx_retries+=sent.attempts>0?sent.attempts-1:0;
        if(!sent.ok()){
          last_can_tx_errno=sent.error;
          motion->stop("CAN transmit error (errno "+std::to_string(sent.error)+")");
          ROS_ERROR_THROTTLE(2,"CAN TX failed: errno=%d (%s)",sent.error,std::strerror(sent.error));
          // Do not report an unsent moving command as the current output.
          // Try zero immediately; subsequent fault ticks also send only zero.
          cmd={};const auto stopped=send(cmd);
          can_tx_retries+=stopped.attempts>0?stopped.attempts-1:0;
        }
      }
      previous_cmd=cmd;
      geometry_msgs::Twist out; out.linear.x=cmd.v; out.angular.z=cmd.w; cmd_pub.publish(out);
      if(motion->stage!=last_stage) { ROS_INFO("STATE %s %s",motion->name(),motion->fault.c_str()); last_stage=motion->stage; }
      std_msgs::String status; status.data=std::string(motion->name())+" "+motion->fault; status_pub.publish(status);
      if(++ticks%5==0 || (!console_mode&&(motion->stage==lab26::Stage::Done || motion->stage==lab26::Stage::Fault))) {
        log<<ros::WallTime::now().toSec()<<','<<motion->name()<<','<<pose.x<<','<<pose.y<<','<<pose.yaw<<','<<motion->progress<<','<<motion->cross_track<<','<<motion->displacement<<','<<cmd.v<<','<<cmd.w<<','<<feedback_v<<','<<feedback_w<<','<<mode<<','<<turn_obstacle<<','<<drive_obstacle<<','<<(motion->cfg.avoidance?motion->avoidanceState():"")<<','<<(use_global_route&&motion->cfg.xy_goal?int(global_route->replans):motion->avoidanceReplans())<<','<<motion->avoidanceRemaining()<<','<<csvField(motion->avoidanceReason())<<','<<csvField(motion->fault)<<','<<last_can_tx_errno<<','<<can_tx_retries<<','<<front_gap<<','<<rear_gap<<std::endl;
        if(console) {
          std::ostringstream s; s<<"{\"stage\":\""<<motion->name()<<"\",\"mode\":\""<<(manual?"manual":"auto")<<"\",\"paused\":"<<(operator_pause?"true":"false")<<",\"obstacle\":"<<(obstacle_pause?"true":"false")<<",\"progress\":"<<motion->progress<<",\"v\":"<<cmd.v<<",\"w\":"<<cmd.w<<",\"chassis_mode\":"<<mode<<",\"feedback_v\":"<<feedback_v<<",\"feedback_w\":"<<feedback_w<<",\"reason\":\""<<motion->fault<<"\"}";
          auto status_json=s.str();status_json.pop_back();
          status_json+=",\"map_generation\":\""+map_generation+"\"";
          if(cloud_time>0 && t-cloud_time<.5){
            const auto metres=[](double d){return std::isfinite(d)?std::to_string(d):std::string("null");};
            status_json+=",\"front_gap_m\":"+metres(front_gap)+",\"rear_gap_m\":"+metres(rear_gap);
          }
          status_json+=",\"total_distance\":"+std::to_string(motion->cfg.figure_eight?4*lab26::pi*motion->cfg.radius:motion->cfg.distance);
          const bool ready=pose_time>0&&t-pose_time<0.5&&cloud_time>0&&t-cloud_time<0.5;
          const bool feedback_fresh=dry||(system_time>0&&t-system_time<0.8&&t-motion_time<0.5&&t-rc_time<0.8);
          status_json+=std::string(",\"control_permitted\":")+(controlPermitted(t)?"true":"false");
          status_json+=std::string(",\"sensors_ready\":")+(ready?"true":"false")+",\"feedback_fresh\":"+(feedback_fresh?"true":"false")+"}";
          status_json.pop_back();const auto blocked=manualBlocked(t);
          status_json+=std::string(",\"manual_ready\":")+(blocked?"false":"true")+",\"manual_block_reason\":\""+(blocked?blocked:"")+"\",\"remote_switch\":"+std::to_string(rc_swb)+",\"chassis_errors\":"+std::to_string(errors)+"}";
          status_json.pop_back();status_json+=",\"manual_vector\":true}";
          status_json.pop_back();
          status_json+=std::string(",\"avoidance_state\":\"")+(motion->cfg.avoidance?motion->avoidanceState():"")+"\",\"front_left_points\":"+std::to_string(avoid_left)+",\"front_right_points\":"+std::to_string(avoid_right)+"}";
          status_json.pop_back();status_json+=",\"avoidance_reason\":\""+motion->avoidanceReason()+"\",\"avoidance_replans\":"+std::to_string((use_global_route&&motion->cfg.xy_goal?int(global_route->replans):motion->avoidanceReplans()))+",\"goal_distance\":"+std::to_string(motion->avoidanceRemaining())+",\"avoidance_version\":6,\"last_can_tx_errno\":"+std::to_string(last_can_tx_errno)+",\"can_tx_retries\":"+std::to_string(can_tx_retries)+"}";
          status_json.pop_back();status_json+=std::string(",\"dwa_enabled\":")+(use_global_route?"true":"false")+",\"dwa_ready\":"+(global_route->ready()?"true":"false")+"}";
          status_json.pop_back();
          status_json+=",\"maze_protocol\":2,\"maze_height_observations\":true,\"maze_leg\":\""+maze_leg+"\"";
          status_json+=std::string(",\"gap_roam\":true,\"roaming\":")+(gap_roaming?"true":"false");
          status_json+=",\"onboard_protocol\":1,\"onboard_id\":\""+onboard_id+"\",\"onboard_experiment\":\""+onboard_experiment+"\"";
          status_json+=std::string(",\"xy_planner\":\"")+(use_global_route?"navfn_pursuit":"local_xy")+"\",\"xy_local_recovery\":false}";
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
  std::unique_ptr<GlobalRouteClient> global_route;
  ros::Publisher obstacle_pub,clearing_pub,maze_obstacle_pub;
  std::string master_id,map_generation;
  std::string maze_leg;
  std::string onboard_id,onboard_experiment;
  lab26::GapRoam gap_roam;
  bool gap_roaming=false;
  double maze_extension=.18;
  void bindRosIO(){
    pose_sub.shutdown();cloud_sub.shutdown();world_cloud_sub.shutdown();
    cmd_pub.shutdown();status_pub.shutdown();obstacle_pub.shutdown();clearing_pub.shutdown();maze_obstacle_pub.shutdown();global_route.reset();
    global_route.reset(new GlobalRouteClient(nh));
    obstacle_pub=nh.advertise<sensor_msgs::PointCloud2>("/lab26/obstacles",1);
    clearing_pub=nh.advertise<sensor_msgs::PointCloud2>("/lab26/clearing_rays",1);
    maze_obstacle_pub=nh.advertise<sensor_msgs::PointCloud2>("/lab26/maze_obstacles",1);
    cmd_pub=nh.advertise<geometry_msgs::Twist>("/lab26/command",1);
    status_pub=nh.advertise<std_msgs::String>("/lab26/status",1,true);
    pose_sub=nh.subscribe("/Odometry",1,&Experiment::onPose,this);
    cloud_sub=nh.subscribe("/velodyne_points",1,&Experiment::onCloud,this);
    world_cloud_sub=nh.subscribe("/cloud_registered",1,&Experiment::onWorldCloud,this);
  }
  bool use_global_route=false,navigation_idle=true;
  lab26::PathTracker path_tracker;
  bool console_mode=false, operator_pause=false, manual=false, obstacle_pause=false;
  bool back_obstacle=true,turn_clear=false,drive_clear=false,back_clear=false;
  double last_heartbeat=-1,last_manual=-1,last_tick=0,obstacle_clear_since=-1;
  double last_cloud_preview=-1;
  double last_world_cloud=-1,last_pose_preview=-1;
  size_t world_preview_phase=0;
  using PreviewCell=std::array<int,3>;
  std::set<PreviewCell> preview_cells;
  unsigned preview_seq=0;
  std::string preview_frame,preview_generation;
  lab26::Command manual_cmd;
  lab26::Command previous_cmd;
  int last_can_tx_errno=0;
  unsigned long can_tx_retries=0;
  unsigned can_index=0;
  double last_can_check=-1;
  double linear_rate=0.20,angular_rate=0.60;
  double default_distance=5.0;
  double manual_linear_rate=0.60;
  double manual_speed=1.50,manual_angular_speed=0.50;
  double pose_stamp=0;
  double last_rebind=0;
  bool rear_obstacle_pause=false;
  std::vector<lab26::ObstaclePoint> avoid_points;
  lab26::Pose avoid_cloud_pose,ray_pose;
  lab26::ScanAlignment scan_alignment;
  lab26::LowObstacles low_obstacles;
  std::deque<lab26::ObstacleScan> pending_scans;
  double latest_raw_start=-1.;
  static void publishPoints(ros::Publisher& publisher,const std::vector<lab26::ObstaclePoint>& points,double stamp,
                            const std::vector<lab26::TimedReturn>* measured=nullptr){
    sensor_msgs::PointCloud2 msg;msg.header.frame_id="body";msg.header.stamp.fromSec(stamp);
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(points.size());
    if(!points.empty()){
      sensor_msgs::PointCloud2Iterator<float> x(msg,"x"),y(msg,"y"),z(msg,"z");
      for(size_t i=0;i<points.size();++i){*x=points[i].x;*y=points[i].y;*z=measured?(*measured)[i].z:0.;++x;++y;++z;}
    }
    publisher.publish(msg);
  }
  void publishAlignedScans(){
    std::vector<lab26::ObstaclePoint> aligned,clearing,marks;
    while(!pending_scans.empty()){
      const auto& scan=pending_scans.front();
      const double end=scan.start+scan.duration;
      if(lab26::ScanAlignment::awaitingPose(end,pose_stamp))break;
      lab26::Pose reference;
      if(ros::Time::now().toSec()-end<.45 && scan_alignment.align(scan,aligned,&clearing,&reference)){
        low_obstacles.update(scan,aligned,reference,scan_alignment.stillDuring(scan.start,end));
        marks.clear();
        for(size_t i=0;i<aligned.size();++i)if(scan.returns[i].mark)marks.push_back(aligned[i]);
        const double stamp=std::min(end,pose_stamp);
        publishPoints(clearing_pub,clearing,stamp);publishPoints(obstacle_pub,marks,stamp);
        // Navfn masks rear marks. The persistent maze map needs every observed
        // obstacle for checking turns and the path back from a dead end.
        if(maze_obstacle_pub.getNumSubscribers())publishPoints(maze_obstacle_pub,aligned,stamp,&scan.returns);
        if(latest_raw_start==scan.start){
          avoid_points.swap(aligned);avoid_cloud_pose=reference;
          front_gap=lab26::bumperClearance(avoid_points,0.);rear_gap=lab26::bumperClearance(avoid_points,lab26::pi);
        }
      }
      pending_scans.pop_front();
    }
  }
  int avoid_left=0,avoid_right=0;
  double front_gap=0.,rear_gap=0.;
  lab26::NearPlaneRays near_plane_rays;
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
  void command(std::string c,double t) {
    if(c=="cloudkey"){preview_seq=0;preview_cells.clear();return;}
    if(c=="heartbeat") { last_heartbeat=t; return; }
    if(c=="stop") { motion->stop("Operator stop");manual_cmd={};operator_pause=true;return; }
    if(c=="zero") { manual_cmd={};return; }
    if(c=="pause") { operator_pause=true;manual_cmd={};return; }
    if(t-last_heartbeat>0.4) { if(c=="manual")manualReply(false,"heartbeat_missing");return; }
    if(!controlPermitted(t)){if(c=="manual")manualReply(false,networkSelected()?"feedback_unavailable":"chassis_mode");return;}
    if(c.rfind("extendmaze ",0)==0){
      std::istringstream input(c);std::string verb,previous,next,frame,generation,extra;
      lab26::ObstaclePoint goal;
      if(!(input>>verb>>previous>>next>>goal.x>>goal.y>>frame>>generation)||(input>>extra)||
         operator_pause||manual||!use_global_route||motion->stage!=lab26::Stage::Driving||maze_leg.empty()||
         previous!=maze_leg||next==previous||next.size()!=36||next.find_first_not_of("0123456789abcdef-")!=std::string::npos||
         generation!=map_generation||frame!=pose_frame||manualBlocked(t)||!global_route->ready()||
         !lab26::mazeExtensionFits(pose,{motion->cfg.goal_x,motion->cfg.goal_y},goal))return;
      motion->cfg.goal_x=goal.x;motion->cfg.goal_y=goal.y;maze_leg=next;
      global_route->extend();return;
    }
    std::string requested_id;
    if(c.rfind("onboard ",0)==0){
      const auto separator=c.find(' ',8);
      if(separator!=44)return;
      requested_id=c.substr(8,36);c=c.substr(separator+1);
      if(requested_id.find_first_not_of("0123456789abcdef-")!=std::string::npos||
         requested_id==onboard_id||manualBlocked(t)||std::abs(feedback_v)>=.02||std::abs(feedback_w)>=.02)return;
      if(c!="start"&&c!="start3"&&c.rfind("startxy ",0)!=0&&c.rfind("start8 ",0)!=0&&c.rfind("startroam ",0)!=0)return;
    }
    const bool maze=c.rfind("startmaze ",0)==0;
    const bool roam=c.rfind("startroam ",0)==0;
    if(c=="start"||c=="start3"||roam||maze||c.rfind("startxy ",0)==0||c.rfind("start8 ",0)==0) {
      if(motion->stage!=lab26::Stage::Waiting&&motion->stage!=lab26::Stage::Done&&motion->stage!=lab26::Stage::Fault) return;
      auto cfg=motion->cfg;cfg.figure_eight=c.rfind("start8 ",0)==0;cfg.avoidance=c=="start3"||roam;
      if(roam){
        std::istringstream input(c);std::string verb,extra;double extension;
        if(!(input>>verb>>extension)||(input>>extra)||!std::isfinite(extension)||extension<.02||extension>.60||manualBlocked(t))return;
        maze_extension=extension;
      }
      cfg.xy_goal=maze||c.rfind("startxy ",0)==0;
      std::string leg;
      if(cfg.xy_goal){
        std::istringstream input(c);std::string verb,frame,extra;
        if(!(input>>verb>>cfg.goal_x>>cfg.goal_y>>frame)||
           !std::isfinite(cfg.goal_x)||!std::isfinite(cfg.goal_y)||frame!=pose_frame||
           t-pose_time>.5||std::hypot(cfg.goal_x-pose.x,cfg.goal_y-pose.y)>30)return;
        if(maze){
          std::string generation;
          double extension;
          if(!(input>>generation>>leg>>extension)||generation!=map_generation||!use_global_route||
             leg.empty()||leg.size()>40||leg.find_first_not_of("0123456789abcdef-")!=std::string::npos||
             !std::isfinite(extension)||extension<.02||extension>.60||
             std::hypot(cfg.goal_x-pose.x,cfg.goal_y-pose.y)>2.05)return;
          if(input>>extra)return;
          maze_extension=extension;
        }else if(input>>extra)return;
        cfg.avoidance=true;
      }
      cfg.distance=cfg.avoidance?10.0:default_distance;
      if(cfg.figure_eight){std::istringstream input(c);std::string verb,point,extra;double radius;
        if(!(input>>verb>>radius>>point)||(input>>extra)||!std::isfinite(radius)||radius<0.5||radius>10||(point!="cross"&&point!="outer"))return;
        cfg.radius=radius;cfg.outer_start=point=="outer";
      }
      run_timeout=cfg.figure_eight?4*lab26::pi*cfg.radius/std::min(cfg.speed,0.8*cfg.max_w*cfg.radius)*2+90:180;
      motion.reset(new lab26::Motion(cfg));
      onboard_id=requested_id;
      onboard_experiment=onboard_id.empty()?"":roam?"maze":cfg.xy_goal?"lab3-xy":cfg.figure_eight?"lab2-eight":c=="start3"?"lab3":"lab2";
      maze_leg=leg;
      gap_roaming=roam;gap_roam=lab26::GapRoam{};
      global_route->reset();path_tracker=lab26::PathTracker();navigation_idle=false;
      manual=false;manual_cmd={};operator_pause=false;obstacle_pause=false;created=t;clear_since=-1;no_response_since=-1;
    } else if(c=="resume") { operator_pause=false;navigation_idle=false; }
    else if(c=="manual") {
      if(const auto blocked=manualBlocked(t)){manualReply(false,blocked);return;}
      if(motion->stage==lab26::Stage::Fault||motion->stage==lab26::Stage::Done) {
        auto cfg=motion->cfg;motion.reset(new lab26::Motion(cfg));created=t;
      }
      manual=true;operator_pause=true;manual_cmd={};
      onboard_id.clear();onboard_experiment.clear();
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
  int canfd=-1,lockfd=-1,mode=-1,vehicle_state=-1,errors=-1,rc_swb=-1;
  double rc_time=-1;
  bool networkSelected() const {return mode==1&&(rc_swb==0||rc_swb==2);}
  bool controlPermitted(double t) const {
    return dry||(networkSelected()&&t-rc_time<.8&&t-system_time<.8&&t-motion_time<.5&&vehicle_state==0&&errors==0);
  }
  double created=0,pose_time=0,cloud_time=0,system_time=0,motion_time=0,run_timeout=180;
  double feedback_v=0,feedback_w=0; std::ofstream log;
  double no_response_since=-1;
  double clear_since=-1;
  std::string pose_frame;
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
    std::string next_frame=msg->header.frame_id;
    if(!next_frame.empty()&&next_frame[0]=='/')next_frame.erase(0,1);
    if(!pose_frame.empty()&&pose_frame!=next_frame)motion->stop("Odometry frame changed");
    if(next_frame!=pose_frame||msg->header.stamp.toSec()<pose_stamp||
       (pose_time>0 && std::hypot(next.x-previous.x,next.y-previous.y)>.15+plausible_speed*(t-pose_time))){
      if(pose_time>0 && msg->header.stamp.toSec()<pose_stamp)motion->stop("Odometry time reset");
      global_route->reset();path_tracker=lab26::PathTracker();
      scan_alignment.clear();pending_scans.clear();low_obstacles.clear();avoid_points.clear();near_plane_rays={};cloud_time=0.;
    }
    pose_frame=next_frame;
    pose=next; previous=next; pose_time=t; pose_stamp=msg->header.stamp.toSec();
    if(use_global_route){scan_alignment.append(pose_stamp,pose);publishAlignedScans();}
    if(console&&console->connected&&t-last_pose_preview>=0.05&&std::isfinite(p.z)) {
      std::ostringstream out;out<<std::setprecision(16)<<"{\"type\":\"pose\",\"stamp\":"<<pose_stamp<<",\"frame\":\""<<safeFrame(msg->header.frame_id)<<"\",\"pose\":["<<pose.x<<','<<pose.y<<','<<pose.yaw<<"],\"position\":["<<p.x<<','<<p.y<<','<<p.z<<"],\"quaternion\":["<<q.x<<','<<q.y<<','<<q.z<<','<<q.w<<"]}";
      console->status(out.str());last_pose_preview=t;
    }
    if(stable_count==0||std::hypot(next.x-stable_ref.x,next.y-stable_ref.y)>0.06||std::abs(lab26::wrap(next.yaw-stable_ref.yaw))>0.04) { stable_ref=next; stable_count=1; }
    else ++stable_count;
  }
  void onCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    const double age=(ros::Time::now()-msg->header.stamp).toSec();
    if(age>0.5||age< -0.1||msg->width*msg->height<100) return;
    int turn_count=0,drive_count=0,back_count=0,turn_release=0,drive_release=0,back_release=0;
    std::vector<lab26::ObstaclePoint> next_points;
    lab26::NearPlaneRays next_rays;
    lab26::ClearingRays clearing;
    size_t finite_points=0;
    const bool preview=console&&console->canPreview()&&now()-last_world_cloud>2.0&&now()-last_cloud_preview>=1.0;
    std::ostringstream cloud_json;
    if(preview) cloud_json<<std::fixed<<std::setprecision(3)<<"{\"type\":\"cloud\",\"registered\":false,\"frame\":\""<<safeFrame(msg->header.frame_id)<<"\",\"stamp\":"<<std::setprecision(6)<<msg->header.stamp.toSec()<<std::setprecision(3)<<",\"points\":[";
    size_t point_index=0,preview_count=0;
    const size_t stride=std::max<size_t>(1,(size_t(msg->width)*msg->height+7999)/8000);
    try {
      lab26::GroundReturns ground;
      if(use_global_route){
        sensor_msgs::PointCloud2ConstIterator<float> gx(*msg,"x"),gy(*msg,"y"),gz(*msg,"z");
        for(;gx!=gx.end();++gx,++gy,++gz)ground.observe(*gx,*gy,*gz);
        ground.finish();
      }
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg,"x"),y(*msg,"y"),z(*msg,"z");
      lab26::ObstacleScan scan;scan.start=msg->header.stamp.toSec();
      std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> offset;
      if(use_global_route){
        for(const auto& field:msg->fields)if(field.name=="time" && field.datatype==sensor_msgs::PointField::FLOAT32)
          offset.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*msg,"time"));
      }
      bool timing_ok=true;
      for(;x!=x.end();++x,++y,++z) {
        const double point_time=offset?**offset:0.;
        if(offset)++(*offset);
        if(!std::isfinite(point_time)||point_time<0.||point_time>.25)timing_ok=false;
        else scan.duration=std::max(scan.duration,point_time);
        if(std::isfinite(*x)&&std::isfinite(*y)&&std::isfinite(*z))++finite_points;
        const bool sampled=(point_index++%stride)==0;
        if(preview&&sampled&&std::isfinite(*x)&&std::isfinite(*y)&&std::isfinite(*z)&&std::abs(*x)<50&&std::abs(*y)<50&&std::abs(*z)<20) {
          if(preview_count++) cloud_json<<',';
          cloud_json<<'['<<*x<<','<<*y<<','<<*z<<']';
        }
        if(!std::isfinite(*x)||!std::isfinite(*y)||!std::isfinite(*z))continue;
        const bool outside_body=std::abs(*x)>lab26::body_half_length||std::abs(*y)>lab26::body_half_width;
        const bool obstacle=ground.obstacle(*x,*y,*z);
        if(use_global_route && outside_body){
          if(*z<-.14 && std::hypot(*x,*y)<8.)scan.lower_rays.push_back({{*x,*y},point_time,false,*z});
          next_rays.add(*x,*y,*z);
          clearing.add(*x,*y,*z,point_time,obstacle);
        }
        const bool in_band=std::abs(*z)<=.35;
        if(!in_band && (!use_global_route || !obstacle))continue;
        const double r=std::hypot(*x,*y);
        // Centered lidar; use the assignment's 650 x 550 mm rectangle.
        if(outside_body) {
          const double range=6.0;
          if(r<range)next_points.push_back({*x,*y});
          if(use_global_route && r<range)
            scan.returns.push_back({{*x,*y},point_time,*x>=-lab26::body_half_length,*z});
        }
        if(!in_band)continue; // Preserve the sector indicators of the other experiments.
        if(r>0.45&&r<1.0) ++turn_count;
        if(r>0.45&&r<1.2) ++turn_release;
        if(std::abs(*y)<0.5) {
          if(*x>0.45&&*x<1.0) ++drive_count;
          if(*x>0.45&&*x<1.2) ++drive_release;
          if(*x< -0.45&&*x> -1.0) ++back_count;
          if(*x< -0.45&&*x> -1.2) ++back_release;
        }
      }
      if(finite_points<100)return;
      avoid_points.swap(next_points);
      next_rays.received_at=now();
      near_plane_rays=next_rays;
      avoid_cloud_pose=pose;ray_pose=pose;latest_raw_start=scan.start;
      front_gap=lab26::bumperClearance(avoid_points,0.);
      rear_gap=lab26::bumperClearance(avoid_points,lab26::pi);
      if(use_global_route && timing_ok){
        scan.clearing=clearing.endpoints();
        if(!pending_scans.empty() && scan.start<=pending_scans.back().start)pending_scans.clear();
        pending_scans.push_back(std::move(scan));
        if(pending_scans.size()>5)pending_scans.pop_front();
        publishAlignedScans();
      }
      turn_obstacle=turn_count>=3; drive_obstacle=drive_count>=3; cloud_time=now();
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
    if(!console||!console->canPreview()||now()-last_cloud_preview<1.0)return;
    const size_t stride=std::max<size_t>(1,(size_t(msg->width)*msg->height+1199)/1200);
    const size_t phase=(world_preview_phase++/4)%stride;size_t index=0;
    std::set<PreviewCell> current;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg,"x"),y(*msg,"y"),z(*msg,"z");
      for(;x!=x.end();++x,++y,++z){
        if(index++%stride!=phase||!std::isfinite(*x)||!std::isfinite(*y)||!std::isfinite(*z)||
           std::max({std::abs(*x),std::abs(*y),std::abs(*z)})>10000.)continue;
        current.insert({int(std::lround(*x*100)),int(std::lround(*y*100)),int(std::lround(*z*100))});
      }
      const auto frame=safeFrame(msg->header.frame_id);
      std::vector<PreviewCell> added,removed;
      std::set_difference(current.begin(),current.end(),preview_cells.begin(),preview_cells.end(),std::back_inserter(added));
      std::set_difference(preview_cells.begin(),preview_cells.end(),current.begin(),current.end(),std::back_inserter(removed));
      const bool full=!preview_seq||preview_frame!=frame||preview_generation!=map_generation||added.size()+removed.size()>current.size();
      std::ostringstream out;out<<std::fixed<<std::setprecision(6)<<"{\"type\":\"cloud_delta\",\"frame\":\""<<frame<<"\",\"stamp\":"<<msg->header.stamp.toSec()
        <<",\"base\":"<<(full?0:preview_seq)<<",\"seq\":"<<(preview_seq+1)<<",\"add\":[";
      auto list=[&](const auto& cells){bool first=true;for(const auto& p:cells){if(!first)out<<',';out<<'['<<p[0]<<','<<p[1]<<','<<p[2]<<']';first=false;}};
      if(full)list(current);else list(added);
      out<<"],\"del\":[";if(!full)list(removed);out<<"]}";
      console->status(out.str());preview_cells.swap(current);++preview_seq;preview_frame=frame;preview_generation=map_generation;last_cloud_preview=now();
    }catch(const std::runtime_error& e){ROS_WARN_THROTTLE(2,"Invalid registered cloud: %s",e.what());}
  }
  void readCan() {
    if(canfd<0) return;
    can_frame f{};
    for(int i=0;i<500;++i) {
      ssize_t n=read(canfd,&f,sizeof(f)); if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)) break;
      if(n<0&&errno==EINTR)continue;
      if(n!=sizeof(f)) { motion->stop("CAN receive error"); break; }
      if(f.can_id&CAN_ERR_FLAG){if(f.can_id&CAN_ERR_BUSOFF)motion->stop("CAN bus-off");continue;}
      if(f.can_id==0x211&&f.can_dlc==8) { vehicle_state=f.data[0]; mode=f.data[1]; errors=(uint16_t(f.data[4])<<8)|f.data[5]; system_time=now(); }
      if(f.can_id==0x221&&f.can_dlc==8) { feedback_v=decode(f.data); feedback_w=decode(f.data+2); motion_time=now(); }
      if(f.can_id==0x241&&f.can_dlc==8){rc_swb=(f.data[0]>>2)&3;rc_time=now();}
    }
  }
  bool bindCan() {
    const unsigned index=if_nametoindex("can0");if(!index)return false;
    const int fd=socket(PF_CAN,SOCK_RAW|SOCK_NONBLOCK,CAN_RAW);if(fd<0)return false;
    sockaddr_can addr{};addr.can_family=AF_CAN;addr.can_ifindex=index;
    const can_filter filters[]={{0x211,CAN_SFF_MASK|CAN_EFF_FLAG|CAN_RTR_FLAG},
                                {0x221,CAN_SFF_MASK|CAN_EFF_FLAG|CAN_RTR_FLAG},
                                {0x241,CAN_SFF_MASK|CAN_EFF_FLAG|CAN_RTR_FLAG}};
    const can_err_mask_t mask=CAN_ERR_BUSOFF;
    if(bind(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0||
       setsockopt(fd,SOL_CAN_RAW,CAN_RAW_FILTER,filters,sizeof(filters))<0||
       setsockopt(fd,SOL_CAN_RAW,CAN_RAW_ERR_FILTER,&mask,sizeof(mask))<0){close(fd);return false;}
    if(canfd>=0)close(canfd);
    canfd=fd;can_index=index;system_time=motion_time=rc_time=-1;mode=rc_swb=-1;last_can_tx_errno=0;return true;
  }
  static std::string csvField(const std::string& value) {
    std::string escaped="\"";for(char c:value){if(c=='\"')escaped+='\"';escaped+=c;}return escaped+'\"';
  }
  lab26::CanSendResult send(lab26::Command c) {
    if(canfd<0) return {};
    can_frame f{}; f.can_id=0x111; f.can_dlc=8;
    auto encode=[](double value,uint8_t* b) { const uint16_t raw=static_cast<uint16_t>(static_cast<int16_t>(std::lround(value*1000))); b[0]=raw>>8; b[1]=raw&255; };
    encode(c.v,f.data); encode(c.w,f.data+2);
    // At most two 1 ms waits: a transient full queue need not abort a task,
    // but persistent failure and a down interface remain immediate faults.
    return lab26::transmitCan([&](){const auto n=write(canfd,&f,sizeof(f));return n==sizeof(f)?0:n<0?errno:EIO;},[](){usleep(1000);});
  }
};
int main(int argc,char**argv) {
  ros::init(argc,argv,"basicControl");
  try { Experiment experiment; return experiment.run(); }
  catch(const std::exception& e) { ROS_FATAL("%s",e.what()); return 1; }
}
