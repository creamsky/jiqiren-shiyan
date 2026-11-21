#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include "figure_eight.h"
#include "basic_control/control_core.h"
#include "basic_control/footprint.h"

namespace lab26 {
constexpr double pi = 3.14159265358979323846;
inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline double clip(double v, double bound) { return std::max(-bound, std::min(bound, v)); }
struct Pose { double x=0, y=0, yaw=0; };
struct Command { double v=0, w=0; };
struct Settings {
  double turn=pi/4, distance=5, speed=0.50, max_w=0.30;
  double yaw_tolerance=0.025, heading_gain=0.8, cross_gain=0.6;
  double heading_d=0.08, derivative_filter=0.15;
  bool figure_eight=false,outer_start=false;
  bool avoidance=false;
  double radius=2.0;
};
enum class Stage { Waiting, Turning, Driving, Done, Fault };
class Motion {
 public:
  explicit Motion(Settings settings={}) : cfg(settings) {
    if (!(cfg.distance>0 && cfg.speed>0 && cfg.speed<=0.5 && cfg.max_w>0 && cfg.max_w<=0.5))
      throw std::invalid_argument("Invalid motion limits");
    if(!std::isfinite(cfg.heading_gain)||cfg.heading_gain<=0||!std::isfinite(cfg.heading_d)||cfg.heading_d<0||
       !std::isfinite(cfg.derivative_filter)||cfg.derivative_filter<=0)
      throw std::invalid_argument("Invalid PD gains");
  }
  Settings cfg;
  Stage stage=Stage::Waiting;
  std::string fault;
  void hold(double seconds) { begin+=seconds; held_seconds+=seconds; if(settled>=0) settled+=seconds; derivative_ready=false; filtered_rate=0; }
  int avoidance_flag=0;
  const char* avoidanceState() const { return basic_control::stateName(avoid.state()); }
  Pose origin;
  double target_yaw=0, progress=0, cross_track=0, displacement=0;
  void start(Pose p, double now) {
    if (stage!=Stage::Waiting) return;
    origin=p; target_yaw=wrap(p.yaw+cfg.turn); begin=now; stage=Stage::Turning;
    derivative_ready=false;filtered_rate=0;
    if(cfg.figure_eight){route=lab2ext::path(cfg.radius,cfg.outer_start,p.x,p.y,p.yaw);cursor=0;stage=Stage::Driving;}
    if(cfg.avoidance) {
      basic_control::ControlConfig settings;
      settings.straight_yaw=p.yaw;settings.straight_speed=avoidance_speed;settings.avoid_speed=0.6;
      settings.k_angle=3.0;settings.max_turn=1.2;settings.max_radius=cfg.distance;
      settings.track_reference=true;
      settings.avoid_distance=1.0;
      avoid=basic_control::ControlCore(settings);stage=Stage::Driving;
    }
  }
  void stop(const std::string& why) { if (stage!=Stage::Done && stage!=Stage::Fault) { stage=Stage::Fault; fault=why; } }
  const char* name() const {
    switch(stage) {
      case Stage::Waiting:return "WAITING"; case Stage::Turning:return "TURNING";
      case Stage::Driving:return "DRIVING"; case Stage::Done:return "DONE";
      default:return "FAULT";
    }
  }
  Command step(Pose p, double now) {
    if (stage==Stage::Waiting || stage==Stage::Done || stage==Stage::Fault) return {};
    if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.yaw)) {
      stop("Non-finite pose"); return {};
    }
    if(cfg.avoidance) {
      displacement=std::hypot(p.x-origin.x,p.y-origin.y);
      progress=std::max(0.0,std::min(cfg.distance,(p.x-origin.x)*std::cos(origin.yaw)+(p.y-origin.y)*std::sin(origin.yaw)));
      avoid.onPose(p.x-origin.x,p.y-origin.y,p.yaw,now-held_seconds);
      avoid.onObstacle(avoidance_flag,now-held_seconds);
      const auto command=avoid.tick(now-held_seconds);
      if(avoid.state()==basic_control::State::Finished)stage=Stage::Done;
      if(avoid.state()==basic_control::State::Fault)stop("No clear direction within exploration limit");
      return {command.speed,command.turn};
    }
    if(cfg.figure_eight) return stepEight(p,now);
    const double dt=now-previous_time;
    if(derivative_ready&&dt>0&&dt<0.5) {
      const double measured_rate=wrap(p.yaw-previous_yaw)/dt;
      const double alpha=dt/(cfg.derivative_filter+dt);
      filtered_rate+=alpha*(measured_rate-filtered_rate);
    } else filtered_rate=0;
    previous_yaw=p.yaw;previous_time=now;derivative_ready=true;
    if (now-begin > cfg.distance/cfg.speed+70) { stop("Experiment timeout"); return {}; }
    const double dx=p.x-origin.x, dy=p.y-origin.y;
    progress=dx*std::cos(target_yaw)+dy*std::sin(target_yaw);
    cross_track=-dx*std::sin(target_yaw)+dy*std::cos(target_yaw);
    displacement=std::hypot(dx,dy);
    if (std::abs(cross_track)>0.5) { stop("Cross-track error > 0.5 m"); return {}; }
    if (stage==Stage::Turning) {
      if (now-begin>40) { stop("Turn timeout"); return {}; }
      const double e=wrap(target_yaw-p.yaw);
      if (std::abs(e)<=cfg.yaw_tolerance) {
        if (settled<0) settled=now;
        if (now-settled>=0.5) stage=Stage::Driving;
        return {};
      }
      settled=-1;
      double w=clip(cfg.heading_gain*e-cfg.heading_d*filtered_rate,cfg.max_w);
      if(w*e>0&&std::abs(w)<0.08) w=std::copysign(std::min(0.08,cfg.max_w),w);
      return {0,w};
    }
    if (displacement>=cfg.distance || progress>=cfg.distance) { stage=Stage::Done; return {}; }
    const double desired=wrap(target_yaw-std::atan2(cfg.cross_gain*cross_track,0.3));
    const double e=wrap(desired-p.yaw);
    const double w=clip(cfg.heading_gain*e-cfg.heading_d*filtered_rate,cfg.max_w);
    if (std::abs(e)>0.65) return {0,w};
    const double remaining=cfg.distance-std::max(progress,displacement);
    const double v=std::min(cfg.speed,std::max(0.025,0.4*remaining));
    return {v*std::max(0.0,std::cos(e)),w};
  }
 private:
  basic_control::ControlCore avoid;
  double held_seconds=0;
  std::vector<lab2ext::Point> route;
  size_t cursor=0;
  Command stepEight(Pose p,double now) {
    const double cruise=std::min(cfg.speed,0.8*cfg.max_w*cfg.radius);
    const double length=4*pi*cfg.radius;
    if(now-begin>length/cruise*2+60){stop("Figure-eight timeout");return {};}
    size_t nearest=cursor;double best=1e9;
    for(size_t i=cursor;i<std::min(route.size(),cursor+51);++i){double d=std::hypot(p.x-route[i].x,p.y-route[i].y);if(d<best){best=d;nearest=i;}}
    cursor=nearest;progress=length*cursor/(route.size()-1);cross_track=best;displacement=std::hypot(p.x-origin.x,p.y-origin.y);
    if(best>0.8){stop("Figure-eight tracking error > 0.8 m");return {};}
    if(cursor>=route.size()-4&&std::hypot(p.x-route.back().x,p.y-route.back().y)<0.10){progress=length;stage=Stage::Done;return {};}
    size_t goal=cursor;double along=0;const double look=std::min(0.5,cfg.radius*0.5);
    while(goal+1<route.size()&&along<look){along+=std::hypot(route[goal+1].x-route[goal].x,route[goal+1].y-route[goal].y);++goal;}
    const double dx=route[goal].x-p.x,dy=route[goal].y-p.y;
    const double error=wrap(std::atan2(dy,dx)-p.yaw);
    if(std::abs(error)>0.8)return {0,clip(cfg.heading_gain*error,cfg.max_w)};
    const double v=std::min(cruise,std::max(0.04,(length-progress)*0.5));
    return {v,clip(2*v*std::sin(error)/std::max(0.10,std::hypot(dx,dy)),cfg.max_w)};
  }
  double begin=0, settled=-1;
  bool derivative_ready=false;
  double previous_time=0,previous_yaw=0,filtered_rate=0;
};
} // namespace lab26
