#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace lab26 {
constexpr double avoidance_speed=0.8;
constexpr double avoidance_reaction_time=0.2;
constexpr double avoidance_linear_deceleration=0.2;
// Assignment Figure 4.1: 650 x 550 mm, with the lidar centered on the body.
constexpr double body_half_length=0.325,body_half_width=0.275;
constexpr double body_length_margin=0.01,body_width_margin=0.015;
struct ObstaclePoint { double x,y; bool remembered=false; };
// Each endpoint is clipped to the part of a real ray lying within 10 cm
// of the lidar plane. These observations describe coverage, not obstacles.
struct NearPlaneRays {
  std::array<ObstaclePoint,360> ends{};
  double received_at=-1.;
  void add(double x,double y,double z){
    const double r=std::hypot(x,y);
    if(r<.40 || r>20. || !std::isfinite(z) || !std::isfinite(r))return;
    const double scale=std::abs(z)>.10?.10/std::abs(z):1.;
    const int bin=std::min(359,int((std::atan2(y,x)+std::acos(-1.))*180./std::acos(-1.)));
    auto& old=ends[std::max(0,bin)];
    if(r*r*scale*scale>old.x*old.x+old.y*old.y)old={x*scale,y*scale};
  }
};
struct RayView {
  const NearPlaneRays* scan=nullptr;
  double x=0,y=0,yaw=0;
};
struct FootprintGuard { bool blocked=false,clear=true; int left=0,right=0; };

inline double bumperClearance(const std::vector<ObstaclePoint>& scan,double angle){
  const double c=std::cos(angle),s=std::sin(angle);
  double nearest=std::numeric_limits<double>::infinity();
  for(const auto& p:scan){
    const double along=c*p.x+s*p.y;
    if(along>0 && std::abs(-s*p.x+c*p.y)<=body_half_width+body_width_margin)
      nearest=std::min(nearest,along-body_half_length-body_length_margin);
  }
  return std::max(0.,nearest);
}

inline double stoppingTravel(double speed){
  speed=std::abs(speed);
  return speed*avoidance_reaction_time+speed*speed/(2*avoidance_linear_deceleration);
}

// A return in each angular band must reach past the far edge. Any closer
// obstacle within the corridor vetoes the maneuver, also between the bands.
inline bool observedCorridor(const std::vector<ObstaclePoint>& scan,double angle,double travel,RayView rays={}) {
  const double c=std::cos(angle),s=std::sin(angle);
  const double far=body_half_length+body_length_margin+travel;
  const double width=body_half_width+body_width_margin;
  unsigned coverage=0;
  const auto cover=[&](double lateral){
    if(std::abs(lateral)<width){
      const int band=std::min(4,int((lateral+width)*5/(2*width)));
      coverage|=1u<<band;
    }
  };
  for(const auto& point:scan){
    const double along=c*point.x+s*point.y,lateral=-s*point.x+c*point.y;
    if(along<=0.)continue;
    if(along<=far && std::abs(lateral)<=width)return false;
    if(along<=far || point.remembered)continue;
    cover(lateral*far/along);
  }
  if(coverage!=31u && rays.scan){
    const double ox=c*rays.x+s*rays.y,oy=-s*rays.x+c*rays.y;
    const double rc=std::cos(rays.yaw-angle),rs=std::sin(rays.yaw-angle);
    if(ox<far)for(const auto& end:rays.scan->ends){
      const double dx=rc*end.x-rs*end.y;
      if(dx<=0 || ox+dx<=far)continue;
      cover(oy+(rs*end.x+rc*end.y)*(far-ox)/dx);
    }
  }
  return coverage==31u;
}

// Check only the requested angular sector. Enlarge each sampled rectangle by
// the maximum corner travel to its nearest sample, covering intermediate yaw.
inline bool pivotClear(const std::vector<ObstaclePoint>& scan,double angle) {
  const double length=body_half_length+body_length_margin,width=body_half_width+body_width_margin;
  const double radius=std::hypot(length,width);
  const int steps=std::max(1,int(std::ceil(std::abs(angle)/.04)));
  const double pad=radius*std::abs(angle)/(2*steps);
  for(const auto& p:scan){
    if(std::hypot(p.x,p.y)>radius+pad)continue;
    for(int i=0;i<=steps;++i){
      const double a=angle*i/steps,c=std::cos(a),s=std::sin(a);
      if(std::abs(c*p.x+s*p.y)<=length+pad && std::abs(-s*p.x+c*p.y)<=width+pad)return false;
    }
  }
  return true;
}

inline FootprintGuard sweptFootprint(const std::vector<ObstaclePoint>& points,
                                    double v,double w) {
  // Follow the current motion during reaction time, then integrate braking.
  // Holding full speed for the entire stopping time doubles the braking sweep.
  const double horizon=avoidance_reaction_time+std::max(std::abs(v)/avoidance_linear_deceleration,std::abs(w)/0.6);
  const double half_length=body_half_length+body_length_margin,half_width=body_half_width+body_width_margin;
  const double edge_speed=std::abs(v)+std::hypot(half_length,half_width)*std::abs(w);
  const double sample_time=std::min(0.02,0.005/std::max(0.01,edge_speed));
  const int steps=std::max(1,int(std::ceil(horizon/sample_time)));
  struct Sample { double x,y,c,s; };
  std::vector<Sample> path;
  double x=0,y=0,angle=0;
  const double dt=horizon/steps;
  double min_x=1e9,max_x=-1e9,min_y=1e9,max_y=-1e9;
  for(int i=0;i<=steps;++i) {
    if(i){
      const double braking=std::max(0.0,(i-.5)*dt-avoidance_reaction_time);
      const double linear=std::copysign(std::max(0.0,std::abs(v)-avoidance_linear_deceleration*braking),v);
      const double angular=std::copysign(std::max(0.0,std::abs(w)-0.6*braking),w);
      x+=linear*std::cos(angle+.5*angular*dt)*dt;
      y+=linear*std::sin(angle+.5*angular*dt)*dt;angle+=angular*dt;
    }
    path.push_back({x,y,std::cos(angle),std::sin(angle)});
    const auto& q=path.back();
    const double ex=half_length*std::abs(q.c)+half_width*std::abs(q.s),ey=half_length*std::abs(q.s)+half_width*std::abs(q.c);
    min_x=std::min(min_x,q.x-ex);max_x=std::max(max_x,q.x+ex);
    min_y=std::min(min_y,q.y-ey);max_y=std::max(max_y,q.y+ey);
  }
  int hits=0,left=0,right=0;bool release_hit=false;
  for(const auto& p:points) {
    if(p.x<min_x||p.x>max_x||p.y<min_y||p.y>max_y)continue;
    if(std::abs(w)<1e-8 && ((v>0 && p.x<=-body_half_length) ||
                              (v<0 && p.x>=body_half_length)))continue;
    bool hit=false;
    for(const auto& q:path) {
      const double dx=p.x-q.x,dy=p.y-q.y;
      const double x=std::abs(q.c*dx+q.s*dy),y=std::abs(-q.s*dx+q.c*dy);
      if(x<half_length&&y<half_width){hit=true;release_hit=true;}
      if(hit)break;
    }
    if(hit) { ++hits;if(p.y>=0)++left;else ++right; }
  }
  // A corner can touch only one return; do not wait for three penetrations.
  return {hits>0,!release_hit,left,right};
}
inline bool brakingBlocked(const std::vector<ObstaclePoint>& scan,double v,double w){
  return (v!=0. || w!=0.) && sweptFootprint(scan,v,w).blocked;
}
inline bool feasibleForward(const std::vector<ObstaclePoint>& points,double& v,double& w) {
  if(v<=0||!sweptFootprint(points,v,w).blocked)return false;
  for(double scale:{0.75,0.5,0.375,0.25,0.1875}) {
    if(v*scale<0.10)continue;
    if(!sweptFootprint(points,v*scale,w*scale).blocked) {
      v*=scale;w*=scale;return true;
    }
  }
  return false;
}
inline bool clearanceAdvance(const std::vector<ObstaclePoint>& points,double& v,double& w) {
  if(std::abs(v)>1e-8||std::abs(w)<1e-8||!sweptFootprint(points,v,w).blocked)return false;
  // Finish passing a side obstacle before turning back toward the route.
  if(sweptFootprint(points,0.30,0).blocked)return false;
  v=0.30;w=0;return true;
}
}
