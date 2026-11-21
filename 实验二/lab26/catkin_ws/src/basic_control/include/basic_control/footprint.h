#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace lab26 {
constexpr double avoidance_speed=0.8;
constexpr double avoidance_reaction_time=0.2;
constexpr double avoidance_linear_deceleration=0.2;
// Assignment Figure 4.1: 650 x 550 mm, with the lidar centered on the body.
constexpr double body_half_length=0.325,body_half_width=0.275;
constexpr double body_length_margin=0.01,body_width_margin=0.015;
struct ObstaclePoint { double x,y; };
struct FootprintGuard { bool blocked=false,clear=true; int left=0,right=0; };

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
    if(v>=0&&std::abs(w)<1e-8&&p.x<0)continue;
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
