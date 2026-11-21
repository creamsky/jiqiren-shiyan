#pragma once
#include "basic_control/motion.h"
#include "basic_control/footprint.h"

namespace lab26 {
inline bool mazeExtensionFits(Pose pose,ObstaclePoint previous,ObstaclePoint next){
  const double ax=previous.x-pose.x,ay=previous.y-pose.y,bx=next.x-pose.x,by=next.y-pose.y;
  const double old_range=std::hypot(ax,ay),range=std::hypot(bx,by);
  return old_range>=.35&&old_range<=1.15&&range<=2.05&&range>old_range+.30&&
    ax*bx+ay*by>=old_range*range*std::cos(.32)&&
    bx*std::cos(pose.yaw)+by*std::sin(pose.yaw)>=range*std::cos(.38);
}
// Each return bounds a possible unseen base extending beyond the visible cone.
inline bool mazeBrakeClear(const std::vector<ObstaclePoint>& scan, Command velocity, double extension) {
  const double reaction = .25, dt = .08;
  const double time = reaction + std::max(std::abs(velocity.v)/.20, std::abs(velocity.w)/.60);
  const double body_radius = std::hypot(body_half_length+body_length_margin, body_half_width+body_width_margin);
  const double pad = extension + dt*(std::abs(velocity.v)+body_radius*std::abs(velocity.w));
  const double range = body_radius + pad + std::abs(velocity.v)*time;
  double x=0., y=0., yaw=0.;
  for(double t=0.; t<=time+dt; t+=dt) {
    const double c=std::cos(yaw), s=std::sin(yaw);
    for(const auto& p:scan) {
      if(std::abs(p.x)>range || std::abs(p.y)>range)continue;
      const double dx=p.x-x, dy=p.y-y;
      const double u=std::max(0.,std::abs(c*dx+s*dy)-body_half_length-body_length_margin);
      const double v=std::max(0.,std::abs(-s*dx+c*dy)-body_half_width-body_width_margin);
      if(u*u+v*v<=pad*pad)return false;
    }
    x+=velocity.v*dt*std::cos(yaw+velocity.w*dt*.5);
    y+=velocity.v*dt*std::sin(yaw+velocity.w*dt*.5);
    yaw+=velocity.w*dt;
    if(t>=reaction) {
      velocity.v=std::copysign(std::max(0.,std::abs(velocity.v)-.20*dt),velocity.v);
      velocity.w=std::copysign(std::max(0.,std::abs(velocity.w)-.60*dt),velocity.w);
    }
  }
  return true;
}
// Retain curvature and try a slower command before surrendering this passage.
inline Command mazeSafeCommand(const std::vector<ObstaclePoint>& scan, Command measured,
                               Command proposed, double extension) {
  if(!mazeBrakeClear(scan,measured,extension))return {};
  const double limit=std::max({1.,std::abs(proposed.v)/.18,std::abs(proposed.w)/.38});
  for(const double fraction:{1.,.75,.5,.25}) {
    const Command candidate{proposed.v*fraction/limit,proposed.w*fraction/limit};
    if(mazeBrakeClear(scan,candidate,extension))return candidate;
  }
  return {};
}
}
