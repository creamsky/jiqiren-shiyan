#pragma once
#include "basic_control/motion.h"

namespace lab26 {
// The nearest approach to a rotating rectangle occurs at an endpoint or
// when a corner points at the return. A sign change of the cross product
// tests both ends of each diagonal without per-point trigonometry.
inline bool turnFits(const std::vector<ObstaclePoint>& points,double yaw,double extension,double shift=0.){
  const double length=body_half_length+body_length_margin,width=body_half_width+body_width_margin;
  const double reach=std::hypot(length,width)+extension,c=std::cos(yaw),s=std::sin(yaw);
  const auto overlaps=[&](double x,double y){
    const double u=std::max(0.,std::abs(x)-length),v=std::max(0.,std::abs(y)-width);
    return u*u+v*v<=extension*extension;
  };
  for(const auto& p:points){
    const double x=p.x-shift,y=p.y;
    if(x*x+y*y>reach*reach)continue;
    const double u=c*x+s*y,v=c*y-s*x;
    if(std::abs(yaw)>=pi||overlaps(x,y)||overlaps(u,v))return false;
    if((length*y-width*x)*(length*v-width*u)<=0.||
       (length*y+width*x)*(length*v+width*u)<=0.)return false;
  }
  return true;
}

// Translation adds a strip beyond the leading face. Its starting
// footprint is occupied by the chassis or verified by the approach turn.
inline bool shiftFits(const std::vector<ObstaclePoint>& points,double shift,double extension){
  const double length=body_half_length+body_length_margin,width=body_half_width+body_width_margin;
  for(const auto& p:points){
    const double along=shift>0.?p.x:-p.x;
    const double u=std::max({0.,length-along,along-length-std::abs(shift)});
    const double v=std::max(0.,std::abs(p.y)-width);
    if(u*u+v*v<=extension*extension)return false;
  }
  return true;
}

// Distance from the leading face to the first possible base in a straight
// sweep. A disk wholly behind that face recedes from the new swept strip.
inline double translationRoom(const std::vector<ObstaclePoint>& points,int sign,double extension){
  const double length=body_half_length+body_length_margin,width=body_half_width+body_width_margin;
  double room=2.;
  for(const auto& p:points){
    const double x=sign*p.x-length,y=std::max(0.,std::abs(p.y)-width);
    if(y>extension||x+extension<0.||x-extension>=room)continue;
    const double reach=std::sqrt(std::max(0.,extension*extension-y*y));
    if(x+reach>=0.)room=std::min(room,x-reach);
  }
  return std::max(0.,room);
}
}
