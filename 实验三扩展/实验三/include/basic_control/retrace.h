#pragma once
#include <algorithm>
#include <cmath>
namespace lab3 {
// Only the most recent straight, actually traversed segment is eligible.
// This records geometry, not proof that the environment remains unchanged.
class Retrace {
 public:
  void update(double x,double y,double yaw,double forward,double now){
    if(!ready_){px_=x;py_=y;heading_=yaw;ready_=true;return;}
    const double dx=x-px_,dy=y-py_;px_=x;py_=y;
    const double error=std::atan2(std::sin(yaw-heading_),std::cos(yaw-heading_));
    if(std::abs(error)>.05||std::hypot(dx,dy)>.10||now-last_forward_>3){length_=0;heading_=yaw;}
    const double along=dx*std::cos(heading_)+dy*std::sin(heading_);
    const double lateral=-dx*std::sin(heading_)+dy*std::cos(heading_);
    if(std::abs(lateral)>.01){length_=0;heading_=yaw;return;}
    if(along<0)length_=std::max(0.0,length_+along);
    else if(forward>.02&&along>.0001){length_=std::min(.35,length_+along);last_forward_=now;}
  }
  bool allowed(double now)const{return ready_&&now-last_forward_<=3&&length_>.065;}
 private:
  bool ready_=false;
  double px_=0,py_=0,heading_=0,length_=0,last_forward_=-100;
};
}
