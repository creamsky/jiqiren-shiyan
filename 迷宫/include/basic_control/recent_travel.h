#pragma once
#include "basic_control/maze_guard.h"
#include "basic_control/rate_limit.h"
#include <deque>

namespace lab26 {
// Measured positions and orientations preserve how the chassis entered a
// pocket, including the rotation between two nearly coincident positions.
class RecentTravel {
  std::deque<Pose> trail_;
  Pose start_{},blockedPose_{};
  double blockedAt_=-1.,lastAttempt_=-1.;
  static double separation(Pose a,Pose b){return std::hypot(a.x-b.x,a.y-b.y);}
  static double yawDifference(Pose a,Pose b){return wrap(a.yaw-b.yaw);}
 public:
  bool active=false;
  void observe(Pose p){
    if(active)return;
    if(!trail_.empty()){
      const double d=separation(p,trail_.back()),a=std::abs(yawDifference(p,trail_.back()));
      if(d<.035&&a<.07)return;
      if(d>.30||a>.50){trail_.clear();blockedAt_=-1.;}
    }
    trail_.push_back(p);
    if(trail_.size()>160)trail_.pop_front();
  }
  void encounter(Pose p,double t){
    if(blockedAt_<0.||t-lastAttempt_>.40||separation(p,blockedPose_)>.03||
       std::abs(yawDifference(p,blockedPose_))>.15){blockedAt_=t;blockedPose_=p;}
    lastAttempt_=t;
    if(t-blockedAt_>=.65&&trail_.size()>2){active=true;start_=p;blockedAt_=-1.;}
  }
  bool step(Pose p,const std::vector<ObstaclePoint>& hits,RayView rays,Command feedback,
            Command previous,double dt,Command& cmd){
    if(separation(start_,p)>.28||std::abs(yawDifference(p,start_))>.55)return active=false;
    while(!trail_.empty()&&separation(p,trail_.back())<.048&&
          std::abs(yawDifference(trail_.back(),p))<.09)trail_.pop_back();
    if(trail_.empty())return active=false;
    // Look back along the contiguous sequence rather than aiming at a
    // nearby parallel aisle or jumping across a corner.
    size_t i=trail_.size()-1;
    while(i>0&&separation(p,trail_[i])<.12&&std::abs(yawDifference(trail_[i],p))<.24)--i;
    const Pose goal=trail_[i];
    const double yaw=yawDifference(goal,p),dx=goal.x-p.x,dy=goal.y-p.y;
    const double mid=p.yaw+.5*yaw,along=dx*std::cos(mid)+dy*std::sin(mid);
    if(std::hypot(dx,dy)>.25||std::abs(yaw)>.48||std::abs(-dx*std::sin(mid)+dy*std::cos(mid))>.045)
      return active=false;
    const double halfDistance=.5*along/std::cos(.25*yaw);
    const Pose halfway{p.x+halfDistance*std::cos(p.yaw+.25*yaw),
                       p.y+halfDistance*std::sin(p.yaw+.25*yaw),mid};
    const double radius=std::hypot(body_half_length,body_half_width);
    const bool onTrail=std::any_of(trail_.begin()+i,trail_.end(),[&](Pose sample){
      return separation(sample,halfway)+radius*std::abs(yawDifference(sample,halfway))<.055;
    });
    if(!onTrail)return active=false;
    const double time=std::max({.35,std::abs(along)/.13,std::abs(yaw)/.42});
    const Command target{along/time,yaw/time};
    const auto next=rateLimit(target,previous,dt,.20,.60);
    const double speed=std::max({std::abs(next.v),std::abs(feedback.v),std::abs(target.v)});
    if(speed>.005&&!observedCorridor(hits,along>=0.?0.:pi,.035+speed*(.25+dt)+speed*speed/.40,rays))
      return active=false;
    // Only the path just occupied by the real chassis has this evidence.
    // Test the current braking arc with the same pose tolerance.
    if(!mazeBrakeClear(hits,feedback,.055)||!mazeBrakeClear(hits,next,.055)||
       !mazeBrakeClear(hits,target,.055))return active=false;
    cmd=next;return true;
  }
};
}
