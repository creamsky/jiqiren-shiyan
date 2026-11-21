#pragma once
#include <limits>
#include "basic_control/motion.h"

namespace lab26 {
// A short observed side passage used once per stalled DWA goal.
class SideExit {
 public:
  enum class Phase { Idle, Turning, Settling, Advancing };
  Phase phase() const { return phase_; }
  void setRays(RayView rays){rays_=rays;}
  double heading() const { return heading_; }
  double length() const { return length_; }
  bool straight() const { return straight_; }
  // Recheck the next section before reaching the end of a straight escape.
  // A changing goal bearing, a corner or lost coverage ends the extension.
  void extendForward(const std::vector<ObstaclePoint>& scan,Pose p,double gx,double gy,double now){
    if(!straight_ || phase_!=Phase::Advancing)return;
    const double travel=(p.x-origin_.x)*std::cos(heading_)+(p.y-origin_.y)*std::sin(heading_);
    if(travel<.30 || std::abs(wrap(heading_-p.yaw))>.06 ||
       std::abs(wrap(std::atan2(gy-p.y,gx-p.x)-p.yaw))>.14)return;
    const double reach=std::min(.65,std::hypot(gx-p.x,gy-p.y)-.12);
    if(reach<=length_-travel+.10 || !observedCorridor(scan,0.,reach+.06,rays_))return;
    length_=reach;origin_=p;best_distance_=0.;started_=last_progress_=now;
  }

  bool selectForward(const std::vector<ObstaclePoint>& scan,Pose p,double gx,double gy,double now,
                     const std::vector<double>& tried){
    const double bearing=wrap(std::atan2(gy-p.y,gx-p.x)-p.yaw);
    if(std::abs(bearing)>.65 || std::any_of(tried.begin(),tried.end(),[&](double yaw){
        return std::abs(wrap(yaw-p.yaw))<.55;
    }))return false;
    const double goal_range=std::hypot(gx-p.x,gy-p.y);
    if(goal_range<=.20)return false;
    const double space=bumperClearance(scan,0.);
    double reach=std::min({.65,goal_range,space-.06});
    // A short, observable motion can escape a blocked turning envelope.
    // Preserve braking room for the minimum effective speed.
    const double min_reach=.04+stoppingTravel(.10);
    while(reach>=min_reach){
      if(observedCorridor(scan,0.,reach,rays_) && !sweptFootprint(scan,.10,0.).blocked){
        phase_=Phase::Advancing;straight_=require_observations_=true;heading_=p.yaw;
        length_=reach;origin_=p;started_=last_progress_=now;best_distance_=0.;return true;
      }
      reach-=.05;
    }
    return false;
  }

  bool select(const std::vector<ObstaclePoint>& scan,Pose p,double gx,double gy,double now,
              double failed_heading=std::numeric_limits<double>::quiet_NaN(),
              const std::vector<double>& tried={}) {
    phase_=Phase::Idle;
    straight_=false;
    require_observations_=std::isfinite(failed_heading);
    const double remaining=std::hypot(gx-p.x,gy-p.y);
    if(remaining<=.30)return false;
    const double bearing=wrap(std::atan2(gy-p.y,gx-p.x)-p.yaw);
    if(!std::isfinite(failed_heading) && std::abs(bearing)<.25 && freeAhead(scan,bearing)>std::min(.65,remaining)+.06)return false;
    double best=-1e9;
    for(double angle:{.52,-.52,.79,-.79,1.05,-1.05,1.57,-1.57}){
      if(std::any_of(tried.begin(),tried.end(),[&](double yaw){
          return std::abs(wrap(p.yaw+angle-yaw))<.75;
      }))continue;
      if(std::isfinite(failed_heading) && std::abs(wrap(p.yaw+angle-failed_heading))<.70)continue;
      if(!pivotClear(scan,angle))continue;
      const double reach=std::min({remaining,.65,freeAhead(scan,angle)-.06});
      if(reach<.25 || (require_observations_ && !observedCorridor(scan,angle,reach,rays_)))continue;
      const double yaw=p.yaw+angle;
      const double end_distance=std::hypot(gx-p.x-reach*std::cos(yaw),gy-p.y-reach*std::sin(yaw));
      const double score=remaining-end_distance+.85*reach-.12*std::abs(angle);
      if(score>best){best=score;heading_=yaw;length_=reach;phase_=Phase::Turning;}
    }
    origin_=p;started_=last_progress_=now;best_error_=std::abs(wrap(heading_-p.yaw));best_distance_=0.;
    return phase_!=Phase::Idle;
  }

  Command step(const std::vector<ObstaclePoint>& scan,Pose p,Command feedback,double now) {
    if(phase_==Phase::Idle)return {};
    const double error=wrap(heading_-p.yaw);
    const double travel=(p.x-origin_.x)*std::cos(heading_)+(p.y-origin_.y)*std::sin(heading_);
    if(now-started_>10.)return finish();
    if(require_observations_ && !observedCorridor(scan,straight_?0.:error,std::min(.20,std::max(0.,length_-travel)),rays_))return finish();
    if(phase_==Phase::Turning){
      if(best_error_-std::abs(error)>.04){best_error_=std::abs(error);last_progress_=now;}
      if(!pivotClear(scan,error)||freeAhead(scan,error)<.25||now-last_progress_>2.)return finish();
      if(std::abs(error)>.08){
        if(std::abs(feedback.v)>.02)return {};
        return {0.,std::copysign(std::min(.50,std::sqrt(2*.60*(std::abs(error)-.06))),error)};
      }
      phase_=Phase::Settling;last_progress_=now;
    }
    if(phase_==Phase::Settling){
      if(now-last_progress_>2.)return finish();
      if(std::abs(feedback.w)>.04||std::abs(feedback.v)>.02)return {};
      if(std::abs(error)>.15)return finish();
      origin_=p;phase_=Phase::Advancing;last_progress_=now;return {};
    }
    if(travel-best_distance_>.025){best_distance_=travel;last_progress_=now;}
    const double left=std::min(length_-travel,freeAhead(scan,0.)-.06);
    if(left<=.04||std::abs(error)>.22||now-last_progress_>2.)return finish();
    if(straight_ && std::abs(error)>.08)return finish();
    const Command target{std::min(.20,std::sqrt(.4*(left-.04))),straight_?0.:clip(error,.18)};
    if(sweptFootprint(scan,target.v,target.w).blocked)return finish();
    return target;
  }
 private:
  RayView rays_;
  Command finish(){phase_=Phase::Idle;return {};}
  static double freeAhead(const std::vector<ObstaclePoint>& scan,double angle){
    return bumperClearance(scan,angle);
  }
  Phase phase_=Phase::Idle;
  bool require_observations_=false,straight_=false;
  Pose origin_;
  double heading_=0,length_=0,started_=0,last_progress_=0,best_error_=0,best_distance_=0;
};
}
