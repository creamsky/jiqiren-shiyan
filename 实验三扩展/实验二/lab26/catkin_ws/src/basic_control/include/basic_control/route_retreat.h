#pragma once
#include "basic_control/motion.h"
#include "basic_control/side_exit.h"

namespace lab26 {

// An ordered stack preserves the route through self-intersections. Each edge
// was observed during forward chassis motion or an in-place turn on this goal.
class RouteRetreat {
 public:
  enum class Phase { Idle, Backing, SeekExit, Aborted };
  Phase phase() const { return phase_; }
  void setRays(RayView rays){rays_=rays;}
  double backed() const { return backed_; }
  double failedHeading() const { return approach_yaw_; }
  const char* reason() const { return reason_; }

  void observe(Pose p,Command sent,Command actual,double now){
    if(phase_!=Phase::Idle)return;
    if(last_observation_>=0 && (now<last_observation_ || now-last_observation_>.5))trail_.clear();
    last_observation_=now;
    const bool forward=sent.v>.015 && actual.v>.015;
    const bool stopped_forward=was_forward_ && !forward;
    was_forward_=forward;
    if(trail_.empty()){trail_.push_back({p,false});return;}
    const Pose q=trail_.back().pose;
    const double gap=distance(p,q),dyaw=wrap(p.yaw-q.yaw);
    if(gap>.13 || std::abs(dyaw)>.35){resetTrail(p);return;}
    if(actual.v<-.015){
      // A normal planner short-reverse consumes traversed edges as well.
      while(trail_.size()>1 && !trail_.back().pivot){
        const Pose prev=trail_[trail_.size()-2].pose;
        if(distance(p,prev)>distance(p,trail_.back().pose))break;
        trail_.pop_back();
      }
      if(distance(p,trail_.back().pose)>.065)resetTrail(p);
      return;
    }
    const bool turning=std::abs(sent.w)>.035 && std::abs(actual.w)>.035;
    // Preserve the last partial translation when the chassis stops to turn.
    if((forward && gap>=.035) || (stopped_forward && gap>.008 && std::abs(dyaw)<.18)){
      const double along=(p.x-q.x)*std::cos(q.yaw)+(p.y-q.y)*std::sin(q.yaw);
      if(along<.7*gap){resetTrail(p);return;}
      trail_.push_back({p,false});
    }else if(turning && !forward && std::abs(dyaw)>=.09 && gap<=.035){
      trail_.push_back({p,true});
    }else if(gap>.065 || std::abs(dyaw)>.20)resetTrail(p);
  }

  std::vector<double> triedAt(Pose p)const{
    std::vector<double> tried;
    for(const auto& branch:branches_)if(distance(branch,p)<.40)tried.push_back(branch.yaw);
    return tried;
  }

  void remember(Pose p,double heading){
    for(double old:triedAt(p))if(std::abs(wrap(heading-old))<.55)return;
    p.yaw=heading;branches_.push_back(p);
  }

  bool begin(Pose p,Command actual,const std::vector<ObstaclePoint>& scan,double now){
    if(phase_!=Phase::Idle || std::abs(actual.v)>.02 || std::abs(actual.w)>.03)return false;
    if(trail_.size()<2 || last_observation_<0 || now-last_observation_>.5 ||
       distance(p,trail_.back().pose)>.07 || std::abs(wrap(p.yaw-trail_.back().pose.yaw))>.18 ||
       std::none_of(trail_.begin()+1,trail_.end(),[](const Node& n){return !n.pivot;})){
      reason_="No continuous measured approach to retrace";return false;
    }
    if(!observedCorridor(scan,pi,rearLook(actual),rays_)){reason_=rearReason(scan,actual);return false;}
    if(brakingBlocked(scan,actual.v,actual.w))return false;
    start_=last_pose_=p;last_step_=last_progress_=now;backed_=0.;
    turn_space_=0.;approach_yaw_=trail_.back().pose.yaw;phase_=Phase::Backing;
    reason_="Retracing the measured approach";return true;
  }

  bool beginTurnSpace(Pose p,Command actual,const std::vector<ObstaclePoint>& scan,double gx,double gy,double now){
    if(phase_!=Phase::Idle || std::abs(actual.v)>.02 || std::abs(actual.w)>.03)return false;
    for(auto old:attempts_)if(distance(old,p)<.30 && std::abs(wrap(old.yaw-p.yaw))<.65)return false;
    const double span=2*std::hypot(body_half_length+body_length_margin,body_half_width+body_width_margin);
    for(double d=.10;d<=span;d+=.10){
      if(!observedCorridor(scan,pi,d+.06,rays_))continue;
      const Pose stop{p.x-d*std::cos(p.yaw),p.y-d*std::sin(p.yaw),p.yaw};
      auto projected=scan;
      for(auto& point:projected)point.x+=d;
      SideExit exit;
      RayView shifted=rays_;shifted.x+=d;exit.setRays(shifted);
      if(!exit.select(projected,stop,gx,gy,now,p.yaw,triedAt(stop)))continue;
      attempts_.push_back(p);trail_.clear();start_=last_pose_=p;last_step_=last_progress_=now;
      backed_=progress_mark_=0.;turn_space_=d;approach_yaw_=p.yaw;phase_=Phase::Backing;
      reason_="Backing to an observed turning space";return true;
    }
    return false;
  }

  Command step(Pose p,Command actual,const std::vector<ObstaclePoint>& scan,double now){
    if(phase_!=Phase::Backing)return {};
    const double dt=now-last_step_,travel=distance(p,last_pose_);
    if(dt<0 || dt>.5 || travel>.035+.18*dt ||
       std::abs(wrap(p.yaw-last_pose_.yaw))>.07+.65*dt)
      return abort("Retreat departed from recorded approach");
    if(actual.v<-.012)backed_+=travel;
    last_pose_=p;last_step_=now;
    if(turn_space_>0.){
      const double c=std::cos(start_.yaw),s=std::sin(start_.yaw);
      const double dx=start_.x-p.x,dy=start_.y-p.y,along=dx*c+dy*s,left=turn_space_-along;
      if(std::abs(-dx*s+dy*c)>.035 || std::abs(wrap(p.yaw-start_.yaw))>.08 || along<-.03)
        return abort("Retreat departed from recorded approach");
      if(left<=.015){forkSeen();return {};}
      if(along>progress_mark_+.015){progress_mark_=along;last_progress_=now;}
      if(now-last_progress_>1.5)return abort("No measured retreat progress");
      if(!observedCorridor(scan,pi,std::max(rearLook(actual),left+.06),rays_))
        return hold("Observed reverse corridor changed");
      if(brakingBlocked(scan,actual.v,actual.w))return hold("Measured braking path obstructed");
      return {-std::min(.10,std::sqrt(.4*left)),0.};
    }
    while(trail_.size()>1){
      const Node& tip=trail_.back();
      const Pose target=trail_[trail_.size()-2].pose;
      const double error=wrap(target.yaw-p.yaw);
      const double remaining=distance(p,target);
      // Consume only the immediate predecessor, including intermediate pivots.
      if(tip.pivot ? (remaining<=.058 && std::abs(error)<=.055) : remaining<=.025){
        trail_.pop_back();last_progress_=now;continue;
      }
      if(now-last_progress_>1.5)return abort("No measured retreat progress");
      if(remaining>.15 || distance(p,tip.pose)>.15)return abort("Retreat departed from recorded approach");
      approach_yaw_=tip.pose.yaw;
      if(!observedCorridor(scan,pi,rearLook(actual),rays_))return hold(rearReason(scan,actual));
      if(brakingBlocked(scan,actual.v,actual.w))return hold("Measured braking path obstructed");
      Command cmd;
      if(tip.pivot){
        if(remaining>.058)return abort("Retreat departed from recorded approach");
        if(std::abs(actual.v)>.02)return {};
        if(!pivotClear(scan,error))return hold("Recorded turn is obstructed");
        cmd.w=std::copysign(std::min(.40,std::sqrt(1.2*std::abs(error))),error);
      }else{
        // Aim farther down the same connected polyline for smooth steering.
        // A recorded pivot is a mandatory waypoint. Never cut across it.
        Pose look=target;
        for(size_t i=trail_.size()-2;i>0 && !trail_[i].pivot;--i){
          const Pose next=trail_[i-1].pose;
          if(distance(p,next)>.18 || std::abs(wrap(next.yaw-tip.pose.yaw))>.24)break;
          look=next;
        }
        const double c=std::cos(tip.pose.yaw),s=std::sin(tip.pose.yaw);
        if(std::abs(-(p.x-tip.pose.x)*s+(p.y-tip.pose.y)*c)>.055 ||
           std::abs(wrap(tip.pose.yaw-p.yaw))>.28)return abort("Retreat departed from recorded approach");
        const double bearing=std::atan2(p.y-look.y,p.x-look.x);
        const double steering=wrap(bearing-p.yaw);
        if(std::abs(steering)>.65)return abort("Retreat departed from recorded approach");
        cmd.v=-std::min(.10,std::sqrt(.4*distance(p,look)));
        cmd.w=std::abs(steering)<.015?0.:clip(1.5*steering,.32);
      }
      if(sweptFootprint(scan,cmd.v,cmd.w).blocked)return hold("Recorded turn is obstructed");
      reason_="Retracing the measured approach";return cmd;
    }
    phase_=Phase::SeekExit;reason_="Reached the start of the recorded route";return {};
  }

  void forkSeen(){phase_=Phase::SeekExit;reason_="Backing stopped at a possible side exit";}
  // Called after a settled rescan, only for transient obstructions or a fork
  // that vanished between discovery and braking. Keep our position on the stack.
  bool canContinue(Pose p,const std::vector<ObstaclePoint>& scan)const{
    if(!(transient_ || phase_==Phase::SeekExit) ||
       distance(p,last_pose_)>.045 || std::abs(wrap(p.yaw-last_pose_.yaw))>.10 ||
       !observedCorridor(scan,pi,rearLook({}),rays_))return false;
    if(turn_space_>0.){
      const double left=turn_space_-((start_.x-p.x)*std::cos(start_.yaw)+(start_.y-p.y)*std::sin(start_.yaw));
      return left>.015 && observedCorridor(scan,pi,left+.06,rays_);
    }
    if(trail_.size()<2)return false;
    const auto& tip=trail_.back();
    return !tip.pivot || pivotClear(scan,wrap(trail_[trail_.size()-2].pose.yaw-p.yaw));
  }
  void resumeBacking(Pose p,double now){
    last_pose_=p;last_progress_=last_step_=now;phase_=Phase::Backing;transient_=false;
    reason_="Retracing the measured approach";
  }
  void cancel(){
    phase_=Phase::Idle;reason_="";transient_=was_forward_=false;
    trail_.clear();branches_.clear();attempts_.clear();turn_space_=0.;last_observation_=-1;
  }
  void exitSelected(double now){phase_=Phase::Idle;turn_space_=0.;last_observation_=now;transient_=false;}
  bool formerApproachOpen(const std::vector<ObstaclePoint>& scan,Pose p,double gx,double gy)const{
    const double bearing=std::atan2(gy-p.y,gx-p.x);
    const double along=(start_.x-p.x)*std::cos(p.yaw)+(start_.y-p.y)*std::sin(p.yaw);
    const double cross=-(start_.x-p.x)*std::sin(p.yaw)+(start_.y-p.y)*std::cos(p.yaw);
    if(std::abs(wrap(bearing-p.yaw))>.25 || std::abs(cross)>.10 || along<-.05)return false;
    return observedCorridor(scan,0.,std::max(0.,along)+.81,rays_);
  }

 private:
  RayView rays_;
  struct Node {Pose pose;bool pivot;};
  static double distance(Pose a,Pose b){return std::hypot(a.x-b.x,a.y-b.y);}
  static double rearLook(Command actual){
    const double speed=std::max(.10,std::abs(actual.v));
    return .06+stoppingTravel(speed);
  }
  static const char* rearReason(const std::vector<ObstaclePoint>& scan,Command actual){
    return bumperClearance(scan,pi)<=rearLook(actual)?"Insufficient rear braking clearance":
                                                       "Rear corridor lacks covering returns";
  }
  void resetTrail(Pose p){trail_.clear();trail_.push_back({p,false});}
  Command abort(const char* why){phase_=Phase::Aborted;reason_=why;transient_=false;return {};}
  Command hold(const char* why){abort(why);transient_=true;return {};}
  std::vector<Node> trail_;
  std::vector<Pose> branches_,attempts_;
  double turn_space_=0.,progress_mark_=0.;
  Pose start_,last_pose_;
  double last_observation_=-1.,last_step_=0.,last_progress_=0.,backed_=0.,approach_yaw_=0.;
  Phase phase_=Phase::Idle;
  bool transient_=false,was_forward_=false;
  const char* reason_="";
};
}
