#pragma once
#include "basic_control/rate_limit.h"
#include "basic_control/route_retreat.h"

namespace lab26 {
using Waypoints=std::vector<ObstaclePoint>; // XY in the map frame

inline ObstaclePoint routeTarget(const Waypoints& route,Pose p,double look){
  // Plans start at the vehicle and are refreshed while moving. Restrict
  // projection to the first metre to avoid jumping across a hairpin.
  ObstaclePoint nearest=route.front();
  size_t next=1;
  double best=std::hypot(nearest.x-p.x,nearest.y-p.y),along=0.;
  for(size_t i=1;i<route.size() && along<1.;++i){
    const auto a=route[i-1],b=route[i];
    const double dx=b.x-a.x,dy=b.y-a.y,len=std::hypot(dx,dy);
    if(len<1e-6)continue;
    const double f=std::max(0.,std::min({1.,(1.-along)/len,((p.x-a.x)*dx+(p.y-a.y)*dy)/(len*len)}));
    const ObstaclePoint projected{a.x+f*dx,a.y+f*dy};
    const double d=std::hypot(p.x-projected.x,p.y-projected.y);
    if(d<best){best=d;nearest=projected;next=i;}
    along+=len;
  }
  for(;next<route.size();++next){
    const auto b=route[next];const double dx=b.x-nearest.x,dy=b.y-nearest.y,len=std::hypot(dx,dy);
    if(len>look)return {nearest.x+look*dx/len,nearest.y+look*dy/len};
    nearest=b;look-=len;
  }
  return nearest;
}

// Distance along a constant-curvature path before the swept rectangular body
// meets a return. Inflate each sample by the largest inter-sample corner travel.
inline double arcClearance(const std::vector<ObstaclePoint>& points,double k,double horizon){
  if(std::abs(k)<1e-7){
    double free=horizon;
    for(auto p:points)if(p.x>-body_half_length && std::abs(p.y)<=body_half_width+body_width_margin)
      free=std::min(free,std::max(0.,p.x-body_half_length-body_length_margin));
    return free;
  }
  const double length=body_half_length+body_length_margin,width=body_half_width+body_width_margin;
  const double radius=std::hypot(length,width);
  if(!std::isfinite(k))return 0.;
  const double travel=std::min(horizon,2*pi/std::abs(k));
  const int steps=std::max(1,int(std::ceil(travel*(1.+radius*std::abs(k))/.045)));
  const double ds=travel/steps,pad=.5*ds*(1.+radius*std::abs(k));
  struct Sample{double x,y,c,s;};
  std::vector<Sample> path;path.reserve(steps+1);
  for(int i=0;i<=steps;++i){
    const double d=i*ds,a=k*d,c=std::cos(a),s=std::sin(a);
    path.push_back({s/k,2*std::pow(std::sin(.5*a),2)/k,c,s});
  }
  int first=steps+1;
  for(const auto& p:points){
    if(std::abs(p.x)>horizon+radius+pad||std::abs(p.y)>horizon+radius+pad)continue;
    for(int i=0;i<first;++i){
      const auto& q=path[i];const double dx=p.x-q.x,dy=p.y-q.y;
      if(std::abs(q.c*dx+q.s*dy)<=length+pad && std::abs(-q.s*dx+q.c*dy)<=width+pad){first=i;break;}
    }
  }
  return first>steps?horizon:std::max(0.,(first-.5)*ds);
}

class PathTracker {
 public:
  const char* state="waiting_route";
  const char* reason="";
  bool finished=false;
  Command step(const Waypoints& route,Pose p,ObstaclePoint goal,const std::vector<ObstaclePoint>& scan,
               RayView rays,Command previous,Command actual,double now,double dt,double acc,double turn_acc){
    const double remaining=std::hypot(goal.x-p.x,goal.y-p.y);
    const bool stopped=std::abs(actual.v)<=.02 && std::abs(actual.w)<=.03;
    if(remaining<=.20)arrived_=true;
    if(arrived_){
      state="settling_goal";reason="";
      if(!stopped)settled_=-1.;
      else if(settled_<0.)settled_=now;
      else if(now-settled_>=.30){finished=true;state="finished";}
      return {};
    }
    if(!(dt>0.) || dt>.25 || !std::isfinite(dt) || brakingBlocked(scan,actual.v,actual.w)){
      state="path_braking";reason="Measured braking path obstructed";return {};
    }
    retreat_.setRays(rays);retreat_.observe(p,previous,actual,now);
    Command target;
    auto track=[&](ObstaclePoint carrot){
      const double dx=carrot.x-p.x,dy=carrot.y-p.y,range=std::hypot(dx,dy);
      const double angle=wrap(std::atan2(dy,dx)-p.yaw);
      if(range>.025){
        const double k=std::abs(angle)<.08 && range*std::abs(std::sin(angle))<.025?0.:2.*std::sin(angle)/range;
        // Pivot when the tangent is far off-heading or an advance would cut
        // the inside corner. Use hysteresis through the last part of the turn.
        const double preview=std::min(1.15,remaining);
        const double free=arcClearance(scan,k,preview);
        const double needed=std::min(range,remaining-.12);
        const bool arc_fits=free>=needed-.001;
        if(std::abs(angle)>(pivoting_?.13:.53) || (!arc_fits && std::abs(angle)>.14)){
          if(pivotClear(scan,angle) && bumperClearance(scan,angle)>=std::min(range,remaining-.12)){
            pivoting_=true;state="pivot_to_path";reason="Pivoting toward the planned route";
            if(std::abs(actual.v)<=.02)target.w=std::copysign(std::min(.55,std::sqrt(1.2*std::abs(angle))),angle);
            return true;
          }
        }else if(arc_fits){
          const double room=std::min(remaining-.10,free-(free<preview?.10:0.));
          const double reaction=acc*avoidance_reaction_time;
          const double safe=std::sqrt(reaction*reaction+2*acc*std::max(0.,room))-reaction;
          const double speed=std::min({.50,safe,.62/std::max(.01,std::abs(k))});
          if(speed>=.09){
            state="path_tracking";reason="";
            if(!pivoting_||std::abs(actual.w)<.10){pivoting_=false;target={speed,k*speed};}
            return true;
          }
        }
      }
      return false;
    };
    const bool on_route=!route.empty() && std::hypot(route.front().x-p.x,route.front().y-p.y)<1.;
    const double look=.42+.65*std::max(0.,actual.v);
    if(branch_ && std::hypot(corner_.x-p.x,corner_.y-p.y)<.18)branch_=false;
    if(branch_){
      const double dx=corner_.x-p.x,dy=corner_.y-p.y;
      if(!observedCorridor(scan,wrap(std::atan2(dy,dx)-p.yaw),std::hypot(dx,dy),rays))branch_=false;
    }
    bool usable=branch_ && track(corner_);
    if(branch_ && !usable)branch_=false;
    if(!usable && on_route)usable=track(routeTarget(route,p,look));
    const double join_range=std::min(remaining,1.7);
    const ObstaclePoint local_join{p.x+(goal.x-p.x)*join_range/remaining,p.y+(goal.y-p.y)*join_range/remaining};
    if(!usable && !on_route){
      const double a=wrap(std::atan2(goal.y-p.y,goal.x-p.x)-p.yaw);
      // The global inflation grid can seal the start next to a rear return.
      // Advance only a measured local prefix, rechecking every tick while
      // the global search continues. Never substitute a straight unobserved path.
      const double prefix=std::min(remaining-.10,std::max(.85,stoppingTravel(actual.v)+.30));
      if(observedCorridor(scan,a,prefix+.07,rays)){
        const double fraction=std::min(look,prefix)/remaining;
        usable=track({p.x+(goal.x-p.x)*fraction,p.y+(goal.y-p.y)*fraction});
        if(usable && target.v>0.)state="forward_escape";
      }
    }
    if(!usable && now>=branch_check_){
      branch_check_=now+.20;
      const auto join=on_route?routeTarget(route,p,1.7):local_join;
      branch_=findCorner(p,join,scan,rays,corner_);
      if(branch_)usable=track(corner_);
    }
    if(usable && branch_ && target.v>0.)state="following_detour";
    if(usable){
      // A restored forward path can terminate backing only at a stopped handover.
      if(retreat_.phase()!=RouteRetreat::Phase::Idle){
        if(!stopped){state="retreat_replan";return {};}
        retreat_.exitSelected(now);
      }
      blocked_=-1.;
    }else{
      state="waiting_route";reason="Waiting for a footprint-clear path";
      if(blocked_<0.)blocked_=now;
      if(!stopped && retreat_.phase()!=RouteRetreat::Phase::Backing)return {};
      if(now-blocked_<.45)return {};
      if(retreat_.phase()!=RouteRetreat::Phase::Backing && now>=retry_){
        retry_=now+.30;
        if(retreat_.phase()==RouteRetreat::Phase::Idle){
          if(!retreat_.begin(p,actual,scan,now))
            retreat_.beginTurnSpace(p,actual,scan,goal.x,goal.y,now);
        }else if(retreat_.canContinue(p,scan))retreat_.resumeBacking(p,now);
      }
      if(retreat_.phase()!=RouteRetreat::Phase::Backing)return {};
      target=retreat_.step(p,actual,scan,now);state="route_retreat";reason=retreat_.reason();
    }
    if(target.v==0. && target.w==0.)return {};
    auto cmd=rateLimit(target,previous,dt,acc,turn_acc);
    if(target.v==0.)cmd.v=0.;
    // Steering and drive acceleration differ: validate the actual next arc,
    // including residual yaw from CAN, before any positive CAN command.
    if(sweptFootprint(scan,cmd.v,cmd.w).blocked){state="path_braking";reason="Swept body or braking path is obstructed";return {};}
    if(cmd.v>0.){
      const double brake=stoppingTravel(std::max(cmd.v,actual.v))+.08;
      if(arcClearance(scan,cmd.w/cmd.v,brake)<brake){state="path_braking";reason="Swept body or braking path is obstructed";return {};}
    }
    return cmd;
  }
 private:
  // Join the same global route beyond the obstruction. Both legs and the
  // rotation between them must fit; an open-looking side ray alone is insufficient.
  static bool findCorner(Pose p,ObstaclePoint join,const std::vector<ObstaclePoint>& scan,RayView rays,ObstaclePoint& result){
    const double dx=join.x-p.x,dy=join.y-p.y,c=std::cos(p.yaw),s=std::sin(p.yaw);
    const double x=c*dx+s*dy,y=-s*dx+c*dy;
    double best=std::numeric_limits<double>::infinity();
    for(double a:{0.,.48,-.48,.78,-.78,1.08,-1.08,1.48,-1.48}){
      if(a!=0. && !pivotClear(scan,a))continue;
      const double ca=std::cos(a),sa=std::sin(a);
      for(double length:{.6,.9,1.2}){
        const double ex=length*ca,ey=length*sa;
        const double out=std::atan2(y-ey,x-ex),next=std::hypot(x-ex,y-ey),bend=wrap(out-a);
        const double cost=length+next+.22*(std::abs(a)+std::abs(bend));
        if(cost>=best || std::abs(bend)>2.1 || !observedCorridor(scan,a,length+.06,rays))continue;
        // Work in the waypoint body frame so the full corner sweep is checked.
        std::vector<ObstaclePoint> at_corner;at_corner.reserve(scan.size());
        for(auto q:scan){
          const double qx=q.x-ex,qy=q.y-ey;
          at_corner.push_back({ca*qx+sa*qy,-sa*qx+ca*qy});
        }
        if(!pivotClear(at_corner,bend) || bumperClearance(at_corner,bend)<next+.06)continue;
        best=cost;result={p.x+c*ex-s*ey,p.y+s*ex+c*ey};
      }
    }
    return std::isfinite(best);
  }
  RouteRetreat retreat_;
  ObstaclePoint corner_{};
  bool branch_=false;
  double branch_check_=-1.;
  bool arrived_=false,pivoting_=false;
  double settled_=-1.,blocked_=-1.,retry_=0.;
};
}
