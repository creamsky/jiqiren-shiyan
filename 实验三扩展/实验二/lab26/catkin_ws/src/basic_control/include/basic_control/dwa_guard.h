#pragma once
#include <array>
#include "basic_control/rate_limit.h"

namespace lab26 {
struct GuardedCommand {
  Command command;
  const char* reason;
};

// Reserve turning room only for a forward approach which is still pointing
// almost straight into a return. A wall alongside the body is outside this lane.
inline bool approachingObstacle(Command request,Command measured,double gap,double goal_distance){
  if(request.v<=0. || std::abs(request.w)>.50*request.v || measured.v<-.02)return false;
  return goal_distance>gap+.20 && gap<=.16+stoppingTravel(std::max(request.v,measured.v));
}

inline Command approachSpeed(Command request,double gap){
  // Leave a nose margin after reaction and braking; solve stoppingTravel(v)
  // for the available distance. Very short residual moves cannot steer the base.
  const double reaction=avoidance_linear_deceleration*avoidance_reaction_time;
  const double speed=std::sqrt(reaction*reaction+2.*avoidance_linear_deceleration*std::max(0.,gap-.12))-reaction;
  if(speed>=request.v)return request;
  if(speed<.10)return {};
  return {speed,request.w*(speed/request.v)};
}

inline GuardedCommand guardDwa(const std::vector<ObstaclePoint>& points,
                               Command target, Command previous, Command actual,
                               double dt, double linear_rate, double angular_rate,
                               bool goal_ahead=false,RayView rays={}) {
  if (target.v == 0 && target.w == 0) return {{}, ""};
  if (!(dt > 0) || !std::isfinite(dt)) return {{}, "Invalid control interval"};
  const auto desired = rateLimit(target, previous, dt, linear_rate, angular_rate);
  if (brakingBlocked(points, actual.v, actual.w))
    return {{}, "Measured braking path obstructed"};
  if (!sweptFootprint(points, desired.v, desired.w).blocked) return {desired, ""};

  // Keep the curvature and both acceleration bounds. Check the fastest
  // candidate first, down to the lowest reachable speed on this arc.
  double minimum = 0;
  for (const auto& axis : {std::array<double,3>{desired.v, previous.v, linear_rate},
                           std::array<double,3>{desired.w, previous.w, angular_rate}}) {
    if (axis[0] != 0)
      minimum = std::max(minimum, axis[1]/axis[0] - axis[2]*dt/std::abs(axis[0]));
  }
  for (int i=1; i<=4; ++i) {
    const double scale = 1 - (1-minimum)*i/4;
    if (scale <= 0 || scale >= 1) continue;
    const Command slower{scale*desired.v, scale*desired.w};
    if (!sweptFootprint(points, slower.v, slower.w).blocked)
      return {slower, "Slowing for proposed braking path"};
  }
  // A rear return may forbid a slight turn while allowing translation away
  // from it. Only straighten forward travel on an observed, goal-facing lane.
  if(goal_ahead && target.v>0 && desired.v>0 && previous.v>=0 && actual.v>=0 && std::abs(target.w)<=.5*target.v &&
     std::abs(previous.w)<=angular_rate*dt && std::abs(actual.w)<1e-8 &&
     observedCorridor(points,0.,std::max(.40,stoppingTravel(desired.v)+.16),rays) &&
     !sweptFootprint(points,desired.v,0.).blocked)
    return {{desired.v,0.},"Straightening in observed forward corridor"};
  return {{}, "No clear braking path in reachable velocity window"};
}
}
