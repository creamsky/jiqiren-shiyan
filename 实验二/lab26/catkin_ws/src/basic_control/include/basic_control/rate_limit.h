#pragma once
#include "basic_control/motion.h"

namespace lab26 {
inline Command rateLimit(Command target,Command previous,double dt,double linear_rate,double angular_rate) {
  if(!std::isfinite(dt)||dt<=0) return previous;
  return {previous.v+clip(target.v-previous.v,linear_rate*dt),
          previous.w+clip(target.w-previous.w,angular_rate*dt)};
}
}
