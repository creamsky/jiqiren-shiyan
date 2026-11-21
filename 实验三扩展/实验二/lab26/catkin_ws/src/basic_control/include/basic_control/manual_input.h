#pragma once
#include "motion.h"
#include <sstream>
namespace lab26 {
// One packet replaces both axes atomically. No sticky axis on key release.
inline bool manualInput(const std::string& text,double max_v,double max_w,Command& out) {
  Command next;
  if(text=="w")next.v=std::min(1.0,max_v);
  else if(text=="s")next.v=-std::min(1.0,max_v);
  else if(text=="a")next.w=std::min(0.3,max_w);
  else if(text=="d")next.w=-std::min(0.3,max_w);
  else {
    std::istringstream input(text);std::string verb,extra;double linear,angular,speed,turn;
    if(!(input>>verb>>linear>>angular>>speed>>turn)||(input>>extra)||verb!="drive"||
      !std::isfinite(linear)||!std::isfinite(angular)||!std::isfinite(speed)||!std::isfinite(turn)||
      (linear!=-1&&linear!=0&&linear!=1)||(angular!=-1&&angular!=0&&angular!=1)||
      speed<0.1||speed>max_v||turn<0.05||turn>max_w)return false;
    next={linear*speed,angular*turn};
  }
  out=next;return true;
}
inline Command manualRampOrigin(Command target,Command previous) {
  if(target.v==0||target.v*previous.v<0)previous.v=0;
  if(target.w==0||target.w*previous.w<0)previous.w=0;
  return previous;
}
}
