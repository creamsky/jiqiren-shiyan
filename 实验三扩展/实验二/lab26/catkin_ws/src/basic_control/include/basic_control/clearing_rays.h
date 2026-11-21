#pragma once
#include "basic_control/scan_alignment.h"

namespace lab26 {
class ClearingRays {
 public:
  void add(double x,double y,double z,double time,bool obstacle){
    const double range=std::hypot(x,y);
    if(range<.28 || range>25.)return;
    const int bin=std::min(359,std::max(0,int((std::atan2(y,x)+pi)*180./pi)));
    if(obstacle){
      // Adjacent angular bins may see opposite sides of the same post.
      for(int d=-1;d<=1;++d){
        auto& limit=bins_[(bin+d+360)%360].limit;limit=std::min(limit,range);
      }
    }
    auto& ray=bins_[bin];
    const double scale=std::abs(z)>.14?.14/std::abs(z):1.;
    if(range*scale>ray.range){ray.range=range*scale;ray.end={{x*scale,y*scale},time};}
  }
  std::vector<TimedReturn> endpoints()const{
    std::vector<TimedReturn> result;
    for(const auto& ray:bins_){
      if(ray.range<=0.)continue;
      const double distance=std::min({ray.range-.07,ray.limit-.07,5.85});
      if(distance<=.35)continue;
      result.push_back({{ray.end.point.x*distance/ray.range,ray.end.point.y*distance/ray.range},ray.end.offset});
    }
    return result;
  }
 private:
  struct Bin{double range=0.,limit=26.;TimedReturn end{};};
  std::array<Bin,360> bins_{};
};
}
