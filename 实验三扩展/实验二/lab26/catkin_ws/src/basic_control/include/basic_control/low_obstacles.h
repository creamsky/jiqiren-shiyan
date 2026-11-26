#pragma once
#include "basic_control/scan_alignment.h"
#include <map>
#include <tuple>

namespace lab26 {
// A downward return can disappear underneath the centered lidar on approach.
// Keep local hits in map coordinates. Only a ray through the recorded height
// may clear one, on two different stationary scans with no matching hit.
class LowObstacles {
 public:
  void clear(){cells_.clear();last_stamp_=-1.;}
  void update(const ObstacleScan& scan,const std::vector<ObstaclePoint>& aligned,Pose pose,bool still){
    if(scan.start<=last_stamp_)return;
    last_stamp_=scan.start;
    const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
    for(size_t i=0;i<aligned.size();++i){
      const auto q=aligned[i];const double z=scan.returns[i].z;
      if(z>=-.14 || std::hypot(q.x,q.y)>3.)continue;
      const double x=pose.x+c*q.x-s*q.y,y=pose.y+s*q.x+c*q.y;
      auto& cell=cells_[std::make_tuple(int(std::floor(x/.04)),int(std::floor(y/.04)),int(std::floor(z/.04)))];
      if(cell.seen>=0.&&scan.start>cell.seen&&scan.start-cell.seen<=.40)cell.confirmed=true;
      cell.x=x;cell.y=y;cell.z=z;cell.seen=scan.start;cell.misses=0;
    }
    std::array<std::vector<Beam>,360> beams;
    if(still)for(const auto& ray:scan.lower_rays){
      const double a=std::atan2(ray.point.y,ray.point.x),r=std::hypot(ray.point.x,ray.point.y);
      if(r>.4)beams[bin(a)].push_back({a,r,ray.z/r});
    }
    for(auto it=cells_.begin();it!=cells_.end();){
      auto& cell=it->second;
      const double dx=cell.x-pose.x,dy=cell.y-pose.y,r=std::hypot(dx,dy);
      if(r>3.){it=cells_.erase(it);continue;}
      if(still && cell.seen!=scan.start){
        const double angle=wrap(std::atan2(dy,dx)-pose.yaw);
        bool crossed=false;
        for(int d=-1;d<=1 && !crossed;++d)for(const auto& beam:beams[(bin(angle)+d+360)%360]){
          if(beam.range>r+.10 && r*std::abs(wrap(beam.angle-angle))<.015 &&
             std::abs(r*beam.slope-cell.z)<.02){crossed=true;break;}
        }
        if(crossed && ++cell.misses>=2){it=cells_.erase(it);continue;}
      }
      ++it;
    }
  }
  void append(Pose pose,std::vector<ObstaclePoint>& points,bool requireRepeat=false)const{
    const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
    for(const auto& entry:cells_){
      const auto& cell=entry.second;
      if(requireRepeat&&!cell.confirmed&&last_stamp_-cell.seen>.40)continue;
      const double dx=cell.x-pose.x,dy=cell.y-pose.y;
      points.push_back({c*dx+s*dy,-s*dx+c*dy,true});
    }
  }
 private:
  struct Beam{double angle,range,slope;};
  struct Cell{double x=0.,y=0.,z=0.,seen=-1.;int misses=0;bool confirmed=false;};
  static int bin(double angle){return std::min(359,std::max(0,int((angle+pi)*180./pi)));}
  std::map<std::tuple<int,int,int>,Cell> cells_;
  double last_stamp_=-1.;
};
}
