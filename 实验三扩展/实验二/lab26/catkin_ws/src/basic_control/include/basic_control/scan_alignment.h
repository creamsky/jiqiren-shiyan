#pragma once
#include "basic_control/motion.h"
#include <deque>

namespace lab26 {
struct TimedPose { double stamp; Pose pose; };
struct TimedReturn { ObstaclePoint point; double offset; bool mark=true; double z=0.; };
struct ObstacleScan {
  double start=0, duration=0;
  std::vector<TimedReturn> returns;
  std::vector<TimedReturn> clearing;
  std::vector<TimedReturn> lower_rays;
};

class ScanAlignment {
 public:
  // FAST-LIO and the lidar driver can round the end of the same scan differently.
  static bool awaitingPose(double scan_end,double pose_stamp){return scan_end-pose_stamp>.0001;}
  void clear(){poses.clear();}
  bool stillDuring(double start,double end)const{
    const Pose ref=at(start);
    const auto near=[&](Pose p){return std::hypot(p.x-ref.x,p.y-ref.y)<.008 && std::abs(wrap(p.yaw-ref.yaw))<.01;};
    for(const auto& sample:poses)if(sample.stamp>start && sample.stamp<end && !near(sample.pose))return false;
    return near(at(end));
  }
  void append(double stamp,Pose p){
    if(!poses.empty() && stamp==poses.back().stamp)return;
    if(!poses.empty() && (stamp<poses.back().stamp || stamp-poses.back().stamp>.25))clear();
    poses.push_back({stamp,p});
    while(poses.size()>2 && stamp-poses[1].stamp>1.)poses.pop_front();
  }
  bool align(const ObstacleScan& scan,std::vector<ObstaclePoint>& output,
             std::vector<ObstaclePoint>* clear=nullptr,Pose* reference=nullptr) const {
    const double end=scan.start+scan.duration;
    if(poses.empty() || !std::isfinite(end) || scan.duration<0 || scan.duration>.25 ||
       scan.start<poses.front().stamp || awaitingPose(end,poses.back().stamp))return false;
    const Pose ref=at(end);
    if(reference)*reference=ref;
    const double c=std::cos(ref.yaw),s=std::sin(ref.yaw);
    struct Transform { double x,y,c,s; };
    std::vector<Transform> bins;
    const int last=int(std::ceil(scan.duration*1000));
    for(int i=0;i<=last;++i){
      const Pose p=at(std::min(end,scan.start+i*.001));
      const double dx=p.x-ref.x,dy=p.y-ref.y,a=p.yaw-ref.yaw;
      bins.push_back({c*dx+s*dy,-s*dx+c*dy,std::cos(a),std::sin(a)});
    }
    const auto transform=[&](const std::vector<TimedReturn>& input,std::vector<ObstaclePoint>& out){
      out.clear();out.reserve(input.size());
      for(const auto& r:input){
        if(!std::isfinite(r.offset)||r.offset<0||r.offset>scan.duration)return false;
        const auto& tf=bins[std::min(last,int(std::lround(r.offset*1000)))];
        out.push_back({tf.x+tf.c*r.point.x-tf.s*r.point.y,tf.y+tf.s*r.point.x+tf.c*r.point.y});
      }
      return true;
    };
    return transform(scan.returns,output) && (!clear || transform(scan.clearing,*clear));
  }
 private:
  std::deque<TimedPose> poses;
  Pose at(double t) const {
    if(t>=poses.back().stamp)return poses.back().pose;
    auto b=std::lower_bound(poses.begin(),poses.end(),t,[](const TimedPose& p,double v){return p.stamp<v;});
    if(b==poses.begin()||b->stamp==t)return b->pose;
    const auto& a=*(b-1);
    const double f=(t-a.stamp)/(b->stamp-a.stamp);
    return {a.pose.x+f*(b->pose.x-a.pose.x),a.pose.y+f*(b->pose.y-a.pose.y),
            a.pose.yaw+f*wrap(b->pose.yaw-a.pose.yaw)};
  }
};
}
