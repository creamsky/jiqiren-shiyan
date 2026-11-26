#pragma once
#include "basic_control/maze_guard.h"
#include "basic_control/rate_limit.h"

namespace lab26 {
// Sparse passage marks are updated by measured travel, never by a planned turn.
class PassageMemory {
  struct Node {double x,y;std::vector<std::pair<double,unsigned>> passages;};
  std::vector<Node> nodes_;
  size_t current_=0;
  static double difference(double a,double b){return std::abs(std::atan2(std::sin(a-b),std::cos(a-b)));}
  static void mark(Node& node,double bearing){
    for(auto& edge:node.passages)if(difference(edge.first,bearing)<.78){edge.second=std::min(2u,edge.second+1);return;}
    node.passages.emplace_back(bearing,1);
  }
 public:
  void observe(Pose p){
    if(nodes_.empty()){nodes_.push_back({p.x,p.y,{}});return;}
    const auto& from=nodes_[current_];
    if(std::hypot(p.x-from.x,p.y-from.y)<1.05)return;
    size_t next=nodes_.size();double nearest=.68;
    for(size_t i=0;i<nodes_.size();++i){
      const double d=std::hypot(p.x-nodes_[i].x,p.y-nodes_[i].y);
      if(d<nearest){nearest=d;next=i;}
    }
    const double bearing=std::atan2(p.y-from.y,p.x-from.x);
    mark(nodes_[current_],bearing);
    if(next==nodes_.size())nodes_.push_back({p.x,p.y,{}});
    mark(nodes_[next],bearing+pi);current_=next;
  }
  unsigned visits(Pose p,double heading)const{
    if(nodes_.empty()||std::hypot(p.x-nodes_[current_].x,p.y-nodes_[current_].y)>.65)return 0;
    unsigned count=0;
    for(const auto& edge:nodes_[current_].passages)if(difference(edge.first,heading)<.78)count=std::max(count,edge.second);
    return count;
  }
};

// Follow observed gaps in the current body frame. Keep a turn target across scans
// so a symmetric dead end cannot flip the preferred side at each update.
class GapRoam {
  double next_scan_=0.,heading_=0.,turn_sign_=1.;
  bool turning_=false,backing_=false,chosen_=false;
  Pose back_start_{};
  PassageMemory memory_;
  double next_window_=0.;
  double cruise_=.18;
  Command window_{};
  static double angle(double x){return std::atan2(std::sin(x),std::cos(x));}
  // Sample the acceleration-limited velocity window and roll out each admissible
  // arc against the inflated rectangular body. Emergency braking runs every tick.
  static Command dynamicWindow(const std::vector<ObstaclePoint>& scan,Command actual,
                               Command desired,double bearing,double pad){
    const double loV=std::max(-.10,actual.v-.20*.25),hiV=std::min(.18,actual.v+.20*.25);
    const double loW=std::max(-.38,actual.w-.60*.25),hiW=std::min(.38,actual.w+.60*.25);
    if(loV>hiV||loW>hiW)return {};
    const auto bound=[](double x,double lo,double hi){return std::max(lo,std::min(hi,x));};
    std::vector<ObstaclePoint> near;
    // Every point lies within 2.9 cm of its 4 cm cell centre.
    std::array<bool,64*64> seen{};
    for(const auto& p:scan){
      if(std::abs(p.x)>=1.27||std::abs(p.y)>=1.27)continue;
      const int x=int(std::floor(p.x/.04))+32,y=int(std::floor(p.y/.04))+32;
      if(!seen[y*64+x]){seen[y*64+x]=true;near.push_back({(x-31.5)*.04,(y-31.5)*.04});}
    }
    Command best{};double bestScore=-1e9;
    for(double v:{loV,(loV+hiV)*.5,hiV,bound(desired.v,loV,hiV),0.}){
      if(v<loV||v>hiV||(desired.v==0.?v!=0.:v*desired.v<=0.))continue;
      if(desired.v>0.&&v>std::max(desired.v,loV))continue;
      for(double w:{loW,(loW+hiW)*.5,hiW,bound(desired.w,loW,hiW),0.}){
        if(w<loW||w>hiW||(desired.v<0.&&w!=0.)||(desired.v==0.&&w*desired.w<=0.))continue;
        double x=0.,y=0.,yaw=0.,gap=.45;
        // Bound body corner travel between samples, including the quantization error.
        const double margin=pad+.029+.10*(std::abs(v)+.46*std::abs(w));
        for(int i=0;i<8&&gap>0.;++i){
          x+=.20*v*std::cos(yaw+.10*w);y+=.20*v*std::sin(yaw+.10*w);yaw+=.20*w;
          const double c=std::cos(yaw),s=std::sin(yaw);
          for(const auto& p:near){
            const double dx=p.x-x,dy=p.y-y;
            const double u=std::max(0.,std::abs(c*dx+s*dy)-body_half_length-body_length_margin);
            const double z=std::max(0.,std::abs(-s*dx+c*dy)-body_half_width-body_width_margin);
            gap=std::min(gap,std::hypot(u,z)-margin);
            if(gap<=0.)break;
          }
        }
        if(gap<=0.)continue;
        const double progress=desired.v<0.?-x:x*std::cos(bearing)+y*std::sin(bearing);
        const double score=3.*progress-std::abs(angle(bearing-yaw))+.5*gap-.18*std::abs(w-desired.w);
        if(score>bestScore){bestScore=score;best={v,w};}
      }
    }
    return best;
  }
  static bool turnSpace(const std::vector<ObstaclePoint>& points,RayView rays,double extension){
    const double radius=std::hypot(body_half_length+body_length_margin,body_half_width+body_width_margin)+extension;
    for(const auto& p:points)if(std::hypot(p.x,p.y)<=radius)return false;
    // A full pivot needs observed space on both flanks and behind the vehicle.
    std::array<bool,36> seen{};
    const double c=std::cos(rays.yaw),s=std::sin(rays.yaw);
    for(const auto& p:rays.scan->ends){
      if(std::hypot(p.x,p.y)<radius+std::hypot(rays.x,rays.y))continue;
      const double a=std::atan2(rays.y+s*p.x+c*p.y,rays.x+c*p.x-s*p.y);
      seen[std::min(35,int((a+pi)*18/pi))]=true;
    }
    return std::all_of(seen.begin(),seen.end(),[](bool v){return v;});
  }
 public:
  const char* state="roam_wait";
  Command step(Pose pose,const std::vector<ObstaclePoint>& points,RayView rays,
               Command measured,Command previous,double t,double dt,double extension){
    state="roam_wait";
    memory_.observe(pose);
    if(!rays.scan||!(dt>0.&&dt<=.25))return {};
    // For pure forward translation a return strictly behind the rear bumper
    // gets farther from the entire rectangle throughout the braking sweep.
    // Keep the full scan for steering, pivots, reverse and passage selection.
    const auto straightAway=[&]() -> Command {
      if(backing_||turning_||measured.v<0.||previous.v<0.||
         std::abs(measured.w)>1e-4||std::abs(previous.w)>1e-4||
         !observedCorridor(points,0.,.55+extension,rays))return {};
      std::vector<ObstaclePoint> ahead;
      for(const auto& p:points)if(p.x>=-body_half_length-body_length_margin)ahead.push_back(p);
      if(ahead.size()==points.size())return {};
      const auto cmd=mazeSafeCommand(ahead,measured,rateLimit({.18,0.},previous,dt,.20,.60),extension);
      if(cmd.v>0.)state="roam_forward";
      return cmd;
    };
    if(!mazeBrakeClear(points,measured,extension))return straightAway();
    if(backing_&&std::hypot(pose.x-back_start_.x,pose.y-back_start_.y)>=.45){
      backing_=false;chosen_=false;next_scan_=t;return {};
    }
    if(!turning_&&!backing_&&t>=next_scan_){
      const bool continuing=chosen_;
      const double prior_heading=heading_;
      next_scan_=t+.25;chosen_=false;
      constexpr int bins=24;
      std::array<double,bins> depth{};
      for(int i=0;i<bins;i++){
        const double a=i*2*pi/bins,c=std::cos(a),s=std::sin(a);
        double clearance=2.;
        for(const auto& p:points){
          const double along=c*p.x+s*p.y;
          if(along>0.&&std::abs(-s*p.x+c*p.y)<=body_half_width+body_width_margin+extension)
            clearance=std::min(clearance,along-body_half_length-body_length_margin-extension);
        }
        if(clearance<.55)continue; // Leave turning room at a known wall.
        for(double length:{1.6,.95,.55,.25})if(clearance>=length&&observedCorridor(points,a,length,rays)){
          depth[i]=length;break;
        }
      }
      double best=-1e9;
      const bool canTurn=turnSpace(points,rays,extension);
      for(int i=0;i<bins;i++){
        if(depth[i]==0.)continue;
        const double a=angle(i*2*pi/bins);
        if((std::abs(a)>.60||depth[0]==0.)&&!canTurn)continue;
        const double neighbors=depth[(i+bins-1)%bins]+depth[(i+1)%bins];
        const double keep=continuing&&std::abs(angle(pose.yaw+a-prior_heading))<.2?.20:0.;
        const double score=depth[i]+.30*neighbors+.70*std::cos(a)+keep-2.5*memory_.visits(pose,pose.yaw+a);
        if(score>best){best=score;chosen_=true;heading_=pose.yaw+a;}
      }
      turning_=chosen_&&(std::abs(angle(heading_-pose.yaw))>.60||depth[0]==0.);
      cruise_=depth[0]>=.55?.18:.08;
      next_window_=t;
      if(turning_)turn_sign_=std::copysign(1.,angle(heading_-pose.yaw));
      if(!chosen_&&observedCorridor(points,pi,.55+extension,rays)){
        backing_=true;back_start_=pose;
      }
    }
    Command desired{};
    if(backing_){
      if(!observedCorridor(points,pi,.55+extension,rays)){backing_=false;next_scan_=t;return {};}
      if(std::abs(measured.w)>.04||measured.v>.02)return {};
      desired.v=-.10;state="route_retreat";
    }else if(chosen_){
      const double delta=angle(heading_-pose.yaw);
      if(turning_&&std::abs(delta)<.10){turning_=false;chosen_=false;next_scan_=t;return {};}
      if(turning_){
        if(!turnSpace(points,rays,extension)||!observedCorridor(points,delta,.30,rays)){
          turning_=chosen_=false;next_scan_=t;return {};
        }
        if(std::abs(measured.v)>.02)return {};
        desired.w=std::copysign(std::min(.38,std::max(.13,std::abs(delta))),std::abs(delta)>pi/2?turn_sign_:delta);state="pivot_to_path";
      }else{
        // Look far to select a passage, but validate the next braking distance
        // on every tick. A shorter observed lane is still usable at a crawl.
        const double look=extension+stoppingTravel(std::max(std::abs(measured.v),cruise_))+.04;
        if(!observedCorridor(points,0.,look,rays)||measured.v<-.02)return {};
        desired.v=cruise_;desired.w=std::max(-.38,std::min(.38,delta));state="roam_forward";
        if(std::abs(delta)<.08&&std::abs(measured.w)<.04){
          const auto straight=rateLimit({cruise_,0.},previous,dt,.20,.60);
          return mazeSafeCommand(points,measured,straight,extension);
        }
      }
    }else return {};
    if(t>=next_window_){
      next_window_=t+.20;
      window_=dynamicWindow(points,measured,desired,backing_?0.:angle(heading_-pose.yaw),extension);
    }
    if(window_.v==0.&&window_.w==0.){state="roam_wait";return straightAway();}
    return mazeSafeCommand(points,measured,rateLimit(window_,previous,dt,.20,.60),extension);
  }
};
}
