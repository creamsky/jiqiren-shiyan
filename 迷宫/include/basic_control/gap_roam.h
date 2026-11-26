#pragma once
#include "basic_control/maze_guard.h"
#include "basic_control/rate_limit.h"
#include <map>

namespace lab26 {
// Sparse passage marks are updated by measured travel, never by a planned turn.
class PassageMemory {
  struct Node {
    double x,y;
    std::vector<std::pair<double,unsigned>> passages;
    std::vector<size_t> links;
    std::vector<double> openings;
    bool dead_end=false;
  };
  std::vector<Node> nodes_;
  size_t current_=0;
  std::map<std::pair<int,int>,unsigned> trail_;
  Pose last_sample_{};
  bool have_sample_=false;
  static std::pair<int,int> cell(double x,double y){return {int(std::floor(x/.25)),int(std::floor(y/.25))};}
  static unsigned axis(double a){return unsigned(int(std::lround(a*8/pi))+32)%8;}
  static double difference(double a,double b){return std::abs(std::atan2(std::sin(a-b),std::cos(a-b)));}
  static void mark(Node& node,double bearing){
    for(auto& edge:node.passages)if(difference(edge.first,bearing)<.78){edge.second=std::min(2u,edge.second+1);return;}
    node.passages.emplace_back(bearing,1);
  }
 public:
  void observe(Pose p,bool junction=false,bool following_links=false,double spacing=1.05){
    if(!have_sample_){last_sample_=p;have_sample_=true;}
    const double dx=p.x-last_sample_.x,dy=p.y-last_sample_.y,d=std::hypot(dx,dy);
    if(d>=.12){
      if(d<=1.25){
        const unsigned mask=1u<<axis(std::atan2(dy,dx));
        const int steps=int(std::ceil(d/.10));
        for(int i=0;i<=steps;++i)trail_[cell(last_sample_.x+dx*i/steps,last_sample_.y+dy*i/steps)]|=mask;
      }
      last_sample_=p;
    }
    if(nodes_.empty()){nodes_.push_back({p.x,p.y,{}});return;}
    const auto& from=nodes_[current_];
    size_t next=nodes_.size();double nearest=.68;
    for(auto j:from.links){
      if(std::hypot(p.x-nodes_[j].x,p.y-nodes_[j].y)<.30){next=j;break;}
    }
    if(next==nodes_.size()&&(following_links||std::hypot(p.x-from.x,p.y-from.y)<(junction?.56:spacing)))return;
    if(next==nodes_.size())for(size_t i=0;i<nodes_.size();++i){
      if(i==current_)continue;
      const double d=std::hypot(p.x-nodes_[i].x,p.y-nodes_[i].y);
      if(d<nearest){nearest=d;next=i;}
    }
    const double bearing=std::atan2(p.y-from.y,p.x-from.x);
    mark(nodes_[current_],bearing);
    if(next==nodes_.size())nodes_.push_back({p.x,p.y,{}});
    for(auto edge:{std::make_pair(current_,next),std::make_pair(next,current_)}){
      auto& links=nodes_[edge.first].links;
      if(std::find(links.begin(),links.end(),edge.second)==links.end())links.push_back(edge.second);
    }
    mark(nodes_[next],bearing+pi);current_=next;
  }
  bool closeDeadEnd(Pose p){
    observe(p,false,false,.35);
    if(nodes_[current_].links.empty())return false;
    // A short spur can end close to a junction. Preserve its other exits.
    if(nodes_[current_].links.size()==1&&nodes_[current_].openings.size()<3)
      nodes_[current_].dead_end=true;
    return true;
  }
  void opening(Pose p,double bearing){
    auto& node=nodes_[current_];
    if(std::hypot(p.x-node.x,p.y-node.y)>.55)return;
    for(double old:node.openings)if(difference(old,bearing)<.65)return;
    node.openings.push_back(bearing);
  }
  // Breadth-first on actually traversed edges. Return the first hop toward
  // the closest junction with an observed, still unexplored branch.
  enum class Route { none, branch, hop, home };
  Route route(Pose p,double& heading,bool backtrack)const{
    if(nodes_.empty()||(!backtrack&&std::hypot(p.x-nodes_[current_].x,p.y-nodes_[current_].y)>.70))return Route::none;
    std::vector<size_t> queue{current_},first(nodes_.size(),nodes_.size());
    first[current_]=current_;
    for(size_t k=0;k<queue.size();++k){
      const size_t i=queue[k];const auto& n=nodes_[i];
      double nearest=4.;
      if(!n.dead_end)for(double a:n.openings){
        const bool used=std::any_of(n.passages.begin(),n.passages.end(),[&](const auto& e){return difference(e.first,a)<.78;});
        if(!used&&difference(p.yaw,a)<nearest){nearest=difference(p.yaw,a);heading=a;}
      }
      if(nearest<4.){
        if(i!=current_){const auto& hop=nodes_[first[i]];heading=std::atan2(hop.y-p.y,hop.x-p.x);}
        return i==current_?Route::branch:Route::hop;
      }
      for(auto j:n.links)if(first[j]==nodes_.size()){
        first[j]=i==current_?j:first[i];queue.push_back(j);
      }
    }
    if(backtrack){
      if(current_==0)return Route::home;
      const auto& hop=nodes_[first[0]];
      heading=std::atan2(hop.y-p.y,hop.x-p.x);
      return Route::hop;
    }
    return Route::none;
  }
  bool frontierHeading(Pose p,double& heading)const{
    return route(p,heading,false)!=Route::none;
  }
  unsigned visits(Pose p,double heading)const{
    if(nodes_.empty()||std::hypot(p.x-nodes_[current_].x,p.y-nodes_[current_].y)>.65)return 0;
    unsigned count=0;
    for(const auto& edge:nodes_[current_].passages)if(difference(edge.first,heading)<.78)count=std::max(count,edge.second);
    return count;
  }
  double retraced(Pose p,double heading,double reach)const{
    const unsigned a=axis(heading);
    const unsigned mask=(1u<<a)|(1u<<((a+1)%8))|(1u<<((a+7)%8));
    unsigned samples=0,hits=0;
    for(double s=.35;s<=reach;s+=.20){
      ++samples;
      auto it=trail_.find(cell(p.x+s*std::cos(heading),p.y+s*std::sin(heading)));
      if(it!=trail_.end()&&(it->second&mask))++hits;
    }
    return samples?double(hits)/samples:0.;
  }
};

// Follow observed gaps in the current body frame. Keep a turn target across scans
// so a symmetric dead end cannot flip the preferred side at each update.
class GapRoam {
  double next_scan_=0.,heading_=0.,turn_sign_=1.;
  bool turning_=false,backing_=false,chosen_=false,rolling_turn_=false;
  bool backtracking_=false,dead_end_turn_=false,explored_=false;
  bool detouring_=false;
  int return_direction_=0;
  bool short_advance_=false;
  bool steering_clear_=false;
  bool short_return_turn_=false;
  const char* no_lane_="roam_no_passage";
  Pose detour_start_{};
  double detour_scan_yaw_=0.;
  Pose coverage_origin_{};
  std::array<double,36> coverage_{};
  double coverage_stamp_=-1.;
  Pose back_start_{};
  PassageMemory memory_;
  double cruise_=.30;
  static double angle(double x){return std::atan2(std::sin(x),std::cos(x));}
  // Fit a small heading correction between the two flanks. In a narrow,
  // symmetric gate neither flank has room for a correction.
  static double passingCorrection(const std::vector<ObstaclePoint>& points,double bearing){
    const double c=std::cos(bearing),s=std::sin(bearing);
    const double width=body_half_width+.055;
    double left=.14,right=.14;
    for(const auto& p:points){
      const double x=c*p.x+s*p.y,y=-s*p.x+c*p.y;
      if(x<body_half_length||x>1.35||std::abs(y)>width+.22)continue;
      double& room=y>0.?left:right;
      room=std::min(room,(std::abs(y)-width)/x);
    }
    if(left<0.&&right>0.)return -std::atan(std::min(-left,right));
    if(right<0.&&left>0.)return std::atan(std::min(-right,left));
    return 0.;
  }
  // Estimate the corridor midpoint at several distances along the chosen
  // passage. A single cone or a missing flank is not a pair of walls.
  static double corridorBearing(const std::vector<ObstaclePoint>& scan,double bearing){
    const double c=std::cos(bearing),s=std::sin(bearing);
    std::array<double,5> left,right,offset{};
    left.fill(1.5);right.fill(1.5);
    for(const auto& p:scan){
      if(p.remembered)continue;
      const double x=c*p.x+s*p.y,y=-s*p.x+c*p.y;
      if(x<.10||x>=1.10||std::abs(y)<=body_half_width)continue;
      const int i=std::min(4,int((x-.10)/.20));
      auto& side=y>0?left[i]:right[i];
      side=std::min(side,std::abs(y));
    }
    size_t n=0;
    for(size_t i=0;i<left.size();++i)
      if(left[i]<1.5&&right[i]<1.5)offset[n++]=(left[i]-right[i])*.5;
    if(n<3)return bearing;
    std::sort(offset.begin(),offset.begin()+n);
    const double middle=offset[n/2];
    if(std::abs(middle)<.05)return bearing;
    return bearing+std::max(-.25,std::min(.25,std::atan2(middle,.9)));
  }
  // Project current laser endpoints into five strips spanning the car width.
  static bool seenLane(const std::vector<ObstaclePoint>& points,double a,double travel,RayView rays){
    const double c=std::cos(a),s=std::sin(a),far=body_half_length+travel;
    unsigned bands=0;
    const auto cover=[&](double y){
      if(std::abs(y)<body_half_width)
        bands|=1u<<std::min(4,int((y+body_half_width)*5/(2*body_half_width)));
    };
    for(const auto& p:points){
      const double x=c*p.x+s*p.y,y=-s*p.x+c*p.y;
      if(x>0.&&x<=far&&std::abs(y)<=body_half_width)return false;
      if(x>far&&!p.remembered)cover(y*far/x);
    }
    if(bands==31)return true;
    const double ox=c*rays.x+s*rays.y,oy=-s*rays.x+c*rays.y;
    const double ca=std::cos(rays.yaw-a),sa=std::sin(rays.yaw-a);
    for(const auto& e:rays.scan->ends){
      const double dx=ca*e.x-sa*e.y;
      if(dx<=0.||ox>=far||ox+dx<far)continue;
      const double y=oy+(sa*e.x+ca*e.y)*(far-ox)/dx;
      cover(y);
      if(bands==31)return true;
    }
    return false;
  }
  // Test only the rectangular body swept by the requested braking motion.
  static bool brakeClear(const std::vector<ObstaclePoint>& scan,Command v){
    const double dt=.04,delay=.25;
    const double end=delay+std::max(std::abs(v.v)/.20,std::abs(v.w)/.60);
    if(v.w==0.){
      const double d=v.v*delay+v.v*std::abs(v.v)/.40;
      for(const auto& p:scan)if(std::abs(p.y)<=body_half_width&&
          p.x>=std::min(0.,d)-body_half_length&&p.x<=std::max(0.,d)+body_half_length)return false;
      return true;
    }
    double x=0.,y=0.,yaw=0.;
    for(double t=0.;t<=end+dt;t+=dt){
      const double c=std::cos(yaw),s=std::sin(yaw);
      const double step=dt*(std::abs(v.v)+std::hypot(body_half_length,body_half_width)*std::abs(v.w));
      for(const auto& p:scan){
        const double dx=p.x-x,dy=p.y-y;
        if(std::abs(dx)>.6+step||std::abs(dy)>.6+step)continue;
        if(std::abs(dx*c+dy*s)<=body_half_length+step&&std::abs(-dx*s+dy*c)<=body_half_width+step)return false;
      }
      x+=dt*v.v*std::cos(yaw+.5*dt*v.w);y+=dt*v.v*std::sin(yaw+.5*dt*v.w);yaw+=dt*v.w;
      if(t>=delay){
        v.v=std::copysign(std::max(0.,std::abs(v.v)-.20*dt),v.v);
        v.w=std::copysign(std::max(0.,std::abs(v.w)-.60*dt),v.w);
      }
    }
    return true;
  }
  static bool pivotObstacle(const std::vector<ObstaclePoint>& points,double extra=0.){
    const double radius=std::hypot(body_half_length,body_half_width)+extra;
    for(const auto& p:points)if(p.x*p.x+p.y*p.y<=radius*radius)return true;
    return false;
  }
  // The graph edge is a goal, not a constraint on a local escape maneuver.
  // Evaluate both translations in the current body frame on every tick.
  Command reposition(const std::vector<ObstaclePoint>& points,RayView rays,Command measured,
                     Command previous,double t,double dt,double extension,int preferred){
    const double stamp=rays.scan->received_at;
    if(stamp<0.||t-stamp>.30||stamp>t+.02)return hold("roam_return_scan");
    if(previous.w!=0.||std::abs(measured.w)>.02)return hold("roam_settle_pivot");
    for(int sign:{preferred,-preferred}){
      if(sign*previous.v<0.||sign*measured.v<-.02)continue;
      const double bearing=sign==1?0.:pi;
      const double speed=std::max({.12,std::abs(measured.v),std::abs(previous.v)});
      const double travel=std::max(.20,.04+speed*(dt+.25)+speed*speed/.40);
      if(!observedCorridor(points,bearing,travel,rays))continue;
      // A strictly straight translation increases separation from every
      // return behind the trailing bumper. For residual yaw or rollback,
      // keep that bumper in the calibrated braking sweep as well.
      std::vector<ObstaclePoint> sweep;
      for(const auto& p:points)
        if(measured.w!=0.||sign*measured.v<0.||sign*p.x>=-body_half_length)
          sweep.push_back(p);
      const auto cmd=rateLimit({sign*.12,0.},previous,dt,.20,.60);
      if(!mazeBrakeClear(sweep,measured,extension)||!mazeBrakeClear(sweep,{sign*.12,0.},extension)||
         !mazeBrakeClear(sweep,cmd,extension)||!brakeClear(points,cmd))continue;
      if(backtracking_)state=sign==1?"roam_return_forward":"roam_return_reverse";
      else state=sign==1?"roam_probe_forward":"roam_probe_reverse";
      return cmd;
    }
    return hold(backtracking_?"roam_return_blocked":no_lane_);
  }
  static bool pivotObserved(RayView rays,double extra=0.){
    const double radius=std::hypot(body_half_length,body_half_width)+extra;
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
  // Combine short-lived angular coverage around one pivot centre. Compute
  // where each real ray crosses the swept circle in the world frame, so
  // turning the robot cannot turn a blind sector into observed free space.
  bool pivotCoverage(Pose p,RayView rays,double now){
    const double stamp=rays.scan->received_at<0.?now:rays.scan->received_at;
    if(now-stamp>.30||stamp>now+.02)return false;
    constexpr double drift=.025;
    if(coverage_stamp_<0.||stamp<coverage_stamp_||
       std::hypot(p.x-coverage_origin_.x,p.y-coverage_origin_.y)>drift){
      coverage_.fill(-1.);coverage_origin_=p;coverage_stamp_=-1.;
    }
    if(stamp!=coverage_stamp_){
      coverage_stamp_=stamp;
      const double c=std::cos(p.yaw),s=std::sin(p.yaw);
      const double ox=p.x-coverage_origin_.x+c*rays.x-s*rays.y;
      const double oy=p.y-coverage_origin_.y+s*rays.x+c*rays.y;
      const double radius=std::hypot(body_half_length,body_half_width)+drift;
      const double inside=radius*radius-ox*ox-oy*oy;
      const double ca=std::cos(p.yaw+rays.yaw),sa=std::sin(p.yaw+rays.yaw);
      if(inside>0.)for(const auto& e:rays.scan->ends){
        const double dx=ca*e.x-sa*e.y,dy=sa*e.x+ca*e.y,len2=dx*dx+dy*dy;
        if(len2==0.||std::hypot(ox+dx,oy+dy)<radius)continue;
        const double dot=ox*dx+oy*dy;
        const double f=inside/(std::sqrt(dot*dot+inside*len2)+dot);
        const double a=std::atan2(oy+f*dy,ox+f*dx);
        coverage_[std::min(35,int((a+pi)*18/pi))]=stamp;
      }
    }
    return std::all_of(coverage_.begin(),coverage_.end(),[&](double t){return t>=0.&&now-t<=.30;})
      ||pivotObserved(rays);
  }
 public:
  const char* state="roam_wait";
  Command hold(const char* reason){state=reason;return {};}
  Command step(Pose pose,const std::vector<ObstaclePoint>& points,RayView rays,
               Command measured,Command previous,double t,double dt,double coneAllowance){
    state="roam_wait";
    memory_.observe(pose,false,backtracking_);
    if(!rays.scan)return hold("roam_no_scan");
    if(!(dt>0.&&dt<=.25))return hold("roam_control_interval");
    if(!brakeClear(points,measured))return hold("roam_braking_obstacle");
    const bool rescanDetour=detouring_&&!dead_end_turn_&&std::abs(angle(pose.yaw-detour_scan_yaw_))>=pi/6;
    if(turning_&&(std::abs(angle(heading_-pose.yaw))<.10||rescanDetour)){
      turning_=dead_end_turn_=false;next_scan_=t;
    }
    if(!turning_&&t>=next_scan_){
      const bool continuing=chosen_;
      const double prior_heading=heading_;
      next_scan_=t+.25;chosen_=false;
      constexpr int bins=24;
      std::array<double,bins> depth{};
      double forwardClearance=0.;
      bool rememberedLimit=false;
      for(int i=0;i<bins;i++){
        const double a=i*2*pi/bins,c=std::cos(a),s=std::sin(a);
        double clearance=2.;
        for(const auto& p:points){
          const double along=c*p.x+s*p.y;
          if(along>0.&&std::abs(-s*p.x+c*p.y)<=body_half_width&&along-body_half_length<clearance){
            clearance=along-body_half_length;
            if(i==0)rememberedLimit=p.remembered;
          }
        }
        if(i==0)forwardClearance=clearance;
        if(clearance<.55)continue; // Leave turning room at a known wall.
        for(double length:{1.75,1.15,.75,.50,.25})if(clearance>=length&&seenLane(points,a,length,rays)){
          depth[i]=length;break;
        }
      }
      double best=-1e9;
      const bool observed=pivotCoverage(pose,rays,t);
      const bool diskClear=!pivotObstacle(points);
      const bool canTurn=observed&&diskClear;
      const bool retreating=backing_&&std::hypot(pose.x-back_start_.x,pose.y-back_start_.y)<.18;
      // Each contiguous angular opening represents one passage, even when
      // several adjacent sampled headings point into the same corridor.
      std::vector<int> openings;
      for(int i=0;i<bins;++i){
        if(depth[i]<.75||depth[(i+bins-1)%bins]>=.75)continue;
        int count=1;
        while(count<bins&&depth[(i+count)%bins]>=.75)++count;
        openings.push_back((i+count/2)%bins);
      }
      if(!backtracking_&&openings.size()>=3)memory_.observe(pose,true);
      if((!backtracking_||openings.size()>=3)&&openings.size()>=2)
        for(int i:openings)memory_.opening(pose,pose.yaw+i*2*pi/bins);
      // A cone row can split the return aisle into several angular openings.
      // Classify by where usable lanes lead, independently of their count.
      const bool deadEnd=depth[0]==0.&&depth[bins/2]>=.50&&
        std::none_of(depth.begin(),depth.begin()+bins/3,[](double d){return d>=.75;})&&
        std::none_of(depth.begin()+2*bins/3+1,depth.end(),[](double d){return d>=.75;});
      if(deadEnd&&!backtracking_)backtracking_=memory_.closeDeadEnd(pose);
      double route=0.;
      const auto destination=memory_.route(pose,route,backtracking_);
      const bool routed=destination==PassageMemory::Route::branch||destination==PassageMemory::Route::hop;
      double bypassScore=-1e9,bypassHeading=0.;
      explored_=destination==PassageMemory::Route::home;
      if(explored_){backing_=false;return hold("roam_explored");}
      for(int i=0;i<bins;i++){
        if(depth[i]==0.)continue;
        const double a=angle(i*2*pi/bins);
        // While reversing out of a pocket, select a side exit or a U-turn.
        // The old approach can look clear again after only a short retreat.
        if(retreating||(backing_&&std::abs(angle(pose.yaw+a-back_start_.yaw))<pi/3))continue;
        if((backing_||std::abs(a)>.60||depth[0]==0.)&&
           (!observed||(!diskClear&&!pivotClear(points,a))))continue;
        const double neighbors=depth[(i+bins-1)%bins]+depth[(i+1)%bins];
        const double keep=continuing&&std::abs(angle(pose.yaw+a-prior_heading))<.2?.20:0.;
        const double heading=pose.yaw+a;
        const double routeError=angle(heading-route);
        if(backtracking_&&routed&&std::abs(routeError)>.65){
          // The graph edge is a chord between odometry samples. A real
          // passage can bend around a cone on either side of that chord.
          const double score=2.*std::cos(routeError)+.9*std::cos(a)+.25*depth[i]+keep;
          if(score>bypassScore){bypassScore=score;bypassHeading=heading;}
          continue;
        }
        const double score=depth[i]+.30*neighbors+.70*std::cos(a)+keep
          -2.5*memory_.visits(pose,heading)-1.2*memory_.retraced(pose,heading,depth[i])
          +(routed&&depth[i]>=.75&&std::abs(angle(heading-route))<.60?2.5*std::cos(heading-route):0.);
        if(score>best){best=score;chosen_=true;heading_=pose.yaw+a;}
      }
      // A rearward graph hop only needs a clear rear lane to translate.
      // Prefer turning to face it when feasible. When the pivot cannot
      // fit, the forward-facing bypass must not outrank the rear hop.
      const double alignment=std::cos(route-pose.yaw);
      return_direction_=routed&&std::abs(alignment)>.85?(alignment>0.?1:-1):0;
      const bool reverseHop=!chosen_&&!backing_&&backtracking_&&routed&&
        std::abs(angle(route-pose.yaw-pi))<.55&&depth[bins/2]>=.50&&
        reposition(points,rays,measured,previous,t,dt,coneAllowance,-1).v<0.;
      const bool bypass=!reverseHop&&!chosen_&&bypassScore>-1e9;
      if(bypass){
        detour_scan_yaw_=pose.yaw;
        if(!detouring_)detour_start_=pose;
        chosen_=true;heading_=bypassHeading;
        // If the old edge stays closed for a junction's worth of measured
        // travel, resume mapping the observed passage from this position.
        if(std::hypot(pose.x-detour_start_.x,pose.y-detour_start_.y)>=.56){
          memory_.observe(pose,true);backtracking_=false;
        }
      }
      detouring_=bypass&&backtracking_;
      // A short side opening can still accommodate the next braking
      // distance. Face it before translating, with the entire calibrated
      // turning disk observed and clear. Keep the target through the turn.
      bool shortTurn=false;
      if(!chosen_&&!backing_&&!reverseHop&&rays.scan->received_at>=0.&&
        t-rays.scan->received_at<=.30&&rays.scan->received_at<=t+.02&&
        !pivotObstacle(points,coneAllowance+body_width_margin)&&
        pivotObserved(rays,coneAllowance+body_width_margin)){
        if(backtracking_&&routed){
          shortTurn=std::abs(angle(route-pose.yaw))>.40&&observedCorridor(points,angle(route-pose.yaw),.20,rays);
          if(shortTurn)heading_=route;
        }else{
          // Prefer straight travel on ties. Only turn for a materially
          // longer verified prefix; score left and right identically.
          double bestReach=0.;
          for(int i=0;i<bins;i++){
            const double a=angle(i*2*pi/bins);
            for(double reach:{.50,.40,.30,.20}){
              const double score=reach+.04*std::cos(a);
              if(score<=bestReach)break;
              if(!observedCorridor(points,a,reach,rays))continue;
              if(i==0)bestReach=score+.055;
              else if(std::abs(a)>.40){bestReach=score;heading_=pose.yaw+a;shortTurn=true;}
              break;
            }
          }
        }
      }
      if(shortTurn)chosen_=true;
      short_return_turn_=shortTurn;
      if(!backing_&&!backtracking_&&std::all_of(depth.begin(),depth.end(),[](double d){return d>=1.15;})){
        // In an observed open area there is no junction to resolve.
        chosen_=true;heading_=continuing&&std::abs(angle(prior_heading-pose.yaw))<.25?prior_heading:pose.yaw;
      }
      turning_=chosen_&&(std::abs(angle(heading_-pose.yaw))>.60||depth[0]==0.);
      if(!retreating&&canTurn&&deadEnd&&(!backtracking_||std::abs(angle(route-pose.yaw-pi))<.65)){
        // Aim down the actual return aisle. A fixed +pi can choose the
        // long rotation when the vehicle entered the pocket at an angle.
        double weight=depth[bins/2],offset=0.;
        for(int sign:{-1,1}){
          for(int j=1;j<=bins/6;++j){
            const double d=depth[bins/2+sign*j];
            if(d<.50)break;
            weight+=d;offset+=sign*j*d;
          }
        }
        const double centre=pi+(2*pi/bins)*offset/weight;
        const double bearing=std::abs(centre-pi)>.08&&seenLane(points,centre,.50,rays)?centre:pi;
        heading_=pose.yaw+bearing;chosen_=turning_=dead_end_turn_=true;
      }
      steering_clear_=false;
      if(chosen_&&!turning_){
        const double bearing=angle(heading_-pose.yaw);
        const double centered=corridorBearing(points,bearing);
        if(centered!=bearing&&seenLane(points,centered,.40,rays))
          heading_=pose.yaw+centered;
        const double planned=angle(heading_-pose.yaw);
        const double correction=passingCorrection(points,planned);
        if(std::abs(correction)>.005&&seenLane(points,planned+correction,.55,rays)){
          heading_+=correction;steering_clear_=true;
        }
      }
      // Actual braking uses .25 s reaction and .20 m/s² deceleration.
      // Validate the observed prefix for each speed, from cruise down.
      cruise_=0.;
      for(double speed:{.30,.24,.18,.12}){
        const double travel=.04+.25*speed+speed*speed/.40;
        if(seenLane(points,0.,travel,rays)){cruise_=speed;break;}
      }
      rolling_turn_=turning_&&!shortTurn&&!backing_&&std::abs(angle(heading_-pose.yaw))<=.55*pi;
      if(chosen_&&backing_){backing_=false;turning_=true;}
      if(chosen_&&!bypass&&!shortTurn&&destination==PassageMemory::Route::branch)backtracking_=false;
      if(turning_)turn_sign_=std::copysign(1.,angle(heading_-pose.yaw));
      if(!chosen_&&!backing_&&!backtracking_&&seenLane(points,pi,.20,rays)){
        backing_=true;back_start_=pose;
      }
      // A turn or long lane may be unobservable while the next straight
      // stopping distance is fully visible. Use this only as a last resort
      // and never creep towards an already known close barrier.
      short_advance_=!chosen_&&!backing_&&forwardClearance>=.55&&cruise_>0.&&
        (!backtracking_||!routed||std::cos(route-pose.yaw)>.25);
      if(short_advance_)cruise_=std::min(.18,cruise_);
      no_lane_=forwardClearance<.55?(rememberedLimit?"roam_near_memory":"roam_near_return"):
        cruise_==0.?"roam_lane_unobserved":"roam_no_passage";
    }
    Command desired{};
    if(backing_){
      const double reverseSpeed=std::max(.12,-measured.v);
      const double rearLook=std::max(.20,.04+.25*reverseSpeed+reverseSpeed*reverseSpeed/.40);
      if(!seenLane(points,pi,rearLook,rays)){
        backing_=false;next_scan_=t;return hold("roam_rear_blocked");
      }
      if(std::abs(measured.w)>.04||measured.v>.02)return hold("roam_settle_reverse");
      desired.v=-.12;state="route_retreat";
    }else if(chosen_){
      const double delta=angle(heading_-pose.yaw);
      if(turning_){
        if(short_return_turn_&&(rays.scan->received_at<0.||t-rays.scan->received_at>.30||
           rays.scan->received_at>t+.02||pivotObstacle(points,coneAllowance+body_width_margin)||
           !pivotObserved(rays,coneAllowance+body_width_margin))){
          turning_=chosen_=false;next_scan_=t;
          return hold("roam_return_rescan");
        }
        const bool observed=pivotCoverage(pose,rays,t);
        const bool obstructed=pivotObstacle(points)&&!pivotClear(points,delta);
        if(!observed&&!obstructed&&dead_end_turn_)return hold("roam_pivot_unobserved");
        if(obstructed||!observed){
          turning_=chosen_=dead_end_turn_=false;next_scan_=t;
          if(!backtracking_&&seenLane(points,pi,.20,rays)){backing_=true;back_start_=pose;}
          return hold(obstructed?"roam_pivot_obstacle":"roam_pivot_unobserved");
        }
        // Allow for chassis response before the angular deceleration takes
        // effect. Keep the full rate on the long part of a U-turn.
        const double toward=std::max(0.,measured.w*std::copysign(1.,delta));
        const double remaining=std::max(0.,std::abs(delta)-.04-.18*toward);
        const double speed=std::min(.52,std::sqrt(2.*.60*remaining));
        desired.w=std::copysign(speed,std::abs(delta)>pi/2?turn_sign_:delta);
        state=dead_end_turn_?"roam_uturn":short_return_turn_&&!backtracking_?"roam_probe_turn":"pivot_to_path";
        // A translating turn can sweep farther than a stationary pivot. Bound
        // its entire braking arc by the corner radius plus centre travel.
        const double entrySpeed=std::max(std::abs(measured.v),std::abs(previous.v));
        if(rolling_turn_||entrySpeed>0.){
          const double cornerSpeed=rolling_turn_?std::min(.24,.30/(1.+std::abs(delta))):0.;
          const double v=std::max(entrySpeed,cornerSpeed);
          const double travel=v*(dt+.25)+v*v/.40;
          // Reserve the next planning interval before accelerating around
          // the bend; a lagging chassis also travels during that interval.
          const double extra=travel+coneAllowance+body_width_margin+.25*v+.04;
          if(!pivotObstacle(points,extra)&&pivotObserved(rays,extra)){
            if(rolling_turn_){desired.v=cornerSpeed;state="roam_turn";}
          }else if(entrySpeed>.02){
            // Tightening clearance: ramp the linear component down while
            // checking the actual arc, including the calibrated cone base.
            const auto slowing=rateLimit(desired,previous,dt,.20,.60);
            if((entrySpeed>.12&&std::abs(previous.w)<.02)||!pivotObserved(rays,travel)||!mazeBrakeClear(points,slowing,coneAllowance)||
               !mazeBrakeClear(points,measured,coneAllowance)){
              const double bearing=(std::abs(measured.v)>std::abs(previous.v)?measured.v:previous.v)<0.?pi:0.;
              if(!seenLane(points,bearing,travel,rays)||std::abs(previous.w)>.02||std::abs(measured.w)>.02)
                return hold("roam_settle_pivot");
              desired={};
            }
            state="roam_slow_for_turn";
          }
        }
      }else{
        // Look far to select a passage, but validate the next braking distance
        // on every tick. A shorter observed lane is still usable at a crawl.
        const double speed=std::max(std::abs(measured.v),cruise_);
        const double look=.04+.25*speed+speed*speed/.40;
        if(cruise_==0.||!seenLane(points,0.,look,rays))return hold("roam_forward_unobserved");
        if(measured.v<-.02)return hold("roam_settle_forward");
        desired.v=cruise_;desired.w=std::max(-.38,std::min(.38,delta));
        state=detouring_?"roam_return_detour":backtracking_?"roam_backtrack":"roam_forward";
        if(std::abs(delta)<(steering_clear_?.005:.08))desired.w=0.;
      }
    }else if(backtracking_&&!explored_){
      const double forward=std::min(1.,bumperClearance(points,0.));
      const double rear=std::min(1.,bumperClearance(points,pi));
      return reposition(points,rays,measured,previous,t,dt,coneAllowance,return_direction_?return_direction_:(rear>forward?-1:1));
    }else if(short_advance_&&!explored_){
      const double speed=std::max(std::abs(measured.v),std::max(previous.v,cruise_));
      const double travel=.04+.25*speed+speed*speed/.40;
      if(rays.scan->received_at<0.||t-rays.scan->received_at>.30||rays.scan->received_at>t+.02||
         !observedCorridor(points,0.,travel,rays))return hold("roam_lane_unobserved");
      if(previous.w!=0.||std::abs(measured.w)>.02||previous.v<0.||measured.v<-.02)
        return hold("roam_settle_forward");
      desired.v=cruise_;state="roam_short_advance";
    }else if(!explored_){
      return reposition(points,rays,measured,previous,t,dt,coneAllowance,1);
    }else return hold("roam_explored");
    const auto cmd=rateLimit(desired,previous,dt,.20,.60);
    if(short_advance_&&(!mazeBrakeClear(points,measured,coneAllowance)||!mazeBrakeClear(points,cmd,coneAllowance)))
      return hold("roam_short_clearance");
    if(!brakeClear(points,cmd))return hold("roam_braking_obstacle");
    return cmd;
  }
};
}
