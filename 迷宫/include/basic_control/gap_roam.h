#pragma once
#include "basic_control/maze_guard.h"
#include "basic_control/rate_limit.h"
#include "basic_control/frontier_search.h"
#include "basic_control/turn_sweep.h"
#include "basic_control/recent_travel.h"
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
  bool backtracking_=false,dead_end_turn_=false;
  bool detouring_=false;
  FrontierSearch search_;
  bool guided_=false;
  int return_direction_=0;
  bool short_advance_=false;
  bool steering_clear_=false;
  bool short_return_turn_=false;
  // A constrained turn may need a small pivot and a translation before
  // the final rotation will fit. Targets stay fixed in odometry.
  enum class TurnPhase { idle, angle, shift };
  TurnPhase turn_phase_=TurnPhase::idle;
  bool partial_turn_=false;
  Pose shift_start_{};
  double approach_yaw_=0.,shift_distance_=0.,exit_yaw_=0.,shift_blocked_=-1.;
  double pivot_blocked_=-1.;
  double turn_retry_at_=-1.,last_turn_retry_=-1.;
  Pose turn_retry_pose_{};
  int escape_sign_=0;
  double escape_blocked_=-1.;
  Pose escape_start_{};
  const char* no_lane_="roam_no_passage";
  Pose detour_start_{};
  double detour_scan_yaw_=0.;
  Pose coverage_origin_{};
  std::array<double,36> coverage_{};
  double coverage_stamp_=-1.;
  Pose back_start_{};
  double back_distance_=.18;
  PassageMemory memory_;
  RecentTravel recent_;
  double cruise_=.30;
  static double angle(double x){return std::atan2(std::sin(x),std::cos(x));}
  struct Flank {
    double slope=0.,offset=0.,first=0.,last=0.;
    bool valid=false;
  };
  struct Surface {double x=0.,y=0.;bool seen=false;};
  using FlankSamples=std::array<Surface,11>;
  static double median(std::vector<double>& values){
    auto mid=values.begin()+values.size()/2;
    std::nth_element(values.begin(),mid,values.end());
    return *mid;
  }
  // Fit the inner surfaces separately. Cones on opposing rows need not
  // appear at the same longitudinal position, or even in the same ring.
  static Flank fitFlank(const FlankSamples& samples){
    std::vector<double> slopes,offsets;
    for(const auto& a:samples)if(a.seen)for(const auto& b:samples)
      if(b.seen&&b.x-a.x>.52)slopes.push_back((b.y-a.y)/(b.x-a.x));
    if(slopes.empty())return {};
    Flank f;f.slope=median(slopes);
    if(std::abs(f.slope)>.50)return {};
    for(const auto& p:samples)if(p.seen)offsets.push_back(p.y-f.slope*p.x);
    f.offset=median(offsets);unsigned support=0;
    for(const auto& p:samples)if(p.seen&&std::abs(p.y-f.offset-f.slope*p.x)<.085){
      if(!support)f.first=p.x;
      ++support;
      f.last=p.x;
    }
    f.valid=support>=3&&f.last-f.first>=.65;
    return f;
  }
  static bool aisleTarget(const std::vector<ObstaclePoint>& hits,double& bearing,double& look){
    FlankSamples left{},right{};
    for(const auto& p:hits){
      if(p.remembered||p.x<-.50||p.x>=2.25||std::abs(p.y)<.18||std::abs(p.y)>1.25)continue;
      auto& sample=(p.y>0.?left:right)[size_t((p.x+.50)*4.)];
      if(!sample.seen||std::abs(sample.y)>std::abs(p.y))sample={p.x,p.y,true};
    }
    const Flank l=fitFlank(left),r=fitFlank(right);
    const auto fits=[](double width){return width>=2.*(body_half_width+body_width_margin)&&width<=1.85;};
    if(l.valid&&r.valid&&std::abs(l.slope-r.slope)<.20&&fits(l.offset-r.offset)&&
       std::min(l.last,r.last)-std::max(l.first,r.first)>.48&&std::min(l.last,r.last)>.55){
      const double x=std::min(1.1,std::min(l.last,r.last));
      if(!fits(l.offset-r.offset+(l.slope-r.slope)*x))return false;
      bearing=std::atan2((l.offset+r.offset+(l.slope+r.slope)*x)*.5,x);
      look=x;return true;
    }
    if(l.valid||r.valid)return false;
    // A single opposed pair defines a gate, but a forward wall or a
    // lone reflector does not. Aim through the nearest measured midpoint.
    double nearest=2.;
    for(const auto& a:left)if(a.seen)for(const auto& b:right)if(b.seen){
      const double x=(a.x+b.x)*.5;
      if(x<.40||x>=nearest||std::abs(a.x-b.x)>.20||!fits(a.y-b.y))continue;
      nearest=x;bearing=std::atan2((a.y+b.y)*.5,x);look=x;
    }
    return nearest<2.;
  }
  // Fit a small heading correction between the two flanks. In a narrow,
  // symmetric gate neither flank has room for a correction.
  static double passingCorrection(const std::vector<ObstaclePoint>& points,double bearing,double extension){
    const double c=std::cos(bearing),s=std::sin(bearing);
    const double width=body_half_width+body_width_margin+extension;
    double left=.24,right=.24;
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
  static void coverBand(unsigned& bands,double y){
    if(std::abs(y)<body_half_width)
      bands|=1u<<std::min(4,int((y+body_half_width)*5/(2*body_half_width)));
  }
  static bool rayBands(RayView rays,double a,double far,unsigned bands){
    const double c=std::cos(a),s=std::sin(a);
    const double ox=c*rays.x+s*rays.y,oy=-s*rays.x+c*rays.y;
    const double ca=std::cos(rays.yaw-a),sa=std::sin(rays.yaw-a);
    for(const auto& e:rays.scan->ends){
      const double dx=ca*e.x-sa*e.y;
      if(dx<=0.||ox>=far||ox+dx<far)continue;
      const double y=oy+(sa*e.x+ca*e.y)*(far-ox)/dx;
      coverBand(bands,y);
      if(bands==31)return true;
    }
    return false;
  }
  // Check every obstacle, including low returns. Once all five strips have
  // evidence, further rings at the same azimuth need no more projections.
  static bool seenLane(const std::vector<ObstaclePoint>& points,double a,double travel,RayView rays){
    const double c=std::cos(a),s=std::sin(a),far=body_half_length+travel;
    unsigned bands=0;
    for(const auto& p:points){
      const double x=c*p.x+s*p.y,y=-s*p.x+c*p.y;
      if(x>0.&&x<=far&&std::abs(y)<=body_half_width)return false;
      if(bands!=31&&x>far&&!p.remembered)coverBand(bands,y*far/x);
    }
    return bands==31||rayBands(rays,a,far,bands);
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
  // A far obstruction can shorten a pivot without invalidating the angle
  // needed to react and brake from both the current and the proposed speed.
  static bool canContinuePivot(const std::vector<ObstaclePoint>& points,double delta,double extension,
                               Command measured,Command previous,double dt){
    if(previous.v!=0.||std::abs(measured.v)>.02||std::abs(previous.w)<=.02||
       delta*previous.w<=0.||delta*measured.w<0.)return false;
    const double speed=std::max({.24,std::abs(measured.w),std::abs(previous.w)});
    const double stopAngle=speed*(.25+dt)+speed*speed/1.20;
    return turnFits(points,std::copysign(stopAngle,delta),extension);
  }
  void noteTurnRetry(Pose pose,double t){
    if(turn_retry_at_<0.){turn_retry_at_=t;turn_retry_pose_=pose;}
    last_turn_retry_=t;
  }
  // The graph edge is a goal, not a constraint on a local escape maneuver.
  // Evaluate both translations in the current body frame on every tick.
  // On a long reverse leg keep room to swing the rear corners into the
  // next passage. The last few centimetres remain available for escape.
  static bool rearCruiseFits(const std::vector<ObstaclePoint>& points,double extension,double speed){
    std::vector<ObstaclePoint> leading;
    for(const auto& p:points)if(-p.x+extension+body_length_margin+.08*speed>=body_half_length)
      leading.push_back(p);
    return mazeBrakeClear(leading,{-speed,0.},extension);
  }
  Command reposition(const std::vector<ObstaclePoint>& points,RayView rays,Command measured,
                     Command previous,double t,double dt,double extension,int preferred,
                     bool allowOpposite=true,double targetSpeed=.12){
    const double stamp=rays.scan->received_at;
    if(stamp<0.||t-stamp>.30||stamp>t+.02)return hold("roam_return_scan");
    if(previous.w!=0.||std::abs(measured.w)>.02)return hold("roam_settle_pivot");
    const double reaction=.25+dt,b=.20*reaction;
    const double yawDrift=std::abs(measured.w)*(reaction+std::abs(measured.w)/1.20);
    const double cornerDrift=std::hypot(body_half_length,body_half_width)*yawDrift;
    for(int sign:{preferred,-preferred}){
      if(sign!=preferred&&!allowOpposite)break;
      if(sign*previous.v<0.||sign*measured.v<-.02)continue;
      const double room=translationRoom(points,sign,extension+cornerDrift);
      if(backing_&&sign<0&&!rearCruiseFits(points,extension,targetSpeed))continue;
      const double limit=std::sqrt(b*b+.40*std::max(0.,room-.006))-b;
      if(limit<std::min(.035,targetSpeed)||std::max(sign*previous.v,sign*measured.v)>limit)continue;
      if(sign*measured.v<0.&&!shiftFits(points,measured.v*(reaction+std::abs(measured.v)/.40),extension))continue;
      const double speed=std::min(targetSpeed,limit);
      const double travel=.04+speed*reaction+speed*speed/.40;
      if(!observedCorridor(points,sign>0?0.:pi,travel,rays))continue;
      const auto cmd=rateLimit({sign*speed,0.},previous,dt,.20,.60);
      if(!brakeClear(points,cmd))continue;
      if(backtracking_)state=sign==1?"roam_return_forward":"roam_return_reverse";
      else state=sign==1?"roam_probe_forward":"roam_probe_reverse";
      return cmd;
    }
    return hold(backtracking_?"roam_return_blocked":no_lane_);
  }
  Command escape(Pose pose,const std::vector<ObstaclePoint>& points,RayView rays,Command measured,
                 Command previous,double t,double dt,double extension,int preferred){
    recent_.encounter(pose,t);
    if(escape_sign_&&std::hypot(pose.x-escape_start_.x,pose.y-escape_start_.y)<.45){
      const auto same=reposition(points,rays,measured,previous,t,dt,extension,escape_sign_,false);
      if(same.v!=0.){escape_blocked_=-1.;return same;}
      if(escape_blocked_<0.)escape_blocked_=t;
      if(t-escape_blocked_<.40)return same;
      preferred=-escape_sign_;
    }
    escape_sign_=0;
    const auto next=reposition(points,rays,measured,previous,t,dt,extension,preferred);
    if(next.v!=0.){escape_sign_=next.v>0.?1:-1;escape_start_=pose;escape_blocked_=-1.;}
    return next;
  }
  template<size_t N>
  bool planTurn(Pose pose,const std::vector<ObstaclePoint>& points,RayView rays,
                const std::array<double,N>& depth,double extension,bool routed,double route){
    std::vector<size_t> lanes;
    for(size_t i=1;i<N;++i)if(depth[i]>=.50&&std::abs(angle(i*2*pi/N))>.40)lanes.push_back(i);
    if(lanes.empty())return false;
    partial_turn_=false;
    const auto score=[&](size_t i){
      const double a=i*2*pi/N;
      return depth[i]+.5*std::cos(a)+(routed?2.*std::cos(pose.yaw+a-route):0.);
    };
    std::stable_sort(lanes.begin(),lanes.end(),[&](size_t a,size_t b){return score(a)>score(b);});
    std::vector<ObstaclePoint> local,turned;
    const double radius=std::hypot(body_half_length+body_length_margin,body_half_width+body_width_margin)+extension+.44;
    for(const auto& p:points)if(p.x*p.x+p.y*p.y<=radius*radius)local.push_back(p);
    turned.reserve(local.size());
    for(double start:{0.,.40,-.40,.24,-.24,.60,-.60,.85,-.85,1.05,-1.05}){
      if(start!=0.&&!turnFits(local,start,extension))continue;
      turned.clear();
      const double c=std::cos(start),s=std::sin(start);
      for(const auto& p:local)turned.push_back({c*p.x+s*p.y,c*p.y-s*p.x,p.remembered});
      for(double distance:{.04,.055,.075,.11,.16,.24,.32,.42})for(int sign:{1,-1}){
        const double shift=sign*distance;
        if(!shiftFits(turned,shift,extension))continue;
        for(size_t i:lanes){
          const double final=angle(i*2*pi/N);
          if(!turnFits(turned,angle(final-start),extension,shift))continue;
          if(!seenLane(points,start+(sign<0?pi:0.),distance,rays))break;
          approach_yaw_=pose.yaw+start;exit_yaw_=pose.yaw+final;shift_distance_=shift;
          shift_start_=pose;shift_blocked_=-1.;escape_sign_=0;
          turn_phase_=start==0.?TurnPhase::shift:TurnPhase::angle;
          return true;
        }
      }
    }
    // A narrow pocket may take several moves. Commit a short translation
    // only if it unlocks a larger verified rotation toward an open aisle.
    for(double reserve:{.012,0.})for(size_t i:lanes){
      const double target=angle(i*2*pi/N);
      for(double amount:{.90,.72,.55,.40,.27,.16}){
        if(amount>=std::abs(target)-.10)continue;
        const double part=std::copysign(amount,target);
        for(double travel:{0.,.025,.05,.075,.11,.16,.24,.32})for(int sign:{1,-1}){
          const double shift=sign*travel;
          if(!turnFits(local,part,extension+reserve,shift))continue;
          if(travel>0.&&(!shiftFits(local,shift,extension)||!seenLane(points,sign<0?pi:0.,travel,rays)))continue;
          partial_turn_=true;shift_start_=pose;shift_blocked_=-1.;escape_sign_=0;
          shift_distance_=shift;exit_yaw_=pose.yaw+part;
          approach_yaw_=travel>0.?pose.yaw:exit_yaw_;
          turn_phase_=travel>0.?TurnPhase::shift:TurnPhase::angle;
          return true;
        }
      }
    }
    return false;
  }
  // Return true while this maneuver owns the command. A failed scan stops
  // the command immediately. Brief feedback noise does not reverse the plan.
  bool followTurn(Pose pose,const std::vector<ObstaclePoint>& points,RayView rays,
                  Command measured,Command previous,double t,double dt,double extension,Command& cmd){
    const auto cancel=[&]{turn_phase_=TurnPhase::idle;next_scan_=t;};
    const auto waitForScan=[&]{
      noteTurnRetry(pose,t);
      cmd=hold("roam_return_rescan");
      if(shift_blocked_<0.)shift_blocked_=t;
      else if(t-shift_blocked_>.45)cancel();
      return true;
    };
    const double stamp=rays.scan->received_at;
    if(stamp<0.||t-stamp>.30||stamp>t+.02){cancel();cmd=hold("roam_return_scan");return true;}
    const double approach=angle(approach_yaw_-pose.yaw),final=angle(exit_yaw_-pose.yaw);
    if(turn_phase_==TurnPhase::angle&&std::abs(approach)>.025){
      const bool fits=turnFits(points,approach,extension);
      if(!pivotCoverage(pose,rays,t)||(!fits&&!canContinuePivot(points,approach,extension,measured,previous,dt)))
        return waitForScan();
      if(std::abs(measured.v)>.02||previous.v!=0.){cmd=hold("roam_settle_pivot");return true;}
      if(!fits)noteTurnRetry(pose,t);
      const double speed=std::min(fits?.50:.24,std::sqrt(1.2*std::max(0.,std::abs(approach)-.012-.15*std::abs(measured.w))));
      cmd=rateLimit({0.,std::copysign(speed,approach)},previous,dt,.20,.60);
      if(!mazeBrakeClear(points,measured,extension)||!mazeBrakeClear(points,cmd,extension))return waitForScan();
      shift_blocked_=-1.;state="roam_turn_setup";return true;
    }
    if(shift_distance_==0.){cancel();return false;}
    if(turn_phase_==TurnPhase::angle){turn_phase_=TurnPhase::shift;shift_start_=pose;}
    const int sign=shift_distance_>0.?1:-1;
    const double distance=sign*((pose.x-shift_start_.x)*std::cos(approach_yaw_)+(pose.y-shift_start_.y)*std::sin(approach_yaw_));
    const double remaining=std::abs(shift_distance_)-distance;
    const bool atCenter=remaining<=.007;
    if((atCenter||distance>.025)&&pivotCoverage(pose,rays,t)&&turnFits(points,final,extension+(atCenter?0.:.016))&&
       (partial_turn_||seenLane(points,final,.25,rays))){
      if(std::abs(measured.v)>.02||previous.v!=0.){cmd=hold("roam_settle_pivot");return true;}
      cancel();pivot_blocked_=-1.;heading_=exit_yaw_;turning_=chosen_=true;turn_sign_=std::copysign(1.,final);
      rolling_turn_=short_return_turn_=backing_=detouring_=false;return false;
    }
    if(remaining<=.002||std::abs(approach)>.16)return waitForScan();
    const double reaction=.20*(.25+dt);
    const double speed=std::min(.12,std::sqrt(reaction*reaction+.40*remaining)-reaction);
    cmd=reposition(points,rays,measured,previous,t,dt,extension,sign,false,speed);
    if(cmd.v!=0.){shift_blocked_=-1.;state=sign>0?"roam_turn_forward":"roam_turn_reverse";}
    else if(shift_blocked_<0.)shift_blocked_=t;
    else if(t-shift_blocked_>.6)cancel();
    return true;
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
    recent_.observe(pose);
    memory_.observe(pose,false,backtracking_);
    if(!rays.scan)return hold("roam_no_scan");
    if(!(dt>0.&&dt<=.25))return hold("roam_control_interval");
    if(search_.observePose(pose,t)){
      turn_phase_=TurnPhase::idle;escape_sign_=0;
      turning_=chosen_=dead_end_turn_=backing_=detouring_=false;next_scan_=t;
      // The same pocket can admit alternating partial turns without ever
      // opening an exit. Move back a junction's width along the arrival
      // aisle when both the measured trail and the rear scan agree.
      const double age=t-rays.scan->received_at;
      if(rays.scan->received_at>=0.&&age>=-.02&&age<=.30&&
         !seenLane(points,0.,.45,rays)&&memory_.retraced(pose,pose.yaw+pi,.85)>.45&&seenLane(points,pi,.32,rays)&&
         translationRoom(points,-1,coneAllowance)>.14){
        backing_=true;back_start_=pose;back_distance_=.55;next_scan_=t+.25;
      }
    }
    if(!brakeClear(points,measured))return hold("roam_braking_obstacle");
    if(recent_.active){
      Command retrace{};
      if(pivotCoverage(pose,rays,t)&&recent_.step(pose,points,rays,measured,previous,dt,retrace)&&
         (!backtracking_||return_direction_*retrace.v>=0.)){
        state="roam_retrace";return retrace;
      }
      recent_.active=false;next_scan_=t;
    }
    // Ignore an isolated interruption and let an actually progressing turn finish.
    if(turn_retry_at_>=0.&&(t-last_turn_retry_>.85||
       std::abs(angle(pose.yaw-turn_retry_pose_.yaw))>.10||
       std::hypot(pose.x-turn_retry_pose_.x,pose.y-turn_retry_pose_.y)>.08))turn_retry_at_=-1.;
    if(turn_retry_at_>=0.&&t-turn_retry_at_>=.75&&last_turn_retry_-turn_retry_at_>=.30){
      double route=0.;
      if(memory_.route(pose,route,true)==PassageMemory::Route::hop&&std::cos(route-pose.yaw)<-.90){
        // A traversed edge gives the direction. Current observations and the
        // calibrated stopping sweep still have to approve every reverse step.
        const auto retreat=reposition(points,rays,measured,previous,t,dt,coneAllowance,-1,false);
        if(retreat.v<0.){
          turn_retry_at_=-1.;turn_phase_=TurnPhase::idle;
          turning_=chosen_=dead_end_turn_=false;backing_=true;back_start_=pose;next_scan_=t+.25;
          state="route_retreat";return retreat;
        }
      }
    }
    Command setup{};
    if(turn_phase_!=TurnPhase::idle&&followTurn(pose,points,rays,measured,previous,t,dt,coneAllowance,setup))return setup;
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
      std::array<double,bins> sideRoom{};
      double forwardClearance=0.;
      bool rememberedLimit=false;
      for(int i=0;i<bins;i++){
        const double a=i*2*pi/bins,c=std::cos(a),s=std::sin(a);
        double clearance=2.,side=coneAllowance+body_width_margin;
        for(const auto& p:points){
          const double along=c*p.x+s*p.y,lateral=std::abs(-s*p.x+c*p.y);
          if(along>=body_half_length&&along<=1.25)
            side=std::min(side,std::max(0.,lateral-body_half_width));
          if(along>0.&&lateral<=body_half_width&&along-body_half_length<clearance){
            clearance=along-body_half_length;
            if(i==0)rememberedLimit=p.remembered;
          }
        }
        sideRoom[i]=side;
        if(i==0)forwardClearance=clearance;
        if(clearance<.55)continue; // Leave turning room at a known wall.
        // Every return was already checked for an obstacle along this lane.
        // When the physical rays cover its width, avoid a second full cloud
        // projection. Sparse scans can still use their original endpoints.
        for(double length:{1.75,1.15,.75,.50,.25})if(clearance>=length&&
            ((clearance>length&&rayBands(rays,a,body_half_length+length,0))||seenLane(points,a,length,rays))){
          depth[i]=length;break;
        }
      }
      double best=-1e9;
      const bool observed=pivotCoverage(pose,rays,t);
      const bool diskClear=!pivotObstacle(points,coneAllowance+std::hypot(body_length_margin,body_width_margin));
      const bool canTurn=observed&&diskClear;
      const bool retreating=backing_&&std::hypot(pose.x-back_start_.x,pose.y-back_start_.y)<back_distance_;
      if(!backing_)back_distance_=.18;
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
      if(destination==PassageMemory::Route::home){
        search_.activate();backtracking_=false;
      }
      const bool fresh=rays.scan->received_at>=0.&&t-rays.scan->received_at<=.30&&rays.scan->received_at<=t+.02;
      if(fresh)search_.observeLanes(pose,depth);
      guided_=fresh&&search_.heading(pose,t,depth,[&](double a,double d){return seenLane(points,a,d,rays);},route);
      const bool routed=guided_||destination==PassageMemory::Route::branch||destination==PassageMemory::Route::hop;
      double bypassScore=-1e9,bypassHeading=0.;
      for(int i=0;i<bins;i++){
        if(depth[i]==0.)continue;
        const double a=angle(i*2*pi/bins);
        // While reversing out of a pocket, select a side exit or a U-turn.
        // The old approach can look clear again after only a short retreat.
        if(retreating||(backing_&&std::abs(angle(pose.yaw+a-back_start_.yaw))<pi/3))continue;
        if((backing_||std::abs(a)>.60||depth[0]==0.)&&
           (!observed||(!diskClear&&!turnFits(points,a,coneAllowance))))continue;
        const double neighbors=depth[(i+bins-1)%bins]+depth[(i+1)%bins];
        const double keep=continuing&&std::abs(angle(pose.yaw+a-prior_heading))<.2?.20:0.;
        const double heading=pose.yaw+a;
        const double routeError=angle(heading-route);
        if((backtracking_||guided_)&&routed&&std::abs(routeError)>.65){
          // The graph edge is a chord between odometry samples. A real
          // passage can bend around a cone on either side of that chord.
          const double score=2.*std::cos(routeError)+.9*std::cos(a)+.25*depth[i]+keep+3.*sideRoom[i];
          if(score>bypassScore){bypassScore=score;bypassHeading=heading;}
          continue;
        }
        const double score=guided_?8.*std::cos(routeError)+.35*depth[i]+.12*neighbors+.30*std::cos(a)+keep:
          depth[i]+.30*neighbors+.70*std::cos(a)+keep
          -2.5*memory_.visits(pose,heading)-1.2*memory_.retraced(pose,heading,depth[i])
          +(routed&&depth[i]>=.75&&std::abs(angle(heading-route))<.60?2.5*std::cos(heading-route):0.);
        const double roomScore=score+3.*sideRoom[i];
        if(roomScore>best){best=roomScore;chosen_=true;heading_=pose.yaw+a;}
      }
      // A rearward graph hop only needs a clear rear lane to translate.
      // Prefer turning to face it when feasible. When the pivot cannot
      // fit, the forward-facing bypass must not outrank the rear hop.
      const double alignment=std::cos(route-pose.yaw);
      return_direction_=routed&&std::abs(alignment)>.85?(alignment>0.?1:-1):0;
      const bool reverseHop=!chosen_&&!backing_&&(backtracking_||guided_)&&routed&&
        std::abs(angle(route-pose.yaw-pi))<.55&&depth[bins/2]>=.50&&
        (bypassScore<=-1e9||rearCruiseFits(points,coneAllowance,.12))&&
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
        if((backtracking_||guided_)&&routed){
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
      if(!backing_&&!backtracking_&&!guided_&&std::all_of(depth.begin(),depth.end(),[](double d){return d>=1.15;})){
        // In an observed open area there is no junction to resolve.
        chosen_=true;heading_=continuing&&std::abs(angle(prior_heading-pose.yaw))<.25?prior_heading:pose.yaw;
      }
      double aim=0.,look=0.;
      const bool aisle=!backing_&&!backtracking_&&!shortTurn&&!deadEnd&&fresh&&
        (!routed||std::abs(angle(route-pose.yaw))<.65)&&
        aisleTarget(points,aim,look)&&std::abs(aim)<.48&&
        seenLane(points,aim,std::max(.42,look-body_half_length+.14),rays);
      if(aisle){heading_=pose.yaw+aim;chosen_=true;}
      const double change=std::abs(angle(heading_-pose.yaw));
      turning_=chosen_&&(change>.60||(depth[0]==0.&&change>.10));
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
      steering_clear_=aisle;
      if(chosen_&&!turning_){
        const double bearing=angle(heading_-pose.yaw);
        const double centered=aisle?bearing:corridorBearing(points,bearing);
        if(centered!=bearing&&seenLane(points,centered,.40,rays))
          heading_=pose.yaw+centered;
        const double planned=angle(heading_-pose.yaw);
        const double correction=passingCorrection(points,planned,coneAllowance);
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
      const bool clearRetreat=deadEnd&&!chosen_&&!backing_&&
        reposition(points,rays,measured,previous,t,dt,coneAllowance,-1,false).v<0.;
      if(observed&&fresh&&!chosen_&&!reverseHop&&!clearRetreat&&!backing_&&
         planTurn(pose,points,rays,depth,coneAllowance,routed,route)&&
         followTurn(pose,points,rays,measured,previous,t,dt,coneAllowance,setup))return setup;
      if(!chosen_&&!backing_&&!backtracking_&&!guided_&&seenLane(points,pi,.20,rays)&&
         reposition(points,rays,measured,previous,t,dt,coneAllowance,-1,false).v<0.){
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
      if(previous.w!=0.||std::abs(measured.w)>.02||measured.v>.02)return hold("roam_settle_reverse");
      const auto retreat=reposition(points,rays,measured,previous,t,dt,coneAllowance,-1,false);
      if(retreat.v<0.)state="route_retreat";
      else{backing_=false;next_scan_=t;}
      return retreat;
    }else if(chosen_){
      double delta=angle(heading_-pose.yaw);
      if(turning_){
        if(std::abs(delta)>pi/2&&delta*turn_sign_<0.)delta+=turn_sign_*2*pi;
        if(short_return_turn_&&(rays.scan->received_at<0.||t-rays.scan->received_at>.30||
           rays.scan->received_at>t+.02||pivotObstacle(points,coneAllowance+body_width_margin)||
           !pivotObserved(rays,coneAllowance+body_width_margin))){
          noteTurnRetry(pose,t);
          turning_=chosen_=false;next_scan_=t;
          return hold("roam_return_rescan");
        }
        const bool observed=pivotCoverage(pose,rays,t);
        const bool obstructed=!turnFits(points,delta,coneAllowance);
        const bool shortSweep=observed&&obstructed&&canContinuePivot(points,delta,coneAllowance,measured,previous,dt);
        if((backtracking_||dead_end_turn_)&&(!observed||obstructed))noteTurnRetry(pose,t);
        if(observed&&obstructed&&!shortSweep&&pivotClear(points,delta)){
          if(pivot_blocked_<0.)pivot_blocked_=t;
          if(t-pivot_blocked_<.36)return hold("roam_return_rescan");
        }else pivot_blocked_=-1.;
        if(!observed&&!obstructed&&dead_end_turn_)return hold("roam_pivot_unobserved");
        if((obstructed&&!shortSweep)||!observed){
          turning_=chosen_=dead_end_turn_=false;next_scan_=t;
          if(!backtracking_&&seenLane(points,pi,.20,rays)){backing_=true;back_start_=pose;}
          if(obstructed)return reposition(points,rays,measured,previous,t,dt,coneAllowance,
              bumperClearance(points,pi)>bumperClearance(points,0.)?-1:1);
          return hold("roam_pivot_unobserved");
        }
        // Allow for chassis response before the angular deceleration takes
        // effect. Keep the full rate on the long part of a U-turn.
        const double toward=std::max(0.,measured.w*std::copysign(1.,delta));
        const double remaining=std::max(0.,std::abs(delta)-.04-.18*toward);
        const double speed=std::min(shortSweep?.24:.52,std::sqrt(2.*.60*remaining));
        desired.w=std::copysign(speed,delta);
        state=shortSweep?"roam_slow_for_turn":dead_end_turn_?"roam_uturn":short_return_turn_&&!backtracking_?"roam_probe_turn":"pivot_to_path";
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
        if(cruise_==0.)return hold("roam_forward_unobserved");
        if(measured.v<-.02)return hold("roam_settle_forward");
        desired.v=cruise_;desired.w=std::max(-.38,std::min(.38,delta));
        if(std::abs(delta)<(steering_clear_?.005:.08))desired.w=0.;
        const double speed=std::max(std::abs(measured.v),cruise_);
        const double look=.04+.25*speed+speed*speed/.40;
        if(!seenLane(points,0.,look,rays)){
          if(previous.v<=0.||desired.w!=0.||previous.w!=0.||std::abs(measured.w)>.02)
            return hold("roam_forward_unobserved");
          const auto slowing=rateLimit({std::min(.12,cruise_),desired.w},previous,dt,.20,.60);
          const double v=std::max({std::abs(measured.v),std::abs(previous.v),slowing.v});
          const double stop=.04+v*(.25+dt)+v*v/.40;
          const double age=t-rays.scan->received_at;
          if(slowing.v<=0.||rays.scan->received_at<0.||age>.30||age<-.02||
             !seenLane(points,0.,stop,rays)||!mazeBrakeClear(points,measured,coneAllowance)||
             !mazeBrakeClear(points,slowing,coneAllowance)||!brakeClear(points,slowing))
            return hold("roam_forward_unobserved");
          state="roam_slow_for_gap";return slowing;
        }
        state=guided_?"roam_frontier":detouring_?"roam_return_detour":backtracking_?"roam_backtrack":"roam_forward";
      }
    }else if(backtracking_||guided_){
      const double forward=std::min(1.,bumperClearance(points,0.));
      const double rear=std::min(1.,bumperClearance(points,pi));
      return escape(pose,points,rays,measured,previous,t,dt,coneAllowance,return_direction_?return_direction_:(rear>forward?-1:1));
    }else if(short_advance_){
      const double speed=std::max(std::abs(measured.v),std::max(previous.v,cruise_));
      const double travel=.04+.25*speed+speed*speed/.40;
      if(rays.scan->received_at<0.||t-rays.scan->received_at>.30||rays.scan->received_at>t+.02||
         !observedCorridor(points,0.,travel,rays))return hold("roam_lane_unobserved");
      if(previous.w!=0.||std::abs(measured.w)>.02||previous.v<0.||measured.v<-.02)
        return hold("roam_settle_forward");
      desired.v=cruise_;state="roam_short_advance";
    }else{
      return escape(pose,points,rays,measured,previous,t,dt,coneAllowance,1);
    }
    const auto cmd=rateLimit(desired,previous,dt,.20,.60);
    if(short_advance_&&(!mazeBrakeClear(points,measured,coneAllowance)||!mazeBrakeClear(points,cmd,coneAllowance))){
      const auto escape=reposition(points,rays,measured,previous,t,dt,coneAllowance,1);
      if(escape.v!=0.)return escape;
      return hold("roam_short_clearance");
    }
    if(!brakeClear(points,cmd))return hold("roam_braking_obstacle");
    return cmd;
  }
};
}
