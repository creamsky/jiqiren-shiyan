#include "basic_control/navigation.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace lab3 {
namespace {
constexpr double resolution=.1;
constexpr double half_length=lab26::body_half_length+lab26::body_length_margin;
constexpr double half_width=lab26::body_half_width+lab26::body_width_margin;
const double radius=std::hypot(lab26::body_half_length,lab26::body_half_width);
double distance(Point a,Point b){return std::hypot(a.x-b.x,a.y-b.y);}
double clamp(double v,double bound){return std::max(-bound,std::min(bound,v));}
double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
}

std::vector<lab26::ObstaclePoint> reframe(const std::vector<lab26::ObstaclePoint>& scan,Pose sampled,Pose current) {
  const double a=sampled.yaw-current.yaw,c=std::cos(a),s=std::sin(a);
  const double dx=sampled.x-current.x,dy=sampled.y-current.y;
  const double tx=std::cos(current.yaw)*dx+std::sin(current.yaw)*dy,ty=-std::sin(current.yaw)*dx+std::cos(current.yaw)*dy;
  std::vector<lab26::ObstaclePoint> result;result.reserve(scan.size());
  for(const auto& p:scan)result.push_back({c*p.x-s*p.y+tx,s*p.x+c*p.y+ty});
  return result;
}

std::vector<Point> Navigator::obstacles(Pose pose,const std::vector<lab26::ObstaclePoint>& scan) const {
  // Aggregate height returns in 10 cm cells. Isolated single returns do not
  // invent whole walls; the independent raw-point swept guard remains active.
  struct Cell { int count=0;double x=0,y=0; };
  std::unordered_map<int,Cell> cells;
  const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
  for(const auto& p:scan){
    const double x=pose.x+c*p.x-s*p.y,y=pose.y+s*p.x+c*p.y;
    if(x< -1||x>distance_+2||std::abs(y)>4.5)continue;
    const int ix=int(std::floor((x+1)/resolution)),iy=int(std::floor((y+4.5)/resolution));
    auto& cell=cells[ix*1000+iy];++cell.count;cell.x=x;cell.y=y;
  }
  std::vector<Point> result;
  for(const auto& item:cells){
    int support=0;for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y){auto it=cells.find(item.first+x*1000+y);if(it!=cells.end())support+=it->second.count;}
    if(support>=3)result.push_back({item.second.x,item.second.y});
  }
  // Stable ordering keeps equal-cost decisions deterministic across frames.
  std::sort(result.begin(),result.end(),[](Point a,Point b){return a.x==b.x?a.y<b.y:a.x<b.x;});
  return result;
}

bool Navigator::visible(Point a,Point b,const std::vector<Point>& points) const {
  // A 670 x 580 mm rectangle aligned with this segment.
  const double dx=b.x-a.x,dy=b.y-a.y,length=std::hypot(dx,dy);
  const double c=length>1e-9?dx/length:1,s=length>1e-9?dy/length:0;
  for(const auto& p:points){
    const double x=c*(p.x-a.x)+s*(p.y-a.y),y=-s*(p.x-a.x)+c*(p.y-a.y);
    if(x> -half_length&&x<length+half_length&&std::abs(y)<half_width)return false;
  }
  return true;
}

bool Navigator::plan(Pose pose,const std::vector<Point>& points) {
  ++replans_;
  const Point start{pose.x,pose.y},goal{distance_,0};
  if(visible(start,goal,points)){path_={goal};return true;}
  // Bounded local search: do not solve a dead end by taking an unobserved,
  // arbitrarily long excursion behind the initial position.
  const int width=int((distance_+1.5)/resolution)+1,height=81,count=width*height;
  auto position=[&](int id){return Point{(id%width)*resolution-.5,(id/width)*resolution-4};};
  auto index=[&](Point p){const int x=int(std::lround((p.x+.5)/resolution)),y=int(std::lround((p.y+4)/resolution));return x>=0&&x<width&&y>=0&&y<height?y*width+x:-1;};
  const int first=index(start),last=index(goal);
  if(first<0||last<0){path_.clear();return false;}
  std::vector<unsigned char> occupied(count,0),closed(count,0);
  std::vector<double> clearance(count,2.0);
  // Four rectangle orientations cover all eight travel directions. Opposite
  // directions share a footprint; no circumscribed-circle inflation is used.
  const double cs[4]={1,std::sqrt(.5),0,-std::sqrt(.5)};
  const double sn[4]={0,std::sqrt(.5),1,std::sqrt(.5)};
  for(const auto& p:points){
    const int cx=int(std::lround((p.x+.5)/resolution)),cy=int(std::lround((p.y+4)/resolution));
    for(int y=std::max(0,cy-9);y<=std::min(height-1,cy+9);++y)for(int x=std::max(0,cx-9);x<=std::min(width-1,cx+9);++x){
      const int id=y*width+x;const double d=distance(position(id),p);clearance[id]=std::min(clearance[id],d);
      if(d<std::hypot(half_length,half_width))for(int angle=0;angle<4;++angle){
        const double dx=p.x-position(id).x,dy=p.y-position(id).y;
        if(std::abs(cs[angle]*dx+sn[angle]*dy)<half_length&&std::abs(-sn[angle]*dx+cs[angle]*dy)<half_width)
          occupied[id]|=1<<angle;
      }
    }
  }
  if(occupied[last]&1){path_.clear();return false;}
  for(const auto& p:points){
    const double dx=p.x-pose.x,dy=p.y-pose.y,c=std::cos(pose.yaw),s=std::sin(pose.yaw);
    if(std::abs(c*dx+s*dy)<half_length&&std::abs(-s*dx+c*dy)<half_width){path_.clear();return false;}
  }
  std::vector<double> cost(count,std::numeric_limits<double>::infinity());std::vector<int> parent(count,-1);
  using Entry=std::pair<double,int>;std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
  cost[first]=0;open.push({distance(start,goal),first});
  while(!open.empty()){
    int id=open.top().second;open.pop();if(closed[id])continue;closed[id]=1;if(id==last)break;
    const int x=id%width,y=id/width;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
      if(!dx&&!dy)continue;
      const int xx=x+dx,yy=y+dy;
      if(xx<0||xx>=width||yy<0||yy>=height)continue;
      const int next=yy*width+xx;if(closed[next])continue;
      const int angle=dy==0?0:dx==0?2:dx*dy>0?1:3,mask=1<<angle;
      if(occupied[next]&mask)continue;
      if(id==first){if(!visible(start,position(next),points))continue;}
      else if(occupied[id]&mask)continue;
      const double step=resolution*std::hypot(dx,dy);
      const double side_cost=detour_side_&&detour_side_*position(next).y<-.15?1.5:0;
      double bend=0;
      if(parent[id]>=0){
        const Point before=position(parent[id]),current=position(id);
        const double bx=current.x-before.x,by=current.y-before.y;
        bend=.05*(1-(bx*dx+by*dy)/(std::hypot(bx,by)*std::hypot(dx,dy)));
      }
      const double g=cost[id]+step*(1+.12/std::max(.08,clearance[next]-radius)+.25*std::abs(position(next).y)+side_cost)+bend;
      if(g<cost[next]){cost[next]=g;parent[next]=id;open.push({g+distance(position(next),goal),next});}
    }
  }
  if(parent[last]<0){path_.clear();return false;}
  std::vector<Point> raw;for(int id=last;id!=first;id=parent[id])raw.push_back(position(id));
  std::reverse(raw.begin(),raw.end());raw.back()=goal;
  // Shortcut only when the same oriented rectangle fits along the segment.
  path_.clear();Point anchor=start;
  for(size_t i=0;i<raw.size();){size_t far=i;while(far+1<raw.size()&&visible(anchor,raw[far+1],points))++far;
    if(!visible(anchor,raw[far],points)){path_.clear();return false;}
    path_.push_back(raw[far]);anchor=raw[far];i=far+1;
  }
  if(!detour_side_)for(const auto& p:path_)if(std::abs(p.y)>.20){detour_side_=p.y>0?1:-1;break;}
  return !path_.empty();
}

Velocity Navigator::recover(Pose pose,const std::vector<lab26::ObstaclePoint>& scan,Velocity previous,Velocity feedback,double now,double dt,const char* reason) {
  auto feasible=[&](Velocity target,Velocity& output){
    output={previous.v+clamp(target.v-previous.v,.20*dt),previous.w+clamp(target.w-previous.w,.60*dt)};
    return !lab26::sweptFootprint(scan,target.v,target.w).blocked&&
      !lab26::sweptFootprint(scan,output.v,output.w).blocked&&
      !lab26::sweptFootprint(scan,feedback.v,feedback.w).blocked;
  };
  Velocity output;
  if(now<recovery_until_&&feasible(recovery_command_,output))return output;
  const double side=detour_side_?detour_side_:1;
  std::vector<Velocity> choices={{.12,0},{0,side*.25},{0,-side*.25},{-.12,0},{-.10,side*.20}};
  // A failed forward/turn attempt gets one short retreat before trying again.
  const bool stalled=recovery_attempts_>0&&distance({pose.x,pose.y},{recovery_origin_.x,recovery_origin_.y})<.08;
  if(stalled&&recovery_command_.v>=0)std::rotate(choices.begin(),choices.begin()+3,choices.end());
  if(recovery_command_.v<0)choices={{0,side*.25},{0,-side*.25},{-.10,side*.20},{.12,0},{-.12,0}};
  for(const auto& target:choices)if(feasible(target,output)){
    recovery_command_=target;recovery_until_=now+1.5;++recovery_attempts_;
    recovery_origin_=pose;
    state_=target.v>0?"recover_forward":target.v<0?"recover_reverse":"recover_turn";reason_=reason;
    progress_time_=now;progress_pose_={pose.x,pose.y};
    path_.clear();avoiding_=true;rejoining_=false;last_plan_=-1;last_check_=-1;
    return output;
  }
  recovery_until_=-1;state_="blocked";reason_="No collision-free forward, turning or reverse motion";
  return {};
}

Velocity Navigator::step(Pose pose,const std::vector<lab26::ObstaclePoint>& scan,
                         Velocity previous,Velocity feedback,double now) {
  if(finished_||failed_)return {};
  if(!std::isfinite(now)||!std::isfinite(pose.x)||!std::isfinite(pose.y)||!std::isfinite(pose.yaw)){
    failed_=true;state_="fault";reason_="Invalid navigation input";return {};
  }
  const double dt=last_tick_<0?.02:std::max(0.0,std::min(.1,now-last_tick_));last_tick_=now;
  remaining_=std::hypot(distance_-pose.x,pose.y);
  auto recovery=[&](const char* why){return recover(pose,scan,previous,feedback,now,dt,why);};
  if(recovery_command_.v<0){
    const double backed=-(pose.x-recovery_origin_.x)*std::cos(recovery_origin_.yaw)-(pose.y-recovery_origin_.y)*std::sin(recovery_origin_.yaw);
    if(recovery_until_>=0&&(backed>=.10||now>=recovery_until_)){
      recovery_until_=-1;
      return recovery("Turning to find a route after short retreat");
    }
  }
  if(now<recovery_until_)return recovery(reason_.c_str());
  if(remaining_<=.13&&std::abs(pose.y)<=.03&&!goal_align_)goal_align_=true;
  if(goal_align_&&(remaining_>.20||std::abs(pose.y)>.03)){goal_align_=false;settled_since_=-1;}
  if(goal_align_){
    state_="aligning_goal";reason_.clear();
    // Rotate to the initial heading at the endpoint without a timed wait first.
    const double error=wrap(-pose.yaw);
    if(std::abs(error)<=8.0*std::acos(-1.0)/180.0){
      if(std::abs(feedback.w)>.03||std::abs(feedback.v)>.02){settled_since_=-1;return {};}
      if(settled_since_<0)settled_since_=now;
      if(now-settled_since_>=.30){finished_=true;state_="finished";}
      return {};
    }
    settled_since_=-1;
    const double target=clamp(1.2*error-.45*feedback.w,.35);
    const double turn=previous.w+clamp(target-previous.w,.60*dt);
    if(lab26::sweptFootprint(scan,0,target).blocked||lab26::sweptFootprint(scan,0,turn).blocked||
       lab26::sweptFootprint(scan,feedback.v,feedback.w).blocked)
      return recovery("Final heading rotation is obstructed");
    return {0,turn};
  }
  if(progress_time_<0){progress_time_=now;progress_pose_={pose.x,pose.y};}
  if(distance(progress_pose_,{pose.x,pose.y})>.15){progress_time_=now;progress_pose_={pose.x,pose.y};}
  if(now-progress_time_>8)return recovery("Seeking a new route after limited progress");
  const Point here{pose.x,pose.y};
  // Figure 4.1 measures 1 m from the bumper, 0.325 m ahead of the lidar.
  int front_left=0,front_right=0;double forward_distance=6;
  for(const auto& p:scan)if(p.x>.325&&std::abs(p.y)<=.35){
    forward_distance=std::min(forward_distance,p.x);
    if(p.x<=1.325){if(p.y>=0)++front_left;else ++front_right;}
  }
  if(!avoiding_&&front_left+front_right>=3){
    avoiding_=true;rejoining_=false;path_.clear();last_check_=-1;last_plan_=-1;direct_clear_since_=-1;
    const int side=front_left<front_right?1:-1;
    entry_turn_side_=side;entry_yaw_=pose.yaw;
    detour_side_=std::cos(pose.yaw)>=0?side:-side;
  }
  if(rejoining_){
    rejoin_target_.x=std::max(rejoin_target_.x,std::min(distance_,pose.x+.6));
    if(std::abs(pose.y)<.03&&std::abs(wrap(pose.yaw))<.06){
      if(rejoin_settled_since_<0)rejoin_settled_since_=now;
      if(now-rejoin_settled_since_>=.30)rejoining_=false;
    }else rejoin_settled_since_=-1;
  }
  if(!avoiding_&&!rejoining_&&std::abs(pose.y)>.05){
    rejoining_=true;rejoin_target_={std::min(distance_,pose.x+.6),0};rejoin_settled_since_=-1;
  }
  if(!avoiding_){path_=rejoining_?std::vector<Point>{rejoin_target_,{distance_,0}}:std::vector<Point>{{distance_,0}};recovery_attempts_=0;}
  while(path_.size()>1&&distance(here,path_.front())<.20)path_.erase(path_.begin());
  if(avoiding_&&(last_check_<0||now-last_check_>=.20)){
    last_check_=now;const auto points=obstacles(pose,scan);
    Point rejoin;bool can_rejoin=false;
    for(double ahead=.6;ahead<=1.6&&!can_rejoin;ahead+=.2){
      rejoin={std::min(distance_,std::max(0.0,pose.x)+ahead),0};
      // Reaching a point before the obstacle is not a completed bypass.
      // Check the outgoing original-line segment too, over the trigger range.
      const Point onward{std::min(distance_,rejoin.x+1.0),0};
      can_rejoin=front_left+front_right<3&&visible(here,rejoin,points)&&visible(rejoin,onward,points);
    }
    if(can_rejoin){
      if(direct_clear_since_<0)direct_clear_since_=now;
      if(now-direct_clear_since_>=.30){rejoin_target_=rejoin;rejoining_=true;path_={rejoin,{distance_,0}};detour_side_=0;avoiding_=false;}
    } else direct_clear_since_=-1;
    bool valid=!path_.empty();Point from=here;
    if(avoiding_)for(const auto& p:path_){if(!visible(from,p,points)){valid=false;break;}from=p;}
    // A stale route must not cancel an emerging turn for a timed straight
    // recovery just because the previous plan was less than 0.5 s ago.
    if(!valid){
      last_plan_=now;if(!plan(pose,points))return recovery("Searching around an obstructed route");
    }
  }
  if(path_.empty())return recovery("Searching around an obstructed route");
  // Look along the next segment before reaching a corner, rather than
  // jumping the target to the opposite side when a waypoint is removed.
  Point target=path_.front();
  const double leg=distance(here,target);
  const double look=.55+.35*std::max(previous.v,feedback.v);
  if(leg>look){target={here.x+(target.x-here.x)*look/leg,here.y+(target.y-here.y)*look/leg};}
  else if(path_.size()>1){
    const double next=distance(target,path_[1]);
    const double fraction=std::min(1.0,(look-leg)/std::max(.001,next));
    const Point ahead{target.x+(path_[1].x-target.x)*fraction,target.y+(path_[1].y-target.y)*fraction};
    if(visible(here,ahead,obstacles(pose,scan)))target=ahead;
  }
  // Track the original line locally even after rejoining; a far endpoint
  // produces almost no steering for a small but persistent lateral offset.
  if(!avoiding_&&!rejoining_)target={std::min(distance_,pose.x+look),0};
  double heading=wrap(std::atan2(target.y-pose.y,target.x-pose.x)-pose.yaw);
  const bool front_near=front_left+front_right>=3;
  // Enter the bypass as soon as the bumper's 1 m zone is occupied. A
  // nearly tangent path must not postpone steering until the wheel is close.
  if(avoiding_&&front_near&&entry_turn_side_){
    const double entry_error=wrap(entry_yaw_+entry_turn_side_*.60-pose.yaw);
    if(entry_turn_side_*entry_error>0&&entry_turn_side_*heading>=-.05&&std::abs(heading)<std::abs(entry_error))heading=entry_error;
  }
  // CAN feedback reveals angular momentum not yet visible in the latest pose.
  const double anticipated=wrap(heading-.25*feedback.w);
  // Arrive at the trigger zone slowly enough to steer while advancing.
  // Distant points may reduce speed, but never select an early detour.
  const double braking_space=std::max(0.0,forward_distance-1.325-.25*std::max(previous.v,feedback.v));
  const double approach_speed=std::sqrt(.25*.25+2*.20*braking_space);
  Velocity selected;bool found=false;
  for(double speed:{1.0,.80,.60,.44,.30,.20,.10}){
    const double approach=std::max(.04,.7*(remaining_-.10));
    double v=std::min({speed,approach,approach_speed}),w=0;
    if(avoiding_||rejoining_)v=std::min(v,.05+.65*std::max(0.0,1-std::abs(heading)/.65));
    if(std::abs(heading)>.65)v=std::min(v,.05);
    // Turn response must not disappear when slowing down: otherwise a tight
    // approach advances into the corner faster than it can change heading.
    w=clamp((front_near?1.45:1.2)*anticipated-.15*feedback.w,.80);
    if(path_.size()>1)v=std::min(v,.12+.60*leg);
    // Couple forward bypass speed and turn rate instead of swivelling first
    // while the body has barely advanced. Recovery turns remain independent.
    if(avoiding_||rejoining_)w=clamp(w,v/.35);
    // If this turn does not fit, try a shallower turn and straight advance
    // before rejecting forward motion. Never turn to the opposite side here.
    for(double turn_scale:{1.0,.5,.25,0.0}){
      if(front_near&&std::abs(heading)>.10&&turn_scale<.5)continue;
      Velocity requested{v,w*turn_scale};
      Velocity limited{previous.v+clamp(v-previous.v,.20*dt),previous.w+clamp(requested.w-previous.w,.60*dt)};
      if(lab26::sweptFootprint(scan,requested.v,requested.w).blocked||
         lab26::sweptFootprint(scan,limited.v,limited.w).blocked||
         lab26::sweptFootprint(scan,previous.v,previous.w).blocked||
         lab26::sweptFootprint(scan,feedback.v,feedback.w).blocked)continue;
      selected=limited;found=true;break;
    }
    if(found)break;
  }
  if(!found)return recovery("Seeking a collision-free direction");
  reason_.clear();state_=std::abs(heading)>.65?"aligning":rejoining_?"rejoining":path_.size()>1?"following_detour":"returning";
  if(!avoiding_&&!rejoining_&&std::abs(pose.y)<.03&&std::abs(wrap(pose.yaw))<.06)state_="straight";
  return selected;
}
}
