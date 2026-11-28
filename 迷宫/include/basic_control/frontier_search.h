#pragma once
#include "basic_control/footprint.h"
#include "basic_control/motion.h"
#include <map>
#include <queue>

namespace lab26 {
// The graph contains connections seen across the chassis width, plus the
// measured trail. It supplies waypoints; current scans authorize motion.
class FrontierSearch {
  using Key=std::pair<int,int>;
  struct Cell {
    unsigned edges=0,search=0;
    bool approached=false,candidate=false;
    double lastPass=-1.,retryAfter=0.,trailDistance=2.;
    double cost=0.;
  };
  static constexpr double resolution=.30;
  std::map<Key,Cell> cells_;
  static Key key(double x,double y){return {int(std::floor(x/resolution)),int(std::floor(y/resolution))};}
  static Pose center(Key k){return {(k.first+.5)*resolution,(k.second+.5)*resolution,0.};}
  static unsigned bit(int dx,int dy){return 1u<<((dy+1)*3+dx+1);}
  void link(Key a,Key b){
    const int dx=b.first-a.first,dy=b.second-a.second;
    if(a==b||std::abs(dx)>1||std::abs(dy)>1)return;
    cells_[a].edges|=bit(dx,dy);cells_[b].edges|=bit(-dx,-dy);
  }
  bool active_=false,havePose_=false,haveGoal_=false,haveWaypoint_=false,survey_=false;
  Key goal_{};
  Pose waypoint_{};
  Pose progress_{},sample_{};
  double progressAt_=0.,travel_=0.;
  unsigned search_=0;
  // The shortest route through observed connections can bend behind the
  // robot. Use graph distance, rather than the bearing to a distant goal.
  std::vector<Key> distances(Key root){
    using Entry=std::pair<double,Key>;
    std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
    ++search_;
    queue.push({0.,root});auto& origin=cells_[root];origin.search=search_;origin.cost=0.;
    std::vector<Key> reached;
    while(!queue.empty()){
      const Entry current=queue.top();queue.pop();
      const double cost=current.first;const Key at=current.second;
      const auto& cell=cells_[at];if(cost>cell.cost)continue;
      reached.push_back(at);
      for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++){
        if(!(cell.edges&bit(dx,dy)))continue;
        const Key next{at.first+dx,at.second+dy};auto& neighbor=cells_[next];
        const double distance=cost+resolution*(dx&&dy?1.41421356237:1.);
        if(neighbor.search==search_&&neighbor.cost<=distance)continue;
        neighbor.search=search_;neighbor.cost=distance;queue.push({distance,next});
      }
    }
    return reached;
  }
  void deferGoal(double now){
    // Adjacent endpoints often describe the same obstructed opening.
    if(haveGoal_)for(int y=-1;y<=1;y++)for(int x=-1;x<=1;x++){
      auto it=cells_.find({goal_.first+x,goal_.second+y});
      if(it!=cells_.end())it->second.retryAfter=now+18.;
    }
    haveGoal_=haveWaypoint_=false;
  }
 public:
  void activate(){active_=true;}
  // A full commanded turn is allowed to finish. Repeated turns at the
  // same spot and repeated circuits both need a different destination.
  bool observePose(Pose p,double now){
    const bool first=!havePose_;
    if(first){sample_=progress_=p;progressAt_=now;havePose_=true;}
    const double moved=std::hypot(p.x-sample_.x,p.y-sample_.y);
    if(first||moved>=.10){
      const auto from=key(sample_.x,sample_.y),at=key(p.x,p.y);
      if(moved<1.)link(from,at);
      travel_+=moved;sample_=p;
      for(int dy=-6;dy<=6;dy++)for(int dx=-6;dx<=6;dx++){
        const Key k{at.first+dx,at.second+dy};
        const auto pos=center(k);const double distance=std::hypot(pos.x-p.x,pos.y-p.y);
        if(distance>1.8)continue;
        auto& cell=cells_[k];
        cell.trailDistance=std::min(cell.trailDistance,distance);
        if(distance<.72)cell.approached=true;
      }
      if(from!=at){
        auto& current=cells_[at];
        if(current.lastPass>=0.&&travel_-current.lastPass>3.)active_=true;
        current.lastPass=travel_;
      }
    }
    if(std::hypot(p.x-progress_.x,p.y-progress_.y)>.28){progress_=p;progressAt_=now;}
    if(now-progressAt_<9.)return false;
    progressAt_=now;progress_=p;active_=true;deferGoal(now);return true;
  }
  template<size_t N> void observeLanes(Pose p,const std::array<double,N>& depth){
    const Key here=key(p.x,p.y);
    cells_[here].approached=true;
    for(size_t i=0;i<N;i++){
      const double a=p.yaw+2*pi*i/N,c=std::cos(a),s=std::sin(a);
      Key last=here;
      for(double d=.12;d<=depth[i];d+=.12){
        const Key at=key(p.x+d*c,p.y+d*s);
        if(at==last)continue;
        link(last,at);last=at;
        auto& cell=cells_[at];
        if(d<=.78)cell.approached=true;
        if(d>=.84)cell.candidate=true;
      }
    }
  }
  // A waypoint stays fixed in the odometry frame during a turn. Select the
  // farthest visible point along the graph route to an unapproached opening.
  template<size_t N,class Visible>
  bool heading(Pose p,double now,const std::array<double,N>& depth,Visible visible,double& bearing){
    if(!active_)return false;
    const Key here=key(p.x,p.y);
    auto reachable=[&](Pose pos){
      const double dx=pos.x-p.x,dy=pos.y-p.y,d=std::hypot(dx,dy);
      if(d<.30||d>1.65)return false;
      const double a=std::atan2(dy,dx);
      if(!visible(wrap(a-p.yaw),d))return false;
      bearing=a;return true;
    };
    const auto endpoint=center(goal_);
    if(haveGoal_&&(survey_?std::hypot(endpoint.x-p.x,endpoint.y-p.y)<.48:cells_[goal_].approached))
      haveWaypoint_=haveGoal_=false;
    if(haveWaypoint_&&reachable(waypoint_))return true;
    haveWaypoint_=false;
    if(!haveGoal_){
      double best=std::numeric_limits<double>::infinity(),oldest=travel_;
      Key revisit{};bool canRevisit=false;
      for(const Key& at:distances(here)){
        const auto& cell=cells_[at];if(cell.retryAfter>now)continue;
        if(cell.cost>1.5&&cell.lastPass>=0.&&cell.lastPass<oldest){
          oldest=cell.lastPass;revisit=at;canRevisit=true;
        }
        if(!cell.candidate||cell.approached)continue;
        const auto pos=center(at);
        const double a=std::atan2(pos.y-p.y,pos.x-p.x);
        const double score=cell.cost+.60*(1.-std::cos(a-p.yaw))-3.*cell.trailDistance;
        if(score<best){best=score;goal_=at;haveGoal_=true;}
      }
      survey_=!haveGoal_;
      if(survey_&&canRevisit){goal_=revisit;haveGoal_=true;}
      if(!haveGoal_)return false;
    }
    distances(goal_);
    // Snap costs to the map, but steer to a point on an actual observed lane.
    // A rounded grid centre can sit to the wrong side of a narrow corner.
    double best=std::numeric_limits<double>::infinity();
    for(size_t i=0;i<N;i++){
      const double a=2*pi*i/N,c=std::cos(p.yaw+a),s=std::sin(p.yaw+a);
      for(double d=.48;d<=depth[i]&&d<=1.60;d+=.24){
        const Pose aim{p.x+d*c,p.y+d*s,0.};
        const auto it=cells_.find(key(aim.x,aim.y));
        if(it==cells_.end()||it->second.search!=search_)continue;
        const double score=it->second.cost+.30*d+.24*(1.-std::cos(a));
        if(score<best){best=score;waypoint_=aim;haveWaypoint_=true;}
      }
    }
    if(haveWaypoint_&&reachable(waypoint_))return true;
    deferGoal(now);return false;
  }
};
}
