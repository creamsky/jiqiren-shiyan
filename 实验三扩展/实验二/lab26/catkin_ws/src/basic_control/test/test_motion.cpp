#include "basic_control/motion.h"
#include "basic_control/rate_limit.h"
#include "basic_control/manual_input.h"
#include "basic_control/footprint.h"
#include "basic_control/dwa_guard.h"
#include "basic_control/side_exit.h"
#include "basic_control/route_retreat.h"
#include "basic_control/scan_alignment.h"
#include "basic_control/path_tracker.h"
#include "basic_control/ground_returns.h"
#include "basic_control/clearing_rays.h"
#include <iostream>
#include <cstdlib>
#include <stdexcept>
void require(bool ok,const char* msg) { if(!ok) { std::cerr<<"FAIL: "<<msg<<std::endl;std::exit(1); } }
void testRetreat(){
  using namespace lab26;
  RouteRetreat recorded;
  for(int i=0;i<=20;++i)recorded.observe({.03*i,0,0},{.15,0},{.15,0},.2*i);
  std::vector<ObstaclePoint> world{{1.05,0}};
  for(double x=.53;x<1.2;x+=.02){world.push_back({x,.31});world.push_back({x,-.31});}
  for(double y=-1.;y<=1.;y+=.025)world.push_back({-1.,y});
  for(double x=-2.;x<=4.;x+=.025){world.push_back({x,2.});world.push_back({x,-2.});}
  const Pose start{.60,0,0};
  for(int i=1;i<=5;++i)recorded.observe(start,{},{},4.+i*.2);
  const auto scan=lab3::reframe(world,{},{start.x,0,0});
  SideExit exit;
  require(!exit.select(scan,start,3.,0.,5.),"Cul-de-sac initially has no side exit");
  RouteRetreat from_rest;
  require(!from_rest.begin(start,{},scan,5.) && from_rest.beginTurnSpace(start,{},scan,3.,0.,5.),
          "A fresh goal can back to a verified turn without an invented driving history");
  require(from_rest.step(start,{},scan,5.02).v<0,"Use reverse when it creates a checked exit");
  auto intrusion=scan;intrusion.push_back({-.41,0});
  require(from_rest.step(start,{},intrusion,5.04).v==0 && !from_rest.canContinue(start,intrusion),
          "Live rear occupancy immediately vetoes the inferred turning space");
  RouteRetreat blind_reverse;
  require(!blind_reverse.beginTurnSpace(start,{},{{-2,0}},3.,0.,5.),
          "A missing approach cannot authorize reverse into sparse rear observations");
  RouteRetreat stale=recorded, unknown=recorded, interrupted=recorded, moved=recorded;
  require(!stale.begin(start,{},scan,30.),"Expired breadcrumbs must not authorize backing");
  require(!unknown.begin(start,{}, {},5.),"Rear occlusion invalidates recovery");
  const std::vector<ObstaclePoint> one_ray{{-2.,0.}};
  require(!unknown.begin(start,{},one_ray,5.),"A single rear beam cannot cover a vehicle width");
  require(!moved.begin({.6,.18,0},{},scan,5.),"Do not back along a displaced trajectory");
  require(!moved.begin(start,{.12,0},scan,5.),"Wait for chassis standstill before changing direction");
  RouteRetreat shorter=recorded;
  auto limited=scan;limited.push_back({-.70,0});
  require(shorter.begin(start,{},limited,5.),"A farther rear obstacle still leaves a usable short retreat");
  for(int i=1;i<=80 && shorter.phase()==RouteRetreat::Phase::Backing;++i){
    const double travel=.005*i;
    const auto updated=lab3::reframe(limited,{},{-travel,0,0});
    shorter.step({start.x-travel,0,0},{-.1,0},updated,5.+.05*i);
  }
  require(shorter.phase()==RouteRetreat::Phase::Aborted && shorter.backed()<.31,
          "Stop before reaching a new obstacle on a previously clear route");
  require(!shorter.canContinue({start.x-.4,.2,0},scan),"Never resume from an unrelated pose");
  require(interrupted.begin(start,{},scan,5.),"Valid approach with rear returns is eligible");
  RouteRetreat jumped=interrupted, blind=interrupted, stuck=interrupted;
  require(jumped.step({.26,0,0},{},scan,5.02).v==0 &&
          jumped.phase()==RouteRetreat::Phase::Aborted,"A localization jump must not masquerade as completed retreat");
  require(blind.step(start,{},one_ray,5.1).v==0 && blind.phase()==RouteRetreat::Phase::Aborted,
          "Loss of rear coverage while backing must stop the robot");
  for(int i=1;i<=17;++i)stuck.step(start,{},scan,5.+i*.1);
  require(stuck.phase()==RouteRetreat::Phase::Aborted,"Detect commanded reverse without measured progress");
  auto changed=scan;changed.push_back({-.42,0});
  require(interrupted.step(start,{},changed,5.1).v==0 &&
          interrupted.phase()==RouteRetreat::Phase::Aborted,"A new rear obstacle stops recovery immediately");
  require(recorded.begin(start,{},scan,5.),"Detected dead end should trigger recent-route retreat");
  Pose p=start;Command previous;
  for(int i=1;i<=350 && recorded.phase()==RouteRetreat::Phase::Backing;++i){
    const auto observation=lab3::reframe(world,{},{p.x,p.y,p.yaw});
    const auto target=recorded.step(p,previous,observation,5.+i*.02);
    if(recorded.backed()>=.12 && exit.select(observation,p,3.,0.,5.+i*.02,recorded.failedHeading())){
      recorded.forkSeen();break;
    }
    auto cmd=guardDwa(observation,target,previous,previous,.02,.2,.6).command;
    require(cmd.v>=-.100001&&cmd.v<=0&&std::abs(cmd.w)<1e-6,"A straight recorded approach stays straight");
    p.x+=cmd.v*.02;previous=cmd;
  }
  require(recorded.phase()==RouteRetreat::Phase::SeekExit && start.x-p.x>.12,
          "Reach the observed fork");
  const auto fork=lab3::reframe(world,{},{p.x,p.y,p.yaw});
  require(exit.select(fork,p,3.,0.,12.,recorded.failedHeading()),"Find another direction from the recorded approach");
  SideExit unseen;
  require(!unseen.select({},p,3.,0.,12.,recorded.failedHeading()),"An unseen side passage cannot release recovery");
  require(!recorded.formerApproachOpen(fork,p,3.,0.),"Do not resume the dead end just because its near segment cleared");
  std::vector<ObstaclePoint> reopened;
  for(double y=-2.;y<=2.;y+=.025)reopened.push_back({2.,y});
  require(recorded.formerApproachOpen(reopened,p,3.,0.),"Allow the same route once the previous obstruction clears");
  recorded.remember(p,recorded.failedHeading());recorded.remember(p,exit.heading());
  SideExit other;
  if(other.select(fork,p,3.,0.,12.,recorded.failedHeading(),recorded.triedAt(p)))
    require(std::abs(wrap(other.heading()-exit.heading()))>=.75,"Skip the tried branch on revisiting a fork");
  recorded.exitSelected(12.);
  bool passed=false;
  for(int i=1;i<=500&&exit.phase()!=SideExit::Phase::Idle;++i){
    const auto observation=lab3::reframe(world,{},{p.x,p.y,p.yaw});
    const auto cmd=exit.step(observation,p,previous,12.+i*.02);
    p.yaw+=cmd.w*.02;p.x+=cmd.v*std::cos(p.yaw)*.02;p.y+=cmd.v*std::sin(p.yaw)*.02;
    previous=cmd;passed|=std::abs(p.y)>.15;
  }
  require(passed,"After retracing, pivot and traverse the alternative branch");
  SideExit left,right;
  require(left.select(reopened,{},3.,0.,20.,0.)==false,"Forward-only rays cannot verify either side branch");
  std::vector<ObstaclePoint> junction;
  for(int i=0;i<360;++i)junction.push_back({3*std::cos(pi*i/180),3*std::sin(pi*i/180)});
  junction.push_back({.55,0});
  RouteRetreat choices;
  require(left.select(junction,{},3.,0.,20.,0.),"Find the first observed branch");
  choices.remember({},left.heading());
  require(right.select(junction,{},3.,0.,20.,0.,choices.triedAt({})) &&
          left.heading()*right.heading()<0,"Choose the opposite branch when this fork is revisited");
  for(int i=0;i<=20;++i)recorded.observe({2.+.03*i,0,0},{.15,0},{.15,0},30.+.2*i);
  require(recorded.begin({2.6,0,0},{},scan,34.1),"A different dead end on the same goal can be retraced");
}

void testLongRetreat(bool pivot){
  using namespace lab26;
  RouteRetreat route;
  Pose p;double t=0.;
  auto measure=[&](Command v,int ticks){
    for(int i=0;i<ticks;++i){
      route.observe(p,v,v,t);
      p.x+=v.v*.05*std::cos(p.yaw+v.w*.025);
      p.y+=v.v*.05*std::sin(p.yaw+v.w*.025);p.yaw+=v.w*.05;t+=.05;
    }
  };
  measure({.15,0},170);
  if(pivot)measure({0,pi/6},60);
  else measure({.15,.15},140);
  measure({.15,0},125);
  measure({},22);
  std::vector<ObstaclePoint> scan;
  for(int i=0;i<360;++i)scan.push_back({3*std::cos(i*pi/180),3*std::sin(i*pi/180)});
  require(route.begin(p,{},scan,t),"The entire goal retains its travelled approach");
  const Pose end=p;Command last;bool held=false;
  for(int i=0;i<3500 && route.phase()==RouteRetreat::Phase::Backing;++i){
    t+=.02;
    if(!held && route.backed()>.65){
      auto occupied=scan;occupied.push_back({-.40,0});
      const auto stop=route.step(p,last,occupied,t);
      require(stop.v==0 && stop.w==0 && !route.canContinue(p,occupied),"Stop for new occupancy far down the return");
      require(route.canContinue(p,scan),"Keep the route cursor while waiting for the rear to clear");
      t+=1.;route.resumeBacking(p,t);last={};held=true;
    }
    const Command target=route.step(p,last,scan,t);
    const auto cmd=guardDwa(scan,target,last,last,.02,.2,.6).command;
    p.x+=.02*cmd.v*std::cos(p.yaw+.01*cmd.w);p.y+=.02*cmd.v*std::sin(p.yaw+.01*cmd.w);
    p.yaw+=.02*cmd.w;last=cmd;
    if(pivot)require(std::min(std::abs(p.y),std::abs(p.x-end.x))<.06,"Do not cut across the recorded right angle");
  }
  std::cout<<(pivot?"Corner":"Curve")<<" return: "<<route.reason()<<" at "<<p.x<<","<<p.y
           <<" yaw="<<p.yaw<<" measured_reverse="<<route.backed()<<" m\n";
  require(held && route.phase()==RouteRetreat::Phase::SeekExit && std::hypot(p.x,p.y)<.05 &&
          route.backed()>2.,"Follow the whole approach home, through its curve or stopped turn");
}

void testRecheckDuringTurn(){
  lab3::Navigator nav(6.,true);
  lab3::Pose pose;lab3::Velocity prev;
  std::vector<lab26::ObstaclePoint> wall;
  for(double y=-3.;y<3.;y+=.012)wall.push_back({.65,y});
  double now=0.;
  for(int i=0;i<100 && std::string(nav.state())!="recover_turn";++i){
    prev=nav.step(pose,wall,prev,prev,now);now+=.02;
  }
  require(std::string(nav.state())=="recover_turn","Enter a search turn when the local window is sealed");
  bool advanced=false;
  for(int i=0;i<50;++i){
    prev=nav.step(pose,{},prev,prev,now);pose.yaw+=prev.w*.02;now+=.02;
    if(prev.v>.04){advanced=true;break;}
  }
  require(advanced,"Look again and move when the exit opens during a search turn");
  std::cout<<"Search turn released after "<<now-.02<<" s from test start\n";
}
void testGlobalRouteTracking(){
  using namespace lab26;
  GroundReturns ground;
  for(double x=-3.;x<=3.;x+=.08)for(double y=-3.;y<=3.;y+=.08)ground.observe(x,y,-.51+.01*x);
  ground.finish();
  ClearingRays old_wall,occupied;
  old_wall.add(5.,0,0.,.07,true);occupied.add(5.,0,0.,.07,true);occupied.add(.61,.003,-.40,.02,true);
  const auto free=old_wall.endpoints(),limited=occupied.endpoints();
  require(free.size()==1 && free[0].point.x>4.8 && limited.size()==1 && limited[0].point.x<.60 &&
          limited[0].offset==.07,"Clear a measured empty ray, never through a nearer low obstacle");
  require(ground.obstacle(1,0,-.40) && !ground.obstacle(1,0,-.49) &&
          !GroundReturns().obstacle(1,0,-.40),"Low returns require measured clearance above the local floor");
  require(arcClearance({{.9,0}},0,1.)<.6 && arcClearance({{.9,.65}},0,1.)==1.,
          "Use the footprint swept along the route for near and distant returns");
  for(int side:{-1,0,1}){
    const ObstaclePoint goal{3.,0.};
    Waypoints route{{0,0}};
    if(side){route.push_back({.65,side*.7});route.push_back({1.35,side*1.05});route.push_back({2.,side*.83});}
    route.push_back(goal);
    Waypoints world;
    if(side)for(double x=1.15;x<=1.48;x+=.045)for(double y=-.22;y<=.22;y+=.045)world.push_back({x,y});
    else for(double x=-1.;x<4.;x+=.1){world.push_back({x,.56});world.push_back({x,-.56});}
    PathTracker tracker;Pose pose;Command previous,actual;
    double min_gap=10.;int idle=0;size_t leg=1;
    for(int tick=0;tick<2600&&!tracker.finished;++tick){
      const double t=tick*.02,c=std::cos(pose.yaw),s=std::sin(pose.yaw);
      const auto points=lab3::reframe(world,{},{pose.x,pose.y,pose.yaw});
      for(auto q:points){
        const double gap=std::hypot(std::max(0.,std::abs(q.x)-body_half_length),std::max(0.,std::abs(q.y)-body_half_width));
        min_gap=std::min(min_gap,gap);
      }
      if(leg+1<route.size() && std::hypot(pose.x-route[leg].x,pose.y-route[leg].y)<.22)++leg;
      Waypoints updated{{pose.x,pose.y}};updated.insert(updated.end(),route.begin()+leg,route.end());
      const auto cmd=tracker.step(updated,pose,goal,points,{},previous,actual,t,.02,.20,.60);
      if(!side && tick>40 && pose.x<2.5 && cmd.v<.09)++idle;
      actual.v+=clip(cmd.v-actual.v,.20*.02);actual.w+=clip(cmd.w-actual.w,.60*.02);
      pose.x+=c*actual.v*.02;pose.y+=s*actual.v*.02;pose.yaw+=actual.w*.02;previous=cmd;
    }
    std::cout<<"Global path side="<<side<<" end="<<pose.x<<","<<pose.y<<" min_body_gap="<<min_gap<<" state="<<tracker.state<<"\n";
    require(tracker.finished && min_gap>.06 && idle==0,"Close the entire measured-pose path loop through both detours and a clear hallway");
  }
  PathTracker tracker;const Waypoints path{{0,0},{3.,0}};
  const auto collision=tracker.step(path,{},path.back(),{{.39,0}},{},{.32,0},{.32,0},1,.02,.2,.6);
  require(!collision.v && !collision.w,"A new obstacle vetoes a previously valid route even with a moving chassis");
  const auto away=tracker.step(path,{},path.back(),{{-.33,0}},{},{},{},2,.02,.2,.6);
  require(away.v>0 && away.w==0,"The global path can move straight away from a close rear return");
  PathTracker detour;NearPlaneRays observed;
  for(int i=0;i<360;++i){const double a=pi*i/180.;observed.add(4*std::cos(a),4*std::sin(a),0);}
  PathTracker empty_map;
  const auto first=empty_map.step({}, {},{7.,0},{{-.33,0}},{&observed,0,0,0},{},{},.2,.02,.2,.6);
  require(first.v>0. && first.w==0,"An empty global path can follow a verified local prefix toward a distant goal");
  const auto blind=empty_map.step({}, {},{7.,0},{{-.33,0}},{},{},{},.3,.02,.2,.6);
  require(!blind.v && !blind.w,"An empty global path plus unobserved free space must not authorize blind forward motion");
  const Waypoints post{{.70,0},{.70,.04},{.70,-.04}};
  const auto bypass=detour.step(path,{},path.back(),post,{&observed,0,0,0},{},{},3,.02,.2,.6);
  require(bypass.v==0 && std::abs(bypass.w)>0,"Choose a footprint-clear observed detour when the global path still points into a new post");
  PathTracker sealed;Waypoints wall;
  for(double y=-3.;y<=3.;y+=.03)wall.push_back({.54,y});
  const auto wait=sealed.step(path,{},path.back(),wall,{&observed,0,0,0},{},{},3,.02,.2,.6);
  require(wait.v==0 && wait.w==0,"A side opening with no collision-free connection across the wall is not a route");
}
int main() {
  using namespace lab26;
  testGlobalRouteTracking();
  testRetreat();
  testLongRetreat(false);testLongRetreat(true);testRecheckDuringTurn();
  const std::vector<ObstaclePoint> nose{{.330,0}},tail{{-.330,0}};
  require(guardDwa(nose,{-.10,0},{},{},.02,.20,.60).command.v<0 &&
          guardDwa(tail,{.10,0},{},{},.02,.20,.60).command.v>0,
          "A return at the opposite bumper cannot forbid pure translation away from it");
  require(guardDwa(nose,{.10,0},{},{},.02,.20,.60).command.v==0 &&
          guardDwa(tail,{-.10,0},{},{},.02,.20,.60).command.v==0 &&
          guardDwa(nose,{0,.4},{},{},.02,.20,.60).command.w==0,
          "Enforce the occupied bumper for approaching and rotating commands");
  SideExit straight;
  NearPlaneRays level,overhead;
  for(double y=-3.;y<=3.;y+=.025){level.add(8.,y,-.50);overhead.add(8.,y,3.5);}
  require(!observedCorridor({},0,.65) && observedCorridor({},0,.65,{&level,0,0,0}) &&
          !observedCorridor({},0,.65,{&overhead,0,0,0}),
          "Use only the near-plane segment of measured rays for free-space evidence");
  require(!observedCorridor(nose,0,.65,{&level,0,0,0}),"A nearer obstacle always vetoes long-ray evidence");
  straight.setRays({&level,0,0,0});
  require(approachingObstacle({.20,.006},{.20,0},.237,2.) &&
          !approachingObstacle({.20,.006},{.20,0},3.93,5.) &&
          !approachingObstacle({.20,.40},{.20,0},.237,2.) &&
          !approachingObstacle({.20,0},{.20,0},.23,.25),
          "Start looking for a side exit on a close straight approach, preserving far, turning, and arriving cases");
  require(approachSpeed({.20,.006},.237).v<.20 && approachSpeed({.184,0},.12).v==0,
          "Protect turning clearance even when the approach is still inside the emergency braking limit");
  const RayView measured_lane{&level,0,0,0};
  for(int turn_sign:{-1,1}){
    const Command slight_turn{.2,.05*turn_sign};
    require(guardDwa(tail,slight_turn,{},{},.02,.2,.6,true,measured_lane).command.v>0 &&
            guardDwa(tail,slight_turn,{},{},.02,.2,.6,true,measured_lane).command.w==0,
            "A nearby rear bumper must permit an observed straight goal-facing exit on either steering sign");
    require(guardDwa(tail,slight_turn,{},{},.02,.2,.6,true).command.v==0 &&
            guardDwa(tail,slight_turn,{},{},.02,.2,.6,false,measured_lane).command.v==0 &&
            guardDwa(tail,slight_turn,{0,.2},{0,.2},.02,.2,.6,true,measured_lane).command.v==0,
            "Unknown space, an off-axis goal or measured turning must not trigger straightening");
  }
  require(straight.selectForward({}, {},2,0,0,{}),"Open indoor range without nearby wall returns can support straight travel");
  straight.extendForward({}, {.34,0,0},2,0,1.);
  require(straight.step({}, {.64,0,0},{.20,0},2.).v>.10,
          "Continue across an arbitrary recovery segment boundary in a still-observed straight passage");
  straight.extendForward({{1.,0}}, {.67,0,0},2,0,2.1);
  require(straight.step({{.36,0}}, {.70,0,0},{.20,0},2.2).v==0,
          "A fresh obstacle blocks the extended forward escape");
  std::vector<ObstaclePoint> lane{{-.33,0},{.53,.05}};
  for(double y=-2.;y<=2.;y+=.025)lane.push_back({1.8,y});
  require(straight.selectForward(lane,{},2.,0.,3.,{}) && straight.step(lane,{}, {},3.02).v>0,
          "Allow a checked short advance even when a quarter-metre maneuver will not fit");
  require(!straight.selectForward(nose,{},2.,0.,3.,{}),"Do not advance into front bumper occupancy");
  ScanAlignment alignment;
  alignment.append(10.,{0.,0.,0.});
  ObstacleScan turning_scan;turning_scan.start=10.;turning_scan.duration=.1;
  const double turn=.06;
  const ObstaclePoint fixed_at_end{1.98*std::cos(turn),-1.98*std::sin(turn)};
  turning_scan.returns={{{2.,0.},0.},{fixed_at_end,.1}};
  std::vector<ObstaclePoint> aligned_points;
  require(!alignment.align(turning_scan,aligned_points),"Never extrapolate obstacle poses past latest odometry");
  alignment.append(10.1,{.02,0.,turn});
  require(alignment.align(turning_scan,aligned_points) && aligned_points.size()==2 &&
          std::hypot(aligned_points[0].x-aligned_points[1].x,aligned_points[0].y-aligned_points[1].y)<1e-6,
          "A stationary obstacle sampled during a moving scan must remain one obstacle");
  turning_scan.duration+=.000016;
  require(alignment.align(turning_scan,aligned_points),
          "A 16 microsecond scan-end rounding difference must not delay obstacles by a whole scan");
  turning_scan.duration+=.005;
  require(!alignment.align(turning_scan,aligned_points),"Wait when the matching odometry is genuinely missing");
  turning_scan.duration=.1;
  alignment.append(11.,{1.,1.,1.});
  require(!alignment.align(turning_scan,aligned_points),"Do not register a scan across a localization gap");
  for(int side:{-1,1}){
    const std::vector<ObstaclePoint> entrance{{.55,0},{.20,-side*.34}};
    require(pivotClear(entrance,side*.52)&&!pivotClear(entrance,-side*.52),
            "A corner in the outer circle must block only the actual swept sector");
    SideExit exit;
    require(exit.select(entrance,{},5,0,0),"Must select the available side passage");
    Pose p;Command actual;bool advanced=false;
    for(int i=1;i<500 && exit.phase()!=SideExit::Phase::Idle;++i){
      const auto scan=lab3::reframe(entrance,{}, {p.x,p.y,p.yaw});
      const auto cmd=exit.step(scan,p,actual,i*.02);
      if(i==1)require(cmd.v==0 && cmd.w*side>0,"Turn toward the open side from rest");
      if(exit.phase()==SideExit::Phase::Turning)require(cmd.v==0,"Pivot without translation");
      require(!sweptFootprint(scan,cmd.v,cmd.w).blocked,"Requested side exit must remain clear");
      actual=cmd;p.yaw+=cmd.w*.02;p.x+=cmd.v*.02*std::cos(p.yaw);p.y+=cmd.v*.02*std::sin(p.yaw);
      advanced|=cmd.v>0;
    }
    require(advanced && side*p.y>.15 && exit.phase()==SideExit::Phase::Idle,
            "Complete the pivot and leave the original corridor");
  }
  SideExit closed;
  require(!closed.select({{.33,0}}, {}, 5,0,0),"Never pivot through a point in the guarded footprint");
  require(!closed.select({}, {}, 5,0,0),"Do not introduce a side turn on a clear straight route");
  lab3::Navigator rejoin_pivot(6,true);
  rejoin_pivot.step({}, {}, {}, {}, 0.);
  auto heading_recovery=rejoin_pivot.step({2.,1.,1.}, {}, {}, {}, .02);
  require(heading_recovery.v==0 && heading_recovery.w<0 &&
          std::string(rejoin_pivot.state())=="pivot_to_path",
          "Large heading error on rejoin must pivot rather than creep forward");
  lab3::Navigator bypass_wait(6,true);
  bypass_wait.step({}, {}, {}, {}, 0.);
  const std::vector<ObstaclePoint> middle(8,{1.1,0.});
  bypass_wait.step({}, middle, {}, {}, 31.);
  require(std::string(bypass_wait.state())=="waiting_route","Stationary planner enters route wait");
  for(int i=1;i<=6;++i)bypass_wait.step({}, middle, {}, {}, 31.+i*.21);
  require(std::string(bypass_wait.state())!="waiting_route" && !bypass_wait.path().empty(),
          "A verified side route must release the wait even while the direct line is blocked");
  const std::vector<ObstaclePoint> ahead{{.478,0}};
  const auto reduced=guardDwa(ahead,{.30,0},{.20,0},{.15,0},.10,.20,.60);
  require(reduced.command.v>=.18 && reduced.command.v<.22 &&
          !sweptFootprint(ahead,reduced.command.v,0).blocked,
          "Keep a reachable slower command when only the proposed braking arc is blocked");
  require(guardDwa(ahead,{.1,0},{.2,0},{.30,0},.1,.2,.6).command.v==0,
          "An obstructed measured braking path must stop immediately");
  require(guardDwa({}, {}, {.3,.2}, {.3,.2},.02,.2,.6).command.v==0,
          "Never ramp an explicit stop into motion");
  require(guardDwa({{-.336,0}}, {-.10,0}, {-.08,0},{-.08,0},.02,.2,.6).command.v==0,
          "A real observed rear obstacle must guard reverse");
  lab3::Navigator near_side;
  const auto clear_forward=near_side.step({},std::vector<ObstaclePoint>(8,{.10,.30}),{},{},0);
  require(clear_forward.v>0&&clear_forward.w==0,"A side obstacle outside the 67 x 58 cm checking rectangle must allow forward motion");
  lab3::Navigator lateral;
  lab3::Navigator early_replan;
  std::vector<ObstaclePoint> changing_obstacle(8,{1.2,0});
  const auto initial_turn=early_replan.step({},changing_obstacle,{},{},0);
  require(early_replan.replans()==1&&!early_replan.path().empty(),"Near-zone obstacle must establish a bypass route");
  const auto blocked_waypoint=early_replan.path().front();
  for(int i=0;i<8;++i)changing_obstacle.push_back({blocked_waypoint.x,blocked_waypoint.y});
  early_replan.step({},changing_obstacle,initial_turn,{},.21);
  require(early_replan.replans()==2,"Invalid bypass route must replan immediately instead of cancelling the turn for a straight recovery segment");
  lateral.step({2,.08,0},{},{},{},0);
  require(std::string(lateral.state())=="rejoining","Aligned heading with lateral offset must still track the original line");
  lab3::Navigator point_target(6.,true);
  point_target.step({}, {}, {}, {}, 0.);
  const auto direct=point_target.step({2.,1.4,std::atan2(-1.4,4.)}, {}, {}, {}, .02);
  require(direct.v>0 && std::abs(direct.w)<1e-6 && point_target.path().size()==1,
          "An XY robot aimed at a clear goal after a detour should advance directly");
  lab3::Navigator offset_goal;
  offset_goal.step({9.95,.08,0},{},{},{},0);
  require(std::string(offset_goal.state())!="aligning_goal","Being near the goal must not bypass lateral line correction");
  lab3::Navigator relaxed_heading;
  const auto settled_heading=relaxed_heading.step({10,0,8.0*pi/180},{},{},{},0);
  relaxed_heading.step({10,0,8.0*pi/180},{},{},{},.31);
  require(settled_heading.v==0&&settled_heading.w==0&&relaxed_heading.finished(),"At ten metres, heading within eight degrees must finish without further turning");
  lab3::Navigator outside_heading;
  require(outside_heading.step({10,0,8.1*pi/180},{},{},{},0).w<0,"Heading outside eight degrees must still turn toward the initial heading");
  lab3::Navigator retreat;
  std::vector<ObstaclePoint> close_wall;
  for(double y=-1;y<=1;y+=.02)close_wall.push_back({.34,y});
  const auto backing=retreat.step({},close_wall,{},{},0);
  require(backing.v<0,"A blocked front and turning sweep should permit a clear short retreat");
  retreat.step({-.11,0,0},{},backing,{},.2);
  require(std::string(retreat.state())=="recover_turn","After ten centimetres of retreat, first turn to find a new route when rotation is clear");
  require(!sweptFootprint(std::vector<ObstaclePoint>(8,{0,.34}),0,.20).blocked,"A small turn must not reserve a full rotation circle");
  std::vector<ObstaclePoint> side(10,{0.0,0.40});
  require(sweptFootprint(side,0.3,0.001).clear,"Tiny heading correction must not use the full rotation circle");
  require(sweptFootprint(side,0.3,0).clear,"Parallel side obstacle must permit forward motion");
  require(sweptFootprint(std::vector<ObstaclePoint>(10,{.20,.34}),0,0.6).blocked,"Actual turn must protect the swept side corner");
  require(sweptFootprint(std::vector<ObstaclePoint>(10,{0.70,0}),0.4,0).blocked,"Forward braking path must be protected");
  require(sweptFootprint(std::vector<ObstaclePoint>(10,{-0.36,0}),0.3,0).clear,"Straight motion away from rear obstacle is permitted");
  require(sweptFootprint(std::vector<ObstaclePoint>(10,{-0.32,-0.29}),0,0.13).blocked,"Turning must retain rear corner collision protection");
  for(int linear:{-1,0,1})for(int angular:{-1,0,1}){
    Command cmd;require(manualInput("drive "+std::to_string(linear)+" "+std::to_string(angular)+" 1.5 0.3",1.5,.3,cmd),"All two-axis combinations must parse");
    require(cmd.v==linear*1.5&&cmd.w==angular*.3,"Both axes must be delivered together");
  }
  Command invalid{.4,.2};
  Command defaults;require(manualInput("w",1.5,.5,defaults)&&defaults.v==1.0,"Default linear speed is 1.0");
  require(manualInput("a",1.5,.5,defaults)&&defaults.w==.3,"Default turn speed is 0.3");
  require(manualInput("drive 1 1 1.5 0.5",1.5,.5,defaults)&&defaults.v==1.5&&defaults.w==.5,"Custom speed supports 1.5 / 0.5 upper limits");
  for(const std::string text:{"drive 2 1 1.5 0.3","drive 1 1 1.6 0.3","drive 1 1 1.5 0.4","drive 1 1 nan 0.3","drive 1 1 1.5 0.3 extra"})
    require(!manualInput(text,1.5,.3,invalid)&&invalid.v==.4&&invalid.w==.2,"Reject invalid drive packets without mutating command");
  auto released=manualRampOrigin({1.5,0},{.8,.2});require(released.v==.8&&released.w==0,"Release turn only, preserve forward ramp");
  released=manualRampOrigin({0,.3},{.8,.2});require(released.v==0&&released.w==.2,"Release translation only, preserve turn ramp");
  released=manualRampOrigin({1.5,.3},{.8,.2});require(released.v==.8&&released.w==.2,"Diagonal motion must preserve both ramp histories");
  auto manualRamp=rateLimit({1.5,0},{},0.1,0.6,0.6);
  require(std::abs(manualRamp.v-0.06)<1e-9,"Manual W/S acceleration must triple to 0.60 m/s2");
  for(double radius:{0.5,1.0,2.0,5.0,10.0})for(bool outer:{false,true}){
    Settings cfg;cfg.figure_eight=true;cfg.radius=radius;cfg.outer_start=outer;
    Motion eight(cfg);Pose p{4,-2,2.9};eight.start(p,0);Command prev;
    double walked=0,maxError=0,lastProgress=0;
    for(int tick=1;tick<150000;++tick){
      const auto raw=eight.step(p,tick*.02+(tick>1000?200:0));
      require(raw.v>=0&&raw.v<=.5&&std::abs(raw.w)<=.3,"Eight commands must obey existing limits");
      require(eight.progress>=lastProgress,"Eight progress must not go backwards");lastProgress=eight.progress;
      if(eight.stage==Stage::Done||eight.stage==Stage::Fault)break;
      auto cmd=rateLimit(raw,prev,.02,.2,.6);prev=cmd;
      p.x+=cmd.v*std::cos(p.yaw)*.02;p.y+=cmd.v*std::sin(p.yaw)*.02;p.yaw=wrap(p.yaw+cmd.w*.02);
      walked+=cmd.v*.02;maxError=std::max(maxError,eight.cross_track);
      if(tick==1000)eight.hold(200);
    }
    std::cout<<"Eight R="<<radius<<" outer="<<outer<<" stage="<<eight.name()<<" walked="<<walked<<" error="<<maxError<<" fault="<<eight.fault<<"\n";
    require(eight.stage==Stage::Done,"Full eight must finish for both start points and supported radii");
    require(walked>4*pi*radius*.85,"Crossing must not finish the route early");
    require(std::hypot(p.x-4,p.y+2)<.11&&maxError<.3,"Eight must return to current-pose origin with limited tracking error");
    require(std::abs(eight.progress-4*pi*radius)<1e-8,"Completed eight progress must be 100 percent");
  }
  auto ramp=rateLimit({0.25,0.30},{},0.1,0.2,0.6);
  require(std::abs(ramp.v-0.02)<1e-9&&std::abs(ramp.w-0.06)<1e-9,"Acceleration uses actual dt");
  ramp=rateLimit({0.05,0.02},{0.25,0.30},0.2,0.2,0.6);
  require(std::abs(ramp.v-0.21)<1e-9&&std::abs(ramp.w-0.18)<1e-9,"Normal deceleration is limited");
  ramp=rateLimit({0.25,0.30},{},2,0.2,0.6);
  require(ramp.v==0.25&&ramp.w==0.30,"Ramp never overshoots target");
  Settings p_cfg; p_cfg.turn=0.2;p_cfg.heading_d=0;
  Settings pd_cfg=p_cfg;pd_cfg.heading_d=0.08;
  Motion proportional(p_cfg),damped(pd_cfg);proportional.start({},0);damped.start({},0);
  proportional.step({},0.01);damped.step({},0.01);
  auto pc=proportional.step({0,0,0.02},0.11);auto dc=damped.step({0,0,0.02},0.11);
  require(dc.w<pc.w&&dc.w>0,"D term damps measured turn rate without changing target direction");
  Motion wrapped(pd_cfg),plain(pd_cfg);
  wrapped.start({0,0,pi-0.01},0);plain.start({0,0,-0.01},0);
  wrapped.step({0,0,pi-0.01},0.01);plain.step({0,0,-0.01},0.01);
  auto wc=wrapped.step({0,0,-pi+0.01},0.11);auto uc=plain.step({0,0,0.01},0.11);
  require(std::abs(wc.w-uc.w)<1e-9,"PD must not spike across the normalized angle boundary");
  damped.hold(10);dc=damped.step({0,0,0.02},10.12);
  require(std::abs(dc.w-pc.w)<1e-9,"Resume resets derivative history to prevent kick");
  Motion m; Pose p{7,-3,3.0}; m.start(p,0);
  auto c=m.step(p,0.1); require(c.v==0&&c.w>0,"Turn must be in place, including wrapped yaw");
  p.yaw=m.target_yaw; m.step(p,1); m.step(p,1.6);
  require(m.stage==Stage::Driving,"Must settle before drive");
  c=m.step(p,1.7); require(c.v>0&&std::abs(c.w)<1e-9,"Drive along target heading");
  p.x=m.origin.x+5*std::cos(m.target_yaw); p.y=m.origin.y+5*std::sin(m.target_yaw);
  // Floating point endpoint just beyond the threshold.
  p.x+=1e-6*std::cos(m.target_yaw); p.y+=1e-6*std::sin(m.target_yaw);
  c=m.step(p,40); require(m.stage==Stage::Done&&c.v==0&&c.w==0,"Endpoint must stop both velocities");
  require(m.step(m.origin,41).v==0,"Completed task must never restart");
  Motion f; f.start({},0); f.stop("stale"); c=f.step({},1); require(c.v==0&&c.w==0,"Fault must latch stop");
  Motion paused; paused.start({},0); const auto origin=paused.origin; const auto target=paused.target_yaw;
  paused.hold(200); c=paused.step({},200.1);
  require(paused.stage==Stage::Turning&&c.w>0,"Pause time must not consume turn timeout");
  require(paused.target_yaw==target&&paused.origin.x==origin.x,"Pause must retain original target");
  require(c.w<=0.30,"Angular speed must be limited");
  Pose aligned{0,0,paused.target_yaw};paused.step(aligned,201);paused.step(aligned,201.6);
  c=paused.step(aligned,201.7);require(std::abs(c.v-0.50)<1e-9,"Automatic cruise speed should be 0.50 m/s");
  f.stop("other"); require(f.fault=="stale","First fault reason must latch");
  Settings avoidance_settings;avoidance_settings.avoidance=true;avoidance_settings.distance=10;
  Motion avoidance(avoidance_settings);Pose avoid_pose{10,20,2.9};avoidance.start(avoid_pose,100);
  auto avoid_command=avoidance.step(avoid_pose,100.1);
  require(avoid_command.v>0&&avoid_command.v<=.0041&&std::abs(avoid_command.w)<1e-8,"Navigation must ramp from rest along its actual start heading");
  lab3::Navigator approach;
  lab3::Navigator early_turn;
  const auto early=early_turn.step({},std::vector<ObstaclePoint>(8,{1.32,.30}),{},{},0);
  require(early.w<0,"An obstacle in the bumper's 1 m zone must initiate steering away immediately, even when the direct path is nearly tangent");
  auto cruise=approach.step({},std::vector<ObstaclePoint>(8,{2,0}),{},{},0);
  require(cruise.v>0&&cruise.w==0&&approach.replans()==0,"Distant obstacle must not trigger a detour");
  approach.step({},std::vector<ObstaclePoint>(8,{1,.36}),{},{},.02);
  require(approach.replans()==0,"Obstacle outside the 70 cm trigger width must not start planning");
  approach.step({},std::vector<ObstaclePoint>(8,{1.32,0}),{},{},.04);
  require(approach.replans()>0,"Obstacle within 1 m of the bumper must trigger planning");
  avoidance.hold(200);
  avoid_command=avoidance.step(avoid_pose,300.12);
  require(avoidance.stage==Stage::Driving&&avoid_command.v>0,"Operator pause must not consume the no-progress timeout");
  Motion route(avoidance_settings);route.start({},0);
  auto route_command=route.step({10,1,0},.1);
  require(route.stage!=Stage::Done&&route_command.w<0,"Distance traveled must not replace the ten-metre goal");
  route_command=route.step({10,0,.4},.2);
  require(route.stage!=Stage::Done&&route_command.v==0&&route_command.w<0,"Arrival must restore the initial heading before finishing");
  for(int i=3;i<9;++i)route_command=route.step({10,0,0},i*.1);
  require(route.stage==Stage::Done&&route_command.v==0,"Goal position and settled heading complete the route");
  require(route.step({},.3).v==0,"Navigation completion must latch");
  Motion blocked(avoidance_settings);blocked.start({},0);
  for(double y=-4;y<=4;y+=.03)blocked.avoidance_points.push_back({.34,y});
  const auto stopped=blocked.step({},.02);
  require(stopped.v<=0,"A blocked front must not command forward motion; a checked reverse maneuver is allowed");
  blocked.stop("operator");
  require(blocked.step({},.04).v==0,"Operator stop must latch during blocked navigation");
  const auto transformed=lab3::reframe({{1,0}},{0,0,0},{.2,0,pi/2});
  require(std::abs(transformed[0].x)<1e-8&&std::abs(transformed[0].y+.8)<1e-8,"Lidar sample frame must follow intervening robot translation and rotation");
  Motion simulated; Pose robot; simulated.start(robot,0);
  double peak_cross=0; bool forward_during_turn=false;
  for(int tick=1;tick<10000;++tick) {
    double t=tick*0.02; auto cmd=simulated.step(robot,t);
    if(simulated.stage==Stage::Turning&&cmd.v!=0) forward_during_turn=true;
    robot.x+=cmd.v*std::cos(robot.yaw)*0.02;
    robot.y+=cmd.v*std::sin(robot.yaw)*0.02;
    robot.yaw=wrap(robot.yaw+cmd.w*0.02);
    peak_cross=std::max(peak_cross,std::abs(simulated.cross_track));
    if(simulated.stage==Stage::Done||simulated.stage==Stage::Fault) break;
  }
  require(simulated.stage==Stage::Done,"Integrated 45 degree / 5 metre run must complete");
  require(!forward_during_turn&&peak_cross<0.05,"Trajectory must stay near y=x");
  std::cout<<"PASS: wrap, turn-first, settle, endpoint stop, latched stop, fault stop, full integration. Distance="<<simulated.displacement<<" max_cross="<<peak_cross<<"\n";
}
