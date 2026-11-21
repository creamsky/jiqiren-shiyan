#include "basic_control/motion.h"
#include "basic_control/rate_limit.h"
#include "basic_control/manual_input.h"
#include "basic_control/footprint.h"
#include <iostream>
#include <cstdlib>
#include <stdexcept>
void require(bool ok,const char* msg) { if(!ok) { std::cerr<<"FAIL: "<<msg<<std::endl;std::exit(1); } }
int main() {
  using namespace lab26;
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
