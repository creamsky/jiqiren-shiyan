#include "basic_control/motion.h"
#include "basic_control/rate_limit.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

using namespace lab26;
struct Scene { std::string name; std::vector<ObstaclePoint> points; bool reachable=true; bool dynamic=false; };
void circle(Scene& s,double x,double y,double r) {
  for(int i=0;i<64;++i) {double a=2*pi*i/64;s.points.push_back({x+r*std::cos(a),y+r*std::sin(a)});}
}
void wall(Scene& s,double x,double y,double xx,double yy) {
  int n=int(std::ceil(std::hypot(xx-x,yy-y)/.035));
  for(int i=0;i<=n;++i)s.points.push_back({x+(xx-x)*i/n,y+(yy-y)*i/n});
}
std::vector<Scene> scenes() {
  std::vector<Scene> all;
  all.push_back({"open"});
  Scene single{"single"};circle(single,2,0,.20);all.push_back(single);
  Scene box{"box_entry"};wall(box,2,-.30,2,.30);wall(box,2,.30,2.6,.30);wall(box,2.6,.30,2.6,-.30);wall(box,2.6,-.30,2,-.30);all.push_back(box);
  Scene staggered{"staggered"};circle(staggered,1.8,-.15,.22);circle(staggered,3.25,.45,.28);all.push_back(staggered);
  Scene gap{"wall_gap"};wall(gap,2,-2,2,.25);wall(gap,2,1.65,2,3);all.push_back(gap);
  Scene corridor{"corridor"};wall(corridor,-1,-.65,6,-.65);wall(corridor,-1,.65,6,.65);all.push_back(corridor);
  Scene dead{"dead_end"};dead.reachable=false;wall(dead,-1,-.65,6,-.65);wall(dead,-1,.65,6,.65);wall(dead,2,-.65,2,.65);all.push_back(dead);
  Scene moving{"temporary_block"};moving.dynamic=true;wall(moving,1.8,-3,1.8,3);all.push_back(moving);
  Scene close{"close_obstacle"};circle(close,1.0,0,.20);all.push_back(close);
  Scene offset{"offset_corridor"};wall(offset,-1,-.65,6,-.65);wall(offset,-1,.65,6,.65);circle(offset,2.5,1,.15);all.push_back(offset);
  return all;
}
int main(int argc,char**argv) {
  const auto begin=std::chrono::steady_clock::now();
  std::ofstream traces;if(argc>1)traces.open(argv[1]);
  if(traces)traces<<"scene,t,x,y,yaw,v,w,phase\n";
  int failures=0;
  for(const auto& scene:scenes()) {
    Settings settings;settings.avoidance=true;
#ifdef NAVIGATION_V2
    settings.distance=10;
#endif
    Motion motion(settings);Pose pose{};motion.start(pose,0);
    Command previous{},actual{};std::vector<ObstaclePoint> cloud;Pose scan_pose{};
    double traveled=0,turned=0,stopped=0,elapsed=0;bool collision=false;int reversals=0,last_sign=0,entry_interruptions=0,premature_rejoins=0;
    for(int tick=1;tick<=6000;++tick) {
      const double t=tick*.02;elapsed=t;
      const bool removed=scene.dynamic&&t>6;
      std::vector<ObstaclePoint> current;
      if(!removed)for(const auto& q:scene.points) {
        double dx=q.x-pose.x,dy=q.y-pose.y;
        double x=dx*std::cos(pose.yaw)+dy*std::sin(pose.yaw),y=-dx*std::sin(pose.yaw)+dy*std::cos(pose.yaw);
        if(std::abs(x)<body_half_length&&std::abs(y)<body_half_width)collision=true;
        if(std::hypot(x,y)<6)current.push_back({x,y});
      }
      // Real lidar is 10 Hz. Points stay in the sampled vehicle frame between scans.
      if(tick==1||tick%5==0){cloud=current;scan_pose=pose;}
      Command command;
#ifdef NAVIGATION_V2
      motion.avoidance_points=lab3::reframe(cloud,{scan_pose.x,scan_pose.y,scan_pose.yaw},{pose.x,pose.y,pose.yaw});motion.avoidance_previous=previous;motion.avoidance_feedback=actual;
      command=motion.step(pose,t);
#else
      const auto scan=sweptFootprint(cloud,.15,0);
      motion.avoidance_flag=scan.blocked?(scan.left<scan.right?1:2):0;
      auto preview=motion;auto proposed=preview.step(pose,t);
      bool adjusted=feasibleForward(cloud,proposed.v,proposed.w);
      auto guard=sweptFootprint(cloud,proposed.v,proposed.w);
      if(proposed.v>0&&guard.blocked) {
        motion.avoidance_flag=guard.left<guard.right?1:2;
        preview=motion;proposed=preview.step(pose,t);guard=sweptFootprint(cloud,proposed.v,proposed.w);
      }
      adjusted=clearanceAdvance(cloud,proposed.v,proposed.w)||adjusted;
      if(adjusted)guard=sweptFootprint(cloud,proposed.v,proposed.w);
      if(guard.blocked||sweptFootprint(cloud,previous.v,previous.w).blocked) { motion.hold(.02);command={}; }
      else {command=motion.step(pose,t);if(adjusted&&motion.stage!=Stage::Done&&motion.stage!=Stage::Fault)command=proposed;}
      if(command.v!=0||command.w!=0)command=rateLimit(command,previous,.02,.2,.6);
#endif
      previous=command;
      if((scene.name=="single"||scene.name=="box_entry")&&pose.x<2.0&&
         (std::string(motion.avoidanceState())=="blocked"||std::string(motion.avoidanceState()).find("recover_")==0))++entry_interruptions;
      if(scene.name=="staggered"&&pose.x<1.8&&std::string(motion.avoidanceState())=="rejoining")++premature_rejoins;
      // Simulated chassis lag; unlike the old test, commands are not applied instantly.
      actual.v+=(command.v-actual.v)*.2;actual.w+=(command.w-actual.w)*.2;
      pose.x+=actual.v*std::cos(pose.yaw)*.02;pose.y+=actual.v*std::sin(pose.yaw)*.02;pose.yaw=wrap(pose.yaw+actual.w*.02);
      traveled+=std::abs(actual.v)*.02;turned+=std::abs(actual.w)*.02;
      if(std::abs(actual.v)<.02)stopped+=.02;
      const int sign=actual.w>.08?1:actual.w<-.08?-1:0;
      if(sign&&last_sign&&sign!=last_sign)++reversals;if(sign)last_sign=sign;
      if(traces&&tick%5==0)traces<<scene.name<<','<<t<<','<<pose.x<<','<<pose.y<<','<<pose.yaw<<','<<command.v<<','<<command.w<<','<<motion.avoidanceState()<<'\n';
      if(collision||motion.stage==Stage::Done||motion.stage==Stage::Fault)break;
    }
    bool reached=motion.stage==Stage::Done&&std::hypot(pose.x-settings.distance,pose.y)<=.17;
#ifdef NAVIGATION_V2
    reached=reached&&std::abs(wrap(pose.yaw))<=8*pi/180;
#endif
    bool pass=!collision&&(scene.reachable?reached:(motion.stage==Stage::Fault&&turned<2*pi));
#ifdef NAVIGATION_V2
    if(!scene.reachable)pass=!collision&&motion.stage!=Stage::Fault;
    if(scene.reachable&&(elapsed>75||reversals>6||turned>3*pi))pass=false;
    if(entry_interruptions||premature_rejoins)pass=false;
#endif
    if(!pass)++failures;
    std::cout<<scene.name<<" pass="<<pass<<" stage="<<motion.name()<<" time="<<elapsed<<" length="<<traveled<<" turning="<<turned<<" stopped="<<stopped<<" reversals="<<reversals<<" collision="<<collision<<" entry_interruptions="<<entry_interruptions<<" premature_rejoins="<<premature_rejoins<<" x="<<pose.x<<" y="<<pose.y<<" reason="<<motion.fault<<std::endl;
  }
  std::cout<<"failures="<<failures<<" compute_seconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()<<std::endl;
  return failures?1:0;
}
