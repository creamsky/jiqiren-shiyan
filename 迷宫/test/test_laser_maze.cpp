#include "basic_control/gap_roam.h"
#include <iostream>
#include <chrono>
#include <cstdlib>
void require(bool ok,const char* msg) { if(!ok) { std::cerr<<"FAIL: "<<msg<<std::endl;std::exit(1); } }
// Cast one revolution against finite barriers. Only the first surface of
// each beam is visible, including at the corner and at the corridor exit.
void testLaserMaze(bool pocket=false,bool wide=false,bool cones=false,bool junction=false,int variant=0){
  using namespace lab26;
  struct Wall {double x,y,dx,dy;};
  std::vector<Wall> walls={{-3,-.85,6,0},{3,-.85,0,4.45},{-3,.85,4.3,0},
    {1.3,.85,0,2.75},{-3,-.85,0,1.7},{-4,5,9,0},{5,-3,0,8},{-4,-3,9,0},{-4,-3,0,8}};
  GapRoam planner;Pose p{0,.16,0};Command cmd{};NearPlaneRays rays;
  if(pocket){
    const double halfWidth=wide?.60:.36;
    walls={{0,-halfWidth,2,0},{2,-halfWidth,0,2*halfWidth},{0,halfWidth,2,0},{-4,-3,8,0},{4,-3,0,6},{-4,3,8,0},{-4,-3,0,6}};
    p={cones?1.25:1.45,0,0};
  }
  if(junction){
    walls={{-3,-.70,6,0},{3,-.70,0,1.4},{.70,.70,2.3,0},{-3,.70,2.3,0},
           {-.70,.70,0,3},{.70,.70,0,3},{-4,-3,9,0},{5,-3,0,8},{-4,5,9,0},{-4,-3,0,8}};
    p={-1.75,0,0};
  }
  if(cones){
    // Discrete square bases along the same aisle. The outer walls return the
    // first background echo through gaps in each row.
    std::vector<Wall> edges;
    for(const auto& wall:walls){
      if(wall.x==-4||wall.y==-3||wall.x==4||wall.x==5){edges.push_back(wall);continue;}
      const int intervals=std::max(1,int(std::ceil(std::hypot(wall.dx,wall.dy)/.65)));
      for(int i=0;i<=intervals;++i){
        const double x=wall.x+wall.dx*i/intervals,y=wall.y+wall.dy*i/intervals;
        edges.insert(edges.end(),{{x-.12,y-.12,.24,0},{x+.12,y-.12,0,.24},
                                  {x-.12,y+.12,.24,0},{x-.12,y-.12,0,.24}});
      }
    }
    walls=edges;
  }
  const int mirror=(variant&1)?-1:1;
  for(auto& wall:walls){wall.y*=mirror;wall.dy*=mirror;}
  p.y=mirror*(p.y+((variant&2)?.075:0.));p.yaw=mirror*((variant&2)?.12:0.);
  const double response=cones?((variant&4)?.055:.10):1.;
  Command measured=cones?Command{.16,0.}:Command{};
  cmd=measured;
  double reversed=0.;bool turned=false;int pivotStops=0;bool firstPivot=false;
  const auto outside=[&]{return pocket?(p.x<-.75||p.x<0&&std::abs(p.y)>1.):mirror*p.y>=3.8;};
  std::vector<ObstaclePoint> hits;Pose scanPose{};
  bool centered=false;double longestHold=0,hold=0;
  std::vector<double> planningUs;
  bool reachedEnd=false,returned=false,followedLinks=false,completedUTurn=false;
  int occludedTicks=0;
  for(int tick=0;tick<3500&&!outside();++tick){
    reachedEnd=reachedEnd||p.x>1.85;
    returned=returned||(reachedEnd&&p.x<.65);
    if(junction&&returned)require(p.x<1.75,"Returned from the junction into the exhausted branch");
    if(junction&&reachedEnd&&std::abs(p.yaw)>1.4&&occludedTicks<35)++occludedTicks;
    if(tick%5==0){
      hits.clear();rays={};rays.received_at=1+tick*.02;scanPose=p;
      for(int degree=0;degree<360;++degree){
        if((degree+tick/5)%7==0)continue;
        const double a=degree*pi/180.,dx=std::cos(p.yaw+a),dy=std::sin(p.yaw+a);
        double nearest=20.;
        for(const auto& wall:walls){
          const double det=dx*wall.dy-dy*wall.dx;
          if(std::abs(det)<1e-9)continue;
          const double x=wall.x-p.x,y=wall.y-p.y;
          const double r=(x*wall.dy-y*wall.dx)/det,u=(x*dy-y*dx)/det;
          if(r>0&&u>=0&&u<=1)nearest=std::min(nearest,r);
        }
        if(nearest>=20.)continue;
        // Never expose a distant obstacle to the planner in this trial.
        const double reach=std::min(1.9,nearest);
        const double x=reach*std::cos(a),y=reach*std::sin(a);
        const int missing=(tick/5)%2==0?75:210;
        const bool blind=occludedTicks>0&&occludedTicks<35;
        const bool partial=occludedTicks==35&&!completedUTurn&&degree>=missing&&degree<missing+30;
        if(!blind&&!partial)rays.add(x,y,0);
        if(nearest<=1.9)hits.push_back({x,y});
      }
    }
    const auto scan=lab3::reframe(hits,{scanPose.x,scanPose.y,scanPose.yaw},{p.x,p.y,p.yaw});
    const double x=scanPose.x-p.x,y=scanPose.y-p.y,c=std::cos(p.yaw),s=std::sin(p.yaw);
    const auto begin=std::chrono::steady_clock::now();
    // In the cone fixture rays already hit the full square base.
    cmd=planner.step(p,scan,{&rays,c*x+s*y,-s*x+c*y,scanPose.yaw-p.yaw},measured,cmd,1+tick*.02,.02,cones?0.:.18);
    if(junction&&reachedEnd&&p.x>1.75&&std::cos(p.yaw)<-.98)completedUTurn=true;
    if(std::string(planner.state)=="roam_backtrack")followedLinks=true;
    // Simulate feedback lag while entering the corner at speed.
    measured.v+=response*(cmd.v-measured.v);
    measured.w+=response*(cmd.w-measured.w);
    if(wide&&std::abs(p.yaw)<2.90){
      if(firstPivot&&cmd.w==0.)++pivotStops;
      firstPivot=firstPivot||std::abs(cmd.w)>.01;
    }
    planningUs.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count());
    hold=cmd.v==0.&&cmd.w==0.?hold+.02:0.;longestHold=std::max(longestHold,hold);
    p.x+=.02*measured.v*c;p.y+=.02*measured.v*s;p.yaw+=.02*measured.w;
    reversed+=.02*std::max(0.,-cmd.v);
    if(std::abs(p.yaw)>1.)turned=true;
    if(p.x<.95&&mirror*p.y<.13)centered=true;
    // Compare the actual rectangular chassis with every ground-truth edge,
    // including the occluded faces of cones between lidar updates.
    const double ch=std::cos(p.yaw),sh=std::sin(p.yaw);
    for(const auto& w:walls){
      const double dx=w.x-p.x,dy=w.y-p.y;
      double lo=0.,hi=1.;
      auto overlap=[&](double pos,double dir,double half){
        if(std::abs(dir)<1e-10)return std::abs(pos)<=half;
        const double a=(-half-pos)/dir,b=(half-pos)/dir;
        lo=std::max(lo,std::min(a,b));hi=std::min(hi,std::max(a,b));
        return lo<=hi;
      };
      require(!(overlap(ch*dx+sh*dy,ch*w.dx+sh*w.dy,body_half_length)&&
                overlap(-sh*dx+ch*dy,-sh*w.dx+ch*w.dy,body_half_width)),"Chassis intersects a ground-truth barrier");
    }
    for(const auto& hit:scan)require(std::abs(hit.x)>body_half_length||std::abs(hit.y)>body_half_width,"Body crossed a barrier in laser maze");
  }
  std::cout<<"Laser maze: x="<<p.x<<" y="<<p.y<<" longest hold="<<longestHold<<" centered="<<centered<<std::endl;
  std::sort(planningUs.begin(),planningUs.end());
  std::cout<<"Planner p95_us="<<planningUs[planningUs.size()*95/100]<<" max_us="<<planningUs.back()<<std::endl;
  if(cones&&!junction)require(longestHold<(variant?.75:.10),"Observed cone turn with lagged chassis feedback paused too long");
  if(junction)require(completedUTurn&&followedLinks&&returned&&outside()&&occludedTicks==35&&longestHold>.12,
    "Finish dead end turn after a blind interval, return to junction, and take its unexplored arm");
  if(pocket){
    std::cout<<"Pocket reverse distance="<<reversed<<" m, turned="<<turned<<std::endl;
    if(wide&&(variant&2))require(outside()&&turned&&reversed>0.&&reversed<.60,"An off-centre chassis backs clear of the near cone before turning");
    else if(wide)require(outside()&&turned&&reversed==0.&&pivotStops==0,"One uninterrupted U-turn in an observed wide dead end");
    else require(outside()&&reversed>1.&&turned,"Back out of a narrow dead end and turn toward an exit");
  }else require(centered&&outside()&&longestHold<2.,"Recenter, turn into side passage and exit with only visible laser returns");
}

void testLocalScanAndTrail(){
  using namespace lab26;
  PassageMemory memory;
  PassageMemory graph;
  graph.observe({});graph.opening({},0.);graph.opening({},pi/2);
  for(int i=1;i<=36;i++)graph.observe({i*.10,0,0});
  double route;
  require(graph.frontierHeading({3.6,0,0},route)&&std::cos(route)<-.9,
      "Plan back across several known edges to the untried junction");
  for(int i=35;i>=0;--i)graph.observe({i*.10,0,pi});
  require(graph.frontierHeading({},route)&&std::sin(route)>.9,
      "Select the unexplored branch on returning to the junction");
  for(int i=1;i<=23;i++)graph.observe({0,i*.10,pi/2});
  require(graph.closeDeadEnd({0,2.3,pi/2})&&
      graph.route({0,2.3,pi/2},route,true)==PassageMemory::Route::hop&&std::sin(route)<-.9,
      "Return home along actual links once the last branch is exhausted");
  for(int i=22;i>=0;--i)graph.observe({0,i*.10,-pi/2},false,true);
  require(graph.route({},route,true)==PassageMemory::Route::home,"Do not restart a completed branch");
  NearPlaneRays open;
  for(int i=0;i<360;i++)open.add(3*std::cos(i*pi/180.),3*std::sin(i*pi/180.),0.);
  GapRoam field;
  for(int i=0;i<=30;++i)field.step({.1*(15-std::abs(15-i)),0,0},{},{},{},{},i*.1,.1,.18);
  const auto onward=field.step({}, {},{&open,0,0,0},{},{},4,.02,.18);
  require(onward.v>0.&&onward.w==0.,"Open space remains straight despite an old trail ahead");
  for(int i=0;i<=30;++i)memory.observe({i*.10,0,0});
  require(memory.retraced({2.,0,0},pi,1.)>.9,"Actual return path is recognized");
  require(memory.retraced({2.,0,0},pi/2,1.)==0.,"Unvisited side passage is preferred");
  require(memory.retraced({2.,.50,0},pi,1.)==0.,"A nearby parallel corridor remains distinct");
  NearPlaneRays ring;std::vector<ObstaclePoint> walls;
  for(int i=0;i<360;++i){
    const double a=i*pi/180.,c=std::cos(a),s=std::sin(a);
    const double r=std::min(4.,.90/std::max(1e-6,std::abs(c)));
    ring.add(r*c,r*s,0.);
    if(r<4.)walls.push_back({r*c,r*s});
  }
  for(int side:{-1,1}){
    GapRoam fork;
    for(int i=0;i<=30;i++)fork.step({0,side*.10*(15-std::abs(15-i)),0},{},{},{},{},1+i*.1,.1,.18);
    const auto next=fork.step({},walls,{&ring,0,0,0},{},{},5,.02,.18);
    require(next.w*side<0.,"At the same T junction the measured trail changes which branch is selected");
  }
  for(double visible:{.62,1.5}){
    GapRoam planner;NearPlaneRays rays;Command last{};
    for(int i=-50;i<=50;++i)rays.add(visible,visible*std::tan(i*pi/180.),0.);
    for(int tick=0;tick<110;++tick)
      last=planner.step({}, {},{&rays,0,0,0},last,last,1+tick*.02,.02,.18);
    require(std::abs(last.v-(visible<1.?.24:.30))<1e-6,"Cruise uses nearby observed braking distance");
    auto blocked=planner.step({},{{.50,0}},{&rays,0,0,0},last,last,3.2,.02,.18);
    require(blocked.v==0.&&blocked.w==0.,"A new barrier overrides the recorded trail and cruise speed");
  }
  GapRoam narrow;Command speed{};walls.clear();
  for(int i=-10;i<40;++i){walls.push_back({i*.05,.285});walls.push_back({i*.05,-.285});}
  NearPlaneRays forward;
  for(int i=-35;i<=35;++i)forward.add(1.5,1.5*std::tan(i*pi/180.),0.);
  for(int i=0;i<110;++i){
    speed=narrow.step({},walls,{&forward,0,0,0},speed,speed,1+.02*i,.02,.18);
    require(speed.v>0.&&speed.w==0.,"55 cm chassis travels continuously between visible barriers 57 cm apart");
  }
}
void testReturnDetour(){
  using namespace lab26;
  for(int side:{-1,1}){
    GapRoam planner;Pose p{};
    for(int i=0;i<=25;i++){p.x=i*.1;planner.step(p,{},{},{},{},1+i*.1,.02,.18);}
    auto scan=[&](double opening,double time){
      NearPlaneRays rays;std::vector<ObstaclePoint> points;
      for(int deg=-180;deg<180;deg++){
        double a=deg*pi/180.;
        bool inGap=std::abs(wrap(a-opening))<.67;
        double r=inGap?3.:.72;
        double x=r*std::cos(a),y=r*std::sin(a);
        rays.add(x,y,0.);if(!inGap)points.push_back({x,y});
      }
      return planner.step(p,points,{&rays,0,0,0},{},{},time,.02,.18);
    };
    require(scan(pi,4).w!=0.,"A dead end starts a return turn");
    p.yaw=side*pi/2;
    NearPlaneRays rays;
    for(int i=0;i<360;i++)rays.add(2*std::cos(i*pi/180.),2*std::sin(i*pi/180.),0.);
    auto blocked=planner.step(p,{{0,.35},{-.45,0}},{&rays,0,0,0},{},{},4.3,.02,.18);
    require(blocked.v==0.&&blocked.w==0.,"New cone interrupts the half-finished turn");
    auto next=scan(side*.18,4.6);
    require(next.v>0.&&std::string(planner.state)=="roam_return_detour","An observed bypass after an interrupted U-turn permits forward progress");
    require(planner.step(p,{{.325,0}},{&rays,0,0,0},next,next,4.62,.02,.18).v==0.,"An obstacle on the bypass still stops the car immediately");
    p.yaw=pi;
    next=scan(0.,4.9);
    require(next.v>0.&&std::string(planner.state)=="roam_backtrack","Rejoin measured return route");
  }
}
void testAngledReturnAisle(){
  using namespace lab26;
  for(int side:{-1,1}){
    GapRoam planner;Pose p{};Command cmd{},feedback{};
    const double exit=side*155*pi/180.;
    for(int tick=0;tick<650;tick++){
      NearPlaneRays rays;std::vector<ObstaclePoint> points;
      for(int degree=-180;degree<180;++degree){
        const double world=degree*pi/180.,a=world-p.yaw;
        const bool gap=std::abs(wrap(world-exit))<.85;
        const double r=gap?3.:.72,x=r*std::cos(a),y=r*std::sin(a);
        rays.add(x,y,0.);if(!gap)points.push_back({x,y});
      }
      cmd=planner.step(p,points,{&rays,0,0,0},feedback,cmd,1+tick*.02,.02,.18);
      if(cmd.v>0.)break;
      feedback.w+=.1*(cmd.w-feedback.w);p.yaw+=.02*feedback.w;
    }
    std::cout<<"Return aisle "<<side*155<<" deg, departure yaw "<<p.yaw*180./pi<<" deg\n";
    require(cmd.v>0.&&std::abs(p.yaw-exit)<.20,"Take the shorter rotation and face the observed angled return aisle");
  }
}
int main(int argc,char** argv){
  if(argc==3){
    const int variant=std::atoi(argv[1]),scene=std::atoi(argv[2]);
    testLaserMaze(scene==1,scene==1,true,scene==2,variant);return 0;
  }
  testAngledReturnAisle();testReturnDetour();testLocalScanAndTrail();testLaserMaze();testLaserMaze(true);testLaserMaze(true,true);
  testLaserMaze(false,false,true);testLaserMaze(true,true,true);testLaserMaze(false,false,true,true);}
