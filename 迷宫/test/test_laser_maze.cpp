#include "basic_control/gap_roam.h"
#include "basic_control/low_obstacles.h"
#include <iostream>
#include <chrono>
#include <cstdlib>
void require(bool ok,const char* msg) { if(!ok) { std::cerr<<"FAIL: "<<msg<<std::endl;std::exit(1); } }
// Cast one revolution against finite barriers. Only the first surface of
// each beam is visible, including at the corner and at the corridor exit.
void testLaserMaze(bool pocket=false,bool wide=false,bool cones=false,bool junction=false,int variant=0,bool hiddenBase=false,
                   bool interruptedTurn=false){
  using namespace lab26;
  struct Wall {double x,y,dx,dy;};
  std::vector<Wall> walls={{-3,-.85,6,0},{3,-.85,0,4.45},{-3,.85,4.3,0},
    {1.3,.85,0,2.75},{-3,-.85,0,1.7},{-4,5,9,0},{5,-3,0,8},{-4,-3,9,0},{-4,-3,0,8}};
  GapRoam planner;Pose p{0,.16,0};Command cmd{};NearPlaneRays rays;
  if(pocket){
    // A 180-degree turn with an unseen 18 cm base needs the chassis
    // diagonal radius plus that margin both ahead and on either side.
    const double halfWidth=wide&&(!cones&&!hiddenBase)?.66:wide?.60:.36;
    walls={{0,-halfWidth,2,0},{2,-halfWidth,0,2*halfWidth},{0,halfWidth,2,0},{-4,-3,8,0},{4,-3,0,6},{-4,3,8,0},{-4,-3,0,6}};
    p={cones||wide&&!hiddenBase?1.25:1.45,0,0};
    if(interruptedTurn)p.x=.18;
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
  int firstTurn=-1,stops=0,tick=0;
  double turnX=0.,retreatTime=-1.;
  bool slowedTurn=false,blockedTurn=false,blockedReverse=false;
  for(;tick<3500&&!outside();++tick){
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
        // Sustained side-coverage gaps interrupt several attempted turns at
        // the same location. Backing out moves the lidar clear of that spot.
        const bool missingFlank=interruptedTurn&&firstTurn>=0&&tick-firstTurn<400&&
          p.x>turnX-.13&&((tick-firstTurn)/5)%6>=1&&degree>=60&&degree<115;
        if(!blind&&!partial&&!missingFlank)rays.add(x,y,0);
        if(nearest<=1.9)hits.push_back({x,y});
      }
    }
    const auto scan=lab3::reframe(hits,{scanPose.x,scanPose.y,scanPose.yaw},{p.x,p.y,p.yaw});
    const double x=scanPose.x-p.x,y=scanPose.y-p.y,c=std::cos(p.yaw),s=std::sin(p.yaw);
    const auto begin=std::chrono::steady_clock::now();
    // The cone-base and 72 cm wall-aisle fixtures return the actual base
    // boundary. Upper-only fixtures carry the full unknown base extension.
    const double baseAllowance=cones||(pocket&&!wide)?0.:.18;
    const bool wasMoving=cmd.v!=0.||cmd.w!=0.;
    const RayView view{&rays,c*x+s*y,-s*x+c*y,scanPose.yaw-p.yaw};
    const double now=1+tick*.02;
    cmd=planner.step(p,scan,view,measured,cmd,now,.02,baseAllowance);
    if(interruptedTurn&&firstTurn<0&&cmd.v==0.&&std::abs(cmd.w)>.02){firstTurn=tick;turnX=p.x;}
    if(firstTurn>=0&&wasMoving&&cmd.v==0.&&cmd.w==0.)++stops;
    if(firstTurn>=0&&tick==firstTurn){
      // A hit beyond the angular stopping sweep should slow the turn. A
      // closer hit on the same side must still veto it immediately.
      auto farther=planner,nearer=planner;
      auto altered=scan;
      const double side=std::copysign(1.,cmd.w);
      altered.push_back({.59,-side*.10});
      const auto slow=farther.step(p,altered,view,measured,cmd,now+.01,.01,baseAllowance);
      slowedTurn=slowedTurn||(slow.w*cmd.w>0.&&std::abs(slow.w)<std::abs(cmd.w));
      altered.back()={.44,-side*.10};
      const auto stop=nearer.step(p,altered,view,measured,cmd,now+.01,.01,baseAllowance);
      blockedTurn=blockedTurn||(stop.v==0.&&stop.w==0.);
    }
    if(firstTurn>=0&&retreatTime<0.&&cmd.v<0.){
      retreatTime=(tick-firstTurn)*.02;
      auto blocked=planner;
      auto altered=scan;altered.push_back({-.40,0.});
      const auto stop=blocked.step(p,altered,view,measured,cmd,now+.01,.01,baseAllowance);
      blockedReverse=stop.v==0.&&stop.w==0.;
    }
    if(junction&&reachedEnd&&p.x>1.75&&std::cos(p.yaw)<-.98)completedUTurn=true;
    if(std::string(planner.state)=="roam_backtrack")followedLinks=true;
    // Simulate feedback lag while entering the corner at speed.
    measured.v+=response*(cmd.v-measured.v);
    // Short ramps below breakaway speed do not start the stationary chassis.
    const double targetW=interruptedTurn&&std::abs(measured.w)<.025&&std::abs(cmd.w)<.18?0.:cmd.w;
    measured.w+=response*(targetW-measured.w);
    if(wide&&!interruptedTurn&&std::abs(p.yaw)<2.90){
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
    if(interruptedTurn){
      const double escapeTime=firstTurn<0?-1.:(tick-firstTurn)*.02;
      std::cout<<"Interrupted turn: stops="<<stops<<" escape_s="<<escapeTime<<" reverse_start_s="<<retreatTime
        <<" slows_for_distant_hit="<<slowedTurn<<" stops_for_close_hit="<<blockedTurn
        <<" stops_for_rear_hit="<<blockedReverse<<std::endl;
      require(outside()&&retreatTime>=0.&&retreatTime<2.&&stops<=6&&escapeTime<21.,
        "Repeated uncertainty must retreat promptly along the observed approach");
      require(slowedTurn&&blockedTurn&&blockedReverse,"Reduce speed for a distant hit and stop for a hit in the stopping space");
    }else if(hiddenBase)require(outside()&&turned&&reversed>0.,"Back out when the upper returns leave insufficient base clearance for a turn");
    else if(wide&&(variant&2))require(outside()&&turned&&reversed>0.&&reversed<.60,"An off-centre chassis backs clear of the near cone before turning");
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
    {
      auto turn=planner;Pose local=p;Command cmd{};
      bool facedRoute=false;
      const double startX=local.x;
      for(int tick=0;tick<950&&startX-local.x<.06;tick++){
        NearPlaneRays sight;std::vector<ObstaclePoint> walls;
        const double now=4.8+.02*tick;sight.received_at=now;
        for(int degree=-180;degree<180;degree++){
          const double a=degree*pi/180.,world=a+local.yaw;
          const double c=std::cos(world),s=std::sin(world);
          const double tx=std::abs(c)<1e-8?10.:(std::copysign(.72,c)-(local.x-startX))/c;
          const double ty=std::abs(s)<1e-8?10.:(std::copysign(.72,s)-(local.y-p.y))/s;
          const double range=std::min(tx,ty),x=range*std::cos(a),y=range*std::sin(a);
          sight.add(x,y,0.);walls.push_back({x,y});
        }
        cmd=turn.step(local,walls,{&sight,0,0,0},cmd,cmd,now,.02,.18);
        if(tick==0){
          require(side*cmd.w>0.,"Turn toward a short visible side return when there is room to pivot");
          auto guard=turn;walls.push_back({.47,0.});
          const auto stop=guard.step(local,walls,{&sight,0,0,0},cmd,cmd,now+.02,.02,.18);
          require(stop.v==0.&&stop.w==0.,"A new cone inside the calibrated turn disk cancels the short-lane pivot");
        }
        facedRoute=facedRoute||std::abs(wrap(local.yaw-pi))<.18;
        local.x+=.02*cmd.v*std::cos(local.yaw);local.y+=.02*cmd.v*std::sin(local.yaw);local.yaw+=.02*cmd.w;
      }
      require(facedRoute&&startX-local.x>=.06,"Face the return lane and make progress without a long straight candidate");
    }
    {
      GapRoam reverse=planner;Pose noseAway=p;noseAway.yaw=0.;Command cmd{};
      const double startX=noseAway.x;
      for(int tick=0;tick<200&&startX-noseAway.x<.22;tick++){
        const double now=4.8+.02*tick;
        NearPlaneRays sight;sight.received_at=now;
        // Both ends of the aisle are visible; the missing side sector
        // prevents a pivot. The actual graph hop is behind the car.
        for(int deg=-180;deg<180;deg++)if(deg<65||deg>105)
          sight.add(2.*std::cos(deg*pi/180.),2.*std::sin(deg*pi/180.),0.);
        cmd=reverse.step(noseAway,{},{&sight,0,0,0},cmd,cmd,now,.02,.18);
        require(cmd.v<0.&&cmd.w==0.,"Follow a visible rear graph edge in reverse before taking a forward detour");
        noseAway.x+=.02*cmd.v;
      }
      require(startX-noseAway.x>=.22,"Maintain reverse along the rear route across replans");
    }
    // At a right angle to the graph edge the vehicle can translate first,
    // then turn once it clears the cone behind the opposite bumper.
    for(int sign:{1,-1}){
      GapRoam boxed=planner;Pose local=p;Command cmd{};double travel=0.;
      for(int tick=0;tick<200&&travel<.07;tick++){
        const double now=4.6+.02*tick;
        NearPlaneRays localRays;localRays.received_at=now;
        std::vector<ObstaclePoint> nearby;
        for(int deg=-180;deg<180;deg++){
          const double a=deg*pi/180.,c=std::cos(a),s=std::sin(a);
          const double range=.69/std::max(std::abs(c),std::abs(s));
          const double x=range*c-sign*travel,y=range*s;
          localRays.add(x,y,0.);nearby.push_back({x,y});
        }
        nearby.push_back({-sign*(.35+travel),0.,true});
        cmd=boxed.step(local,nearby,{&localRays,0,0,0},cmd,cmd,now,.02,.18);
        require(sign*cmd.v>0.&&cmd.w==0.,"Use a short observed forward or rear lane transverse to the graph edge");
        auto probe=boxed;
        auto stale=probe.step(local,nearby,{&localRays,0,0,0},cmd,cmd,now+.31,.02,.18);
        require(stale.v==0.&&stale.w==0.,"Old rays cannot authorize return repositioning");
        nearby.push_back({sign*.48,0.,true});
        auto guard=boxed;
        auto veto=guard.step(local,nearby,{&localRays,0,0,0},cmd,cmd,now+.005,.02,.18);
        require(veto.v==0.&&veto.w==0.,"A new near obstacle vetoes the next repositioning command");
        travel+=sign*.02*cmd.v;local.x+=.02*cmd.v*std::cos(local.yaw);local.y+=.02*cmd.v*std::sin(local.yaw);
      }
      require(travel>=.07,"Make measured progress instead of repeatedly reporting a blocked route");
      // The next full scan reveals the side aisle from the new position.
      // Resume the graph route without resetting the planner or run.
      bool rejoined=false;
      for(int tick=0;tick<500&&!rejoined;tick++){
        NearPlaneRays exit;std::vector<ObstaclePoint> aisle;
        for(int degree=-180;degree<180;degree++){
          const double a=degree*pi/180.;
          const bool open=std::abs(wrap(local.yaw+a-pi))<.70;
          const double r=open?3.:.72,x=r*std::cos(a),y=r*std::sin(a);
          exit.add(x,y,0.);if(!open)aisle.push_back({x,y});
        }
        const double now=10.+.02*tick;exit.received_at=now;
        cmd=boxed.step(local,aisle,{&exit,0,0,0},cmd,cmd,now,.02,.18);
        local.x+=.02*cmd.v*std::cos(local.yaw);local.y+=.02*cmd.v*std::sin(local.yaw);local.yaw+=.02*cmd.w;
        const std::string mode=boxed.state;
        rejoined=cmd.v>0.&&std::abs(wrap(local.yaw-pi))<.25&&(mode=="roam_backtrack"||mode=="roam_frontier");
      }
      require(rejoined,"Automatically rejoin the observed return aisle after moving forward or backward for clearance");
    }
    auto next=scan(side*.18,4.6);
    require(next.v>0.&&std::string(planner.state)=="roam_return_detour","An observed bypass after an interrupted U-turn permits forward progress");
    require(planner.step(p,{{.325,0}},{&rays,0,0,0},next,next,4.62,.02,.18).v==0.,"An obstacle on the bypass still stops the car immediately");
    p.yaw=pi;
    next=scan(0.,4.9);
    require(next.v>0.&&std::string(planner.state)=="roam_backtrack","Rejoin measured return route");
  }
}
void testConePassingMargin(){
  using namespace lab26;
  for(int side:{-1,1}){
    GapRoam planner;Pose p{};Command cmd{};
    double margin=1.;int stops=0;
    for(int tick=0;tick<600&&p.x<2.2;tick++){
      NearPlaneRays rays;std::vector<ObstaclePoint> points;
      const double c=std::cos(p.yaw),s=std::sin(p.yaw);
      for(int degree=-180;degree<180;degree++){
        const double a=degree*pi/180.,dx=std::cos(a+p.yaw),dy=std::sin(a+p.yaw);
        double near=3.;
        auto plane=[&](double t,double coord,double low,double high){
          if(t>0.&&coord>=low&&coord<=high)near=std::min(near,t);
        };
        if(std::abs(dy)>1e-8){
          for(double wall:{-.68,.68}){double t=(wall-p.y)/dy;plane(t,p.x+t*dx,-4.,8.);}
          for(double edge:{side*.293,side*.533}){double t=(edge-p.y)/dy;plane(t,p.x+t*dx,1.20,1.44);}
        }
        if(std::abs(dx)>1e-8)for(double edge:{1.20,1.44}){
          const double t=(edge-p.x)/dx;plane(t,side*(p.y+t*dy),.293,.533);
        }
        const double x=near*std::cos(a),y=near*std::sin(a);
        rays.add(x,y,0.);if(near<3.)points.push_back({x,y});
      }
      for(int i=0;i<=24;i++){
        const double dx=1.20+i*.01-p.x,dy=side*.293-p.y;
        const double x=c*dx+s*dy,y=c*dy-s*dx;
        if(std::abs(x)<=body_half_length)margin=std::min(margin,std::abs(y)-body_half_width);
      }
      rays.received_at=1+tick*.02;
      cmd=planner.step(p,points,{&rays,0,0,0},cmd,cmd,rays.received_at,.02,.18);
      if(cmd.v==0.&&cmd.w==0.)++stops;
      p.x+=.02*cmd.v*c;p.y+=.02*cmd.v*s;p.yaw+=.02*cmd.w;
    }
    std::cout<<"Cone side "<<side<<" passing gap "<<margin<<" m, stops "<<stops<<" end x "<<p.x<<std::endl;
    require(p.x>=2.2&&margin>=.045&&stops==0,"Steer away early from a cone grazing an otherwise clear lane");
  }
}
void testShortForwardObservation(){
  using namespace lab26;
  GapRoam planner;NearPlaneRays rays;Command cmd{};
  const RayView view{&rays,0,0,0};
  // One return almost two metres beyond the bumper, but a complete
  // width of ray evidence only for the immediately usable prefix.
  const std::vector<ObstaclePoint> points={{2.25,0.},{-.62,0.}};
  for(int deg=-36;deg<=36;deg++)rays.add(.55,.55*std::tan(deg*pi/180.),0.);
  for(int i=0;i<30;i++){
    rays.received_at=1+i*.02;
    cmd=planner.step({},points,view,cmd,cmd,1+i*.02,.02,.18);
    require(cmd.v>0.&&cmd.w==0.,"Use a verified straight braking lane when no long passage or rear lane is observed");
  }
  auto stop=planner.step({},{{.50,0.}},view,cmd,cmd,1.60,.02,.18);
  require(stop.v==0.&&stop.w==0.,"A new obstacle vetoes short advance immediately");
  stop=planner.step({},{{.56,0.,true}},view,cmd,cmd,1.62,.02,.18);
  require(stop.v==0.&&stop.w==0.,"A remembered low obstacle vetoes the calibrated braking sweep");
  stop=planner.step({},points,view,cmd,cmd,2.,.02,.18);
  require(stop.v==0.&&stop.w==0.,"A stale scan cannot authorize short advance");
  GapRoam blind;NearPlaneRays pencil;pencil.add(2.25,0.,0.);
  cmd=blind.step({},points,{&pencil,0,0,0},{},{},2,.02,.18);
  require(cmd.v==0.&&cmd.w==0.,"A distant central ray alone cannot clear the chassis width");
}
void testSlowForShorterForwardObservation(){
  using namespace lab26;
  GapRoam planner;NearPlaneRays open,narrow;Command cmd{};
  const std::vector<ObstaclePoint> points={{2.25,0.}};
  for(int degree=-180;degree<180;++degree){
    const double a=degree*pi/180.;
    open.add(2.*std::cos(a),2.*std::sin(a),0.);
  }
  for(int degree=-36;degree<=36;++degree)narrow.add(.55,.55*std::tan(degree*pi/180.),0.);
  for(int tick=0;tick<45;++tick){
    open.received_at=1.+tick*.02;
    cmd=planner.step({},points,{&open,0.,0.,0.},cmd,cmd,open.received_at,.02,.18);
  }
  require(cmd.v>.15&&cmd.w==0.,"Approach the shorter observation with forward speed");
  narrow.received_at=1.90;
  const auto slow=planner.step({},points,{&narrow,0.,0.,0.},cmd,cmd,1.90,.02,.18);
  std::cout<<"Shorter forward observation: before="<<cmd.v<<" after="<<slow.v<<" m/s"<<std::endl;
  require(slow.v>0.&&slow.v<cmd.v&&slow.w==0.,"Keep rolling while slowing into a verified, shorter braking lane");
  const auto stop=planner.step({},{{.44,0.}},{&narrow,0.,0.,0.},slow,slow,1.92,.02,.18);
  require(stop.v==0.&&stop.w==0.,"Stop for a new obstacle within the forward braking lane");
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
void testExplorePastIsolatedHit(){
  using namespace lab26;
  GapRoam planner;LowObstacles memory;ObstacleScan scan;NearPlaneRays rays;
  // Real run: about 38 cm from the front bumper, a clear 20 cm prefix,
  // short rear returns, and no long lane. Initialize in exploration mode.
  const std::vector<ObstaclePoint> live={{.72,0.},{-.34,0.}};
  for(int i=-38;i<=38;i++)rays.add(.62,.62*std::tan(i*pi/180.),0.);
  scan.start=1.;scan.returns={{{.70,0.},0.,true,-.18},{{.70,0.},0.,true,-.18}};
  memory.update(scan,{{.70,0.},{.70,0.}},{},true);
  scan.returns.clear();
  Command cmd{};
  for(int i=0;i<15;i++){
    scan.start=2.+.10*i;memory.update(scan,{},{},true);
    auto points=live;memory.append({},points,true);
    require(points.size()==live.size(),"Duplicate returns from an isolated scan cannot create persistent maze obstacles");
    rays.received_at=scan.start;
    cmd=planner.step({},points,{&rays,0,0,0},cmd,cmd,scan.start,.10,.18);
    require(cmd.v>0.&&cmd.w==0.,"Exploration must use the observed braking lane even below the long-path threshold");
  }
  std::vector<ObstaclePoint> occupied=live;occupied.push_back({.48,0.});
  cmd=planner.step({},occupied,{&rays,0,0,0},cmd,cmd,scan.start+.02,.02,.18);
  require(cmd.v==0.&&cmd.w==0.,"A fresh obstacle still vetoes exploratory motion immediately");
  memory.clear();scan.returns={{{1.,0.},0.,true,-.20}};
  for(double stamp:{4.,4.1}){scan.start=stamp;memory.update(scan,{{1.,0.}},{},true);}
  scan.returns.clear();scan.lower_rays={{{2.,0.},0.,false,-.20}};
  for(double stamp:{5.,5.1}){scan.start=stamp;memory.update(scan,{},{},true);}
  std::vector<ObstaclePoint> saved;memory.append({},saved,true);
  require(saved.size()==1,"An upper beam cannot remove a confirmed low obstacle");
  scan.lower_rays={{{2.,0.},0.,false,-.40}};
  for(double stamp:{5.2,5.3}){scan.start=stamp;memory.update(scan,{},{},true);}
  saved.clear();memory.append({},saved,true);
  require(saved.empty(),"Fresh rays through the measured height clear a confirmed historical return");
  for(int side:{-1,1}){
    GapRoam searching;NearPlaneRays circle;std::vector<ObstaclePoint> walls;
    for(int deg=-180;deg<180;deg++){
      const double a=deg*pi/180.,x=std::cos(a),y=std::sin(a);
      const double r=std::min(.66/std::max(1e-8,std::abs(x)),(side*y>0?.84:.66)/std::max(1e-8,std::abs(y)));
      circle.add(r*x,r*y,0.);walls.push_back({r*x,r*y});
    }
    circle.received_at=6.;
    cmd=searching.step({},walls,{&circle,0,0,0},{},{},6.,.02,.18);
    require(cmd.v==0.&&side*cmd.w>0.,"During ordinary exploration turn toward the longer short side opening");
  }
}
void testEscapeAlongOccupiedFlank(){
  using namespace lab26;
  // The cone is alongside the rear half of the chassis. Its entire
  // possible base lies behind the leading bumper throughout straight travel.
  GapRoam planner;Command cmd{};Pose p{};
  for(int tick=0;tick<100&&p.x<.045;tick++){
    NearPlaneRays rays;const double time=1+tick*.02;rays.received_at=time;
    for(int deg=-36;deg<=36;deg++)rays.add(.61,.61*std::tan(deg*pi/180.),0.);
    const std::vector<ObstaclePoint> points={{.66-p.x,0.,true},{-.34-p.x,0.},{-.05-p.x,.385}};
    auto guard=planner;auto ahead=points;ahead.push_back({.36,.365});
    const auto blocked=guard.step(p,ahead,{&rays,0,0,0},cmd,cmd,time,.02,.18);
    require(blocked.v==0.&&blocked.w==0.,"A possible cone base beside the leading corner blocks escape");
    cmd=planner.step(p,points,{&rays,0,0,0},cmd,cmd,time,.02,.18);
    require(cmd.v>0.&&cmd.w==0.,"A rear flank inside the inflation band cannot veto straight motion away");
    p.x+=.02*cmd.v;
  }
  require(p.x>=.045,"Advance within the short visible lane instead of repeatedly holding at rest");
}
int main(int argc,char** argv){
  if(argc==2&&std::string(argv[1])=="--interrupted-turn"){
    testLaserMaze(true,true,false,false,0,false,true);return 0;
  }
  if(argc==3){
    const int variant=std::atoi(argv[1]),scene=std::atoi(argv[2]);
    testLaserMaze(scene==1,scene==1,true,scene==2,variant);return 0;
  }
  testEscapeAlongOccupiedFlank();testExplorePastIsolatedHit();testConePassingMargin();testShortForwardObservation();testSlowForShorterForwardObservation();testAngledReturnAisle();testReturnDetour();testLocalScanAndTrail();testLaserMaze();testLaserMaze(true);testLaserMaze(true,true);
  testLaserMaze(true,true,false,false,0,true);
  testLaserMaze(false,false,true);testLaserMaze(true,true,true);testLaserMaze(false,false,true,true);}
