#include "basic_control/gap_roam.h"
#include <array>
#include <cstdlib>
#include <iostream>
#include <random>
#include <set>
#include <string>

using namespace lab26;

namespace {
using Cell=std::pair<int,int>;
struct Maze {
  std::vector<std::string> grid;
  struct Cone {double x,y;};
  std::map<Cell,std::vector<Cone>> cones;
  int exitRow=0;
  double scale=1.35;
  bool wall(int x,int y)const{
    return y>=0&&y<int(grid.size())&&x>=0&&x<int(grid[0].size())&&grid[y][x]=='#';
  }
  void placeCones(){
    for(int y=0;y<int(grid.size());y++)for(int x=0;x<int(grid[y].size());x++){
      if(!wall(x,y))continue;
      for(Cell normal:std::array<Cell,4>{{{1,0},{-1,0},{0,1},{0,-1}}}){
        if(wall(x+normal.first,y+normal.second))continue;
        for(int i=0;i<3;i++){
          const double along=(i-1)*scale/3.,normalOffset=scale*.5-.16;
          cones[{x,y}].push_back({(x+.5)*scale+normal.first*normalOffset-normal.second*along,
                                  (y+.5)*scale+normal.second*normalOffset+normal.first*along});
        }
      }
    }
  }
  static double hitSquare(Pose p,double dx,double dy,Cone cone,double half){
    double lo=0.,hi=5.;
    const auto slab=[&](double relative,double direction){
      if(std::abs(direction)<1e-10)return std::abs(relative)<=half;
      const double a=(relative-half)/direction,b=(relative+half)/direction;
      lo=std::max(lo,std::min(a,b));hi=std::min(hi,std::max(a,b));return lo<=hi;
    };
    return slab(cone.x-p.x,dx)&&slab(cone.y-p.y,dy)?lo:5.;
  }
  // Intersections with the first occupied cell only. Beyond the maze a
  // surrounding hall provides distant returns through the one open gate.
  double ray(Pose p,double dx,double dy)const{
    int x=int(std::floor(p.x/scale)),y=int(std::floor(p.y/scale));
    const int sx=dx>0.?1:-1,sy=dy>0.?1:-1;
    const double stepX=scale/std::max(1e-10,std::abs(dx)),stepY=scale/std::max(1e-10,std::abs(dy));
    double tx=std::abs((x+(sx>0))*scale-p.x)/std::max(1e-10,std::abs(dx));
    double ty=std::abs((y+(sy>0))*scale-p.y)/std::max(1e-10,std::abs(dy));
    for(;;){
      const double range=std::min(tx,ty);
      if(range>5.)return 5.;
      if(tx<=ty){x+=sx;tx+=stepX;}
      else{y+=sy;ty+=stepY;}
      if(wall(x,y)){
        if(cones.empty())return range;
        auto it=cones.find({x,y});
        if(it!=cones.end()){
          double nearest=5.;
          for(auto cone:it->second)nearest=std::min(nearest,hitSquare(p,dx,dy,cone,.10));
          if(nearest<5.)return nearest;
        }
      }
    }
  }
  // Independent separating-axis test of the complete body against solid
  // walls, including corners that were occluded from the lidar.
  bool collision(Pose p)const{
    const int ix=int(std::floor(p.x/scale)),iy=int(std::floor(p.y/scale));
    const double c=std::cos(p.yaw),s=std::sin(p.yaw);
    const auto overlaps=[&](double x,double y,double h){
      const double dx=x-p.x,dy=y-p.y;
      return std::abs(dx)<=h+body_half_length*std::abs(c)+body_half_width*std::abs(s)&&
             std::abs(dy)<=h+body_half_length*std::abs(s)+body_half_width*std::abs(c)&&
             std::abs(c*dx+s*dy)<=body_half_length+h*(std::abs(c)+std::abs(s))&&
             std::abs(c*dy-s*dx)<=body_half_width+h*(std::abs(c)+std::abs(s));
    };
    for(int j=iy-1;j<=iy+1;j++)for(int i=ix-1;i<=ix+1;i++){
      if(!wall(i,j))continue;
      if(cones.empty()){if(overlaps((i+.5)*scale,(j+.5)*scale,scale*.5))return true;}
      else{
        auto it=cones.find({i,j});
        if(it!=cones.end())for(auto cone:it->second)if(overlaps(cone.x,cone.y,.16))return true;
      }
    }
    return false;
  }
};

Maze makeMaze(int seed,std::string style){
  Maze maze;
  std::mt19937 generator(seed);
  const int width=style=="large"?13:9,height=style=="large"?9:7;
  if(style=="narrow")maze.scale=1.05;
  maze.grid.assign(height,std::string(width,'#'));
  std::vector<Cell> stack{{1,1}};
  maze.grid[1][1]='.';
  const std::array<Cell,4> directions{{{1,0},{0,1},{-1,0},{0,-1}}};
  while(!stack.empty()){
    const auto here=stack.back();
    std::vector<Cell> options;
    for(auto d:directions){
      const int x=here.first+2*d.first,y=here.second+2*d.second;
      if(x>0&&x<width-1&&y>0&&y<height-1&&maze.wall(x,y))options.push_back(d);
    }
    if(options.empty()){stack.pop_back();continue;}
    const auto d=options[generator()%options.size()];
    const int x=here.first+d.first,y=here.second+d.second;
    maze.grid[y][x]=maze.grid[y+d.second][x+d.first]='.';
    stack.emplace_back(x+d.first,y+d.second);
  }
  // Connect two already accessible arms for looped layouts.
  if(seed%2==0){
    std::vector<Cell> candidates;
    for(int y=1;y<height-1;y++)for(int x=1;x<width-1;x++)
      if(maze.wall(x,y)&&((x+y)%2==1)&&
         ((!maze.wall(x-1,y)&&!maze.wall(x+1,y))||(!maze.wall(x,y-1)&&!maze.wall(x,y+1))))
        candidates.emplace_back(x,y);
    const auto gap=candidates.at(generator()%candidates.size());
    maze.grid[gap.second][gap.first]='.';
  }
  maze.exitRow=1+2*(generator()%(height/2));
  maze.grid[maze.exitRow][width-1]='.';
  if(style=="cones")maze.placeCones();
  return maze;
}

bool run(int seed,const std::string& style,bool verbose){
  const Maze maze=makeMaze(seed,style);
  const bool jitter=style=="turn_jitter";
  GapRoam planner;
  Pose p{maze.scale*1.5,maze.scale*(1.50+.05*(seed%3-1)),(seed%4)*pi/2+.06};
  const double frameYaw=.23*seed,fc=std::cos(frameYaw),fs=std::sin(frameYaw);
  Command cmd{},feedback{};
  const double dt=.04;
  Pose atScan{};NearPlaneRays rays;std::vector<ObstaclePoint> hits;
  std::set<Cell> visited;
  Cell last{-1,-1};
  std::string lastState;
  int lastDirection=0,reversals=0;
  double leavePocket=-1.;
  bool checkedShift=false,checkedTurn=false;
  double hold=0.,maxHold=0.,path=0.;
  const char* outcome="time limit";
  if(verbose)for(const auto& row:maze.grid)std::cout<<row<<"\n";
  const int limit=style=="large"?32000:16000;
  for(int tick=0;tick<limit;++tick){
    const double now=1+dt*tick,c=std::cos(p.yaw),s=std::sin(p.yaw);
    if(tick%3==0){
      atScan=p;rays={};rays.received_at=now;hits.clear();
      for(int degree=-180;degree<180;degree+=2){
        const double a=degree*pi/180.,r=maze.ray(p,std::cos(p.yaw+a),std::sin(p.yaw+a));
        // Alternate the observed range by up to 12 mm; the independent
        // collision check below still uses the fixed, true wall position.
        const double measuredRange=r+(jitter&&r<5.?.012*std::sin(.71*degree+tick*.43):0.);
        const double x=measuredRange*std::cos(a),y=measuredRange*std::sin(a);
        rays.add(x,y,0.);
        if(r<5.)hits.push_back({x,y});
      }
    }
    const auto seen=lab3::reframe(hits,{atScan.x,atScan.y,atScan.yaw},{p.x,p.y,p.yaw});
    const double dx=atScan.x-p.x,dy=atScan.y-p.y;
    const Pose odom{8.+fc*p.x-fs*p.y,-3.+fs*p.x+fc*p.y,p.yaw+frameYaw};
    Command measured=feedback;
    if(jitter){
      measured.v=std::round(feedback.v*1000.)/1000.;
      measured.w=std::round(feedback.w*100.)/100.+.01*((tick/3)%3-1);
    }
    cmd=planner.step(odom,seen,{&rays,c*dx+s*dy,c*dy-s*dx,atScan.yaw-p.yaw},measured,cmd,now,dt,.18);
    if(jitter&&leavePocket<0.){
      const std::string state=planner.state;
      const bool shift=!checkedShift&&(state=="roam_turn_forward"||state=="roam_turn_reverse")&&cmd.v!=0.;
      const bool turn=!checkedTurn&&state=="roam_turn_setup"&&cmd.w!=0.;
      if(shift||turn){
        const RayView view{&rays,c*dx+s*dy,c*dy-s*dx,atScan.yaw-p.yaw};
        auto blocked=planner,stale=planner;
        auto changed=seen;changed.push_back({shift&&cmd.v<0.?-.44:.44,.33});
        const auto stop=blocked.step(odom,changed,view,measured,cmd,now+.01,.01,.18);
        const auto blind=stale.step(odom,seen,view,measured,cmd,now+.35,dt,.18);
        if(stop.v!=0.||stop.w!=0.||blind.v!=0.||blind.w!=0.){outcome="unsafe recovery";break;}
        checkedShift=checkedShift||shift;checkedTurn=checkedTurn||turn;
      }
    }
    feedback.v+=.28*(cmd.v-feedback.v);feedback.w+=.28*(cmd.w-feedback.w);
    const double mid=p.yaw+feedback.w*dt*.5;
    p.x+=dt*feedback.v*std::cos(mid);p.y+=dt*feedback.v*std::sin(mid);p.yaw+=dt*feedback.w;
    path+=dt*std::abs(feedback.v);
    hold=(cmd.v==0.&&cmd.w==0.)?hold+dt:0.;maxHold=std::max(maxHold,hold);
    const Cell cell{int(std::floor(p.x/maze.scale)),int(std::floor(p.y/maze.scale))};
    if(leavePocket<0.){
      if(std::abs(cmd.v)>.015){
        const int direction=cmd.v>0.?1:-1;
        if(lastDirection&&lastDirection!=direction)++reversals;
        lastDirection=direction;
      }
      if(cell!=Cell{1,1})leavePocket=tick*dt;
      if(jitter&&(reversals>3||tick*dt>20.)){outcome="oscillating in pocket";break;}
    }
    visited.insert(cell);
    if(verbose&&(cell!=last||tick%500==0||(jitter&&leavePocket<0.&&lastState!=planner.state)))
      std::cout<<now<<" "<<cell.first<<","<<cell.second<<" pos "<<p.x<<","<<p.y<<" yaw "<<p.yaw<<" "<<planner.state<<"\n";
    if(verbose)lastState=planner.state;
    last=cell;
    if(maze.collision(p)){outcome="collision";break;}
    if(hold>3.){outcome="immobile";break;}
    if(p.x>maze.scale*maze.grid[0].size()+std::hypot(body_half_length,body_half_width)&&cell.second==maze.exitRow){
      outcome=jitter&&(!checkedTurn||!checkedShift)?"recovery not exercised":"exit";break;
    }
  }
  std::cout<<"seed "<<seed<<" "<<style<<" "<<outcome<<" path_m "<<path<<" visited "<<visited.size()<<" max_hold_s "<<maxHold
           <<" first_exit_s "<<leavePocket<<" first_reversals "<<reversals
           <<" pose "<<p.x<<","<<p.y<<","<<p.yaw<<" state "<<planner.state<<"\n";
  return std::string(outcome)=="exit";
}
}
int main(int argc,char** argv){
  return run(argc>1?std::atoi(argv[1]):0,argc>2?argv[2]:"walls",argc>3)?0:1;
}
