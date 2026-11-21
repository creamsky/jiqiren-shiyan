#pragma once
#include "basic_control/footprint.h"
#include <string>
#include <vector>

namespace lab3 {
struct Pose { double x=0,y=0,yaw=0; };
struct Velocity { double v=0,w=0; };
struct Point { double x=0,y=0; };
std::vector<lab26::ObstaclePoint> reframe(const std::vector<lab26::ObstaclePoint>& scan,Pose sampled,Pose current);

// Coordinates are relative to the start pose; the fixed goal is (distance, 0).
// No ROS/CAN access: the production caller and simulation share this entire loop.
class Navigator {
 public:
  explicit Navigator(double distance=10):distance_(distance),remaining_(distance){}
  Velocity step(Pose pose,const std::vector<lab26::ObstaclePoint>& scan,
                Velocity previous,Velocity feedback,double now);
  const char* state() const { return state_; }
  const std::string& reason() const { return reason_; }
  bool finished() const { return finished_; }
  bool failed() const { return failed_; }
  int replans() const { return replans_; }
  double remaining() const { return remaining_; }
  const std::vector<Point>& path() const { return path_; }
 private:
  bool plan(Pose pose,const std::vector<Point>& obstacles);
  bool visible(Point a,Point b,const std::vector<Point>& obstacles) const;
  std::vector<Point> obstacles(Pose pose,const std::vector<lab26::ObstaclePoint>& scan) const;
  Velocity recover(Pose pose,const std::vector<lab26::ObstaclePoint>& scan,Velocity previous,Velocity feedback,double now,double dt,const char* reason);
  double distance_=10,remaining_=10,last_tick_=-1,last_plan_=-1,last_check_=-1;
  double progress_time_=-1;
  double direct_clear_since_=-1;
  int detour_side_=0;
  int entry_turn_side_=0;
  double entry_yaw_=0;
  bool avoiding_=false,goal_align_=false,rejoining_=false;
  double settled_since_=-1,recovery_until_=-1;
  double rejoin_settled_since_=-1;
  Pose recovery_origin_;
  int recovery_attempts_=0;
  Velocity recovery_command_;
  Point rejoin_target_;
  Point progress_pose_;
  const char* state_="waiting_for_data";
  std::string reason_;
  bool finished_=false,failed_=false;
  int replans_=0;
  std::vector<Point> path_;
};
}
