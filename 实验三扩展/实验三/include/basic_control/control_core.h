#ifndef BASIC_CONTROL_CONTROL_CORE_H
#define BASIC_CONTROL_CONTROL_CORE_H

#include <string>

namespace basic_control {

struct MotionCommand {
    double speed = 0.0;
    double turn = 0.0;
};

struct ControlConfig {
    double straight_yaw = 0.7853981633974483;
    double straight_speed = 0.5;
    double avoid_speed = 0.4;
    double k_angle = 0.75;
    double max_turn = 0.8;
    double yaw_tolerance = 0.08;
    double explore_step = 0.08726646259971647;
    double explore_max_angle = 3.14159265358979323846;
    double pose_timeout = 0.5;
    double obstacle_timeout = 0.5;
    double clear_hold_time = 0.2;
    double avoid_distance = 1.5;
    double max_radius = 8.0;
    bool track_reference = false;
    // true: preserve the original 1=left, 2=right turn mapping.
    // false: 1=obstacle on left, 2=obstacle on right; turn away from it.
    bool stop_flag_is_direction = true;

    bool validate(std::string* error = nullptr) const;
};

enum class State {
    WaitingForData,
    Straight,
    Exploring,
    ClearHold,
    AvoidForward,
    Finished,
    Fault
};

const char* stateName(State state);

// This class deliberately contains no ROS/CAN code. Its caller must serialize
// input updates and tick() with the same mutex. Time is monotonic seconds.
class ControlCore {
public:
    explicit ControlCore(const ControlConfig& config = ControlConfig());

    bool onPose(double x, double y, double yaw, double now);
    bool onObstacle(double raw_flag, double now);
    MotionCommand tick(double now);

    State state() const { return state_; }
    double targetYaw() const { return target_yaw_; }
    const ControlConfig& config() const { return config_; }
    static double wrapAngle(double angle);

private:
    void waitForData();
    void startExploration();
    void startClearHold(double now);
    void startAvoidForward();
    double turnCommand() const;
    bool fresh(double timestamp, double now, double timeout) const;

    ControlConfig config_;
    State state_ = State::WaitingForData;
    bool pose_valid_ = false;
    bool obstacle_valid_ = false;
    bool tick_seen_ = false;
    bool recovery_hold_ = false;
    double pose_time_ = 0.0;
    double obstacle_time_ = 0.0;
    double last_tick_ = 0.0;
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    int obstacle_flag_ = 0;
    double target_yaw_ = 0.0;
    int explore_direction_ = 0;
    double explore_origin_yaw_ = 0.0;
    double explored_angle_ = 0.0;
    double clear_since_ = 0.0;
    double avoid_start_x_ = 0.0;
    double avoid_start_y_ = 0.0;
};

}  // namespace basic_control

#endif  // BASIC_CONTROL_CONTROL_CORE_H
