#include "basic_control/control_core.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace basic_control {
namespace {
constexpr double kPi = 3.14159265358979323846;

bool finiteNonnegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}
}  // namespace

bool ControlConfig::validate(std::string* error) const {
    const auto reject = [error](const char* message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };
    if (!std::isfinite(straight_yaw)) {
        return reject("straight_yaw must be finite");
    }
    if (!finiteNonnegative(straight_speed) || straight_speed > 32.767 ||
        !finiteNonnegative(avoid_speed) || avoid_speed > 32.767) {
        return reject("speed must be finite and in [0, 32.767]");
    }
    if (!std::isfinite(k_angle) || k_angle <= 0.0 ||
        !std::isfinite(max_turn) || max_turn <= 0.0 || max_turn > 32.767) {
        return reject("k_angle and max_turn must be positive; max_turn <= 32.767");
    }
    if (!std::isfinite(yaw_tolerance) || yaw_tolerance <= 0.0 ||
        !std::isfinite(explore_step) || explore_step <= yaw_tolerance ||
        explore_step > kPi / 2.0 || !std::isfinite(explore_max_angle) ||
        explore_max_angle < explore_step || explore_max_angle > kPi) {
        return reject("require 0 < yaw_tolerance < explore_step <= pi/2 and "
                      "explore_step <= explore_max_angle <= pi");
    }
    if (!std::isfinite(pose_timeout) || pose_timeout <= 0.0 ||
        !std::isfinite(obstacle_timeout) || obstacle_timeout <= 0.0 ||
        !finiteNonnegative(clear_hold_time)) {
        return reject("timeouts must be positive and clear_hold_time nonnegative");
    }
    if (!std::isfinite(avoid_distance) || avoid_distance <= 0.0 ||
        !std::isfinite(max_radius) || max_radius <= 0.0) {
        return reject("avoid_distance and max_radius must be positive");
    }
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

const char* stateName(State state) {
    switch (state) {
        case State::WaitingForData: return "waiting_for_data";
        case State::Straight: return "straight";
        case State::Exploring: return "exploring";
        case State::ClearHold: return "clear_hold";
        case State::AvoidForward: return "avoid_forward";
        case State::Finished: return "finished";
        case State::Fault: return "fault";
    }
    return "unknown";
}

ControlCore::ControlCore(const ControlConfig& config) : config_(config) {
    std::string error;
    if (!config_.validate(&error)) {
        throw std::invalid_argument(error);
    }
    config_.straight_yaw = wrapAngle(config_.straight_yaw);
    target_yaw_ = config_.straight_yaw;
}

double ControlCore::wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

bool ControlCore::onPose(double x, double y, double yaw, double now) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
        !finiteNonnegative(now) || (pose_valid_ && now < pose_time_)) {
        pose_valid_ = false;
        return false;
    }
    x_ = x;
    y_ = y;
    yaw_ = wrapAngle(yaw);
    pose_time_ = now;
    pose_valid_ = true;
    // Completion is latched, so localization jitter cannot restart the robot.
    const bool reached=config_.track_reference?
        std::hypot(x_-config_.max_radius*std::cos(config_.straight_yaw),
                   y_-config_.max_radius*std::sin(config_.straight_yaw))<=0.15:
        std::hypot(x_, y_)>config_.max_radius;
    if (reached) {
        state_ = State::Finished;
    }
    return true;
}

bool ControlCore::onObstacle(double raw_flag, double now) {
    if (!finiteNonnegative(now) || !std::isfinite(raw_flag) ||
        (raw_flag != 0.0 && raw_flag != 1.0 && raw_flag != 2.0) ||
        (obstacle_valid_ && now < obstacle_time_)) {
        obstacle_valid_ = false;
        return false;
    }
    obstacle_flag_ = static_cast<int>(raw_flag);
    obstacle_time_ = now;
    obstacle_valid_ = true;
    // Preserve even a brief stop event between two output ticks. If a clear
    // message follows immediately, tick() must still require the clear hold.
    if (obstacle_flag_ != 0 && pose_valid_ &&
        (state_ == State::Straight || state_ == State::AvoidForward ||
         state_ == State::ClearHold)) {
        startExploration();
    }
    return true;
}

bool ControlCore::fresh(double timestamp, double now, double timeout) const {
    const double age = now - timestamp;
    return age >= 0.0 && age <= timeout;
}

void ControlCore::waitForData() {
    if (state_ != State::WaitingForData) {
        recovery_hold_ = true;
    }
    state_ = State::WaitingForData;
    explore_direction_ = 0;
    explored_angle_ = 0.0;
}

void ControlCore::startExploration() {
    state_ = State::Exploring;
    explore_direction_ = obstacle_flag_ == 1 ? 1 : -1;
    if (!config_.stop_flag_is_direction) {
        explore_direction_ = -explore_direction_;
    }
    explore_origin_yaw_ = yaw_;
    explored_angle_ = config_.explore_step;
    target_yaw_ = wrapAngle(explore_origin_yaw_ +
                            explore_direction_ * explored_angle_);
}

void ControlCore::startClearHold(double now) {
    state_ = State::ClearHold;
    // A clear signal refers to the present heading. Discard the old scan target.
    target_yaw_ = yaw_;
    clear_since_ = now;
}

void ControlCore::startAvoidForward() {
    state_ = State::AvoidForward;
    // Count bypass distance from the start of actual forward motion, not from
    // the start of the scan or the clear-hold interval.
    avoid_start_x_ = x_;
    avoid_start_y_ = y_;
    recovery_hold_ = false;
}

double ControlCore::turnCommand() const {
    const double error = wrapAngle(target_yaw_ - yaw_);
    return std::max(-config_.max_turn,
                    std::min(config_.max_turn, config_.k_angle * error));
}

MotionCommand ControlCore::tick(double now) {
    MotionCommand command;
    if (state_ == State::Finished || state_ == State::Fault) {
        return command;
    }
    if (!finiteNonnegative(now) || (tick_seen_ && now < last_tick_)) {
        pose_valid_ = false;
        obstacle_valid_ = false;
        waitForData();
        tick_seen_ = false;
        return command;
    }
    last_tick_ = now;
    tick_seen_ = true;
    if (!pose_valid_ || !obstacle_valid_ ||
        !fresh(pose_time_, now, config_.pose_timeout) ||
        !fresh(obstacle_time_, now, config_.obstacle_timeout)) {
        waitForData();
        return command;
    }

    if (state_ == State::WaitingForData) {
        if (obstacle_flag_ != 0) {
            startExploration();
        } else if (recovery_hold_) {
            startClearHold(now);
        } else {
            state_ = State::Straight;
            target_yaw_ = config_.straight_yaw;
        }
    }

    if (state_ == State::Straight) {
        if (obstacle_flag_ != 0) {
            startExploration();
        } else {
            if(config_.track_reference) {
                const double c=std::cos(config_.straight_yaw),s=std::sin(config_.straight_yaw);
                const double along=x_*c+y_*s;
                const double target=std::min(config_.max_radius,std::max(0.0,along)+0.6);
                target_yaw_=std::atan2(target*s-y_,target*c-x_);
            }
            command.speed = config_.straight_speed;
            command.turn = turnCommand();
            if(config_.track_reference&&std::abs(wrapAngle(target_yaw_-yaw_))>0.35)command.speed=0;
            return command;
        }
    }

    if (state_ == State::Exploring) {
        if (obstacle_flag_ == 0) {
            startClearHold(now);
        } else {
            const double error = wrapAngle(target_yaw_ - yaw_);
            if (std::abs(error) <= config_.yaw_tolerance) {
                if (explored_angle_ >= config_.explore_max_angle) {
                    state_ = State::Fault;
                    return command;
                }
                explored_angle_ = std::min(config_.explore_max_angle,
                                            explored_angle_ + config_.explore_step);
                target_yaw_ = wrapAngle(explore_origin_yaw_ +
                                        explore_direction_ * explored_angle_);
            }
            command.turn = turnCommand();
            return command;
        }
    }

    if (state_ == State::ClearHold) {
        if (obstacle_flag_ != 0) {
            startExploration();
            command.turn = turnCommand();
            return command;
        }
        if (now - clear_since_ < config_.clear_hold_time) {
            command.turn = turnCommand();
            return command;
        }
        // Drift while stopped is corrected before releasing forward motion.
        if (std::abs(wrapAngle(target_yaw_ - yaw_)) > config_.yaw_tolerance) {
            command.turn = turnCommand();
            return command;
        }
        startAvoidForward();
    }

    if (state_ == State::AvoidForward) {
        if (obstacle_flag_ != 0) {
            startExploration();
            command.turn = turnCommand();
            return command;
        }
        if (std::hypot(x_ - avoid_start_x_, y_ - avoid_start_y_) >=
            config_.avoid_distance) {
            state_ = State::Straight;
            target_yaw_ = config_.straight_yaw;
            // Re-evaluate the route target before moving after a bypass.
            return tick(now);
        } else {
            command.speed = config_.avoid_speed;
        }
        command.turn = turnCommand();
    }
    return command;
}

}  // namespace basic_control
