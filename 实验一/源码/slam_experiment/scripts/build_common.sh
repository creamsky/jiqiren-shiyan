#!/usr/bin/env bash
# Shared build helpers; sourced by scripts in this directory.
set -euo pipefail

EXPERIMENT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
LIDAR_WS="$EXPERIMENT_ROOT/RS_SDK"
IMU_WS="$EXPERIMENT_ROOT/FAST-LIO-Localization/imu_drive"
SLAM_WS="$EXPERIMENT_ROOT/FAST-LIO-Localization/FAST-LIO-Localization-QN-master"

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

source_setup() {
    local setup_file="$1"
    [[ -f "$setup_file" ]] || fail "Setup file not found: $setup_file"
    # ROS-generated setup files are not guaranteed to be safe with nounset.
    set +u
    if ! source "$setup_file"; then
        set -u
        fail "Could not source ROS setup file: $setup_file"
    fi
    set -u
}

load_ros() {
    [[ "$(uname -s)" == Linux ]] || fail "Run this script in the car's Ubuntu terminal, reached through ToDesk."
    if [[ "${ROS_VERSION:-1}" != 1 ]]; then
        fail "This shell contains ROS 2. Open a fresh Ubuntu terminal for ROS 1 Noetic."
    fi
    ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
    [[ -f "$ROS_SETUP" ]] || fail "ROS Noetic is missing: $ROS_SETUP. Install ROS Noetic on Ubuntu 20.04 first, or set ROS_SETUP to its setup.bash."
    source_setup "$ROS_SETUP"
    [[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]] || fail "Supported target: ROS 1 Noetic. Selected setup reports ROS_VERSION=${ROS_VERSION:-unset}, ROS_DISTRO=${ROS_DISTRO:-unset}."
    ROS_PREFIX="$(cd -- "$(dirname -- "$ROS_SETUP")" && pwd -P)"
    export ROS_PREFIX
    if [[ -n "${TEASERPP_PREFIX:-}" ]]; then
        [[ -d "$TEASERPP_PREFIX" ]] || fail "TEASERPP_PREFIX is not a directory: $TEASERPP_PREFIX"
        export CMAKE_PREFIX_PATH="$TEASERPP_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
        # catkin may rebuild an isolated environment from its explicit --extend.
        # Pass the dependency config directly as well as the prefix, if found.
        if [[ -z "${teaserpp_DIR:-}" ]]; then
            local candidate
            for candidate in "$TEASERPP_PREFIX"/lib/cmake/teaserpp "$TEASERPP_PREFIX"/share/teaserpp/cmake "$TEASERPP_PREFIX"/lib/*/cmake/teaserpp; do
                if [[ -f "$candidate/teaserppConfig.cmake" ]]; then
                    export teaserpp_DIR="$candidate"
                    break
                fi
            done
        fi
    fi
}

validate_jobs() {
    JOBS="${JOBS:-2}"
    [[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || fail "JOBS must be a positive integer, e.g. JOBS=2."
    export JOBS
}
