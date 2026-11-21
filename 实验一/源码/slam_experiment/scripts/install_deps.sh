#!/usr/bin/env bash
# Explicit installation step. Never called automatically by build_all.sh.
set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/build_common.sh"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    printf 'Usage: bash scripts/install_deps.sh\nInstalls apt dependencies on Ubuntu 20.04 with ROS Noetic already installed.\nTEASER++ must be installed separately from a reviewed upstream source.\n'
    exit 0
fi
[[ $# -eq 0 ]] || fail "Unexpected arguments. Use --help."
[[ -r /etc/os-release ]] || fail "This installer requires Ubuntu 20.04."
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 20.04 ]] || fail "Supported target: Ubuntu 20.04 with ROS Noetic. Detected ${PRETTY_NAME:-unknown OS}."
load_ros
command -v sudo >/dev/null 2>&1 || fail "sudo is required to install apt packages."
printf 'Installing system and ROS Noetic dependencies with apt. TEASER++ is a separate step.\n'
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake git pkg-config \
    python3-catkin-tools python3-rosdep python3-dev python3-numpy \
    libeigen3-dev libpcl-dev libpcap-dev libyaml-cpp-dev libtbb-dev \
    libboost-all-dev libapr1-dev pcl-tools \
    ros-noetic-roscpp ros-noetic-rospy ros-noetic-rosbag ros-noetic-roslib \
    ros-noetic-std-msgs ros-noetic-sensor-msgs ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs ros-noetic-visualization-msgs \
    ros-noetic-tf ros-noetic-tf-conversions ros-noetic-eigen-conversions \
    ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-message-filters \
    ros-noetic-message-generation ros-noetic-message-runtime \
    ros-noetic-serial ros-noetic-rviz
printf '\nApt installation completed. TEASER++ is required by Quatro; see the accompanying guide.\n'
