#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source /opt/ros/noetic/setup.bash
mkdir -p logs
cd catkin_ws
catkin_make -j2 -DCMAKE_BUILD_TYPE=Release
./devel/lib/basic_control/test_motion
