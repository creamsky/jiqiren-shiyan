#!/usr/bin/env bash
# Source this file; do not source relocated, mutually overwriting overlays.
source /opt/ros/noetic/setup.bash
export LAB26_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LAB26_ORIGINAL=/home/nvidia/实验代码2026_10_14/实验代码/实验二
imu="$LAB26_ORIGINAL/FAST-LIO-Localization/imu_drive"
lio="$LAB26_ORIGINAL/FAST-LIO-Localization/FAST-LIO-Localization-QN-master"
sdk="$LAB26_ORIGINAL/RS_SDK"
export CMAKE_PREFIX_PATH="$LAB26_ROOT/catkin_ws/devel:$sdk/devel_isolated/rslidar_sdk:$sdk/devel_isolated/rs_to_velodyne:$imu/devel:$lio/devel:$CMAKE_PREFIX_PATH"
export ROS_PACKAGE_PATH="$LAB26_ROOT/catkin_ws/src:$sdk/src:$imu/src:$lio/src:$ROS_PACKAGE_PATH"
export LD_LIBRARY_PATH="$LAB26_ROOT/catkin_ws/devel/lib:$sdk/devel_isolated/rslidar_sdk/lib:$sdk/devel_isolated/rs_to_velodyne/lib:$imu/devel/lib:$lio/devel/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$LAB26_ROOT/catkin_ws/devel/lib/python3/dist-packages:$PYTHONPATH"
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
