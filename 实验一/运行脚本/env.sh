#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
export LAB1_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LAB1_SOURCE="$LAB1_ROOT/slam_experiment"
slam="$LAB1_SOURCE/FAST-LIO-Localization/FAST-LIO-Localization-QN-master"
imu="$LAB1_SOURCE/FAST-LIO-Localization/imu_drive"
sdk="$LAB1_SOURCE/RS_SDK"
export CMAKE_PREFIX_PATH="$slam/devel:$imu/devel:$sdk/devel:$CMAKE_PREFIX_PATH"
export ROS_PACKAGE_PATH="$slam/src:$imu/src:$sdk/src:$ROS_PACKAGE_PATH"
export LD_LIBRARY_PATH="$slam/devel/lib:$imu/devel/lib:$sdk/devel/lib:/usr/local/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$slam/devel/lib/python3/dist-packages:$sdk/devel/lib/python3/dist-packages:$PYTHONPATH"
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
unset ROS_HOSTNAME
