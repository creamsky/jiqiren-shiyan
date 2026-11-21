#!/usr/bin/env bash
source /home/nvidia/lab26/env.sh
prefix=/home/nvidia/lab3-dwa/vendor/root/opt/ros/noetic
export CMAKE_PREFIX_PATH="$prefix:$CMAKE_PREFIX_PATH"
export ROS_PACKAGE_PATH="/home/nvidia/lab3-dwa:$prefix/share:$ROS_PACKAGE_PATH"
export LD_LIBRARY_PATH="$prefix/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$prefix/lib/python3/dist-packages:$PYTHONPATH"
