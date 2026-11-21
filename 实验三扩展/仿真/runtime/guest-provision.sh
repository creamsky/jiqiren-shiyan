#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
sudo mkdir -p /etc/apt/keyrings
curl -fsSL --connect-timeout 15 https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key | sudo gpg --dearmor --yes -o /etc/apt/keyrings/ros.gpg
printf '%s\n' 'deb [arch=amd64 signed-by=/etc/apt/keyrings/ros.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main' | sudo tee /etc/apt/sources.list.d/ros.list >/dev/null
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  ros-noetic-ros-base ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-velodyne-simulator \
  ros-noetic-navigation ros-noetic-mpc-local-planner ros-noetic-teb-local-planner ros-noetic-cartographer-ros ros-noetic-pointcloud-to-laserscan ros-noetic-rosbridge-server python3-numpy \
  python3-yaml xvfb xauth libgl1-mesa-dri mesa-utils unzip \
  novnc python3-websockify x11vnc fluxbox wmctrl x11-apps
mkdir -p "$HOME/scout-lab"
echo GAZEBO_RUNTIME_INSTALLED
