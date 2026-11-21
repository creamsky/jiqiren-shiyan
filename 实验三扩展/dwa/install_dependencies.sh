#!/usr/bin/env bash
set -euo pipefail
root=/home/nvidia/lab3-dwa/vendor
mkdir -p "$root/debs" "$root/root"
cd "$root"
for package in ros-noetic-dwa-local-planner ros-noetic-base-local-planner ros-noetic-costmap-2d ros-noetic-nav-core ros-noetic-voxel-grid ros-noetic-navfn ros-noetic-move-base ros-noetic-move-base-msgs; do
  mapfile -t fields < <(awk -v package="$package" 'BEGIN{RS="";FS="\n"} {found=0;file="";sha="";for(i=1;i<=NF;i++){if($i=="Package: "package)found=1;if($i~/^Filename: /)file=substr($i,11);if($i~/^SHA256: /)sha=substr($i,9);}if(found){print file;print sha;exit}}' Packages)
  [[ ${#fields[@]} == 2 && "${fields[0]}" == pool/*.deb && "${fields[1]}" =~ ^[0-9a-f]{64}$ ]]
  file="debs/${fields[0]##*/}"
  curl -fsSL --connect-timeout 8 --max-time 50 "https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/${fields[0]}" -o "$file"
  printf '%s  %s\n' "${fields[1]}" "$file" | sha256sum -c -
  dpkg-deb -x "$file" root
done
touch "$root/root/opt/ros/noetic/.catkin"
echo 'DWA dependencies extracted into private directory; system packages unchanged.'
