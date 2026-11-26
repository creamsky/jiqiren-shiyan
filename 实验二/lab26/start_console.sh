#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
# ROS setup scripts are not nounset-safe.
set +u
source ./env.sh
set -u
bash ./keep_screen_awake.sh
if ! rostopic list 2>/dev/null | grep -qx /velodyne_points; then
  bash ./start_sensors.sh
fi
# The sensor launch may need to create the master. Starting another roslaunch
# immediately can race it and leave two masters competing for port 11311.
deadline=$((SECONDS+45))
until timeout 3 rosparam list >/dev/null 2>&1; do
  if ((SECONDS>=deadline)); then echo 'ROS master not ready; inspect logs/sensors.log' >&2; exit 1; fi
  sleep 1
done
mkdir -p logs
roslaunch basic_control basicControl.launch node_name:=labConsole console:=true dry_run:=false csv:="$LAB26_ROOT/logs/console_$(date +%Y%m%d_%H%M%S).csv"
