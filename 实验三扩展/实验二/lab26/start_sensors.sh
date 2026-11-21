#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source ./env.sh
mkdir -p logs
exec 8>logs/sensor-start.lock
flock -x 8
if [[ -f logs/sensors.pid ]]; then
  pid=$(cat logs/sensors.pid)
  if [[ "$pid" =~ ^[0-9]+$ ]] && ps -p "$pid" -o args= | grep -Fq "$LAB26_ROOT/launch/sensors.launch"; then
    echo "Sensors already running: $pid"; exit 0
  fi
fi
# Keep the ROS master independent of the sensor group so the live control
# socket, DWA registrations and desktop cloud subscribers survive each prepare.
if ! timeout 2 rosparam get /run_id >/dev/null 2>&1; then
  setsid roscore 8>&- >logs/roscore.log 2>&1 </dev/null &
  core_pid=$!; echo "$core_pid" >logs/roscore.pid
  deadline=$((SECONDS+15))
  until timeout 2 rosparam get /run_id >/dev/null 2>&1; do
    if ! kill -0 "$core_pid" 2>/dev/null || ((SECONDS>=deadline)); then echo 'ROS master startup failed' >&2; exit 1; fi
    sleep .2
  done
fi
rosparam set /lab26/map_generation "'$(date +%s%N)'"
setsid roslaunch --wait "$LAB26_ROOT/launch/sensors.launch" 8>&- >logs/sensors.log 2>&1 </dev/null &
echo $! >logs/sensors.pid
echo "Sensors PID $!; log: $LAB26_ROOT/logs/sensors.log"
