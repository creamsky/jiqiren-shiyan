#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source ./env.sh
mkdir -p logs
if [[ -f logs/sensors.pid ]] && kill -0 "$(cat logs/sensors.pid)" 2>/dev/null && ps -p "$(cat logs/sensors.pid)" -o args= | grep -Fq "$LAB26_ROOT/launch/sensors.launch"; then
  echo "Sensors already running: $(cat logs/sensors.pid)"; exit 0
fi
setsid roslaunch "$LAB26_ROOT/launch/sensors.launch" >logs/sensors.log 2>&1 < /dev/null &
echo $! >logs/sensors.pid
echo "Sensors PID $!; log: $LAB26_ROOT/logs/sensors.log"
