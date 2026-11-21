#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source ./env.sh
export DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus XDG_RUNTIME_DIR=/run/user/1000
mkdir -p logs
if [[ -f logs/rviz.pid ]]; then
  pid=$(cat logs/rviz.pid)
  if [[ "$pid" =~ ^[0-9]+$ ]] && ps -p "$pid" -o args= | grep -Fq "$LAB26_ROOT/launch/lab26.rviz"; then
    if [[ "${1:-}" != --refresh ]]; then echo 'Robot RViz already running'; exit 0; fi
    kill -TERM "$pid"
    for ((i=0;i<10;i++)); do
      if ! ps -p "$pid" -o args= | grep -Fq "$LAB26_ROOT/launch/lab26.rviz"; then break; fi
      sleep 0.5
    done
  fi
fi
# Show on the robot desktop without unlocking it or changing lock settings.
setsid rviz -d "$LAB26_ROOT/launch/lab26.rviz" >logs/rviz.log 2>&1 </dev/null &
pid=$!; echo "$pid" >logs/rviz.pid
sleep 2
if ! kill -0 "$pid" 2>/dev/null; then tail -n 20 logs/rviz.log; exit 1; fi
echo "Robot RViz running: $pid"
