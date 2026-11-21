#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source ./env.sh
rosnode kill /basicControl 2>/dev/null || true
if [[ -f logs/sensors.pid ]]; then
  pid="$(cat logs/sensors.pid)"
  if [[ "$pid" =~ ^[0-9]+$ ]] && ps -p "$pid" -o args= | grep -Fq "$LAB26_ROOT/launch/sensors.launch"; then
    kill -INT -- "-$pid"
    echo "Stopped lab26 sensor launch group $pid"
  fi
fi
