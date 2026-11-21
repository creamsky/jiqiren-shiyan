#!/usr/bin/env bash
set -eo pipefail
cd /home/nvidia/lab3-dwa
source ./env.sh
mkdir -p logs
if [[ -f logs/navigation.pid ]]; then
  pid=$(cat logs/navigation.pid)
  if [[ "$pid" =~ ^[0-9]+$ ]] && ps -p "$pid" -o args= | grep -Fq '/home/nvidia/lab3-dwa/navigation.launch'; then
    echo 'DWA already running';exit 0
  fi
fi
chmod +x bridge.py
setsid roslaunch /home/nvidia/lab3-dwa/navigation.launch >logs/navigation.log 2>&1 </dev/null &
echo $! >logs/navigation.pid
echo 'DWA started; it proposes velocities only and cannot drive without the controller.'
