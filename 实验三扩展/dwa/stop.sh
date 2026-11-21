#!/usr/bin/env bash
set -e
file=/home/nvidia/lab3-dwa/logs/navigation.pid
if [[ -f "$file" ]]; then
  pid=$(cat "$file")
  if [[ "$pid" =~ ^[0-9]+$ ]] && ps -p "$pid" -o args= | grep -Fq '/home/nvidia/lab3-dwa/navigation.launch'; then
    kill -INT -- "-$pid"
    for n in $(seq 1 30); do if ! kill -0 "$pid" 2>/dev/null; then exit 0;fi;sleep .2;done
    echo 'DWA process has not exited' >&2;exit 1
  fi
fi
