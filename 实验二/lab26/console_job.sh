#!/usr/bin/env bash
# Launch once on the robot. SSH only relays the local job's log and exit code.
set -eu
cd "$(dirname "$0")"
id=${1:-}
kind=${2:-}
[[ $id =~ ^[a-f0-9-]{36}$ ]] || exit 2
case "$kind" in
  lab1) command=(bash /home/nvidia/lab1/start.sh 30);;
  stopLab1) command=(bash /home/nvidia/lab1/stop.sh);;
  maze) command=(bash ./select_experiment.sh lab2 --roam);;
  lab2|lab2-eight|lab3|lab3-xy) command=(bash ./select_experiment.sh lab2);;
  *) exit 2;;
esac
dir="logs/console-jobs/$id"
if [[ ${3:-} == worker ]]; then
  exec >>"$dir/output" 2>&1
  echo "$$" > "$dir/pid"
  code=0
  "${command[@]}" || code=$?
  echo "$code" > "$dir/result.tmp"
  mv "$dir/result.tmp" "$dir/result"
  exit "$code"
fi
unit="lab26-job-$id.service"
if [[ ${3:-} != observe ]]; then
  mkdir -p "$dir"
  (
    flock -x 9
    if [[ ! -e "$dir/pid" && ! -e "$dir/result" ]] && ! systemctl --user is-active --quiet "$unit"; then
      touch "$dir/output"
      # The scope stays alive for ROS and RViz after the preparation worker exits.
      systemd-run --user --quiet --collect --unit="$unit" /usr/bin/systemd-run --user --scope --quiet --unit="lab26-runtime-$id" /bin/bash "$PWD/console_job.sh" "$id" "$kind" worker
    fi
  ) 9>"$dir/start.lock"
fi
for ((i=0;i<50;i++)); do
  [[ -s "$dir/pid" || -f "$dir/result" ]] && break
  sleep 0.1
done
if [[ -f "$dir/result" ]]; then
  cat "$dir/output"
elif [[ -s "$dir/pid" ]]; then
  tail -n +1 --pid="$(cat "$dir/pid")" -f "$dir/output"
else
  echo 'Vehicle job has no worker or result'
  exit 4
fi
test -f "$dir/result" || { echo 'Vehicle job exited without a result'; exit 1; }
exit "$(cat "$dir/result")"
