#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source ./env.sh
mode="${1:---dry-run}"
if [[ "$mode" != --dry-run && "$mode" != --live && "$mode" != --preview ]]; then
  echo "Usage: bash ~/lab26/start.sh [--preview|--dry-run|--live]"; exit 1
fi
bash ./start_sensors.sh
bash ./show_rviz.sh
if [[ "$mode" == --preview ]]; then
  python3 "$LAB26_ROOT/check_topics.py"
  exit 0
fi
dry=true
if [[ "$mode" == --live ]]; then
  dry=false
  # Use existing permissions for IMU; nvidia is already in dialout.
  if ! ip -details link show can0 | grep -q 'bitrate 500000'; then
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 500000
  fi
  if ! ip link show can0 | grep -q 'LOWER_UP'; then
    sudo ip link set can0 up
  fi
  python3 "$LAB26_ROOT/enable_can_mode.py"
fi
stamp="$(date +%Y%m%d_%H%M%S)"
csv="$LAB26_ROOT/logs/control_$stamp.csv"
set +e
roslaunch basic_control basicControl.launch dry_run:="$dry" csv:="$csv" run_timeout:="${LAB26_RUN_TIMEOUT:-180}" 2>&1 | tee "$LAB26_ROOT/logs/control_$stamp.log"
launch_result=${PIPESTATUS[0]}
set -e
python3 - "$csv" "$launch_result" <<'PY'
import csv, sys
try:
    rows=list(csv.DictReader(open(sys.argv[1])))
    stage=rows[-1]['stage'] if rows else 'NO DATA'
except OSError:
    stage='NO LOG'
if stage=='DONE' and sys.argv[2]=='0':
    print('EXPERIMENT COMPLETED: both velocities stopped. Log: '+sys.argv[1])
else:
    print('EXPERIMENT STOPPED: '+stage+'. Check the RViz status and the matching .log file.')
    sys.exit(2)
PY
