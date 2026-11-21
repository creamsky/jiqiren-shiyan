#!/usr/bin/env bash
set -euo pipefail
exec 9>/home/nvidia/lab26/.console-service.lock
flock -x 9
if ss -ltn | grep -q ':8766 '; then echo 'Controller still active; park and close project frontend before updating' >&2; exit 1; fi
if pgrep -f '^(/usr/bin/)?python3 /home/nvidia/lab1/run_experiment.py( |$)' >/dev/null; then echo 'Experiment 1 still active; update aborted' >&2; exit 1; fi
stamp=$(date +%Y%m%d_%H%M%S)
tar -czf "/home/nvidia/console-deploy-backups/pre_wasd_ack_$stamp.tar.gz" -C /home/nvidia/lab26 \
  catkin_ws/src/basic_control catkin_ws/devel/lib/basic_control/basicControl
tar -xzf /home/nvidia/lab26-console-stage/manual_fix.tar.gz -C /home/nvidia/lab26
bash /home/nvidia/lab26/build.sh
flock -u 9
bash /home/nvidia/lab26/launch_console_service.sh
echo "DEPLOYED; BACKUP=/home/nvidia/console-deploy-backups/pre_wasd_ack_$stamp.tar.gz"
