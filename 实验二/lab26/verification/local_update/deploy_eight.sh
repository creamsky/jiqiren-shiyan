#!/usr/bin/env bash
set -euo pipefail
# The Windows deployment step closes only the project frontend after checking
# zero feedback; its loss of connection makes the old controller exit parked.
for ((i=0;i<20;i++)); do
  if ! ss -ltn | grep -q ':8766 '; then break; fi
  sleep 1
done
if ss -ltn | grep -q ':8766 '; then echo 'Existing controller still active; update aborted'; exit 1; fi
stamp=$(date +%Y%m%d_%H%M%S)
mkdir -p /home/nvidia/console-deploy-backups
tar -czf "/home/nvidia/console-deploy-backups/pre_eight_$stamp.tar.gz" -C /home/nvidia \
  lab26/catkin_ws/src/basic_control lab26/catkin_ws/devel/lib/basic_control/basicControl \
  lab26/select_experiment.sh lab1/run_experiment.py
tar -xzf /home/nvidia/lab26-console-stage/eight_update.tar.gz -C /home/nvidia/lab26
cp /home/nvidia/lab26-console-stage/run_experiment_update.py /home/nvidia/lab1/run_experiment.py
set +u
source /home/nvidia/lab26/env.sh
set -u
cd /home/nvidia/lab26/catkin_ws
catkin_make -j2 -DCMAKE_BUILD_TYPE=Release > /home/nvidia/lab26/logs/eight_build.log 2>&1
./devel/lib/basic_control/test_motion
bash -n /home/nvidia/lab26/select_experiment.sh
/usr/bin/python3 -m py_compile /home/nvidia/lab1/run_experiment.py
bash /home/nvidia/lab26/launch_console_service.sh
echo "DEPLOYED; BACKUP=/home/nvidia/console-deploy-backups/pre_eight_$stamp.tar.gz"
