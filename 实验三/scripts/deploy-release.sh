#!/usr/bin/env bash
set -eo pipefail
stage=/home/nvidia/lab26-console-stage/avoidance_v2
root=/home/nvidia/lab26
source "$root/env.sh"
exec 9>"$root/.console-service.lock"
flock -x 9
test -f "$stage/results/validated.sha256"
(cd "$stage" && sha256sum -c results/validated.sha256)
mkdir -p /home/nvidia/console-deploy-backups
backup="/home/nvidia/console-deploy-backups/lab3_navigation_$(date +%Y%m%d_%H%M%S).tar.gz"
tar -czf "$backup" -C /home/nvidia lab3/include lab3/src lab26/catkin_ws/src/basic_control lab26/catkin_ws/devel/lib/basic_control/basicControl lab26/catkin_ws/devel/lib/basic_control/test_motion
echo "BACKUP=$backup"
if rosnode list | grep -qx /labConsole; then rosnode kill /labConsole; fi
for n in $(seq 1 20); do if ! ss -ltn | grep -q ':8766 '; then break; fi; sleep 1; done
if ss -ltn | grep -q ':8766 '; then echo 'Controller still running; no files changed';exit 1;fi
restore(){ tar -xzf "$backup" -C /home/nvidia; }
trap restore ERR
cp -a "$stage/lab3/include/." /home/nvidia/lab3/include/
cp -a "$stage/lab3/src/." /home/nvidia/lab3/src/
cp -a "$stage/lab26/catkin_ws/src/basic_control/." "$root/catkin_ws/src/basic_control/"
cp "$stage/lab26/catkin_ws/devel/lib/basic_control/basicControl" "$root/catkin_ws/devel/lib/basic_control/basicControl"
cp "$stage/lab26/catkin_ws/devel/lib/basic_control/test_motion" "$root/catkin_ws/devel/lib/basic_control/test_motion"
trap - ERR
flock -u 9
bash "$root/launch_console_service.sh"
echo NAVIGATION_V2_DEPLOYED
