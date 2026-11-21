#!/usr/bin/env bash
set -eo pipefail
root=/home/nvidia/lab26
source "$root/env.sh"
exec 9>"$root/.console-service.lock"
flock -x 9
mkdir -p /home/nvidia/console-deploy-backups
backup="/home/nvidia/console-deploy-backups/controller_$(date +%Y%m%d_%H%M%S).tar.gz"
tar -czf "$backup" -C "$root" catkin_ws/src/basic_control catkin_ws/devel/lib/basic_control/basicControl
echo "BACKUP=$backup"
if rosnode list | grep -qx /labConsole; then rosnode kill /labConsole; fi
for n in $(seq 1 20); do
  if ! ss -ltn | grep -q ':8766 '; then break; fi
  sleep 1
done
if ss -ltn | grep -q ':8766 '; then echo 'Controller still active; aborting'; exit 1; fi
tar -xzf /home/nvidia/lab26-console-stage/controller_update.tar.gz -C "$root"
if ! bash "$root/build.sh"; then
  tar -xzf "$backup" -C "$root"
  echo 'Build failed; restored source and binary backup'
  exit 1
fi
flock -u 9
bash "$root/launch_console_service.sh"
echo CONTROLLER_DEPLOYED
