#!/usr/bin/env bash
set -eo pipefail
stage=/home/nvidia/lab3-xy-extension
root=/home/nvidia/lab26
source "$root/env.sh"
exec 9>"$root/.console-service.lock"
flock -x 9
backup=/home/nvidia/console-deploy-backups/xy_$(date +%Y%m%d_%H%M%S).tar.gz
mkdir -p /home/nvidia/console-deploy-backups
files=(lab3/include lab3/src lab26/catkin_ws/src/basic_control lab26/catkin_ws/devel/lib/basic_control/basicControl lab26/launch lab26/select_experiment.sh lab26/start_sensors.sh)
if [[ -f "$root/verify_sensor_generation.py" ]]; then files+=(lab26/verify_sensor_generation.py); fi
tar -czf "$backup" -C /home/nvidia "${files[@]}"
echo "BACKUP=$backup"
if rosnode list | grep -qx /labConsole; then rosnode kill /labConsole; fi
for pid in $(pgrep -x basicControl || true); do
  command_line=$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)
  if [[ "$command_line" == "$root/catkin_ws/devel/lib/basic_control/basicControl "* && "$command_line" == *"__name:=labConsole "* ]]; then
    kill -INT "$pid"
  fi
done
for n in $(seq 1 20); do if ! ss -ltn | grep -q ':8766 '; then break; fi; sleep 1; done
if ss -ltn | grep -q ':8766 '; then echo 'Controller still active';exit 1;fi
if [[ -f /home/nvidia/lab3-dwa/stop.sh ]]; then bash /home/nvidia/lab3-dwa/stop.sh;fi
restore(){ tar -xzf "$backup" -C /home/nvidia; flock -u 9; bash "$root/launch_console_service.sh"; }
trap restore ERR
cp -a "$stage/实验二/lab26/catkin_ws/src/basic_control/." "$root/catkin_ws/src/basic_control/"
cp -a "$stage/实验三/include/." /home/nvidia/lab3/include/
cp -a "$stage/实验三/src/." /home/nvidia/lab3/src/
if [[ -f "$stage/实验二/lab26/launch/imu.launch" ]]; then
  cp "$stage/实验二/lab26/launch/imu.launch" "$stage/实验二/lab26/launch/rslidar.yaml" "$stage/实验二/lab26/launch/sensors.launch" "$root/launch/"
  cp "$stage/实验二/lab26/"{select_experiment.sh,start_sensors.sh,verify_sensor_generation.py} "$root/"
fi
bash "$root/build.sh"
if [[ -f "$stage/dwa/navigation.launch" ]]; then
  mkdir -p /home/nvidia/lab3-dwa
  cp -a "$stage/dwa/." /home/nvidia/lab3-dwa/
  chmod +x /home/nvidia/lab3-dwa/bridge.py
fi
trap - ERR
flock -u 9
bash "$root/launch_console_service.sh"
if [[ -f /home/nvidia/lab3-dwa/start.sh ]]; then bash /home/nvidia/lab3-dwa/start.sh;fi
echo XY_NAVIGATION_DEPLOYED
