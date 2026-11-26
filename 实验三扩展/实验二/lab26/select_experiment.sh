#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
[[ "${1:-}" == lab2 ]] || { echo 'Unsupported experiment'; exit 2; }
[[ -z "${2:-}" || "$2" == --sensors-only || "$2" == --roam ]] || { echo 'Unsupported prepare option'; exit 2; }
sensors_only=false
if [[ "${2:-}" == --sensors-only ]]; then sensors_only=true; fi
source ./env.sh
mkdir -p logs
exec 7>logs/experiment-prepare.lock
flock -n 7 || { echo 'Another prepare is in progress' >&2; exit 1; }
# Wait for the identified USB devices, not a stale ROS topic or tty number.
device_deadline=$((SECONDS+30))
until [[ -c /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 && -e /sys/class/net/can0 ]]; do
  if ((SECONDS>=device_deadline)); then echo 'IMU USB serial or USB-CAN missing; check device connection' >&2; exit 1; fi
  sleep 1
done
timeout 3 candump -L -n 25 can0,221:7FF | awk -F'#' 'length($2)==16 && substr($2,1,8)=="00000000"{n++} END{exit n!=25}' || { echo 'Chassis must be stopped before sensor restart' >&2; exit 1; }
if ss -ltn | grep -q ':8766 '; then
  status=$(timeout 4 rostopic echo -n 1 /lab26/status)
  echo "$status" | grep -Eq 'data: .*DONE|data: .*FAULT|data: .*WAITING' || { echo 'Finish or stop the active task before prepare' >&2; exit 1; }
fi
# Read the actual mode and RC switch before requesting network control.
frame() { timeout 2 candump -L -n 1 "can0,$1:7FF" | awk -F'#' 'length($2)==16 {print $2}'; }
check_chassis(){
  system_frame=$(frame 211)
  switch_frame=$(frame 241)
  motion_frame=$(frame 221)
  if [[ ! $system_frame =~ ^[0-9A-Fa-f]{16}$ || ! $switch_frame =~ ^[0-9A-Fa-f]{16}$ || ! $motion_frame =~ ^[0-9A-Fa-f]{16}$ ]]; then
    echo 'Missing CAN system, RC or velocity feedback' >&2; exit 1
  fi
  rc=$(( (16#${switch_frame:0:2} >> 2) & 3 ))
  mode=${system_frame:2:2}
  if [[ ${system_frame:0:2} != 00 || ${system_frame:8:4} != 0000 || ${motion_frame:0:8} != 00000000 ]] ||
     [[ $mode != 00 && $mode != 01 && $mode != 03 ]]; then
    echo "CAN initialization refused: mode=$mode RC=$rc fault=${system_frame:8:4} velocity=${motion_frame:0:8}" >&2; exit 1
  fi
}
check_chassis
if [[ -f /home/nvidia/lab3-dwa/stop.sh ]]; then bash /home/nvidia/lab3-dwa/stop.sh;fi
# The local console pauses and verifies zero output before invoking this script.
if [[ -f /home/nvidia/lab1/stop.sh ]]; then
  bash /home/nvidia/lab1/stop.sh
fi
# A previous sensor launch can remain alive after the master owned by lab1
# exits. Restart this project's sensors before declaring the new task ready.
stop_sensors() {
  bash ./stop.sh
  if [[ -f logs/sensors.pid ]]; then
    local pid deadline=$((SECONDS+40))
    pid=$(cat logs/sensors.pid)
    while [[ "$pid" =~ ^[0-9]+$ ]] && ps -p "$pid" -o args= | grep -Fq "$LAB26_ROOT/launch/sensors.launch"; do
      if ((SECONDS>=deadline)); then echo 'Previous sensors did not exit' >&2; return 1; fi
      sleep 1
    done
  fi
}
wait_topic() {
  local topic="$1" deadline=$((SECONDS+$2))
  until timeout 3 rostopic echo -n 1 "$topic/header" >/dev/null 2>&1; do
    if ((SECONDS>=deadline)); then
      echo "Sensor not ready: $topic; inspect $LAB26_ROOT/logs/sensors.log" >&2
      return 1
    fi
    sleep 1
  done
}
node_pid(){ timeout 4 rosnode info "$1" 2>/dev/null | awk '/^Pid:/{print $2}'; }
nodes=(/rslidar_sdk_node /rs_to_velodyne /laserMapping)
old_pids=()
for node in "${nodes[@]}"; do old_pids+=("$(node_pid "$node")"); done
echo 'Restarting lidar, converter, IMU and FAST-LIO'
after=$(date +%s.%N)
stop_sensors
bash ./start_sensors.sh 7>&-
if ! wait_topic /imu 12; then
  echo 'IMU is silent; restarting sensors once.'
  stop_sensors
  after=$(date +%s.%N)
  bash ./start_sensors.sh 7>&-
fi
"${ROBOT_PYTHON:-/usr/bin/python3}" ./verify_sensor_generation.py "$after"
for i in "${!nodes[@]}"; do
  new=$(node_pid "${nodes[$i]}")
  [[ "$new" =~ ^[0-9]+$ && "$new" != "${old_pids[$i]}" ]] || { echo "Sensor did not restart: ${nodes[$i]}" >&2; exit 1; }
  echo "${nodes[$i]} PID ${old_pids[$i]:-none} -> $new"
done
check_chassis
if ! $sensors_only && [[ $mode == 00 && $rc != 1 && $rc != 3 ]]; then
  echo 'Requesting CAN mode at zero speed'
  "${ROBOT_PYTHON:-/usr/bin/python3}" "$LAB26_ROOT/enable_can_mode.py"
fi
if [[ ${2:-} != --roam && -f /home/nvidia/lab3-dwa/start.sh ]]; then bash /home/nvidia/lab3-dwa/start.sh 7>&-;fi
bash ./keep_screen_awake.sh 7>&-
bash ./display_console.sh 7>&-
if [[ ${2:-} == --roam ]]; then
  binary="$LAB26_ROOT/catkin_ws/devel/lib/basic_control/basicControl"
  pid=$(node_pid /labConsole)
  if [[ ! "$pid" =~ ^[0-9]+$ ]]; then echo 'Cannot identify maze controller' >&2; exit 1; fi
  if [[ ! /proc/$pid/exe -ef "$binary" ]]; then
    systemctl --user is-active --quiet lab26-console.service
    check_chassis
    status=$(timeout 4 rostopic echo -n 1 /lab26/status)
    grep -Eq 'data: .*(DONE|FAULT|WAITING)' <<<"$status" || { echo 'Controller busy; update deferred' >&2; exit 1; }
    echo 'Loading updated maze controller at standstill'
    rosnode kill /labConsole
    deadline=$((SECONDS+45))
    until (( SECONDS >= deadline )); do
      pid=$(ss -ltnpH 'sport = :8766' | sed -n 's/.*pid=\([0-9]*\),.*/\1/p')
      [[ -n "$pid" && /proc/$pid/exe -ef "$binary" ]] && break
      sleep .25
    done
    [[ -n "$pid" && /proc/$pid/exe -ef "$binary" ]] || { echo 'New maze controller not online' >&2; exit 1; }
    echo "Updated maze controller online (PID $pid)"
  fi
fi
if $sensors_only; then echo 'SENSORS_VERIFIED'; else echo 'EXPERIMENT_2_PREPARED'; fi
