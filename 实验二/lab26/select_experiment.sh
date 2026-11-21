#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
[[ "${1:-}" == lab2 ]] || { echo 'Unsupported experiment'; exit 2; }
source ./env.sh
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
stop_sensors
# Wait for the identified USB devices, not a stale ROS topic or tty number.
device_deadline=$((SECONDS+30))
until [[ -c /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 && -e /sys/class/net/can0 ]]; do
  if ((SECONDS>=device_deadline)); then echo 'IMU USB serial or USB-CAN missing; check device connection' >&2; exit 1; fi
  sleep 1
done
sleep 2
bash ./start_sensors.sh
if ! wait_topic /imu 10; then
  echo 'IMU is silent; restarting sensors once.'
  stop_sensors
  sleep 2
  bash ./start_sensors.sh
  wait_topic /imu 15
fi
echo 'Waiting for live lidar and odometry...'
wait_topic /velodyne_points 45
wait_topic /Odometry 45
bash ./keep_screen_awake.sh
bash ./display_console.sh --refresh
echo 'EXPERIMENT_2_PREPARED'
