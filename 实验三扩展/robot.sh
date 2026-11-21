#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
action=${1:-help}
if (( $# )); then shift; fi
navigation_pid() {
  local pid
  [[ -f .navigation.pid ]] || return 1
  pid=$(cat .navigation.pid)
  [[ "$pid" =~ ^[0-9]+$ && -r /proc/$pid/cmdline ]] || return 1
  [[ "$(readlink -f /proc/$pid/cwd)" == "$PWD" ]] || return 1
  tr '\0' ' ' < /proc/$pid/cmdline | grep -Fq './goto_xy.py' || return 1
  echo "$pid"
}
case "$action" in
  pause|resume|stop)
    pid=$(navigation_pid) || { echo '没有通过本入口启动的导航进程'; exit 1; }
    case "$action" in pause) kill -USR1 "$pid";; resume) kill -USR2 "$pid";; stop) kill -INT "$pid";; esac
    echo "已请求 $action；请查看导航终端反馈。"
    ;;
  shutdown)
    if pid=$(navigation_pid); then
      kill -INT "$pid"
      for n in $(seq 1 50); do if ! navigation_pid >/dev/null; then break; fi; sleep .1; done
      if navigation_pid >/dev/null; then echo '导航尚未退出，暂不关闭传感器，请现场停车后重试'; exit 1; fi
    fi
    source /home/nvidia/lab26/env.sh
    if [[ -f /home/nvidia/lab3-dwa/stop.sh ]]; then bash /home/nvidia/lab3-dwa/stop.sh;fi
    if timeout 3 rosnode list 2>/dev/null | grep -qx /labConsole; then
      rosnode kill /labConsole
    fi
    # A dead ROS master cannot deliver rosnode kill to an orphan controller.
    for control_pid in $(pgrep -x basicControl || true); do
      command_line=$(tr '\0' ' ' <"/proc/$control_pid/cmdline" 2>/dev/null || true)
      if [[ "$command_line" == "/home/nvidia/lab26/catkin_ws/devel/lib/basic_control/basicControl "* && "$command_line" == *"__name:=labConsole "* ]]; then
        kill -INT "$control_pid"
      fi
    done
    for n in $(seq 1 30); do
      if ! ss -ltn | grep -q ':8766 '; then break; fi
      sleep .2
    done
    if ss -ltn | grep -q ':8766 '; then echo '控制服务未退出，停止关闭流程'; exit 1; fi
    bash /home/nvidia/lab26/stop.sh
    sensor_pid_file=/home/nvidia/lab26/logs/sensors.pid
    if [[ -f "$sensor_pid_file" ]]; then
      sensor_pid=$(cat "$sensor_pid_file")
      if [[ "$sensor_pid" =~ ^[0-9]+$ ]]; then
        for n in $(seq 1 40); do
          if ! ps -p "$sensor_pid" -o args= | grep -Fq '/home/nvidia/lab26/launch/sensors.launch'; then break; fi
          sleep 1
        done
        if ps -p "$sensor_pid" -o args= | grep -Fq '/home/nvidia/lab26/launch/sensors.launch'; then
          echo '传感器尚未完全退出，请勿立即 prepare'; exit 1
        fi
      fi
    fi
    rviz_file=/home/nvidia/lab26/logs/rviz.pid
    if [[ -f "$rviz_file" ]]; then
      rviz_pid=$(cat "$rviz_file")
      if [[ "$rviz_pid" =~ ^[0-9]+$ ]] && ps -p "$rviz_pid" -o args= | grep -Fq '/home/nvidia/lab26/launch/lab26.rviz'; then
        kill -TERM "$rviz_pid"
      fi
    fi
    echo '控制服务已关闭，已向实验传感器及 RViz 发出退出信号；请确认车体静止。机器人操作系统保持开机。'
    ;;
  prepare)
    if navigation_pid >/dev/null; then echo '请先 stop 当前导航，再准备实验'; exit 1; fi
    echo '重新准备雷达、IMU 和 SLAM；定位原点可能重置，请重新选目标。'
    # Existing sensor preparation includes an IMU retry and waits for actual
    # messages, rather than merely accepting a listed but silent ROS topic.
    bash /home/nvidia/lab26/select_experiment.sh lab2
    bash /home/nvidia/lab26/launch_console_service.sh
    source /home/nvidia/lab26/env.sh
    timeout 20 rostopic echo -n 1 /velodyne_points/header
    timeout 20 rostopic echo -n 1 /Odometry/header
    bash /home/nvidia/lab26/display_console.sh
    echo '准备完成；使用 bash robot.sh check X Y 检查目标，再用 bash robot.sh go X Y 启动。'
    ;;
  check|go)
    if [[ $# != 2 ]]; then echo "用法: bash robot.sh $action X Y"; exit 2; fi
    # Explicit executable; prefer installed conda environments when available.
    interpreter=${ROBOT_PYTHON:-}
    if [[ -z "$interpreter" ]]; then
      for candidate in /home/nvidia/anaconda3/bin/python /home/nvidia/miniconda3/bin/python; do
        if [[ -x "$candidate" ]]; then interpreter=$candidate; break; fi
      done
    fi
    if [[ -z "$interpreter" || ! -x "$interpreter" ]]; then
      echo '请设置 ROBOT_PYTHON 为机器人上可用 Python 3 解释器的绝对路径，见 README。'
      exit 2
    fi
    exec 8>.navigation.lock
    flock -n 8 || { echo '已有命令行导航正在运行；在原终端按 Ctrl+C 停止后再试。'; exit 1; }
    args=(--x "$1" --y "$2" --port 8766)
    if [[ "$action" == go ]]; then args+=(--run); fi
    echo $$ > .navigation.pid
    exec "$interpreter" -u ./goto_xy.py "${args[@]}"
    ;;
  logs)
    latest=$(find "$PWD/logs" -maxdepth 1 -name 'navigation_*.log' -type f 2>/dev/null | sort | tail -n 1)
    [[ -n "$latest" ]] || { echo '暂无导航日志'; exit 1; }
    exec tail -n 30 -f "$latest"
    ;;
  *)
    echo '命令: bash robot.sh prepare | check X Y | go X Y | pause | resume | stop | shutdown | logs'
    echo '导航时 Ctrl+C 停车；修改目标须先停车，再运行 go 新X 新Y。'
    ;;
esac
