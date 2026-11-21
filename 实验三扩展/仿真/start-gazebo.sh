#!/usr/bin/env bash
set -eo pipefail
if [[ ! -r /opt/ros/noetic/setup.bash ]]; then
  echo 'ROS Noetic is not installed in this Linux environment.' >&2
  exit 1
fi
source /opt/ros/noetic/setup.bash
simdir=$(cd "$(dirname "$0")" && pwd)
if [[ -f "$simdir/../实验二/lab26/catkin_ws/src/basic_control/launch/basicControl.launch" ]]; then
  /usr/bin/python3 "$simdir/ros/scout_lab/scripts/source_limits.py" --root "$simdir/.." > "$simdir/ros/scout_lab/config/motion_limits.json.tmp"
  mv "$simdir/ros/scout_lab/config/motion_limits.json.tmp" "$simdir/ros/scout_lab/config/motion_limits.json"
fi
if [[ ! -r "$simdir/ros/scout_lab/config/motion_limits.json" ]]; then
  echo 'Missing source-derived motion_limits.json. Launch from the project runtime or export the real controller limits first.' >&2
  exit 1
fi
export ROS_MASTER_URI=http://127.0.0.1:11431 ROS_IP=127.0.0.1 GAZEBO_MASTER_URI=http://127.0.0.1:11445
export ROS_PACKAGE_PATH="$simdir/ros:$simdir/../dwa:$ROS_PACKAGE_PATH"
for package in gazebo_ros gazebo_plugins velodyne_description velodyne_gazebo_plugins move_base dwa_local_planner navfn rosbridge_server robot_state_publisher scout_lab lab3_dwa; do
  rospack find "$package" >/dev/null || { echo "Missing ROS package: $package" >&2; exit 1; }
done
chmod +x "$simdir/ros/scout_lab/scripts/lab.py"
if [[ "${1:-}" == --check ]]; then
  /usr/bin/python3 "$simdir/ros/scout_lab/scripts/robot_description.py" > /dev/null
  echo 'SCOUT_MODEL_READY'
  exit 0
fi
export GAZEBO_MODEL_DATABASE_URI=
if [[ -z "${DISPLAY:-}" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  exec xvfb-run -a roslaunch -p 11431 scout_lab field.launch gui:=false "$@"
fi
exec roslaunch -p 11431 scout_lab field.launch "$@"
