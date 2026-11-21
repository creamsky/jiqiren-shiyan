#!/usr/bin/env bash
set -eo pipefail
root=/home/nvidia/lab26-console-stage/avoidance_v2
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11314 ROS_IP=127.0.0.1
unset ROS_HOSTNAME
if ss -ltn | grep -Eq ':11314 |:18768 '; then echo 'Isolated test ports already occupied'; exit 1; fi
mkdir -p "$root/results"
catkin_make -C "$root/lab26/catkin_ws" -j2 -DCMAKE_BUILD_TYPE=Release -DLAB3_ROOT="$root/lab3" -DFIGURE8_INCLUDE_DIR=/home/nvidia/lab2-extension/include
"$root/lab26/catkin_ws/devel/lib/basic_control/test_motion" | tee "$root/results/motion.txt"
g++ -std=c++14 -O2 -DNAVIGATION_V2 -I"$root/lab26/catkin_ws/src/basic_control/include" -I"$root/lab3/include" -I/home/nvidia/lab2-extension/include "$root/lab3/tests/navigation_scenarios.cpp" "$root/lab3/src/navigation.cpp" -o "$root/results/scenarios"
"$root/results/scenarios" "$root/results/scenarios.csv" | tee "$root/results/scenarios.txt"
roscore -p 11314 >"$root/results/master.log" 2>&1 & master=$!
controller=''
cleanup(){ if [[ -n "$controller" ]]; then kill "$controller" 2>/dev/null || true; wait "$controller" 2>/dev/null || true; fi; kill "$master" 2>/dev/null || true; wait "$master" 2>/dev/null || true; }
trap cleanup EXIT
for n in $(seq 1 30); do if rosparam list >/dev/null 2>&1; then break; fi; sleep .2; done
"$root/lab26/catkin_ws/devel/lib/basic_control/basicControl" __name:=lab3Test _dry_run:=true _console:=true _console_port:=18768 _csv:="$root/results/ros_commands.csv" >"$root/results/controller.log" 2>&1 & controller=$!
for n in $(seq 1 30); do if ss -ltn | grep -q ':18768 '; then break; fi; sleep .2; done
"$root/lab26/catkin_ws/devel/lib/basic_control/test_navigation_ros" | tee "$root/results/ros_integration.txt"
echo VALIDATION_PASSED
(cd "$root" && find lab3/include lab3/src lab26/catkin_ws/src/basic_control -type f -print0 | sort -z | xargs -0 sha256sum; sha256sum "$root/lab26/catkin_ws/devel/lib/basic_control/basicControl" "$root/lab26/catkin_ws/devel/lib/basic_control/test_motion") >"$root/results/validated.sha256"
