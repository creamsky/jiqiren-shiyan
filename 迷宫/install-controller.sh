#!/usr/bin/env bash
# Install the independent maze build and connect it to the stopped prepare step.
set -euo pipefail
build=${1:?supply the verified maze build directory}
root=/home/nvidia/lab26
maze=/home/nvidia/maze
cd "$root"
mkdir -p "$maze/bin" logs
exec 7>logs/experiment-prepare.lock
flock -w 8 7 || { echo 'Preparation in progress; retry installation when it finishes'; exit 1; }
"$build/test_laser_maze"
lib=catkin_ws/devel/lib/basic_control
launch=catkin_ws/src/basic_control/launch/basicControl.launch
if ! grep -q 'arg name="controller_type"' "$launch"; then
  grep -Fq '<node pkg="basic_control" type="basicControl"' "$launch"
  grep -Fq 'roslaunch basic_control basicControl.launch node_name:=' start_console.sh
  grep -Fq 'binary="$LAB26_ROOT/catkin_ws/devel/lib/basic_control/basicControl"' select_experiment.sh
  backup=$(mktemp -d "$maze/before-install-XXXXXX")
  cp -p "$launch" start_console.sh select_experiment.sh "$backup/"
  sed '/<node pkg="basic_control" type="basicControl"/i\  <arg name="controller_type" default="basicControl"/>' "$launch" |
    sed 's/type="basicControl" name=/type="$(arg controller_type)" name=/' >"$backup/launch.new"
  sed '/^roslaunch basic_control basicControl.launch /i\controller=basicControl\n[[ ! -f /home/nvidia/maze/use-maze ]] || controller=maze_controller' start_console.sh |
    sed 's/basicControl.launch node_name:=/basicControl.launch controller_type:="$controller" node_name:=/' >"$backup/start.new"
  awk '
    /^if \[\[ \$\{2:-\} == --roam \]\]; then$/ { print "if ! $sensors_only; then"; next }
    /^  binary=.*basicControl"$/ {
      print "  controller=basicControl"
      print "  if [[ ${2:-} == --roam ]]; then controller=maze_controller; fi"
      print "  binary=\"$LAB26_ROOT/catkin_ws/devel/lib/basic_control/$controller\""; next
    }
    /    rosnode kill \/labConsole/ {
      print "    if [[ $controller == maze_controller ]]; then touch /home/nvidia/maze/use-maze; else rm -f /home/nvidia/maze/use-maze; fi"
    }
    {print}
  ' select_experiment.sh >"$backup/select.new"
  grep -Fq 'controller=maze_controller' "$backup/select.new"
  bash -n "$backup/start.new"
  bash -n "$backup/select.new"
fi
install -m755 "$build/maze_controller" "$maze/bin/maze_controller.new"
mv "$maze/bin/maze_controller.new" "$maze/bin/maze_controller"
ln -sfn "$maze/bin/maze_controller" "$lib/maze_controller"
if [[ -n ${backup:-} ]]; then
  install -m644 "$backup/launch.new" "$launch.new"
  mv "$launch.new" "$launch"
  for pair in 'start_console.sh start.new' 'select_experiment.sh select.new'; do
    read -r target source <<< "$pair"
    install -m755 "$backup/$source" "$target.new"
    mv "$target.new" "$target"
  done
fi
cmp "$build/maze_controller" "$lib/maze_controller"
echo 'Maze controller installed; prepare maze to activate at standstill.'
