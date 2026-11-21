#!/usr/bin/env bash
# Sourced by run scripts. Call from any directory.
set -euo pipefail
EXPERIMENT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SLAM_WORKSPACE="$EXPERIMENT_ROOT/FAST-LIO-Localization/FAST-LIO-Localization-QN-master"
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
load_workspace() {
  local workspace="$1"
  [[ "$(uname -s)" == Linux ]] || fail 'Run this command in the robot Ubuntu terminal.'
  [[ "${ROS_VERSION:-1}" == 1 ]] || fail 'Open a fresh terminal; this one contains ROS 2.'
  local ros_setup="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
  [[ -r "$ros_setup" ]] || fail "ROS Noetic setup is missing: $ros_setup"
  [[ -r "$workspace/devel/setup.bash" ]] || fail "Workspace not built. Run: bash $EXPERIMENT_ROOT/scripts/build_all.sh"
  set +u
  source "$ros_setup"
  source "$workspace/devel/setup.bash"
  set -u
  [[ "${ROS_VERSION:-}" == 1 && "${ROS_DISTRO:-}" == noetic ]] || fail 'Expected ROS 1 Noetic.'
}
validate_rviz() {
  RVIZ="${RVIZ:-true}"
  [[ "$RVIZ" == true || "$RVIZ" == false ]] || fail 'RVIZ must be true or false.'
}
