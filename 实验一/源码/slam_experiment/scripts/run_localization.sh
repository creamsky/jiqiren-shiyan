#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: bash scripts/run_localization.sh existing_map.bag'
  echo 'Use a completed bag from run_mapping.sh. Set RVIZ=false for no GUI.'; exit 0
fi
[[ $# == 1 ]] || { echo 'Provide a completed map.bag path.' >&2; exit 2; }
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/common.sh"
map_bag="$1"
[[ "$map_bag" == /* ]] || map_bag="$PWD/$map_bag"
[[ "$map_bag" == *.bag && -s "$map_bag" && -r "$map_bag" ]] || fail "Map is missing, empty or unreadable: $map_bag"
validate_rviz
load_workspace "$SLAM_WORKSPACE"
exec roslaunch fast_lio_localization_qn run.launch lidar:=velodyne "map_bag:=$map_bag" "rviz:=$RVIZ"
