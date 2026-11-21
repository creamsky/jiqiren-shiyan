#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: bash scripts/run_mapping.sh [output_map.bag]'
  echo 'Default: maps/map.bag. Existing maps are never overwritten. Set RVIZ=false for no GUI.'
  exit 0
fi
[[ $# -le 1 ]] || { echo 'Expected at most one output bag path.' >&2; exit 2; }
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/common.sh"
map_bag="${1:-$EXPERIMENT_ROOT/maps/map.bag}"
[[ "$map_bag" == /* ]] || map_bag="$PWD/$map_bag"
[[ "$map_bag" == *.bag ]] || fail 'Output map must end in .bag.'
[[ ! -e "$map_bag" && ! -e "$map_bag.active" && ! -e "$map_bag.lock" ]] || fail "Output exists; choose a new map name: $map_bag"
map_pcd="${map_bag%.bag}.pcd"
[[ ! -e "$map_pcd" ]] || fail "PCD output already exists; choose another map name: $map_pcd"
validate_rviz
load_workspace "$SLAM_WORKSPACE"
mkdir -p -- "$(dirname -- "$map_bag")"
printf 'PCD output: %s\nMap bag output: %s\n' "$map_pcd" "$map_bag"
exec roslaunch fast_lio_localization_qn mapping_record.launch "map_bag:=$map_bag" "map_pcd:=$map_pcd" "rviz:=$RVIZ"
