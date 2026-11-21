#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: bash scripts/run_lidar.sh'; exit 0
fi
[[ $# == 0 ]] || { echo 'No arguments expected.' >&2; exit 2; }
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/common.sh"
load_workspace "$EXPERIMENT_ROOT/RS_SDK"
exec roslaunch rslidar_sdk start.launch
