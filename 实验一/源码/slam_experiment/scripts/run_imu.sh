#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: bash scripts/run_imu.sh [serial_port] [baud]'; exit 0
fi
[[ $# -le 2 ]] || { echo 'Expected at most serial_port and baud.' >&2; exit 2; }
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/common.sh"
port="${1:-/dev/ttyUSB0}"
baud="${2:-921600}"
[[ "$port" == /dev/* && "$baud" =~ ^[1-9][0-9]*$ ]] || fail 'Invalid serial path or baud.'
[[ -c "$port" ]] || fail "Serial port not found: $port"
[[ -r "$port" && -w "$port" ]] || fail "Serial permission missing. See imu_drive/net.sh --help."
load_workspace "$EXPERIMENT_ROOT/FAST-LIO-Localization/imu_drive"
exec roslaunch fdilink_ahrs ahrs_data.launch "port:=$port" "baud:=$baud"
