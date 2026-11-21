#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
experiment_root="$(cd -- "$workspace_dir/../.." && pwd -P)"
exec bash "$experiment_root/scripts/build_all.sh" --slam-only "$@"
