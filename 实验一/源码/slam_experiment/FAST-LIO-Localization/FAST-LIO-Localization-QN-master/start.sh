#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
case "${1:-mapping}" in
  mapping)
    [[ $# == 0 ]] || shift
    exec bash "$ROOT/scripts/run_mapping.sh" "$@" ;;
  localization)
    shift
    exec bash "$ROOT/scripts/run_localization.sh" "$@" ;;
  --help|-h)
    echo 'Usage: bash start.sh [mapping [output.bag] | localization existing.bag]' ;;
  *) echo 'Unknown mode. Use --help.' >&2; exit 2 ;;
esac
