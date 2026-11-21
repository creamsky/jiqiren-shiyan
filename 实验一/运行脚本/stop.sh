#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source ./env.sh
exec python3 "$LAB1_ROOT/run_experiment.py" --stop
