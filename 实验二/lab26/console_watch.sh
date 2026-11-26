#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")"
# Adopt an existing listener in place. Checking the kernel listener table never
# consumes the controller's single client slot.
while true; do
  if ! ss -ltnH 'sport = :8766' | grep -q .; then
    bash ./launch_console_service.sh
  fi
  sleep 2
done
