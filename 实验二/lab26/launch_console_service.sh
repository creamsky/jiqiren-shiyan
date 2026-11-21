#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
exec 9>.console-service.lock
flock -x 9
if ss -ltn | grep -q ':8766 '; then exit 0; fi
mkdir -p logs
setsid bash ./start_console.sh 9>&- >logs/console_service.log 2>&1 </dev/null &
pid=$!; echo "$pid" >logs/console_service.pid
for ((i=0;i<55;i++)); do
  if ss -ltn | grep -q ':8766 '; then exit 0; fi
  if ! kill -0 "$pid" 2>/dev/null; then tail -n 20 logs/console_service.log; exit 1; fi
  sleep 1
done
echo 'Console did not become ready; inspect logs/console_service.log' >&2
exit 1
