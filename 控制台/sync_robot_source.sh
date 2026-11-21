#!/usr/bin/env bash
set -euo pipefail
repo=/home/nvidia/robot-experiments
archive=/home/nvidia/lab26-console-stage/robot_source.tar.gz
if [[ -d "$repo/.git" ]] && [[ -n "$(git -C "$repo" status --porcelain)" ]]; then
  echo 'Robot source repository has uncommitted changes; sync aborted.' >&2
  exit 1
fi
mkdir -p "$repo"
tar -xzf "$archive" -C "$repo"
cp "$repo/控制台/robot-source.gitignore" "$repo/.gitignore"
cp "$repo/控制台/机器人端源码仓说明.md" "$repo/README.md"
if [[ ! -d "$repo/.git" ]]; then
  git -C "$repo" init
  git -C "$repo" symbolic-ref HEAD refs/heads/main
fi
git -C "$repo" add --all
if ! git -C "$repo" diff --cached --quiet; then
  git -C "$repo" -c user.name='Robot Lab Automation' -c user.email='robot-lab@localhost' commit -m 'Sync verified experiment sources and WASD acknowledgement fix'
fi
git -C "$repo" log -1 --format='SOURCE_COMMIT=%H'
git -C "$repo" status --short
