#!/usr/bin/env bash
set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/build_common.sh"

mode=all
case "${1:-}" in
    '') ;;
    --slam-only) mode=slam; shift ;;
    --drivers-only) mode=drivers; shift ;;
    -h|--help)
        printf 'Usage: JOBS=2 [TEASERPP_PREFIX=/path] bash scripts/build_all.sh [--slam-only|--drivers-only]\n'
        exit 0 ;;
    *) fail "Unknown option: $1. Use --help." ;;
esac
[[ $# -eq 0 ]] || fail "Unexpected argument: $1"
load_ros
validate_jobs
for cmd in cmake make g++ catkin_make catkin; do
    command -v "$cmd" >/dev/null 2>&1 || fail "Missing command: $cmd. Run bash scripts/install_deps.sh first."
done

build_driver() (
    local workspace="$1"
    cd -- "$workspace"
    printf '\nBuilding %s (JOBS=%s)\n' "$workspace" "$JOBS"
    catkin_make -j"$JOBS" -l"$JOBS" -DCMAKE_BUILD_TYPE=Release
    [[ -f devel/setup.bash ]] || fail "Build returned without creating $workspace/devel/setup.bash."
    source_setup "$workspace/devel/setup.bash"
)

if [[ "$mode" != slam ]]; then
    build_driver "$LIDAR_WS"
    build_driver "$IMU_WS"
fi
if [[ "$mode" != drivers ]]; then
    (
        cd -- "$SLAM_WS"
        printf '\nBuilding SLAM workspace (JOBS=%s)\n' "$JOBS"
        # Extend ROS, not a stale sibling workspace from an earlier shell.
        catkin config --extend "$ROS_PREFIX"
        cmake_args=(-DCMAKE_BUILD_TYPE=Release -DQUATRO_TBB=ON)
        if [[ -n "${teaserpp_DIR:-}" ]]; then
            cmake_args+=("-Dteaserpp_DIR=$teaserpp_DIR")
        fi
        catkin build nano_gicp -j "$JOBS" -p 1 --no-status --cmake-args "${cmake_args[@]}"
        catkin build quatro -j "$JOBS" -p 1 --no-status --cmake-args "${cmake_args[@]}"
        catkin build -j "$JOBS" -p 1 --no-status --cmake-args "${cmake_args[@]}"
        [[ -f devel/setup.bash ]] || fail "Build returned without creating $SLAM_WS/devel/setup.bash."
        source_setup "$SLAM_WS/devel/setup.bash"
    )
fi
printf '\nBuild completed successfully. Use the run_*.sh scripts in separate runtime terminals.\n'
