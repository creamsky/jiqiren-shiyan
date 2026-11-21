#!/usr/bin/env bash
# Read-only: no apt, network changes, compilation, or device access.
set -euo pipefail
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/build_common.sh"
issues=0
ros_ready=0
ok() { printf '[OK] %s\n' "$*"; }
missing() { printf '[MISSING] %s\n' "$*"; issues=$((issues + 1)); }
warn() { printf '[CHECK] %s\n' "$*"; }
printf 'Experiment root: %s\n' "$EXPERIMENT_ROOT"
if [[ "$(uname -s)" != Linux ]]; then
    missing "This is not Linux. Use the car's Ubuntu terminal through ToDesk."
    exit 1
fi
if [[ -r /etc/os-release ]]; then
    source /etc/os-release
    if [[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 20.04 ]]; then
        ok "${PRETTY_NAME:-Ubuntu 20.04} ($(uname -m))"
    else
        warn "Detected ${PRETTY_NAME:-unknown OS}; supported target: Ubuntu 20.04 + ROS 1 Noetic."
    fi
fi
if [[ -f "${ROS_SETUP:-/opt/ros/noetic/setup.bash}" ]]; then
    if (load_ros); then
        load_ros
        ros_ready=1
        ok "ROS $ROS_DISTRO ($ROS_SETUP)"
    else
        missing "Selected ROS setup could not load ROS 1 Noetic."
    fi
else
    missing "ROS setup: ${ROS_SETUP:-/opt/ros/noetic/setup.bash}. Install ROS Noetic first."
fi
for cmd in cmake make g++ catkin_make catkin python3 rospack roslaunch; do
    if command -v "$cmd" >/dev/null 2>&1; then ok "$cmd: $(command -v "$cmd")"; else missing "$cmd"; fi
done
if [[ $ros_ready -eq 1 ]] && command -v python3 >/dev/null 2>&1; then
    if python3 -c 'import rospy, rosbag, numpy, message_filters'; then
        ok "Python ROS modules: rospy, rosbag, numpy, message_filters"
    else
        missing "Python ROS modules. Run install_deps.sh and use Ubuntu's system Python 3."
    fi
fi
if command -v cmake >/dev/null 2>&1; then
    cmake_version="$(cmake --version)"
    cmake_version="${cmake_version%%$'\n'*}"
    if [[ "$cmake_version" =~ ([0-9]+)\.([0-9]+) ]] &&
       (( BASH_REMATCH[1] > 3 || (BASH_REMATCH[1] == 3 && BASH_REMATCH[2] >= 10) )); then
        ok "$cmake_version (requires >= 3.10)"
    else
        missing "$cmake_version; CMake >= 3.10 is required."
    fi
fi
if command -v dpkg-query >/dev/null 2>&1; then
    for package in libeigen3-dev libpcl-dev libpcap-dev libyaml-cpp-dev libtbb-dev libapr1-dev python3-dev ros-noetic-serial ros-noetic-pcl-ros ros-noetic-rviz; do
        if dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q '^install ok installed$'; then
            ok "$package"
        else
            missing "$package (install_deps.sh supplies it)"
        fi
    done
fi
sdk="$SLAM_WS/src/third_party/livox_ros_driver/livox_ros_driver/Livox-SDK"
for file in CMakeLists.txt sdk_core/CMakeLists.txt sdk_core/include/livox_sdk.h sdk_core/src/livox_sdk.cpp; do
    if [[ -s "$sdk/$file" ]]; then ok "Vendored Livox-SDK: $file"; else missing "Vendored SDK file: $sdk/$file (retransfer the complete archive)"; fi
done
teaser_config=''
if [[ -n "${teaserpp_DIR:-}" && -f "$teaserpp_DIR/teaserppConfig.cmake" ]]; then
    teaser_config="$teaserpp_DIR/teaserppConfig.cmake"
else
    IFS=: read -r -a prefixes <<< "${TEASERPP_PREFIX:-/usr/local}:/usr/local:/usr:${CMAKE_PREFIX_PATH:-}"
    for prefix in "${prefixes[@]}"; do
        [[ -n "$prefix" ]] || continue
        for file in "$prefix"/lib/cmake/teaserpp/teaserppConfig.cmake "$prefix"/share/teaserpp/cmake/teaserppConfig.cmake "$prefix"/lib/*/cmake/teaserpp/teaserppConfig.cmake; do
            if [[ -f "$file" ]]; then teaser_config="$file"; break 2; fi
        done
    done
fi
if [[ -n "$teaser_config" ]]; then
    ok "TEASER++ CMake config: $teaser_config"
else
    missing "TEASER++ config (required by Quatro). Install the documented upstream version, then set TEASERPP_PREFIX or teaserpp_DIR for a custom location."
fi
printf '\nNetwork interfaces (read only):\n'
if command -v ip >/dev/null 2>&1; then ip -brief address; else warn "ip command unavailable; inspect the car's Network Settings."; fi
printf '\nSerial devices (read only):\n'
shopt -s nullglob
serial_devices=(/dev/serial/by-id/* /dev/ttyUSB* /dev/ttyACM*)
if [[ ${#serial_devices[@]} -gt 0 ]]; then
    ls -l -- "${serial_devices[@]}"
else
    warn "No USB serial device found. Connect the IMU and check its actual serial port before running."
fi
if id -nG | tr ' ' '\n' | grep -qx dialout; then ok "Current user belongs to dialout"; else warn "Current user is not in dialout; the IMU port may need group permissions and a new login."; fi
default_map="$EXPERIMENT_ROOT/maps/map.bag"
if [[ -s "${MAP_PATH:-$default_map}" ]]; then
    ok "Map present: ${MAP_PATH:-$default_map}"
else
    warn "No map at ${MAP_PATH:-$default_map}. Normal before mapping; localization requires a completed keyframe .bag map."
fi
printf '\nMissing build prerequisites: %s\n' "$issues"
[[ $issues -eq 0 ]]
