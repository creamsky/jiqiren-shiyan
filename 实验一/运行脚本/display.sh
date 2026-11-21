#!/usr/bin/env bash
source "$(dirname "$0")/env.sh"
export DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus XDG_RUNTIME_DIR=/run/user/1000
# Display the experiment without changing lock-screen or power settings.
exec rviz -d "$LAB1_ROOT/config/${1:-mapping}.rviz"
