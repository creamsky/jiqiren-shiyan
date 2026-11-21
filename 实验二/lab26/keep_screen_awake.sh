#!/usr/bin/env bash
# User-requested screen-on setting. Never disable screen locking or unlock sessions.
export DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus XDG_RUNTIME_DIR=/run/user/1000
gsettings set org.gnome.desktop.session idle-delay 0
xset s off
xset -dpms
echo 'Screen idle blanking disabled; lock-enabled setting unchanged.'
