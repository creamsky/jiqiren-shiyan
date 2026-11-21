#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source ./env.sh
export DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus XDG_RUNTIME_DIR=/run/user/1000
mkdir -p logs
if [[ ! -f logs/restore_display.sh ]]; then
  {
    echo '#!/usr/bin/env bash'
    echo 'export DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus'
    for key in 'org.gnome.desktop.session idle-delay' 'org.gnome.desktop.screensaver lock-enabled' 'org.gnome.settings-daemon.plugins.power idle-dim'; do
      read -r schema property <<< "$key"
      printf 'gsettings set %s %s %q\n' "$schema" "$property" "$(gsettings get "$schema" "$property")"
    done
    echo 'xset +dpms'
  } >logs/restore_display.sh
fi
gsettings set org.gnome.desktop.session idle-delay 0
gsettings set org.gnome.desktop.screensaver lock-enabled false
gsettings set org.gnome.settings-daemon.plugins.power idle-dim false
xset s off
xset dpms force on
xset -dpms
loginctl unlock-session 2 2>/dev/null || true
if [[ ! -f logs/feedback.pid ]] || ! kill -0 "$(cat logs/feedback.pid)" 2>/dev/null; then
  setsid python3 "$LAB26_ROOT/monitor_feedback.py" >logs/feedback.log 2>&1 < /dev/null &
  echo $! >logs/feedback.pid
fi
if [[ -f logs/rviz.pid ]] && kill -0 "$(cat logs/rviz.pid)" 2>/dev/null; then
  echo 'RViz is already running'; exit 0
fi
setsid rviz -d "$LAB26_ROOT/launch/lab26.rviz" >logs/rviz.log 2>&1 < /dev/null &
echo $! >logs/rviz.pid
echo "RViz PID $!; screen kept awake"
