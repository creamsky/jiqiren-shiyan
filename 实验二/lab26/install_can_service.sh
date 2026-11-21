#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
install -o root -g root -m 0755 lab26-can-up /usr/local/sbin/lab26-can-up
install -o root -g root -m 0644 lab26-can.service /etc/systemd/system/lab26-can.service
install -o root -g root -m 0644 90-lab26-can.rules /etc/udev/rules.d/90-lab26-can.rules
systemctl daemon-reload
udevadm control --reload-rules
systemctl start lab26-can.service
systemctl is-active lab26-can.service
