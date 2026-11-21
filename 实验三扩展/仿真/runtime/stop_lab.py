#!/usr/bin/python3
"""Stop only this project's isolated ROS/Gazebo processes, including orphans."""
import argparse
import json
import os
from pathlib import Path
import signal
import time

PROJECT = Path('/home/sim/scout-lab')
ROS_MASTER = 'http://127.0.0.1:11431'
GAZEBO_MASTER = 'http://127.0.0.1:11445'


def processes():
    selected = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            command = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode()
            if not command:
                continue
            environment = dict(part.split(b'=', 1) for part in
                               (entry / 'environ').read_bytes().split(b'\0')
                               if b'=' in part)
            ros = environment.get(b'ROS_MASTER_URI', b'').decode().rstrip('/')
            gazebo = environment.get(b'GAZEBO_MASTER_URI', b'').decode().rstrip('/')
            paths = environment.get(b'ROS_PACKAGE_PATH', b'').decode().split(':')
            if ros == ROS_MASTER and gazebo == GAZEBO_MASTER and str(PROJECT / 'sim/ros') in paths:
                selected[int(entry.name)] = command
        except (OSError, UnicodeError, ValueError):
            continue
    return selected


def send(pids, signum):
    # Recheck the process environment immediately before each signal, so a stale
    # PID cannot target another session that reused the same process number.
    for pid in pids:
        if pid in processes():
            try:
                os.kill(pid, signum)
            except ProcessLookupError:
                pass


def wait(seconds):
    deadline = time.monotonic() + seconds
    while processes() and time.monotonic() < deadline:
        time.sleep(.25)
    return processes()


def main(check):
    current = processes()
    if check:
        print(json.dumps(current))
        return
    launchers = [pid for pid, command in current.items()
                 if 'roslaunch' in command and 'scout_lab field.launch' in command]
    send(launchers or list(current), signal.SIGINT)
    remaining = wait(12)
    if remaining:
        send(remaining, signal.SIGTERM)
        remaining = wait(4)
    if remaining:
        send(remaining, signal.SIGKILL)
        remaining = wait(3)
    if remaining:
        raise SystemExit('Isolated simulation processes are still exiting: ' + json.dumps(remaining))
    (PROJECT / 'gazebo.pid').unlink(missing_ok=True)
    print('GAZEBO_STOPPED')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true')
    main(parser.parse_args().check)
