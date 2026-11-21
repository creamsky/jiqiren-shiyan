#!/usr/bin/python3
"""Run the native Gazebo client on a project-only X11 display and loopback VNC."""
import json
import fcntl
import os
from pathlib import Path
import socket
import subprocess
import time

ROOT = Path('/home/sim/scout-lab')
STATE = ROOT / 'native'
DISPLAY = ':110'


def process_is_ours(pid, executable):
    try:
        process = Path('/proc') / str(pid)
        command = (process / 'cmdline').read_bytes().replace(b'\0', b' ').decode()
        environment = (process / 'environ').read_bytes().split(b'\0')
        return (process.stat().st_uid == os.getuid() and executable in command
                and b'SCOUT_NATIVE_VIEWER=1' in environment)
    except (OSError, ValueError):
        return False


def start(role, args, environment, existing):
    pid = existing.get(role)
    if pid and process_is_ours(pid, args[0]):
        return pid
    with (STATE / (role + '.log')).open('ab') as log:
        child = subprocess.Popen(args, env=environment, stdin=subprocess.DEVNULL,
                                 stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return child.pid


def recover_stale_display(existing):
    sock=Path('/tmp/.X11-unix/X110')
    if not sock.exists():return
    pid=existing.get('display')
    if pid and process_is_ours(pid,'/usr/bin/Xvfb'):return
    try:
        with socket.socket(socket.AF_UNIX) as probe:
            probe.settimeout(.5);probe.connect(str(sock))
    except OSError:
        lock=Path('/tmp/.X110-lock')
        try: recorded=int(lock.read_text().strip())
        except (OSError,ValueError):raise SystemExit('Display :110 ownership could not be verified')
        if pid!=recorded or sock.stat().st_uid!=os.getuid() or lock.stat().st_uid!=os.getuid():
            raise SystemExit('Stale display :110 does not match the project manifest')
        sock.unlink();lock.unlink()
        return
    raise SystemExit('Display :110 belongs to another live session')


def main():
    STATE.mkdir(exist_ok=True)
    global viewer_lock
    viewer_lock=(STATE/'viewer.lock').open('a')
    fcntl.flock(viewer_lock,fcntl.LOCK_EX)
    manifest = STATE / 'processes.json'
    existing = json.loads(manifest.read_text()) if manifest.exists() else {}
    environment = dict(os.environ, DISPLAY=DISPLAY, LIBGL_ALWAYS_SOFTWARE='1',
                       LP_NUM_THREADS='2', OMP_NUM_THREADS='2',
                       QT_X11_NO_MITSHM='1', SCOUT_NATIVE_VIEWER='1',
                       ROS_MASTER_URI='http://127.0.0.1:11431',
                       GAZEBO_MASTER_URI='http://127.0.0.1:11445',
                       GAZEBO_MODEL_DATABASE_URI='',
                       ROS_PACKAGE_PATH=str(ROOT / 'sim/ros') + ':' + str(ROOT / 'dwa') + ':/opt/ros/noetic/share')
    for _ in range(90):
        try:
            with socket.create_connection(('127.0.0.1', 11445), timeout=1):
                break
        except OSError:
            time.sleep(1)
    else:
        raise SystemExit('Gazebo master did not become ready')
    recover_stale_display(existing)
    existing['display'] = start('display', ['/usr/bin/Xvfb', DISPLAY, '-screen', '0', '1280x800x24', '-ac', '-nolisten', 'tcp'], environment, existing)
    manifest.write_text(json.dumps(existing))
    for _ in range(40):
        if Path('/tmp/.X11-unix/X110').exists():
            break
        time.sleep(.1)
    existing['window_manager'] = start('window_manager', ['/usr/bin/fluxbox', '-display', DISPLAY], environment, existing)
    existing['gazebo'] = start('gazebo', ['/usr/bin/gzclient', '--verbose'], environment, existing)
    existing['vnc'] = start('vnc', ['/usr/bin/x11vnc', '-display', DISPLAY, '-rfbport', '5901', '-localhost', '-nopw', '-forever', '-shared', '-noxdamage', '-repeat'], environment, existing)
    existing['web'] = start('web', ['/usr/bin/websockify', '--web=/usr/share/novnc', '--heartbeat=30', '127.0.0.1:6080', '127.0.0.1:5901'], environment, existing)
    manifest.write_text(json.dumps(existing))
    for _ in range(100):
        windows=subprocess.run(['/usr/bin/wmctrl', '-l'],env=environment,text=True,capture_output=True)
        window_ready='Gazebo' in windows.stdout
        if window_ready:
            subprocess.run(['/usr/bin/wmctrl', '-r', 'Gazebo', '-b', 'add,maximized_vert,maximized_horz'], env=environment, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            with socket.create_connection(('127.0.0.1', 6080), timeout=.5):
                if window_ready:break
        except OSError:
            pass
        time.sleep(.2)
    for role,executable in [('gazebo','/usr/bin/gzclient'),('vnc','/usr/bin/x11vnc'),('web','/usr/bin/websockify')]:
        if not process_is_ours(existing[role],executable):raise SystemExit('Native viewer process failed: '+role)
    print(json.dumps({'viewer': 'native-gazebo', 'display': DISPLAY,
                      'http': 'http://127.0.0.1:6080', 'websocket': '/websockify',
                      'client': '/vnc_lite.html', 'rfb_module': '/core/rfb.js', 'pids': existing}))


if __name__ == '__main__':
    main()
