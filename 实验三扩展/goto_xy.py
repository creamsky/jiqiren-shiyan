
import argparse
import json
import logging
import math
import os
from pathlib import Path
import re
import socket
import signal
import subprocess
import time
from datetime import datetime, timezone, timedelta


GOAL_X = 0.0
GOAL_Y = 0.0
LOG = logging.getLogger('navigation')
pending_control = None


def request_control(signum, _frame):
    global pending_control
    pending_control = 'pause' if signum == signal.SIGUSR1 else 'resume'


def setup_log():
    folder = Path(__file__).resolve().parent / 'logs'
    folder.mkdir(exist_ok=True)
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    target = folder / f'navigation_{stamp}.log'
    formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s', '%H:%M:%S')
    formatter.converter = lambda t: time.gmtime(t + 8 * 3600)
    LOG.setLevel(logging.INFO)
    for handler in (logging.StreamHandler(), logging.FileHandler(target, encoding='utf-8')):
        handler.setFormatter(formatter)
        LOG.addHandler(handler)
    LOG.info('日志文件：%s', target)


def prepare_controller():
    if os.name != 'nt':
        LOG.info('检查本机 ROS 控制服务（不会启动运动）')
        result = subprocess.run(['/bin/bash', '/home/nvidia/lab26/launch_console_service.sh'],
                                capture_output=True, text=True, timeout=75)
        if result.returncode:
            raise RuntimeError('控制服务启动失败：' + result.stderr + result.stdout)
        LOG.info('本机 ROS 控制服务已就绪')
        return
    # The controller exits after a disconnected interval. Restart it before
    # connecting; never restart/replay a navigation goal after connection loss.
    config = Path(__file__).resolve().parent / '控制台' / 'robot-address.json'
    ip = json.loads(config.read_text(encoding='utf-8-sig'))['ip']
    import ipaddress
    ipaddress.IPv4Address(ip)
    ssh = Path(os.environ.get('WINDIR', 'C:/Windows')) / 'System32/OpenSSH/ssh.exe'
    key = Path.home() / '.ssh/robot_lab_ed25519'
    LOG.info('检查机器人 %s 控制服务（不会启动运动）', ip)
    result = subprocess.run(
        [str(ssh), '-n', '-i', str(key), '-o', 'BatchMode=yes',
         '-o', 'StrictHostKeyChecking=yes', '-o', 'HostKeyAlias=10.226.235.72',
         '-o', 'ConnectTimeout=5', f'nvidia@{ip}',
         'bash /home/nvidia/lab26/launch_console_service.sh'],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        timeout=75, creationflags=subprocess.CREATE_NO_WINDOW)
    if result.returncode:
        raise RuntimeError('控制服务启动失败：' + result.stderr.strip() + result.stdout.strip())
    LOG.info('机器人控制服务已就绪')


def navigate(x, y, port, run):
    if not all(math.isfinite(v) for v in (x, y)):
        raise ValueError('终点必须是有限数值')
    LOG.info('模式=%s | 终点=(%.3f, %.3f) m | 连接端口=%d', '导航' if run else '只检查', x, y, port)
    with socket.create_connection(('127.0.0.1', port), timeout=3) as sock:
        LOG.info('已连接控制端口，等待机器人反馈')
        sock.settimeout(0.1)
        buffer = b''
        pose = None
        pose_at = 0.0
        started = active = False
        begin = time.monotonic()
        last_received = begin
        last_print = 0.0

        def send(command):
            sock.sendall((command + '\n').encode('ascii'))

        try:
            if run:
                send('stop')
            while time.monotonic() - begin < 180:
                global pending_control
                now = time.monotonic()
                send('heartbeat')
                if pending_control is not None:
                    command, pending_control = pending_control, None
                    if started:
                        send(command)
                        LOG.info('已发送%s指令', '暂停' if command == 'pause' else '继续')
                    else:
                        raise RuntimeError('尚未开始导航，取消启动')
                try:
                    data = sock.recv(65536)
                except socket.timeout:
                    if now - last_received > 3:
                        raise RuntimeError('未收到反馈：请关闭占用连接的控制台服务，检查 SSH 隧道')
                    continue
                if not data:
                    raise RuntimeError('机器人连接断开')
                last_received = now
                buffer += data
                if len(buffer) > 2_000_000:
                    raise RuntimeError('反馈数据异常')
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    msg = json.loads(line)
                    if msg.get('type') == 'pose':
                        pose, pose_at = msg, now
                        continue
                    if 'stage' not in msg:
                        continue
                    if not started:
                        if not pose or now - pose_at > .5 or not msg.get('sensors_ready') or not msg.get('feedback_fresh') or (msg.get('dwa_enabled') and not msg.get('dwa_ready')):
                            if now - begin > 15:
                                raise RuntimeError('定位、雷达、底盘或 DWA 未就绪，请先 prepare 并检查 /home/nvidia/lab3-dwa/logs/navigation.log')
                            continue
                        frame = pose['frame'].lstrip('/')
                        if not re.fullmatch(r'[A-Za-z0-9_/]+', frame):
                            raise RuntimeError('定位坐标系无效')
                        distance = math.hypot(x - pose['pose'][0], y - pose['pose'][1])
                        LOG.info('底盘模式=%s（1=CAN） | 遥控开关=%s | 底盘故障码=%s',
                                 msg.get('chassis_mode'), msg.get('remote_switch'), msg.get('chassis_errors'))
                        if msg.get('chassis_mode') != 1:
                            raise RuntimeError('底盘未进入 CAN 模式，请重新 prepare 完成初始化；不是仅看遥控器档位')
                        if msg.get('remote_switch') in (1, 3) or msg.get('chassis_errors') != 0:
                            raise RuntimeError('遥控器正在接管或底盘故障，无法开始导航')
                        if distance > 30:
                            raise ValueError('终点距离超过 30 m')
                        if not run:
                            LOG.info('当前位置 %s | 坐标系 %s | 终点 (%.2f, %.2f) | 距离 %.2f m', pose['pose'], frame, x, y, distance)
                            LOG.info('检查完成；添加 --run 执行导航。')
                            return
                        if not all(isinstance(msg.get(k), (int, float)) and abs(msg[k]) < .02 for k in ('v', 'w', 'feedback_v', 'feedback_w')):
                            continue
                        if msg['stage'] not in ('WAITING', 'DONE', 'FAULT'):
                            continue
                        # C++ 接收此命令，使用定位与雷达闭环控制底盘。
                        send(f'startxy {x:.6f} {y:.6f} {frame}')
                        started = True
                        start_time = now
                        LOG.info('开始导航：(%.2f, %.2f)，Ctrl+C 停车', x, y)
                        continue
                    if msg['stage'] in ('WAITING', 'TURNING', 'DRIVING'):
                        active = True
                    if msg['stage'] == 'WAITING' and now - start_time > 10:
                        raise RuntimeError('启动等待超过 10 秒：底盘模式=%s，阻塞原因=%s，暂停=%s' %
                                           (msg.get('chassis_mode'), msg.get('manual_block_reason'), msg.get('paused')))
                    if not active and now - start_time > 3:
                        raise RuntimeError('控制器未接受目标')
                    if active and msg['stage'] == 'DONE':
                        LOG.info('已到达终点并停车 | 位置=%s | 剩余=%s m', pose['pose'], msg.get('goal_distance'))
                        return
                    if active and msg['stage'] == 'FAULT':
                        raise RuntimeError('控制器故障：' + str(msg.get('reason', msg)))
                    if now - last_print >= 1:
                        LOG.info('状态=%s | 避障=%s | 位置=%s | 定位年龄=%.2fs | 剩余=%s m | 指令 v=%s w=%s | 反馈 v=%s w=%s | 原因=%s',
                                 msg['stage'], msg.get('avoidance_state', ''), pose['pose'], now-pose_at,
                                 msg.get('goal_distance', '未知'), msg.get('v'), msg.get('w'),
                                 msg.get('feedback_v'), msg.get('feedback_w'), msg.get('avoidance_reason') or msg.get('reason', ''))
                        last_print = now
            raise TimeoutError('导航超时')
        finally:
            if run:
                try:
                    send('stop')
                    LOG.info('已发送停车指令')
                except OSError:
                    LOG.warning('连接已断开，无法发送停车指令；控制器依赖心跳超时停车，未确认实车静止')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, default=GOAL_X)
    parser.add_argument('--y', type=float, default=GOAL_Y)
    parser.add_argument('--port', type=int, default=18766 if os.name == 'nt' else 8766)
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    setup_log()
    if os.name != 'nt':
        signal.signal(signal.SIGUSR1, request_control)
        signal.signal(signal.SIGUSR2, request_control)
    try:
        prepare_controller()
        navigate(args.x, args.y, args.port, args.run)
    except (KeyboardInterrupt, OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
        LOG.error('停止：%s', '用户按下 Ctrl+C' if isinstance(error, KeyboardInterrupt) else error)
        if isinstance(error, ConnectionResetError):
            LOG.error('机器人服务或 SSH 转发连接中断；本次任务不会自动重启，请检查服务后重新运行')
        raise SystemExit(1)
