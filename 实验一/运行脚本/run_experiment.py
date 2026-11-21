#!/usr/bin/env python3
"""Build a fresh map, save and validate it, then prove accepted map localization."""
import collections, datetime, fcntl, json, math, os, pathlib, signal, subprocess, sys, time
ROOT=pathlib.Path(__file__).resolve().parent
STATE=ROOT/'state.json'
def save(path,data):
    tmp=path.with_name(path.name+'.%d.tmp'%os.getpid()); tmp.write_text(json.dumps(data,indent=2)); tmp.replace(path)
def stop_group(pid,timeout=35):
    try:
        command=pathlib.Path('/proc/%d/cmdline'%pid).read_bytes().replace(b'\0',b' ').decode()
        if str(ROOT) not in command or os.getpgid(pid)!=pid: return
        os.killpg(pid,signal.SIGINT)
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            try:
                stat=pathlib.Path('/proc/%d/stat'%pid).read_text().split()
                if stat[2]=='Z': return
            except FileNotFoundError: return
            time.sleep(.2)
        os.killpg(pid,signal.SIGTERM)
    except (ProcessLookupError,FileNotFoundError): pass
def stop_existing():
    if STATE.exists():
        state=json.loads(STATE.read_text())
        if state.get('stamp'):
            ROOT.joinpath('logs').mkdir(exist_ok=True)
            save(ROOT/'logs'/(state['stamp']+'_cancelled.json'),{'cancelled':True})
        for label in ['mapping','localization','rviz','dense_map','drivers']:
            pid=state.get('processes',{}).get(label)
            if pid: stop_group(pid)
        state['phase']='STOPPED'; save(STATE,state)
if len(sys.argv)>1 and sys.argv[1]=='--stop':
    stop_existing(); print('Experiment 1 stopped. Any running map recorder was stopped gracefully.'); sys.exit(0)
seconds=int(sys.argv[1]) if len(sys.argv)>1 else 30
if not 5<=seconds<=600: raise SystemExit('Mapping duration must be 5..600 seconds')
ROOT.joinpath('logs').mkdir(exist_ok=True); ROOT.joinpath('maps').mkdir(exist_ok=True)
lock=open(ROOT/'run.lock','w')
try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError: raise SystemExit('Experiment 1 already running')
stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
state={'stamp':stamp,'phase':'STARTING','processes':{},'success':False}
children={}; logfiles=[]
cancel_file=ROOT/'logs'/(stamp+'_cancelled.json')
def phase(text):
    state['phase']=text; save(STATE,state); print('EXPERIMENT 1: '+text,flush=True)

def progress(percent,label):
    print('EXPERIMENT_PROGRESS %d %s'%(percent,label),flush=True)
def spawn(label,args):
    f=open(ROOT/'logs'/(stamp+'_'+label+'.log'),'w'); logfiles.append(f)
    child=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,start_new_session=True)
    children[label]=child; state['processes'][label]=child.pid; save(STATE,state); return child
def stop(label):
    child=children.get(label)
    if child and child.poll() is None:
        os.killpg(child.pid,signal.SIGINT)
        try: child.wait(timeout=40)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid,signal.SIGTERM); child.wait(timeout=5)
def alive(label):
    if cancel_file.exists(): raise RuntimeError('Operator requested stop')
    if children[label].poll() is not None: raise RuntimeError(label+' exited; inspect '+str(ROOT/'logs'))
def wait_for(predicate,limit,label):
    end=time.monotonic()+limit
    while time.monotonic()<end:
        alive('drivers')
        if 'rviz' in children: alive('rviz')
        if predicate(): return
        time.sleep(.1)
    raise RuntimeError('Timeout: '+label+'; received='+str({k:len(v) for k,v in messages.items()}))
try:
    current=subprocess.run(['rosnode','list'],capture_output=True,text=True,timeout=5)
    if '/basicControl' in current.stdout.split(): raise RuntimeError('Experiment 2 controller is running. Stop it first.')
    stop_existing()
    # Release the mutually exclusive physical sensor ports used by experiment 2.
    old=pathlib.Path('/home/nvidia/lab26')
    if (old/'stop.sh').exists(): subprocess.run(['bash',str(old/'stop.sh')],check=True,timeout=45)
    old_pid_file=old/'logs/sensors.pid'
    if old_pid_file.exists():
        try:
            old_pid=int(old_pid_file.read_text()); deadline=time.monotonic()+35
            while pathlib.Path('/proc/%d/cmdline'%old_pid).exists():
                cmd=pathlib.Path('/proc/%d/cmdline'%old_pid).read_bytes()
                if str(old).encode() not in cmd: break
                if time.monotonic()>deadline: raise RuntimeError('Previous experiment drivers did not release the devices')
                time.sleep(.2)
        except (FileNotFoundError,ValueError): pass
    for label in ['rviz','feedback']:
        p=old/'logs'/(label+'.pid')
        if p.exists():
            try:
                pid=int(p.read_text()); cmd=pathlib.Path('/proc/%d/cmdline'%pid).read_bytes()
                if str(old).encode() in cmd: os.kill(pid,signal.SIGTERM)
            except (OSError,ValueError): pass
    time.sleep(2)
    state['processes']={}; phase('STARTING DRIVERS')
    progress(10,'启动雷达和惯导')
    spawn('drivers',['roslaunch',str(ROOT/'drivers.launch')])
    import rosgraph
    end=time.monotonic()+15
    while True:
        try: rosgraph.Master('/lab1_check').getPid(); break
        except Exception:
            if time.monotonic()>end: raise RuntimeError('ROS master did not start')
            time.sleep(.2)
    import rospy, rosbag
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu,PointCloud2
    from geometry_msgs.msg import PoseStamped
    rospy.init_node('lab1_verifier',anonymous=True,disable_signals=True)
    messages=collections.defaultdict(list); latest={}
    def received(msg,topic):
        messages[topic].append(time.monotonic()); latest[topic]=msg
        if len(messages[topic])==1: print('INPUT READY: '+topic,flush=True)
    subscriptions=[rospy.Subscriber(topic,cls,received,callback_args=topic,queue_size=2)
                   for topic,cls in [('/imu',Imu),('/velodyne_points',PointCloud2),('/Odometry',Odometry),('/pose_stamped',PoseStamped)]]
    wait_for(lambda:len(messages['/imu'])>100 and len(messages['/velodyne_points'])>10,30,'sensor data')
    bag=ROOT/'maps'/(stamp+'.bag'); pcd=bag.with_suffix('.pcd')
    phase('MAPPING %d SECONDS'%seconds)
    progress(20,'启动建图')
    spawn('mapping',['roslaunch','fast_lio_localization_qn','mapping_record.launch','rviz:=false','map_bag:='+str(bag),'map_pcd:='+str(pcd)])
    spawn('rviz',['bash',str(ROOT/'display.sh'),'mapping'])
    wait_for(lambda:len(messages['/Odometry'])>=20,25,'FAST-LIO odometry')
    begin=time.monotonic()
    last_percent=-1
    while time.monotonic()-begin<seconds:
        alive('mapping'); alive('drivers'); alive('rviz')
        for t in ['/imu','/velodyne_points','/Odometry']:
            if not messages[t] or time.monotonic()-messages[t][-1]>1: raise RuntimeError('Input stopped: '+t)
        percent=20+int(45*min(1,(time.monotonic()-begin)/seconds))
        if percent!=last_percent: progress(percent,'建图采集'); last_percent=percent
        time.sleep(.2)
    phase('SAVING AND VALIDATING MAP')
    progress(70,'保存并检查地图')
    stop('mapping')
    if not bag.is_file() or not pcd.is_file(): raise RuntimeError('Map output missing')
    with pcd.open('rb') as f:
        header={}
        for _ in range(30):
            line=f.readline().decode('ascii').strip(); parts=line.split()
            if parts: header[parts[0]]=parts[1:]
            if line.startswith('DATA '): break
    points=int(header['POINTS'][0])
    if points<1000 or pcd.stat().st_size<10000: raise RuntimeError('PCD is empty or too small')
    with rosbag.Bag(str(bag)) as b:
        ncloud=b.get_message_count('/keyframe_pcd'); npose=b.get_message_count('/keyframe_pose')
        if ncloud<1 or ncloud!=npose: raise RuntimeError('Keyframe pairs missing or unequal')
        for topic,msg,t in b.read_messages(topics=['/keyframe_pcd','/keyframe_pose']):
            if topic=='/keyframe_pcd' and msg.width*msg.height<100: raise RuntimeError('Empty keyframe')
    state['map']={'bag':str(bag),'pcd':str(pcd),'points':points,'keyframes':ncloud,'pcd_bytes':pcd.stat().st_size,'bag_bytes':bag.stat().st_size}
    phase('LOADING SAVED MAP FOR LOCALIZATION')
    progress(85,'加载地图并验证定位')
    stop('rviz')
    messages['/pose_stamped'].clear()
    spawn('localization',['roslaunch','fast_lio_localization_qn','run.launch','lidar:=velodyne','rviz:=false','map_bag:='+str(bag)])
    spawn('dense_map',['/opt/ros/noetic/lib/pcl_ros/pcd_to_pointcloud',str(pcd),
                       '__name:=lab1_dense_map','_frame_id:=map','_latch:=true','cloud_pcd:=/lab1_dense_map'])
    spawn('rviz',['bash',str(ROOT/'display.sh'),'localization'])
    wait_for(lambda:len(messages['/pose_stamped'])>=15,50,'accepted global map localization')
    alive('localization')
    alive('dense_map')
    alive('rviz')
    dense_map=rospy.wait_for_message('/lab1_dense_map',PointCloud2,timeout=15)
    if dense_map.header.frame_id!='map' or dense_map.width*dense_map.height!=points:
        raise RuntimeError('RViz dense map does not match the saved PCD')
    pose=latest['/pose_stamped']; p=pose.pose.position; q=pose.pose.orientation
    if pose.header.frame_id!='map' or not all(math.isfinite(v) for v in [p.x,p.y,p.z,q.x,q.y,q.z,q.w]): raise RuntimeError('Invalid global pose')
    age=(rospy.Time.now()-pose.header.stamp).to_sec()
    if age>1 or age<-.1 or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.05: raise RuntimeError('Stale or non-normalized global pose')
    logtext=(ROOT/'logs'/(stamp+'_localization.log')).read_text(errors='replace')
    # The patched node withholds this topic until a map match is accepted.
    # Its console output can remain buffered while localization stays running.
    state['localization']={'frame':pose.header.frame_id,'position':[p.x,p.y,p.z],'messages':len(messages['/pose_stamped']),'age_seconds':age,'accepted':True}
    state['rates_hz']={t:round((len(ts)-1)/(ts[-1]-ts[0]),2) for t,ts in messages.items() if len(ts)>1}
    state['success']=True; phase('COMPLETED; LIVE LOCALIZATION AND RVIZ REMAIN RUNNING')
    progress(100,'地图与定位验证完成')
    save(ROOT/'report.json',state); save(ROOT/'logs'/(stamp+'_report.json'),state)
    ROOT.joinpath('latest_map.txt').write_text(stamp+'\n')
    print(json.dumps(state,indent=2),flush=True)
except (Exception,KeyboardInterrupt) as e:
    cancelled=cancel_file.exists() or isinstance(e,KeyboardInterrupt)
    state['error']=str(e); phase('STOPPED' if cancelled else 'FAILED')
    for label in ['mapping','localization','rviz','dense_map','drivers']: stop(label)
    save(ROOT/'report.json',state)
    print(('EXPERIMENT 1 STOPPED: ' if cancelled else 'EXPERIMENT 1 FAILED: ')+str(e),flush=True); sys.exit(0 if cancelled else 2)
finally:
    for f in logfiles: f.close()
