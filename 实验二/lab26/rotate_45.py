#!/usr/bin/env python3
"""Rotate only, with zero linear velocity, wall-time limit and sensor interlocks."""
import math, time, socket, struct, json
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
rospy.init_node('lab26_rotate_45',anonymous=True)
state={'pose':None,'pose_time':0,'clear':False,'cloud_time':0,'can_time':0,'mode':None,'fault':True,'feedback_w':0.,'rc':None}
def pose(msg):
    q=msg.pose.pose.orientation
    if (rospy.Time.now()-msg.header.stamp).to_sec()>.5: return
    state['pose']=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    state['pose_time']=time.monotonic()
def cloud(msg):
    if (rospy.Time.now()-msg.header.stamp).to_sec()>.5: return
    hits=sum(.45<math.hypot(x,y)<.85 and abs(z)<.35 for x,y,z in point_cloud2.read_points(msg,field_names=['x','y','z'],skip_nans=True))
    state['clear']=hits<3; state['cloud_time']=time.monotonic()
subs=[rospy.Subscriber('/Odometry',Odometry,pose,queue_size=1),rospy.Subscriber('/velodyne_points',PointCloud2,cloud,queue_size=1)]
s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW); s.bind(('can0',)); s.setblocking(False)
def send(w):
    s.send(struct.pack('=IB3x8s',0x421,1,bytes([1])+bytes(7)))
    s.send(struct.pack('=IB3x8s',0x111,8,struct.pack('>hh4x',0,round(w*1000))))
def read():
    for _ in range(500):
        try:
            cid,n,data=struct.unpack('=IB3x8s',s.recv(16))
            if cid==0x211:
                state['mode']=data[1]; state['fault']=data[0]!=0 or data[4]!=0 or data[5]!=0; state['can_time']=time.monotonic()
            elif cid==0x221: state['feedback_w']=struct.unpack('>h',data[2:4])[0]/1000
            elif cid==0x241: state['rc']=data[0]
        except BlockingIOError: break
result={'task':'rotate_only_45_deg','success':False}
try:
    start_time=time.monotonic(); ready_since=None
    while time.monotonic()-start_time<8:
        read(); send(0)
        ready=state['pose'] is not None and state['clear'] and state['mode']==1 and state['rc'] is not None and ((state['rc']>>2)&3)==2
        if ready:
            if ready_since is None: ready_since=time.monotonic()
            if time.monotonic()-ready_since>1: break
        else: ready_since=None
        time.sleep(.02)
    if ready_since is None or time.monotonic()-ready_since<1: raise RuntimeError('Not ready: '+str(state))
    start=state['pose']
    if start is None: raise RuntimeError('No odometry')
    target=math.pi/4; t0=time.monotonic(); peak_feedback=0; angle=0.; tick=0
    while time.monotonic()-t0<15:
        read(); t=time.monotonic()
        if not state['clear'] or t-state['cloud_time']>.5 or t-state['pose_time']>.5 or t-state['can_time']>.8 or state['mode']!=1 or state['fault']:
            raise RuntimeError('Sensor/chassis interlock: '+str(state))
        angle=abs(math.atan2(math.sin(state['pose']-start),math.cos(state['pose']-start)))
        peak_feedback=max(peak_feedback,abs(state['feedback_w']))
        if angle>=target-math.radians(.5):
            result.update(success=True,angle_deg=math.degrees(angle)); break
        if t-t0>2 and peak_feedback<.01 and angle<.02:
            raise RuntimeError('Chassis has no rotation response; RC switch byte='+str(state['rc']))
        if angle>target+.15: raise RuntimeError('Unexpected yaw change')
        w=min(.25,max(.08,1.0*(target-angle)))
        send(w)
        if tick%25==0: print('angle_deg=%.2f feedback_w=%.3f command_w=%.3f RC=%s'%(math.degrees(angle),state['feedback_w'],w,state['rc']),flush=True)
        tick+=1; time.sleep(.02)
    if not result['success']: raise RuntimeError('Rotation timeout')
except Exception as e:
    result['error']=str(e)
finally:
    for _ in range(30): send(0); time.sleep(.02)
    result['final_yaw']=state['pose']
    with open('/home/nvidia/lab26/logs/rotate_45_result.json','w') as f: json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2),flush=True)
if not result['success']: raise SystemExit(2)
