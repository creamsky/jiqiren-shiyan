#!/usr/bin/env python3
"""One bounded 0.25 rad/s, one second direction check with live interlocks."""
import rospy, socket, struct, time, math, json
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
rospy.init_node('lab26_turn_direction',anonymous=True)
state={'pose':None,'cloud_time':0,'clear':False,'can_time':0,'mode':None,'fault':True}
max_feedback_v=max_feedback_w=0.0
def pose(msg):
    q=msg.pose.pose.orientation; p=msg.pose.pose.position
    state['pose']=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    state['pose_time']=time.monotonic()
def cloud(msg):
    hits=sum(.45<math.hypot(x,y)<.85 and abs(z)<.35 for x,y,z in point_cloud2.read_points(msg,field_names=['x','y','z'],skip_nans=True))
    state['clear']=hits<3
    state['cloud_time']=time.monotonic()
subs=[rospy.Subscriber('/Odometry',Odometry,pose,queue_size=1),rospy.Subscriber('/velodyne_points',PointCloud2,cloud,queue_size=1)]
s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW); s.bind(('can0',)); s.setblocking(False)
def feedback():
    global max_feedback_v,max_feedback_w
    for _ in range(500):
        try:
            cid,n,data=struct.unpack('=IB3x8s',s.recv(16))
            if cid==0x211:
                state['can_time']=time.monotonic(); state['mode']=data[1]; state['fault']=data[0]!=0 or data[4]!=0 or data[5]!=0
            if cid==0x221:
                v,w=struct.unpack('>hh',data[:4])
                max_feedback_v=max(max_feedback_v,abs(v/1000))
                max_feedback_w=max(max_feedback_w,abs(w/1000))
        except BlockingIOError: break
def send(w): s.send(struct.pack('=IB3x8s',0x111,8,struct.pack('>hh4x',0,round(w*1000))))
try:
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        feedback(); send(0)
        if state['pose'] and state['cloud_time'] and state['can_time']: break
        time.sleep(.02)
    start=state['pose']; t0=time.monotonic()
    if not start: raise RuntimeError('No odometry')
    while time.monotonic()-t0<1.0:
        feedback(); t=time.monotonic()
        if not state['clear'] or t-state['cloud_time']>.5 or t-state.get('pose_time',0)>.5 or t-state['can_time']>.8 or state['mode']!=1 or state['fault']:
            raise RuntimeError('Motion interlock: '+str(state))
        send(.25); time.sleep(.02)
finally:
    for _ in range(30): send(0); time.sleep(.02)
end=state['pose']; delta=math.atan2(math.sin(end[2]-start[2]),math.cos(end[2]-start[2]))
result={'start':start,'end':end,'yaw_delta_rad':delta,'yaw_delta_deg':math.degrees(delta),'displacement':math.hypot(end[0]-start[0],end[1]-start[1]),'max_feedback_v':max_feedback_v,'max_feedback_w':max_feedback_w}
print(json.dumps(result,indent=2))
with open('/home/nvidia/lab26/logs/turn_direction.json','w') as f: json.dump(result,f,indent=2)
if abs(delta)<.02 or abs(delta)>.4: raise RuntimeError('Unexpected turn response')
