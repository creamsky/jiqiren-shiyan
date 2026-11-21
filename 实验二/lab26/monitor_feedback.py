#!/usr/bin/env python3
"""Read-only live status and obstacle markers for the robot's RViz display."""
import math, socket, struct, time
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
rospy.init_node('lab26_feedback',anonymous=True)
pose=None; cloud=None; status='IDLE'; mode=-1; v=w=0.; rc=0; obstacles=[]; status_time=0
def on_pose(msg):
    global pose
    pose=msg
def on_cloud(msg):
    global cloud,obstacles
    cloud=msg
    obstacles=[(x,y,z) for x,y,z in point_cloud2.read_points(msg,field_names=['x','y','z'],skip_nans=True)
               if .45<math.hypot(x,y)<.85 and abs(z)<.35]
def on_status(msg):
    global status,status_time
    status=msg.data; status_time=time.monotonic()
subs=[rospy.Subscriber('/Odometry',Odometry,on_pose,queue_size=1),
      rospy.Subscriber('/velodyne_points',PointCloud2,on_cloud,queue_size=1),
      rospy.Subscriber('/lab26/status',String,on_status,queue_size=1)]
pub=rospy.Publisher('/lab26/markers',MarkerArray,queue_size=1,latch=True)
try:
    s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW); s.bind(('can0',)); s.setblocking(False)
except OSError: s=None
rate=rospy.Rate(5)
while not rospy.is_shutdown():
    if s:
        for _ in range(500):
            try:
                cid,n,b=struct.unpack('=IB3x8s',s.recv(16))
                if cid==0x211: mode=b[1]
                elif cid==0x221: v,w=[a/1000 for a in struct.unpack('>hh',b[:4])]
                elif cid==0x241: rc=(b[0]>>2)&3
            except (BlockingIOError,OSError): break
    if pose:
        p=pose.pose.pose.position; q=pose.pose.pose.orientation
        label=Marker(); label.header.frame_id=pose.header.frame_id; label.header.stamp=rospy.Time.now()
        label.ns='lab26'; label.id=0; label.type=Marker.TEXT_VIEW_FACING; label.action=Marker.ADD
        label.pose.position=Point(p.x,p.y,p.z+1.8); label.pose.orientation.w=1
        label.scale.z=.22; label.color.a=1; label.color.r=1; label.color.g=1; label.color.b=1
        yaw=math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        shown=status if time.monotonic()-status_time<3 else 'STOPPED | last: '+status
        label.text='LAB 2.6  '+shown+'\nCAN mode=%d  SWB=%d\nv=%.3f m/s  w=%.3f rad/s\nyaw=%.1f deg  near obstacle points=%d'%(mode,rc,v,w,yaw,len(obstacles))
        dots=Marker(); dots.header=label.header; dots.ns='lab26'; dots.id=1; dots.type=Marker.POINTS; dots.action=Marker.ADD
        dots.pose.orientation.w=1; dots.scale.x=dots.scale.y=.04; dots.color.r=1; dots.color.a=1
        # Apply the inherited LiDAR-to-IMU extrinsic, then current IMU pose.
        xq,yq,zq,wq=q.x,q.y,q.z,q.w
        R=((1-2*(yq*yq+zq*zq),2*(xq*yq-zq*wq),2*(xq*zq+yq*wq)),
           (2*(xq*yq+zq*wq),1-2*(xq*xq+zq*zq),2*(yq*zq-xq*wq)),
           (2*(xq*zq-yq*wq),2*(yq*zq+xq*wq),1-2*(xq*xq+yq*yq)))
        for x,y,z in obstacles[::3]:
            z+=.28
            dots.points.append(Point(p.x+R[0][0]*x+R[0][1]*y+R[0][2]*z,
                                     p.y+R[1][0]*x+R[1][1]*y+R[1][2]*z,
                                     p.z+R[2][0]*x+R[2][1]*y+R[2][2]*z))
        if not dots.points: dots.action=Marker.DELETE
        pub.publish(MarkerArray([label,dots]))
    rate.sleep()
