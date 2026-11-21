#!/usr/bin/env python3
"""Use a separate ROS master and simulated sensors. No CAN sockets are opened."""
import os
import math
import subprocess
import time
import json
import sys

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11312'
os.environ['ROS_IP'] = '127.0.0.1'
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2

root = os.path.dirname(os.path.abspath(__file__))
master_log = open(os.path.join(root,'logs','test_master.log'),'w')
master = subprocess.Popen(['roscore','-p','11312'], stdout=master_log, stderr=subprocess.STDOUT)
results = []
process = None
try:
    time.sleep(2)
    rospy.init_node('lab26_simulation', anonymous=True)
    odom_pub=rospy.Publisher('/Odometry',Odometry,queue_size=1)
    cloud_pub=rospy.Publisher('/velodyne_points',PointCloud2,queue_size=1)
    command=[0.0,0.0]
    def on_command(msg): command[:]=[msg.linear.x,msg.angular.z]
    sub=rospy.Subscriber('/lab26/command',Twist,on_command,queue_size=1)
    for case in ['complete','odometry_timeout','lidar_timeout','obstacle','invalid_pose']:
        command[:]=[0,0]
        x=y=yaw=0.0
        logfile=os.path.join(root,'logs','test_'+case+'.log')
        with open(logfile,'w') as output:
            process=subprocess.Popen(['rosrun','basic_control','basicControl',
                '_dry_run:=true','_speed:=0.5','_run_timeout:=50',
                '_csv:='+os.path.join(root,'logs','test_'+case+'.csv')],stdout=output,stderr=subprocess.STDOUT)
            started=time.monotonic(); last=started; injected=False
            obstacle_started=None; observed_obstacle_stop=False
            max_cross=0
            while process.poll() is None and time.monotonic()-started<55:
                t=time.monotonic(); dt=min(t-last,0.05); last=t
                v,w=command
                x+=v*math.cos(yaw)*dt; y+=v*math.sin(yaw)*dt; yaw+=w*dt
                elapsed=t-started
                inject=elapsed>3
                if case=='obstacle':
                    if obstacle_started is None and v>0.01: obstacle_started=elapsed
                    inject=obstacle_started is not None and elapsed-obstacle_started<3.0
                    if inject and elapsed-obstacle_started>0.3 and abs(v)<1e-9 and abs(w)<1e-9:
                        observed_obstacle_stop=True
                injected=injected or inject
                msg=Odometry(); msg.header.stamp=rospy.Time.now(); msg.header.frame_id='camera_init'
                msg.pose.pose.position.x=x; msg.pose.pose.position.y=y
                msg.pose.pose.orientation.z=math.sin(yaw/2); msg.pose.pose.orientation.w=math.cos(yaw/2)
                if case=='invalid_pose' and inject: msg.pose.pose.position.x=float('nan')
                if not(case=='odometry_timeout' and inject): odom_pub.publish(msg)
                if not(case=='lidar_timeout' and inject):
                    pts=[(3+i*0.001,3.0,0.0) for i in range(120)]
                    if case=='obstacle' and inject: pts.extend([(0.7,0.0,0.0)]*10)
                    cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.now(),frame_id='velodyne'),pts))
                max_cross=max(max_cross,abs(y-x)/math.sqrt(2))
                time.sleep(0.02)
            if process.poll() is None:
                process.terminate(); process.wait(timeout=5)
                raise RuntimeError('Test hung: '+case)
        text=open(logfile).read()
        expected={'complete':'RESULT DONE','odometry_timeout':'Odometry timeout','lidar_timeout':'Lidar timeout','obstacle':'RESULT DONE','invalid_pose':'Invalid/stale odometry'}[case]
        if expected not in text: raise RuntimeError(case+' failed: '+text[-1500:])
        if case=='obstacle' and not observed_obstacle_stop:
            raise RuntimeError('Obstacle did not produce a zero-command pause')
        if case in ('complete','obstacle') and (process.returncode!=0 or abs(math.hypot(x,y)-5)>0.05 or max_cross>0.06):
            raise RuntimeError('Trajectory inaccurate')
        result={'case':case,'pass':True,'exit_code':process.returncode,'distance':round(math.hypot(x,y),4),'max_cross':round(max_cross,4)}
        results.append(result); print(json.dumps(result),flush=True)
        time.sleep(0.5)
    with open(os.path.join(root,'logs','integration_results.json'),'w') as f: json.dump(results,f,indent=2)
finally:
    if process is not None and process.poll() is None: process.terminate(); process.wait(timeout=5)
    master.terminate()
    try: master.wait(timeout=10)
    except subprocess.TimeoutExpired: master.kill(); master.wait()
    master_log.close()
