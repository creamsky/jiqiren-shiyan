#!/usr/bin/python3
"""Estimate body velocity strictly from consecutive laser-matched poses."""
from collections import deque
import json
import math
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rospy
import tf2_ros
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory
from geometry_msgs.msg import PoseStamped,TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import euler_matrix


class LaserOdometry:
    def __init__(self):
        # Derive a ground-reference link from the expanded tire collision bodies.
        model=ET.fromstring(rospy.get_param('/robot_description'))
        lowest=[]
        for name in ('front_left','rear_left','front_right','rear_right'):
            joint=model.find(f"joint[@name='{name}_wheel']/origin")
            collision=model.find(f"link[@name='{name}_wheel_link']/collision")
            rotation=euler_matrix(*map(float,joint.get('rpy','0 0 0').split()))[:3,:3]
            jxyz=np.fromstring(joint.get('xyz'),sep=' ')
            origin=collision.find('origin')
            offset=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            axis=rotation@np.array([0.,0.,1.])
            cylinder=collision.find('geometry/cylinder')
            extent=float(cylinder.get('radius'))*math.sqrt(max(0.,1-axis[2]**2))+float(cylinder.get('length'))/2*abs(axis[2])
            lowest.append((jxyz+rotation@offset)[2]-extent)
        height=-min(lowest)
        rospy.set_param('~base_height',height)
        self.static=tf2_ros.StaticTransformBroadcaster()
        transform=TransformStamped();transform.header.stamp=rospy.Time.now();transform.header.frame_id='base_footprint'
        transform.child_frame_id='base_link';transform.transform.translation.z=height;transform.transform.rotation.w=1.
        self.static.sendTransform(transform)
        self.lock=threading.RLock();self.samples=deque(maxlen=4)
        self.scan_at=-100.;self.pose_at=-100.;self.scan_good=False
        self.trajectory=0;self.epoch=0;self.reset_stamp=-1.;self.stable_at=None;self.reason='等待激光匹配'
        self.output=rospy.Publisher('/scout_lab/odom',Odometry,queue_size=1)
        self.health=rospy.Publisher('/scout_lab/localization',String,queue_size=1)
        rospy.Subscriber('/scout_lab/scan',LaserScan,self.scan,queue_size=1)
        rospy.Subscriber('/scout_lab/laser_pose',PoseStamped,self.pose,queue_size=1)
        rospy.Service('/scout_lab/reset_localization',Trigger,self.reset)
        rospy.Timer(rospy.Duration(.1),self.status)

    def scan(self,msg):
        ranges=np.asarray(msg.ranges)
        valid=np.isfinite(ranges)&(ranges>=msg.range_min)&(ranges<=msg.range_max)
        # Returns must cover several directions, not only a single nearby post.
        sectors=np.array_split(valid,8)
        with self.lock:
            self.scan_good=np.count_nonzero(valid)>=90 and sum(np.count_nonzero(s)>=8 for s in sectors)>=4
            self.scan_at=msg.header.stamp.to_sec()

    def pose(self,msg):
        if msg.header.frame_id!='camera_init':return
        stamp=msg.header.stamp.to_sec();p=msg.pose.position;q=msg.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if not all(map(math.isfinite,(stamp,p.x,p.y,yaw))):return
        with self.lock:
            if stamp<=self.reset_stamp or self.samples and stamp<=self.samples[-1][0]:return
            if not self.scan_good or abs(stamp-self.scan_at)>.45:
                self.reason='激光覆盖不足或匹配扫描过期';self.stable_at=None;self.samples.clear();return
            if self.samples and stamp-self.samples[-1][0]>.5:self.samples.clear();self.stable_at=None
            self.samples.append((stamp,p.x,p.y,yaw));self.pose_at=stamp
            if len(self.samples)<2:return
            prev=self.samples[0];dt=stamp-prev[0]
            if dt<=0:return
            dyaw=math.atan2(math.sin(yaw-prev[3]),math.cos(yaw-prev[3]))
            midpoint=prev[3]+dyaw/2
            v=((p.x-prev[1])*math.cos(midpoint)+(p.y-prev[2])*math.sin(midpoint))/dt
            w=dyaw/dt
            if abs(v)>2.2 or abs(w)>1.5:
                self.reason='激光匹配跳变';self.stable_at=None;self.samples.clear();return
            if self.stable_at is None:self.stable_at=time.monotonic()
            self.reason='仅激光定位'
            odom=Odometry();odom.header=msg.header;odom.child_frame_id='base_footprint'
            odom.pose.pose=msg.pose;odom.twist.twist.linear.x=v;odom.twist.twist.angular.z=w
            odom.pose.covariance[0]=odom.pose.covariance[7]=.02**2
            odom.pose.covariance[35]=.02**2
            self.output.publish(odom)

    def status(self,_):
        with self.lock:
            now=rospy.Time.now().to_sec()
            ready=self.scan_good and 0<=now-self.pose_at<.5 and self.stable_at is not None and time.monotonic()-self.stable_at>=1
            self.health.publish(String(json.dumps(dict(source='laser_slam',ready=ready,epoch=self.epoch,
                reason=self.reason,pose_age=now-self.pose_at,scan_age=now-self.scan_at,use_odometry=False,use_imu=False))))

    def reset(self,_):
        with self.lock:
            self.epoch+=1;self.reset_stamp=rospy.Time.now().to_sec();self.samples.clear();self.stable_at=None
            self.pose_at=-100.;self.reason='等待重置后的激光匹配'
            old=self.trajectory
        try:
            finish=rospy.ServiceProxy('/finish_trajectory',FinishTrajectory)(old)
            if finish.status.code:raise ValueError(finish.status.message)
            start=rospy.ServiceProxy('/start_trajectory',StartTrajectory)
            request=StartTrajectory._request_class()
            request.configuration_directory=rospy.get_param('~configuration_directory')
            request.configuration_basename='laser_only.lua';request.use_initial_pose=False
            request.initial_pose.orientation.w=1.;request.relative_to_trajectory_id=old
            result=start(request)
            if result.status.code:raise ValueError(result.status.message)
            with self.lock:self.trajectory=result.trajectory_id
            return TriggerResponse(success=True,message='激光轨迹已重建，等待连续有效匹配后就绪')
        except (rospy.ServiceException,ValueError) as exc:
            return TriggerResponse(success=False,message=str(exc))


if __name__=='__main__':
    rospy.init_node('laser_odometry');LaserOdometry();rospy.spin()
