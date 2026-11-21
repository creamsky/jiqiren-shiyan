#!/usr/bin/python3
"""Gazebo-only ROS adapter for scene editing, observations and gated motor commands."""
import json
import math
import os
import threading
import time
from collections import deque

import actionlib
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import SpawnModel, SetModelState
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import Pose, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_msgs.msg import Header
from voxel_memory import VoxelMemory
from tf.transformations import quaternion_matrix
from dynamic_reconfigure.msg import Config, DoubleParameter, StrParameter
from dynamic_reconfigure.srv import Reconfigure
from collision_geometry import CollisionGeometry

def prop_model(kind):
    rings=[(.16,.10),(.123,.14),(.08,.14),(.039,.10)] if kind=='cone' else [(0,.86)]
    links=[];collisions=[];z=0.
    for i,(radius,height) in enumerate(rings):
        shape=(f'<cylinder><radius>{radius}</radius><length>{height}</length></cylinder>' if radius else '<box><size>.48 1.10 .86</size></box>')
        pose=f'<pose>0 0 {z+height/2} 0 0 0</pose>';geom=f'<geometry>{shape}</geometry>'
        color=('.95 .30 .06 1' if i%2==0 else '.94 .94 .85 1') if radius else '.45 .54 .58 1'
        links.append(f'<collision name="hit{i}">{pose}{geom}</collision><visual name="v{i}">{pose}{geom}<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>')
        collisions.append(f'<collision>hit{i}</collision>');z+=height
    sensor='<sensor name="contact" type="contact"><always_on>true</always_on><update_rate>50</update_rate><contact>'+''.join(collisions)+'</contact><plugin name="bumper" filename="libgazebo_ros_bumper.so"><bumperTopicName>/scout_lab/contact</bumperTopicName><frameName>world</frameName></plugin></sensor>'
    return '<sdf version="1.6"><model name="prop"><static>true</static><link name="link">'+''.join(links)+sensor+'</link></model></sdf>'

def finite_point(value):
    if len(value)!=2 or not all(math.isfinite(float(v)) and abs(float(v))<=12 for v in value):raise ValueError("坐标超出场地")
    return list(map(float,value))

class Lab:
    def __init__(self):
        self.lock=threading.RLock();self.scene_lock=threading.Lock()
        self.voxel_memory=VoxelMemory()
        self.tf=tf2_ros.Buffer();self.listener=tf2_ros.TransformListener(self.tf)
        self.client=actionlib.SimpleActionClient('/move_base',MoveBaseAction)
        self.out=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.cloud=rospy.Publisher('/lab26/obstacles',PointCloud2,queue_size=1)
        self.clearing=rospy.Publisher('/scout_lab/clearing',PointCloud2,queue_size=1)
        self.pub=rospy.Publisher('/scout_lab/state',String,queue_size=1)
        self.phase="idle";self.note="等待场景";self.goal=None;self.props={};self.trail=deque(maxlen=1800)
        self.prop_pool={};self.navigation_owner=None
        self.points=[];self.path=[];self.pose=None;self.speed=[0.,0.];self.scan_t=self.pose_t=self.proposal_t=-100.
        self.proposal=[0.,0.];self.last=[0.,0.];self.next_id=0;self.contact=None;self.has_server=False
        self.q=[0,0,0,1];self.z=0.;self.gap=None;self.distance=0.;self.travel_origin=None
        self.body_points=np.empty((0,2));self.reverse_distance=0.;self.forward_distance=0.
        self.localization_restarting=False
        self.manual_owner=None;self.manual_cmd=[0.,0.];self.manual_at=0.;self.ack=None
        self.pivot=False;self.pivot_sign=1.;self.reverse_disabled=False
        self.pivot_watch=None
        self.limits=rospy.get_param('~limits');self.auto_limits=self.limits['automatic'];self.manual_limits=self.limits['manual'];self.manual_last=[0.,0.]
        self.control_t=rospy.Time.now().to_sec();self.status_ticks=0
        self.planner='dwa_early';self.requested_planner=rospy.get_param('~planner','dwa_early')
        self.move_base_configure=rospy.ServiceProxy('/move_base/set_parameters',Reconfigure)
        self.metrics={};self.metric_previous=[0.,0.];self.episode_start=0.
        self.control_epoch=int(time.time()*1000)
        self.geometry=CollisionGeometry(rospy.get_param('/robot_description'))
        self.footprint_ready=False
        footprint=self.geometry.outline.tolist()
        for namespace in ('/move_base/global_costmap','/move_base/local_costmap'):
            rospy.set_param(namespace+'/footprint',footprint)
        for planner in ('MpcLocalPlannerROS','TebLocalPlannerROS'):
            rospy.set_param('/move_base/'+planner+'/footprint_model/vertices',footprint)
        self.pose_wall=0.;self.last_state=None
        self.server_probe=0.
        self.configure=rospy.ServiceProxy('/move_base/DWAPlannerROS/set_parameters',Reconfigure)
        self.spawn=rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)
        self.set_model=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        rospy.Subscriber('/scout_lab/odom',Odometry,self.position,queue_size=1)
        rospy.Subscriber('/velodyne_points',PointCloud2,self.scan,queue_size=1,buff_size=2**22)
        rospy.Subscriber('/scout_lab/proposal',Twist,self.velocity,queue_size=1)
        rospy.Subscriber('/move_base/NavfnROS/plan',Path,self.route,queue_size=1)
        rospy.Subscriber('/scout_lab/request',String,self.request,queue_size=2)
        rospy.Subscriber('/scout_lab/contact',ContactsState,self.touch,queue_size=2)
        rospy.Timer(rospy.Duration(.02),self.tick)
        threading.Thread(target=self.wall_status,daemon=True).start()
        rospy.on_shutdown(lambda:self.out.publish(Twist()))
    def position(self,msg):
        stamp=msg.header.stamp
        if msg.header.frame_id.lstrip('/')!='camera_init' or msg.child_frame_id.lstrip('/') not in ('base_link','base_footprint','body'):return
        if stamp.to_sec()<=self.pose_t or not -.1<=rospy.Time.now().to_sec()-stamp.to_sec()<.5:return
        p=msg.pose.pose;vel=msg.twist.twist;pos=p.position;q=p.orientation
        values=[pos.x,pos.y,pos.z,q.x,q.y,q.z,q.w,vel.linear.x,vel.angular.z]
        if not all(math.isfinite(v) for v in values) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.05:return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        with self.lock:
            if self.pose is not None and self.phase in ('running','manual'):
                dt=stamp.to_sec()-self.pose_t
                plausible=max(.7,self.manual_limits['max_v']+.2) if self.phase=='manual' else .7
                delta_yaw=yaw-self.pose[2]
                if math.hypot(pos.x-self.pose[0],pos.y-self.pose[1])>.15+plausible*dt or abs(math.atan2(math.sin(delta_yaw),math.cos(delta_yaw)))>.1+.8*dt:
                    self.stop('fault','激光定位发生跳变，请重新定位')
            self.pose=[pos.x,pos.y,yaw];self.z=pos.z;self.q=[q.x,q.y,q.z,q.w]
            self.speed=[vel.linear.x,vel.angular.z];self.pose_t=stamp.to_sec()
            self.pose_wall=time.monotonic()
    def scan(self,msg):
        try:
            tr=self.tf.lookup_transform('camera_init',msg.header.frame_id,msg.header.stamp,rospy.Duration(.08)).transform
            body_transform=self.tf.lookup_transform('base_link',msg.header.frame_id,msg.header.stamp,rospy.Duration(.08)).transform
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):return
        raw=np.asarray(list(pc.read_points(msg,field_names=('x','y','z'),skip_nans=True)),dtype=float).reshape(-1,3)
        raw=raw[np.isfinite(raw).all(axis=1)]
        q=tr.rotation;t=tr.translation
        world=raw@quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3].T+[t.x,t.y,t.z]
        q=body_transform.rotation;t=body_transform.translation
        body=raw@quaternion_matrix([q.x,q.y,q.z,q.w])[:3,:3].T+[t.x,t.y,t.z]
        visible=(np.hypot(raw[:,0],raw[:,1])<6)&~self.geometry.own_returns(body)
        visible &= abs(np.arctan2(body[:,1],body[:,0]))<math.radians(140)
        mask=visible&(world[:,2]>.055)&(world[:,2]<1.45)
        header=Header(stamp=msg.header.stamp,frame_id='camera_init')
        self.cloud.publish(pc.create_cloud_xyz32(header,world[mask].tolist()))
        # Ground returns terminate free rays as well. They must not become
        # obstacle marks, nor may missing/occluded rays clear unseen cells.
        endpoints=world[visible].copy()
        endpoints[:,2]=np.maximum(endpoints[:,2],.001)
        self.clearing.publish(pc.create_cloud_xyz32(header,endpoints.tolist()))
        v=body[mask]
        gap=self.geometry.gap(v[:,:2])
        seen=world[mask]
        with self.lock:
            self.points=seen[::max(1,math.ceil(len(seen)/1000))].round(3).tolist()
            self.scan_t=msg.header.stamp.to_sec();self.gap=gap;self.body_points=v[:,:2]
    def velocity(self,msg):
        with self.lock:
            if self.phase=='running' and math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
                self.proposal=[msg.linear.x,msg.angular.z];self.proposal_t=rospy.Time.now().to_sec()
    def route(self,msg):
        with self.lock:self.path=[[p.pose.position.x,p.pose.position.y] for p in msg.poses[::3]]
    def stop(self,phase,note):
        self.control_epoch+=1
        self.navigation_owner=None
        self.phase=phase;self.note=note;self.client.cancel_all_goals();self.out.publish(Twist())
        self.proposal_t=-100;self.last=[0.,0.]
    def planner_limits(self,pivot,allow_reverse=True):
        if self.planner in ('mpc','teb'):return
        limits=self.auto_limits
        values={'min_vel_x':-limits['reverse_v'] if allow_reverse and not pivot else 0.,'max_vel_x':0. if pivot else limits['max_v'],'max_vel_trans':0. if pivot else limits['max_v']}
        config=Config();config.doubles=[DoubleParameter(k,v) for k,v in values.items()]
        self.configure(config)
    def select_planner(self,name):
        if name not in ('dwa_base','dwa_early','mpc','teb'):raise ValueError('未知规划器')
        if self.phase in ('running','manual','scene_updating') or max(map(abs,self.speed))>.04:
            raise ValueError('请先停止并停稳车辆，再切换规划器')
        limits=self.auto_limits
        if name=='mpc':
            import rospkg
            rospkg.RosPack().get_path('mpc_local_planner')
            values={'max_vel_x':limits['max_v'],'max_vel_x_backwards':limits['reverse_v'],
                    'max_vel_theta':limits['max_w'],'acc_lim_x':limits['linear_acceleration'],
                    'dec_lim_x':limits['linear_acceleration'],'acc_lim_theta':limits['angular_acceleration']}
            for key,value in values.items():rospy.set_param('/move_base/MpcLocalPlannerROS/robot/unicycle/'+key,value)
            plugin='mpc_local_planner/MpcLocalPlannerROS'
        elif name=='teb':
            import rospkg
            rospkg.RosPack().get_path('teb_local_planner')
            values={'max_vel_x':limits['max_v'],'max_vel_x_backwards':limits['reverse_v'],'max_vel_theta':limits['max_w'],
                    'acc_lim_x':limits['linear_acceleration'],'acc_lim_theta':limits['angular_acceleration']}
            for key,value in values.items():rospy.set_param('/move_base/TebLocalPlannerROS/'+key,value)
            plugin='teb_local_planner/TebLocalPlannerROS'
        else:plugin='dwa_local_planner/DWAPlannerROS'
        config=Config();config.strs=[StrParameter('base_local_planner',plugin)]
        self.move_base_configure(config)
        self.planner=name;self.requested_planner=name
        if name.startswith('dwa'):
            values=dict(self.limits['dwa_base'])
            if name=='dwa_early':values.update(sim_time=3.,path_distance_bias=13.,occdist_scale=.13)
            values.update(max_vel_x=limits['max_v'],min_vel_x=-limits['reverse_v'],
                          max_vel_trans=limits['max_v'],max_vel_theta=limits['max_w'],
                          acc_lim_x=limits['linear_acceleration'],acc_lim_trans=limits['linear_acceleration'],acc_lim_theta=limits['angular_acceleration'])
            config=Config();config.doubles=[DoubleParameter(k,v) for k,v in values.items()];self.configure(config)
        self.phase='ready';self.note='规划器已切换：'+name;self.control_epoch+=1
    def touch(self,msg):
        with self.lock:
            for c in msg.states:
                if 'scout_mini::' in c.collision1_name or 'scout_mini::' in c.collision2_name:
                    self.contact=c.collision1_name+' / '+c.collision2_name
                    if self.phase in ('running','manual'):self.stop('contact','Gazebo 检测到车体接触')
                    break
    def command(self,d):
        op=d['op']
        if op in ('go','scene','manual') and d.get('control_epoch')!=self.control_epoch:
            raise ValueError('控制周期已变更，旧场景或目标已失效，请重新操作')
        if op=='planner':
            with self.lock:self.select_planner(d['name'])
            return
        if op in ('manual','drive'):
            with self.lock:
                owner=d['owner']
                if op=='manual':
                    if self.phase in ('running','manual') or not self.ready():raise ValueError('请先停止其他任务并等待就绪')
                    self.phase='manual';self.manual_owner=owner;self.manual_cmd=[0.,0.];self.manual_last=[0.,0.];self.note='仿真手动驾驶';self.manual_at=time.monotonic()
                elif self.phase=='manual' and self.manual_owner==owner:
                    v,w=map(float,d['axes'])
                    speed=float(d['speed']);turn=float(d['turn'])
                    if v not in (-1,0,1) or w not in (-1,0,1) or not math.isfinite(speed) or not math.isfinite(turn):raise ValueError('无效的双轴驾驶参数')
                    if not .1<=speed<=self.manual_limits['max_v'] or not .05<=turn<=self.manual_limits['max_w']:raise ValueError('驾驶速度超出真实控制器允许范围')
                    self.manual_cmd=[v*speed,w*turn];self.manual_at=time.monotonic()
            return
        if op=='stop':
            with self.lock:self.stop('stopped','已停止')
            return
        if op=='reset':
            with self.lock:
                if self.phase in ('running','manual') or max(map(abs,self.speed))>.04:raise ValueError('请先停车')
                if any(math.hypot(x,y)<(1.05 if k=='block' else .70) for k,x,y in self.props.values()):raise ValueError('起点被障碍占用')
                self.stop('stopped','复位中')
                self.localization_restarting=True;self.pose=None;self.pose_t=-100.;self.pose_wall=0.
            model=ModelState();model.model_name='scout_mini';model.reference_frame='world'
            model.pose.position.z=.22;model.pose.orientation.w=1.
            ret=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)(model)
            if not ret.success:raise ValueError(ret.status_message)
            rospy.wait_for_service('/scout_lab/reset_localization',timeout=5.)
            localization=rospy.ServiceProxy('/scout_lab/reset_localization',Trigger)()
            if not localization.success:raise ValueError(localization.message)
            with self.lock:self.localization_restarting=False
            rospy.ServiceProxy('/move_base/clear_costmaps',Empty)()
            self.voxel_memory.clear(rospy.Time.now().to_sec())
            with self.lock:
                self.trail.clear();self.path=[];self.distance=0;self.travel_origin=None
                self.phase='ready';self.note='已复位至起点';self.goal=None;self.contact=None
            return
        if op=='scene':
            if len(d['objects'])>40:raise ValueError('最多 40 个障碍')
            objects=[]
            for k,x,y in d['objects']:
                if k not in ('cone','block'):raise ValueError('未知物体')
                objects.append([k,*finite_point([x,y])])
            with self.lock:
                if self.phase in ('running','manual') or not self.pose or abs(self.speed[0])>.02 or abs(self.speed[1])>.04:raise ValueError('停车后再应用场景')
                if any(math.hypot(x-self.pose[0],y-self.pose[1])<(1.05 if k=='block' else .70) for k,x,y in objects):raise ValueError('障碍与车辆过近')
                self.phase='scene_updating';self.note='同步 Gazebo 障碍物'
            available={kind:[key for key,value in self.prop_pool.items() if value==kind] for kind in ('cone','block')}
            applied={}
            for k,x,y in objects:
                if available[k]:
                    key=available[k].pop()
                    model=ModelState();model.model_name=key;model.reference_frame='world'
                    model.pose.position.x=x;model.pose.position.y=y;model.pose.orientation.w=1.
                    ret=self.set_model(model)
                else:
                    self.next_id+=1;key=f'lab_prop_{self.next_id}'
                    p=Pose();p.position.x=x;p.position.y=y;p.orientation.w=1
                    ret=self.spawn(key,prop_model(k),'',p,'world')
                    if ret.success:self.prop_pool[key]=k
                if not ret.success:raise ValueError(ret.status_message)
                applied[key]=[k,x,y]
                with self.lock:self.props[key]=[k,x,y]
            for key in [key for group in available.values() for key in group]:
                # Gazebo sensor model deletion can deadlock while a native client
                # renders its contact sensors. Reuse the pool, park unused props.
                model=ModelState();model.model_name=key;model.reference_frame='world'
                model.pose.position.x=50.+self.next_id;model.pose.position.y=50.;model.pose.orientation.w=1.
                ret=self.set_model(model)
                if not ret.success:raise ValueError(ret.status_message)
            with self.lock:self.props=applied
            rospy.ServiceProxy('/move_base/clear_costmaps',Empty)()
            self.voxel_memory.clear(rospy.Time.now().to_sec())
            with self.lock:self.phase='ready';self.note='场景已同步';self.contact=None
        elif op=='go':
            with self.lock:
                if not self.ready() or self.phase in ('running','manual'):raise ValueError('请先停车并等待规划器、点云和底盘就绪')
                x,y=finite_point(d['goal'])
                bearing=math.atan2(y-self.pose[1],x-self.pose[0])-self.pose[2]
                heading_error=math.atan2(math.sin(bearing),math.cos(bearing))
                self.pivot=abs(heading_error)>1.2;self.pivot_sign=1. if heading_error>=0 else -1.
                self.pivot_watch=(rospy.Time.now().to_sec(),self.pose[2])
                self.planner_limits(self.pivot)
                goal=MoveBaseGoal();goal.target_pose.header.frame_id='camera_init';goal.target_pose.header.stamp=rospy.Time.now()
                goal.target_pose.pose.position.x=x;goal.target_pose.pose.position.y=y;goal.target_pose.pose.orientation.w=1
                self.client.send_goal(goal);self.goal=[x,y];self.phase='running';self.note='正在规划';self.trail.clear();self.path=[]
                self.navigation_owner=d.get('owner') or 'ros'
                self.distance=0.;self.contact=None;self.travel_origin=None;self.proposal_t=-100
                self.reverse_distance=0.;self.forward_distance=0.
                self.reverse_disabled=False
                self.episode_start=rospy.Time.now().to_sec();self.metric_previous=[0.,0.]
                self.metrics=dict(integral_v2=0.,integral_w2=0.,smooth_v=0.,smooth_w=0.,min_observed_gap=None,elapsed=0.)
        else:raise ValueError('未知指令')
    def request(self,msg):
        request={}
        try:
            request=json.loads(msg.data)
            rospy.loginfo('SCOUT_REQUEST op=%s id=%s epoch=%s owner=%s',request.get('op'),request.get('request_id'),request.get('control_epoch'),request.get('owner'))
            with self.scene_lock:self.command(request)
            if request.get('request_id'):
                with self.lock:self.ack=dict(id=str(request['request_id'])[:100],op=request['op'],ok=True)
        except Exception as e:
            with self.lock:
                self.note=str(e)
                if self.phase=='scene_updating':self.phase='stopped'
                if isinstance(request,dict) and request.get('request_id'):
                    self.ack=dict(id=str(request['request_id'])[:100],op=request.get('op',''),ok=False,message=str(e))
    def ready(self):
        t=rospy.Time.now().to_sec()
        return not self.localization_restarting and self.has_server and self.footprint_ready and self.planner==self.requested_planner and self.pose is not None and time.monotonic()-self.pose_wall<.8 and 0<=t-self.scan_t<.6 and 0<=t-self.pose_t<.4
    def readiness(self):
        t=rospy.Time.now().to_sec()
        if self.localization_restarting:return '重建独立激光定位中'
        if self.pose is None or not 0<=t-self.pose_t<.4:
            return '等待 Gazebo 车辆位姿'
        if not 0<=t-self.scan_t<.6:
            return '等待 16 线点云及传感器 TF'
        if not self.has_server:
            return '等待 ROS 导航规划器'
        if not self.footprint_ready:return '配置厂家实际碰撞轮廓'
        if self.planner!=self.requested_planner:return '配置局部规划器'
        return ''
    def wall_status(self):
        while not rospy.is_shutdown():
            state=self.last_state
            if state is not None:
                state=dict(state)
                if time.monotonic()-self.pose_wall>.8:
                    state.update(ready=False,clock_stalled=True,ready_reason='Gazebo 物理时钟暂停或未响应')
                    self.out.publish(Twist())
                else:state['clock_stalled']=False
                self.pub.publish(String(json.dumps(state,allow_nan=False,separators=(',',':'))))
            time.sleep(.1)
    def braking_clear(self,v,w):
        # Sweep the loaded rectangle through reaction time and a braking horizon.
        pts=self.guard_points()
        if not len(pts) or abs(v)+abs(w)<.01:return True
        limits=self.auto_limits
        reaction=limits['reaction_time'];linear_dec=limits['linear_deceleration'];angular_dec=limits['angular_deceleration']
        half_length,half_width=np.abs(self.geometry.guard).max(axis=0)
        horizon=reaction+max(abs(v)/linear_dec,abs(w)/angular_dec)
        edge_speed=abs(v)+math.hypot(half_length,half_width)*abs(w)
        steps=max(1,math.ceil(horizon/min(.02,.005/max(.01,edge_speed))))
        dt=horizon/steps;x=y=angle=0.
        path=[]
        for i in range(steps+1):
            if i:
                braking=max(0.,(i-.5)*dt-reaction)
                linear=math.copysign(max(0.,abs(v)-linear_dec*braking),v)
                angular=math.copysign(max(0.,abs(w)-angular_dec*braking),w)
                x+=linear*math.cos(angle+.5*angular*dt)*dt;y+=linear*math.sin(angle+.5*angular*dt)*dt;angle+=angular*dt
            path.append((x,y,math.cos(angle),math.sin(angle)))
        samples=np.asarray(path)
        radius=float(np.linalg.norm(self.geometry.guard,axis=1).max())
        lower=samples[:,:2].min(axis=0)-radius;upper=samples[:,:2].max(axis=0)+radius
        pts=pts[((pts>=lower)&(pts<=upper)).all(axis=1)]
        if not len(pts):return True
        dx=pts[:,0,None]-samples[:,0];dy=pts[:,1,None]-samples[:,1]
        lx=dx*samples[:,2]+dy*samples[:,3];ly=-dx*samples[:,3]+dy*samples[:,2]
        inside=np.ones(lx.shape,dtype=bool)
        for nx,ny,offset in self.geometry.planes:
            inside&=nx*lx+ny*ly+offset<0
            if not inside.any():return True
        return False
    def guard_points(self):
        points=self.body_points
        if self.pose is None:return points
        memory=self.voxel_memory.body_points(self.pose,rospy.Time.now().to_sec())
        if not len(memory):return points
        # A projected marked voxel is a square, not an exact point. Its corners
        # conservatively cover the half-cell position uncertainty.
        half=self.voxel_memory.cell_radius/math.sqrt(2)
        offsets=np.array([[-half,-half],[-half,half],[half,-half],[half,half]])
        c,s=math.cos(self.pose[2]),math.sin(self.pose[2]);offsets=offsets@np.array([[c,-s],[s,c]])
        memory=(memory[:,None,:]+offsets).reshape(-1,2)
        planes=self.geometry.outline_planes
        outside=(memory@planes[:,:2].T+planes[:,2]>.001).any(axis=1)
        return np.concatenate((points,memory[outside]))
    def turn_clear(self,angle):
        points=self.guard_points()
        if not len(points):return True
        for yaw in np.linspace(0,angle,max(2,math.ceil(abs(angle)/.025))):
            c,s=math.cos(yaw),math.sin(yaw)
            local=points@np.array([[c,-s],[s,c]])
            if np.any((local@self.geometry.planes[:,:2].T+self.geometry.planes[:,2]<0).all(axis=1)):return False
        return True
    def initial_reference(self):
        if not self.path:return None
        xy=np.asarray(self.pose[:2]);path=np.asarray(self.path)
        nearest=int(np.linalg.norm(path-xy,axis=1).argmin())
        for point in path[nearest:]:
            if np.linalg.norm(point-xy)>=.7:return point
        return path[-1]
    def tick(self,event):
        if not self.has_server or time.monotonic()-self.server_probe>1.:
            self.has_server=self.client.wait_for_server(rospy.Duration(.015));self.server_probe=time.monotonic()
        with self.lock:
            if self.has_server and not self.footprint_ready:
                try:
                    config=Config();config.strs=[StrParameter('footprint',json.dumps(self.geometry.outline.tolist()))]
                    for namespace in ('global_costmap','local_costmap'):
                        rospy.ServiceProxy('/move_base/'+namespace+'/set_parameters',Reconfigure)(config)
                    self.footprint_ready=True
                except rospy.ServiceException as error:self.note=str(error)
            if self.has_server and self.planner!=self.requested_planner:
                try:self.select_planner(self.requested_planner)
                except Exception as error:self.note=str(error);self.requested_planner=self.planner
            t=rospy.Time.now().to_sec();cmd=Twist()
            dt=max(0.,min(.1,t-self.control_t));self.control_t=t
            limits=self.auto_limits
            was_running=self.phase=='running'
            if self.phase=='manual':
                if not self.ready() or time.monotonic()-self.manual_at>.30:self.stop('stopped','手动驾驶心跳中断')
                else:
                    target=np.array(self.manual_cmd,dtype=float);previous=np.array(self.manual_last,dtype=float)
                    previous[(target==0)|(target*previous<0)]=0
                    rates=np.array([self.manual_limits['linear_acceleration'],self.manual_limits['angular_acceleration']])
                    next_cmd=previous+np.clip(target-previous,-rates*dt,rates*dt)
                    if self.braking_clear(*next_cmd) and self.braking_clear(*self.speed):cmd.linear.x,cmd.angular.z=map(float,next_cmd)
                    self.manual_last=[cmd.linear.x,cmd.angular.z]
            if self.phase=='running':
                result=self.client.get_state()
                if not self.ready():self.stop('fault','等待新鲜点云或里程计')
                elif result in (GoalStatus.ABORTED,GoalStatus.REJECTED,GoalStatus.LOST):self.stop('blocked','规划器未找到可执行路径')
                elif result==GoalStatus.SUCCEEDED:self.stop('arrived','目标已到达')
                else:
                    pivot_turn=None
                    if self.pivot:
                        reference=self.initial_reference()
                        if reference is not None:
                            angle=math.atan2(reference[1]-self.pose[1],reference[0]-self.pose[0])-self.pose[2]
                            error=math.atan2(math.sin(angle),math.cos(angle))
                            stalled=False
                            if self.pivot_watch and t-self.pivot_watch[0]>1.5:
                                change=self.pose[2]-self.pivot_watch[1]
                                stalled=abs(math.atan2(math.sin(change),math.cos(change)))<.05
                                self.pivot_watch=(t,self.pose[2])
                            if abs(error)<.40 or stalled or not self.turn_clear(error):
                                self.planner_limits(False);self.pivot=False
                            else:
                                direction=self.pivot_sign if abs(error)>2.7 else (1. if error>0 else -1.)
                                pivot_turn=direction*min(limits['max_w'],max(.20,1.2*abs(error)))
                    if t-self.proposal_t<=.30:
                        # DWA already limits acceleration. Additional slew bounds suppress rapid rejoin reversals.
                        v,w=self.proposal
                        target_v=float(np.clip(v,-limits['reverse_v'],limits['max_v']))
                        target_w=float(np.clip(w,-limits['max_w'],limits['max_w']))
                        cmd.linear.x=float(np.clip(target_v,self.last[0]-limits['linear_acceleration']*dt,self.last[0]+limits['linear_acceleration']*dt))
                        cmd.angular.z=float(np.clip(target_w,self.last[1]-limits['angular_acceleration']*dt,self.last[1]+limits['angular_acceleration']*dt))
                        if abs(v)+abs(w)<1e-6:cmd=Twist()
                    if pivot_turn is not None:
                        cmd.linear.x=0.
                        cmd.angular.z=float(np.clip(pivot_turn,self.last[1]-limits['angular_acceleration']*dt,self.last[1]+limits['angular_acceleration']*dt))
                    if self.reverse_distance>=.20 and not self.reverse_disabled:
                        self.planner_limits(self.pivot,False);self.reverse_disabled=True
                    if cmd.linear.x<0 and self.reverse_disabled:cmd.linear.x=0
                    blocked=not self.braking_clear(cmd.linear.x,cmd.angular.z) or not self.braking_clear(*self.speed)
                    if blocked:cmd=Twist()
                    self.last=[cmd.linear.x,cmd.angular.z];self.note='制动范围内有回波' if blocked else ('起步原地对齐目标方向' if self.pivot else ('跟踪局部轨迹' if any(self.last) else '减速或重新规划'))
                    xy=self.pose[:2]
                    if self.travel_origin:
                        ds=math.dist(xy,self.travel_origin);self.distance+=ds
                        if self.speed[0]<-.01:self.reverse_distance+=ds;self.forward_distance=0
                        if self.speed[0]>.01:self.forward_distance+=ds
                        if self.forward_distance>=.15:
                            self.reverse_distance=0
                            if self.reverse_disabled:
                                self.planner_limits(self.pivot);self.reverse_disabled=False
                    self.travel_origin=xy
                    if not self.trail or math.dist(xy,self.trail[-1])>.035:self.trail.append(xy)
            if was_running and dt>0:
                self.metrics['integral_v2']+=self.speed[0]**2*dt;self.metrics['integral_w2']+=self.speed[1]**2*dt
                self.metrics['smooth_v']+=(cmd.linear.x-self.metric_previous[0])**2/dt
                self.metrics['smooth_w']+=(cmd.angular.z-self.metric_previous[1])**2/dt
                self.metric_previous=[cmd.linear.x,cmd.angular.z];self.metrics['elapsed']=t-self.episode_start
                if self.gap is not None:
                    prior=self.metrics['min_observed_gap'];self.metrics['min_observed_gap']=self.gap if prior is None else min(prior,self.gap)
            self.out.publish(cmd)
            self.status_ticks+=1
            if self.status_ticks%5:return
            state=dict(engine='gazebo-classic',ready=self.ready(),ready_reason=self.readiness(),phase=self.phase,note=self.note,ack=self.ack,control_epoch=self.control_epoch,time=t,pose=self.pose,z=self.z,q=self.q,goal=self.goal,
                       points=self.points,path=self.path,trail=list(self.trail),objects=list(self.props.values()),speed=self.speed,
                       command=[cmd.linear.x,cmd.angular.z],gap=self.gap,distance=self.distance,contact=self.contact,pose_source='laser_slam',metrics=self.metrics,limits=self.limits,footprint=self.geometry.outline.tolist(),planner=self.planner,navigation_owner=self.navigation_owner,manual_owner=self.manual_owner if self.phase=='manual' else None,roof_z=rospy.get_param_cached('~roof_z',.175))
            self.last_state=state

if __name__=='__main__':
    if os.environ.get('ROS_MASTER_URI','').rstrip('/')!='http://127.0.0.1:11431':raise SystemExit('Expected isolated simulation ROS master on port 11431')
    rospy.init_node('scout_lab_bridge');lab=Lab();rospy.spin()

