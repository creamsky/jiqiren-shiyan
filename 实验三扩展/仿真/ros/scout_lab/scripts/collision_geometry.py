"""Collision envelopes from the expanded, assembled manufacturer URDF."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import rospkg
from scipy.spatial import ConvexHull


def origin(element):
    if element is None:return np.eye(4)
    rpy=np.fromstring(element.get('rpy','0 0 0'),sep=' ')
    roll,pitch,yaw=rpy;cr,sr=np.cos(roll),np.sin(roll);cp,sp=np.cos(pitch),np.sin(pitch);cy,sy=np.cos(yaw),np.sin(yaw)
    matrix=np.eye(4)
    matrix[:3,:3]=np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    matrix[:3,3]=np.fromstring(element.get('xyz','0 0 0'),sep=' ')
    return matrix


class CollisionGeometry:
    def __init__(self,xml,margins=(.01,.015)):
        robot=ET.fromstring(xml);transforms={'base_link':np.eye(4)}
        joints=list(robot.findall('joint'))
        while joints:
            progressed=False
            for joint in joints[:]:
                parent=joint.find('parent').get('link')
                if parent in transforms:
                    transforms[joint.find('child').get('link')]=transforms[parent]@origin(joint.find('origin'))
                    joints.remove(joint);progressed=True
            if not progressed:raise ValueError('Disconnected URDF collision tree')
        self.parts=[];outline=[];packages=rospkg.RosPack()
        for link in robot.findall('link'):
            for collision in link.findall('collision'):
                matrix=transforms[link.get('name')]@origin(collision.find('origin'))
                shape=collision.find('geometry')[0];kind=shape.tag
                if kind=='mesh':
                    uri=shape.get('filename')
                    if not uri.startswith('package://'):raise ValueError('Expected package mesh URI')
                    package,file=uri[len('package://'):].split('/',1)
                    path=Path(packages.get_path(package))/file
                    if path.suffix.lower()!='.stl':raise ValueError('Collision mesh must be STL: '+str(path))
                    dtype=np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attribute','<u2')])
                    vertices=np.unique(np.fromfile(path,dtype=dtype,offset=84)['vertices'].reshape(-1,3),axis=0)
                    vertices*=np.fromstring(shape.get('scale','1 1 1'),sep=' ')
                    hull=ConvexHull(vertices);planes=hull.equations
                    bounds=(vertices.min(axis=0),vertices.max(axis=0));parameters=planes
                    vertices=vertices[hull.vertices]
                elif kind=='box':
                    half=np.fromstring(shape.get('size'),sep=' ')/2
                    vertices=np.array([[x,y,z] for x in (-half[0],half[0]) for y in (-half[1],half[1]) for z in (-half[2],half[2])])
                    parameters=half;bounds=(-half,half)
                elif kind=='cylinder':
                    radius=float(shape.get('radius'));length=float(shape.get('length'))
                    angle=np.linspace(0,2*np.pi,64,endpoint=False)
                    vertices=np.array([[radius*np.cos(a),radius*np.sin(a),z] for z in (-length/2,length/2) for a in angle])
                    parameters=(radius,length/2);bounds=(np.array([-radius,-radius,-length/2]),np.array([radius,radius,length/2]))
                else:raise ValueError('Unsupported robot collision shape '+kind)
                outline.extend((vertices@matrix[:3,:3].T+matrix[:3,3])[:,:2])
                self.parts.append((np.linalg.inv(matrix),kind,parameters,bounds))
        points=np.unique(np.asarray(outline),axis=0);self.outline=points[ConvexHull(points).vertices]
        self.outline_planes=ConvexHull(self.outline).equations
        offsets=np.array([[x,y] for x in (-margins[0],margins[0]) for y in (-margins[1],margins[1])])
        expanded=(self.outline[:,None,:]+offsets).reshape(-1,2)
        self.guard=expanded[ConvexHull(expanded).vertices]
        self.planes=ConvexHull(self.guard).equations

    def own_returns(self,body_points):
        inside=np.zeros(len(body_points),bool);tolerance=.001
        for inverse,kind,parameters,(low,high) in self.parts:
            points=body_points@inverse[:3,:3].T+inverse[:3,3]
            candidates=np.flatnonzero((points>=low-tolerance).all(axis=1)&(points<=high+tolerance).all(axis=1)&~inside)
            if not len(candidates):continue
            selected=points[candidates]
            if kind=='mesh':contained=(selected@parameters[:,:3].T+parameters[:,3]<=tolerance).all(axis=1)
            elif kind=='box':contained=(np.abs(selected)<=parameters+tolerance).all(axis=1)
            else:
                radius,half_length=parameters
                contained=(np.linalg.norm(selected[:,:2],axis=1)<=radius+tolerance)&(np.abs(selected[:,2])<=half_length+tolerance)
            inside[candidates[contained]]=True
        return inside

    def gap(self,points):
        if not len(points):return None
        a=self.outline;b=np.roll(a,-1,axis=0);delta=b-a
        t=np.clip(np.sum((points[:,None,:]-a)*delta,axis=2)/np.sum(delta*delta,axis=1),0,1)
        distance=np.linalg.norm(points[:,None,:]-(a+t[:,:,None]*delta),axis=2).min(axis=1)
        planes=self.outline_planes
        distance[(points@planes[:,:2].T+planes[:,2]<=0).all(axis=1)]=0.
        return float(distance.min())
