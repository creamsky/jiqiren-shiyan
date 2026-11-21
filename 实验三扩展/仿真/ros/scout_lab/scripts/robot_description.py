#!/usr/bin/python3
"""Expand the pinned AgileX model and select its skid-steer controller."""
import sys
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import rospkg
import xacro


def align_factory_chassis(root):
    """Align the source CAD's +Y front lamp with the ROS +X drive direction.

    Experiment-two Word Figure 2.3 identifies the triangular centre-port end as
    the rear. That feature is the mesh's -Y end. Its opposite long lamp is +Y.
    The CAD bearing centres below were measured on the four circular mounting
    faces of the pinned STL, before the vendor's visual-origin translation.
    """
    yaw=-math.pi/2
    chassis=root.find("link[@name='base_link']")
    for element in list(chassis.findall('visual'))+list(chassis.findall('collision')):
        origin=element.find('origin')
        x,y,z=map(float,origin.get('xyz','0 0 0').split())
        origin.set('xyz',f'{y:.12f} {-x:.12f} {z:.12f}')
        origin.set('rpy',f'0 0 {yaw:.15f}')
    inertial=root.find("link[@name='inertial_link']/inertial/origin")
    if inertial is not None:
        x,y,z=map(float,inertial.get('xyz').split())
        inertial.set('xyz',f'{y:.12f} {-x:.12f} {z:.12f}')
        inertial.set('rpy',f'0 0 {yaw:.15f}')
    old_offset=(.015,.0231316650320557,0.)
    for end,cad_y in (('front',.20835333),('rear',-.24424478)):
        for side,cad_x in (('left',-.18252623),('right',.15347376)):
            joint=root.find(f"joint[@name='{end}_{side}_wheel']/origin")
            joint.set('xyz',f'{cad_y+old_offset[1]:.12f} {-cad_x-old_offset[0]:.12f} {-0.07424039:.12f}')
            joint.set('rpy',f'{-math.pi/2 if side=="left" else math.pi/2:.15f} 0 0')
    skid=root.find("gazebo/plugin[@name='skid_steer_drive_controller']")
    if skid is None:raise ValueError('Pinned factory skid-steer plugin is missing')
    # Tire collision centres sit 0.03 m outward from each axle mounting frame.
    # Keep that factory offset; do not place the tire's geometric centre at the bearing.
    separation=.15347376-(-.18252623)+2*.03
    skid.find('wheelSeparation').text=f'{separation:.12f}'
    skid.find('wheelbase').text=f'{.20835333-(-.24424478):.12f}'


def description(roof_z):
    packages = rospkg.RosPack()
    vendor = Path(packages.get_path('scout_description'))
    laboratory = Path(packages.get_path('scout_lab'))
    if not .15 <= roof_z <= 1.0:
        raise ValueError('roof_z must be between 0.15 and 1.0 metres above base_link')
    document = xacro.process_file(str(vendor / 'urdf/scout_mini.xacro'), mappings={
        'robot_namespace': '/',
        'urdf_extras': str(laboratory / 'urdf/roof.xacro'),
        'roof_z': str(roof_z),
    })
    root = ET.fromstring(document.toxml())
    align_factory_chassis(root)
    # The vendor supplies two alternative actuators. Running both would allow
    # ros_control's idle velocity interface to overwrite skid-steer wheel targets.
    for element in list(root):
        if element.tag == 'transmission':
            root.remove(element)
        elif element.tag == 'gazebo':
            if element.get('reference') in ('laser_link', 'camera_link'):
                root.remove(element)
                continue
            for plugin in list(element.findall('plugin')):
                if plugin.get('filename') == 'libgazebo_ros_control.so':
                    element.remove(plugin)
    gazebo = ET.SubElement(root, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin', name='wheel_joint_states',
                           filename='libgazebo_ros_joint_state_publisher.so')
    ET.SubElement(plugin, 'robotNamespace').text = '/'
    ET.SubElement(plugin, 'updateRate').text = '30'
    ET.SubElement(plugin, 'jointName').text = ','.join((
        'front_left_wheel', 'rear_left_wheel', 'front_right_wheel', 'rear_right_wheel'))
    return ET.tostring(root, encoding='unicode')


if __name__ == '__main__':
    print(description(float(sys.argv[1]) if len(sys.argv) > 1 else .175))
