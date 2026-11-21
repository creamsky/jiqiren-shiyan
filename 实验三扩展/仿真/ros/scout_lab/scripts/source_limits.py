#!/usr/bin/python3
"""Derive the simulator's command envelope from the real XY controller sources."""
import argparse
import hashlib
import json
import re
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml


def derive(root):
    package=root/'实验二/lab26/catkin_ws/src/basic_control'
    files={'launch':package/'launch/basicControl.launch',
           'dwa_client':package/'include/basic_control/dwa_client.h',
           'bridge':root/'dwa/bridge.py','planner':root/'dwa/navigation.yaml'}
    files['footprint']=package/'include/basic_control/footprint.h'
    args={e.get('name'):e.get('default') for e in ET.parse(files['launch']).getroot().findall('arg')}
    planner=yaml.safe_load(files['planner'].read_text(encoding='utf8'))['DWAPlannerROS']
    client=files['dwa_client'].read_text(encoding='utf8')
    minimum=re.search(r'msg->twist\.linear\.x<(-[\d.]+)',client)
    maximum=re.search(r'msg->twist\.linear\.x>([\d.]+)',client)
    angular=re.search(r'std::abs\(msg->twist\.angular\.z\)>([\d.]+)',client)
    bridge=re.search(r'not (-[\d.]+) <= values\[0\] <= ([\d.]+) or abs\(values\[1\]\) > ([\d.]+)',files['bridge'].read_text(encoding='utf8'))
    if not all((minimum,maximum,angular,bridge)):
        raise ValueError('Real XY acceptance limits changed; inspect source before exporting limits')
    auto={'max_v':min(float(args['speed']),float(planner['max_vel_x']),round(float(maximum[1]),3),round(float(bridge[2]),3)),
          'reverse_v':min(abs(float(planner['min_vel_x'])),round(abs(float(minimum[1])),3),round(abs(float(bridge[1])),3)),
          'max_w':min(float(planner['max_vel_theta']),round(float(angular[1]),3),round(float(bridge[3]),3)),
          'linear_acceleration':min(float(args['linear_acceleration']),float(planner['acc_lim_x'])),
          'angular_acceleration':min(float(args['angular_acceleration']),float(planner['acc_lim_theta']))}
    guard=files['footprint'].read_text(encoding='utf8')
    def constant(name):
        match=re.search(r'\b'+re.escape(name)+r'=([\d.]+)',guard)
        if not match:raise ValueError('Missing real footprint constant '+name)
        return float(match[1])
    auto.update(reaction_time=constant('avoidance_reaction_time'),linear_deceleration=constant('avoidance_linear_deceleration'),
                angular_deceleration=float(re.search(r'std::abs\(w\)/([\d.]+)',guard)[1]),
                guard_half_length=constant('body_half_length')+constant('body_length_margin'),
                guard_half_width=constant('body_half_width')+constant('body_width_margin'))
    manual={'max_v':float(args['manual_speed']),'max_w':float(args['manual_angular_speed']),
            'linear_acceleration':float(args['manual_linear_acceleration']),'angular_acceleration':float(args['angular_acceleration'])}
    return {'automatic':auto,'manual':manual,
            'dwa_base':{key:planner[key] for key in ('sim_time','path_distance_bias','occdist_scale')},
            'sources':{k:{'file':str(p.relative_to(root)).replace('\\','/'),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for k,p in files.items()}}


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(derive(args.root),ensure_ascii=True,indent=2))
