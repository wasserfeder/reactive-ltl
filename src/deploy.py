#!/usr/bin/env python
'''
.. module:: deploy.py
   :synopsis: TODO:

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

license_text='''
    TODO:
    Copyright (C) 2018  Cristian Ioan Vasile <cvasile@bu.edu>
    Hybrid and Networked Systems (HyNeSs) Group, BU Robotics Laboratory,
    Boston University

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import os

from spaces.base import ConfigurationSpace, line_translate, Point
from spaces.maps_nd import BoxBoundary, BoxRegion
from robots import BaxterRobot

import rospy
from visualization_msgs.msg import *
from geometry_msgs.msg import Point as RosPoint
import time

def visualize_policy(policy):
    policy_marker_pub = rospy.Publisher('/policy_marker', Marker, queue_size=1)
    policy_marker_line_strip_pub = rospy.Publisher('/policy_marker_line', Marker, queue_size=1)
    
    traj_marker = Marker()
    traj_marker.type = Marker.SPHERE_LIST
    traj_marker.action = Marker.ADD
    traj_marker.header.frame_id = "/world"
    traj_marker.header.stamp = rospy.Time(0)
    
    
    traj_marker_line = Marker()
    traj_marker_line.type = Marker.LINE_STRIP
    traj_marker_line.action = Marker.ADD
    traj_marker_line.header.frame_id = "/world"
    traj_marker_line.header.stamp = rospy.Time(0)
  
    
    ### for test
    policy_list = []
    for i in range(100):
        policy_list.append([i, i, i])
    ###
        
    for p in policy_list:
        pt = RosPoint()
        pt.x = p[0]
        pt.y = p[1]
        pt.z = p[2]
        traj_marker.points.append(pt)
        traj_marker_line.points.append(pt)
        
    traj_marker.scale.x = 0.2
    traj_marker.color.r = 0
    traj_marker.color.g = 0
    traj_marker.color.b = 1
    traj_marker.color.a = 1.0
    traj_marker.id = 3
    
    traj_marker_line.scale.x = 0.01
    traj_marker_line.color.r = 1
    traj_marker_line.color.g = 0
    traj_marker_line.color.b = 0
    traj_marker_line.color.a = 1.0
    traj_marker_line.id = 2
    
    for i in range(10000):
      policy_marker_pub.publish(traj_marker)
      policy_marker_line_strip_pub.publish(traj_marker_line)
      time.sleep(0.1)
      
def load_policy(logfilename):
    '''Parses log file and returns the stored global policy.'''

    with open(logfilename, 'r') as logfile:
        policy = None
        for line in logfile:
            prefix, line_data = line.split('--')
            if prefix.lower().rfind('info') >= 0:
                if line_data.lower().find('global policy') >= 0:
                    policy = eval(line_data)['global policy']
                    break
    return policy

def execute_policy(policy):
    '''Executes the given policy.'''
    # define boundary in 6-dimensional configuration space of the Baxter's arm
    # the joint limits are taken from the Baxter's manual
    boundary = BoxBoundary([[-1.69, 1.69], [-2.1, 1.], [-3., 3.], [0., 2.6],
                            [-3., 3.], [-1.5, 2.]])
    # create robot's configuration space
    cspace = ConfigurationSpace(boundary=boundary)

    # initial configuration
    init_conf = Point([0, 0, 0, 0, 0, 0]) # TODO: load from logfile
    # create robot object
    filename = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                            'robots', 'baxter_robot', 'env_config.json')
    assert os.path.isfile(filename), 'Json environment file not found!'
    robot = BaxterRobot(name="baxter", init=init_conf, cspace=cspace,
                        stepsize=.99, config={'json-filename': filename})

    robot.move(init_conf) # make sure to start from the initial configuration
    prefix, suffix = policy
    for conf in prefix + suffix:
        robot.move(conf)

if __name__ == '__main__':
  logfile = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                         '..', 'data_ijrr', 'example3', 'ijrr_example_3.log')
  policy = load_policy(logfile)
  # print policy
  # execute_policy(policy)


  # print(type(policy))
  # rospy.init_node("test")
  # visualize_policy(policy)