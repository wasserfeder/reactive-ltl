#!/usr/bin/env python
'''
.. module:: fullyActuatedRobot
   :synopsis: Module implements a fully actuated robot (an integrator).

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    Module implements a fully actuated robot (an integrator).
    Copyright (C) 2014-2016  Cristian Ioan Vasile <cvasile@bu.edu>
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

from robot import Robot

import rospy
from geometry_msgs.msg import Twist


class FullyActuatedRobot(Robot):
    '''
    Class representing a fully actuated robot.
    '''
    def __init__(self, name, init=None, wspace=None, stepsize=1):
        Robot.__init__(self, name, initConf=init, cspace=wspace, wspace=wspace,
                       controlspace=stepsize)
        self.isSetup = False
        self.origin = None
    
    def setup(self):
        ground = 'X80Pro1' #TODO:
        self.grd_pub = rospy.Publisher('GroundCommand{}'.format(ground), Twist,
                                  queue_size=20)
        # initialize node
        rospy.init_node('ReLTL_commander', anonymous=True)
        self.rate = rospy.Rate(0.1) # hz
        # create message object
        self.grd_msg = Twist()
        # mark as set up
        self.isSetup = True
    
    def move(self, conf):
        if self.isSetup:
            if rospy.is_shutdown():
                return
            print '[move]', conf.y, -conf.x, self.origin
            self.grd_msg.linear.x, self.grd_msg.linear.y = self.origin.y + conf.y, self.origin.x + conf.x
            self.grd_msg.linear.z = 0
            self.grd_msg.angular.x, self.grd_msg.angular.y, self.grd_msg.angular.z = 0, 0, 0
            self.grd_pub.publish(self.grd_msg)
            print '[move]', self.grd_msg
            print
            self.rate.sleep()
            conf = None # TODO: read from optitrack data
        super(FullyActuatedRobot, self).move(conf)
    
    def sense(self):
        if self.isSetup:
            #TODO: get locally sensed events from external node
            raise NotImplementedError
    
    def getSymbols(self, position, local=False):
        return self.wspace.getSymbols(position, local)
    
    def steer(self, start, target, atol=0):
        '''Returns a position that the robot can move to from the start position
        such that it steers closer to the given target position using the
        robot's dynamics.
        
        Note: It simulates the movement.
        
        Examples: TODO: modify
        ---------
        >>> from spaces.maps2d import Point2D
        >>> robot = FullyActuatedRobot(stepsize=1.0)
        >>> p = robot.steer(Point2D(1, 2), Point2D(5, 5))
        >>> (round(p.x, 1), round(p.y, 1))
        (1.4, 2.3)
        >>> robot.limit = 6
        >>> p = robot.steer(Point2D(1, 2), Point2D(5, 5))
        >>> (int(p.x), int(p.y))
        (5, 5)
        '''
        s = start.coords
        t = target.coords
        dist = self.cspace.metric(s, t)
        if dist <= self.controlspace + atol:
            return target
        return self.initConf.__class__(s + (t-s) * self.controlspace/dist)
    
    def isSimpleSegment(self, u, v):
        '''Returns True if the curve [x, y] = H([u, v]) in the workspace space
        does not intersect other regions than the ones x and y belong to.
        In the definition, H is a submersion mapping the line segment [u, v] in
        the configuration space to a curve [x, v] in the workspace, where the
        trajectory is annotated with the properties of the regions it
        intersects.
        Since the robot is a fully actuate points, the configuration space is
        the same as the workspace and the submersion is the identity map.
        
        Note: assumes non-overlapping regions. 
        '''
        nrRegUV = len(self.cspace.intersectingRegions(u, v))
        regU = self.cspace.intersectingRegions(u)
        regV = self.cspace.intersectingRegions(v)
#         print '[simple] regU:', regU
#         print '[simple] regV:', regV
#         print '[simple] regUV:', self.cspace.intersectingRegions(u, v)
#         print '[simple] nrRegUV:', nrRegUV
#         print
        
        # if both endpoints of the segment are in the free space
        if (not regU) and (not regV):
            return nrRegUV == 0
        # if one endpoint is in the free space
        if (not regU) or (not regV): # FIXME: assumes convex regions
            return nrRegUV == 1
        # if both endpoints are in the same region
        if regU[0] == regV[0]:
            # and regU.contains(u, v): # TODO: uncomment for non-convex regions
            return nrRegUV == 1
        # if the endpoints belong to two different regions
        return nrRegUV == 2
    
    def collision_free(self, plan):
        '''#TODO:
        '''
        if self.isSetup:
            raise NotImplementedError
        else:
            aux = [self.robot.currentConf] + self.local_plan
            for start, stop in zip(aux[:-1], aux[1:]):
                reqs = self.wspace.intersectingRegions(start, stop, local=True)
                if self.localObst in reqs:
                    return False
            return True
    
    def collision_free_segment(self, u, v):
        '''TODO:
        '''
        if self.isSetup:
            raise NotImplementedError
        else:
            pass
    
    def __str__(self):
        return 'Fully actuated robot'


if __name__ == '__main__':
    import doctest
    doctest.testmod()