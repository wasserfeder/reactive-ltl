'''
.. module:: sensors
   :synopsis: Module defining a robot sensors.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    The module defines a robot sensors.
    Copyright (C) 2016  Cristian Ioan Vasile <cvasile@bu.edu>
    Hybrid and Networked Systems (HyNeSs) Laboratory, Boston University

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
import numpy as np


class Sensor(object):
    '''
    classdocs
    '''

    def __init__(self, robot, sensingShape):
        '''
        Constructor
        '''
        self.robot = robot
        self.sensingShape = sensingShape # sensing shape
    
    def sense(self):
        raise NotImplementedError


class SimulatedSensor(Sensor):
    '''
    classdocs
    '''
    
    def __init__(self, robot, sensingShape, requests, obstacles):
        '''
        Constructor
        '''
        Sensor.__init__(self, robot, sensingShape)
        
        self.requests = requests
        self.obstacles = obstacles
    
    def sense(self):
        '''TODO:
        '''
        self.sensingShape.center = np.array(self.robot.currentConf.coords)
        requests = [r for r in self.requests
                            if self.sensingShape.intersects(r.center)]
        
        obstacles = [o for o in self.obstacles
            if (self.robot.cspace.dist(self.robot.sensingShape.center, o.center)
                                < self.robot.sensingShape.radius + o.radius)]
        
        return requests, obstacles
