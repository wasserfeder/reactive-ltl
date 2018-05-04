'''
.. module:: sensors
   :synopsis: Module defining a robot sensors.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    The module defines a robot sensors.
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
import logging

import numpy as np

from spaces.maps2d import intersection


class Sensor(object):
    '''Base class for sensors.'''

    def __init__(self, robot, sensingShape):
        '''
        Constructor
        '''
        self.robot = robot
        self.sensingShape = sensingShape # sensing shape

    def sense(self):
        '''Sensing method that returns requests and local obstacles.'''
        raise NotImplementedError

    def update(self):
        '''Updates requests and local obstacles.'''

    def reset(self):
        '''Resets requests and local obstacles.'''


class SimulatedSensor(Sensor):
    '''Simulated ideal sensor.'''

    def __init__(self, robot, sensingShape, requests, obstacles):
        '''
        Constructor
        '''
        Sensor.__init__(self, robot, sensingShape)

        self.requests = requests
        self.obstacles = obstacles
        self.all_requests = requests

    def sense(self):
        '''Sensing method that returns requests and local obstacles.'''
        v = np.array(self.robot.currentConf.coords) - self.sensingShape.center
        self.sensingShape.translate(v)
        assert np.all(self.sensingShape.center == self.robot.currentConf.coords)

        requests = [r for r in self.requests
                      if self.sensingShape.intersects(r.region.center)]

        obstacles = [intersection(self.sensingShape, o)
                            for o in self.obstacles]
        obstacles = [o for o in obstacles if o is not None]
        logging.debug('Sensor:sense: sensed obstacles: %s', obstacles)

        return requests, obstacles

    def update(self):
        '''Updates requests and local obstacles.'''
        conf = self.robot.currentConf
        # remove serviced requests
        self.requests = [r for r in self.requests
                                            if not r.region.intersects(conf)]
        # move requests on their paths
        for r in self.requests:
            v = next(r.region.path)
            r.region.translate(v)

    def reset(self):
        '''Resets requests and local obstacles.'''
        self.requests = self.all_requests
