'''
.. module:: cozmo_sensor
   :synopsis: Module defining a Cozmo robot's sensors.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    The module defines a robot sensors.
    Copyright (C) 2014-2019  Cristian Ioan Vasile <cvasile@bu.edu>
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
import time
import itertools as it
import logging

import numpy as np

import lcm

from sensors import Sensor
from planning import Request

from cozmo_messages import cube_msg_t
from cozmo_messages import poll_msg_t
from cozmo_messages import world_msg_t


class CozmoSensor(Sensor):
    '''Cozmo/OptiTrack sensor.'''

    def __init__(self, robot, sensingShape, requests, obstacles):
        '''Constructor'''
        Sensor.__init__(self, robot, sensingShape)

        self.requests = requests
        self.obstacles = obstacles
        self.all_requests = requests

        def world_reply_handler(channel, data):
            msg = world_msg_t.decode(data)
            print('World reply from cozmo:', list(msg.cubes),
                  list(msg.cube_light))
            # update requests position
            [r.region.translate(np.asarray(p[:2]) - r.region.center)
                        for r, p in it.izip(self.all_requests, msg.cube_pose)]
            # select visible and active requests (lights on and in camera view)
            self.requests = [r for idx, r in enumerate(self.all_requests)
                                if msg.cubes[idx] and msg.cube_light[idx] > 0]
            self.obstacles = [] #TODO:

        self.lc = lcm.LCM()
        self.lc.subscribe('WORLD', world_reply_handler)

        self.poll_msg = poll_msg_t()
        self.cube_msg = cube_msg_t()

    def sense(self):
        '''Sensing method that returns requests and local obstacles.'''
        v = np.array(self.robot.currentConf.coords) - self.sensingShape.center
        self.sensingShape.translate(v)
        assert np.all(self.sensingShape.center == self.robot.currentConf.coords)

        self.poll_msg.timestamp = int(time.time() * 1000000)
        self.poll_msg.what = 0 # world
        print('Send poll for world data...')
        self.lc.publish('POLL', self.poll_msg.encode())
        self.lc.handle()

        return self.requests, self.obstacles

    def update(self):
        '''Updates requests and local obstacles.'''
        conf = self.robot.currentConf

        for idx, r in enumerate(self.all_requests):
            if r.region.intersects(conf):
                self.cube_msg.timestamp = int(time.time() * 1000000)
                self.cube_msg.which = idx
                self.cube_msg.color = 0 # LIGHTS OFF
                self.lc.publish('CUBE', self.cube_msg.encode())

        # remove serviced requests
        self.requests = [r for r in self.requests
                                            if not r.region.intersects(conf)]

    def reset(self):
        '''Resets requests and local obstacles.'''
        self.requests = self.all_requests

        # reset color to original
        for idx, r in enumerate(self.all_requests):
            self.cube_msg.timestamp = int(time.time() * 1000000)
            self.cube_msg.which = idx
            self.cube_msg.color = r.region.cube_color # LIGHTS OFF
            self.lc.publish('CUBE', self.cube_msg.encode())
