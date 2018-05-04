'''
.. module:: robot
   :synopsis: Module defining an abstract robot class

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    The module defines an abstract robot class.
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

class Robot(object):
    '''Base class for a robot.'''

    def __init__(self, name, initConf=None, cspace=None, wspace=None,
                 controlspace=None, sensor=None, dynamics=None):
        self.name = name

        self.initConf = initConf # initial position
        self.currentConf = self.initConf # current position
        self.sensor = sensor # set sensor

        self.cspace = cspace
        self.wspace = wspace
        self.controlspace = controlspace
        self.dynamics = dynamics

        self.localObst = None # local obstacle symbol

    def steer(self):
        raise NotImplementedError

    def sample(self, local=False):
        ''' Generate a sample in the configuration space (global) or local
        within the sensing area.
        '''
        if local:
            return self.sensor.sensingShape.sample()
        return self.cspace.getSample()

    def move(self, conf):
        '''Moves the robot to the given configuration.'''
        self.currentConf = conf

    def getSymbols(self, position, local=False):
        raise NotImplementedError

    def isSimpleSegment(self, u, v):
        raise NotImplementedError
