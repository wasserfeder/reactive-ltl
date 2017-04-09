'''
Package defining state spaces, workspaces and control spaces to be used by
planning algorithms.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''
'''
    State, work and control spaces package.
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

from base import *
from maps2d import *
from maps3d import *
from maps2hd import *
from maps_nd import *

# register yaml representers
try: # try using the libyaml if installed
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError: # else use default PyYAML loader and dumper
    from yaml import Loader, Dumper

def p2d_representer(dumper, p):
    return dumper.represent_mapping(tag=u'!Point2D',
                                    mapping={'x': float(p.x), 'y': float(p.y)})
Dumper.add_representer(Point2D, p2d_representer)
def p2d_constructor(loader, node):
    data = loader.construct_mapping(node)
    return Point2D([data['x'], data['y']])
Loader.add_constructor(u'!Point2D', p2d_constructor)
