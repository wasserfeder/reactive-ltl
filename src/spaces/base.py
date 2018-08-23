'''
.. module:: base
   :synopsis: Module defining the classes for handling n-dimensional metric
   spaces.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    The module defines classes for defining n-dimensional metric spaces
    (workspace, configuration space, control space).
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

import itertools
import types
from collections import namedtuple

import numpy as np
from numpy import array
from numpy.linalg import norm
from scipy.spatial.distance import euclidean

__all__ = ['Point', 'State', 'Configuration',
           'Region', 'Boundary', 'RealMetricSpace',
           'Workspace', 'StateSpace', 'ConfigurationSpace', 'ControlSpace']

BasePoint = namedtuple('Point', ['coords', 'hash'])

class Point(BasePoint):
    '''Representation of a point in an n-dimensional real metric space.
    The point is represented as NumPy array.

    .. see also:: http://docs.scipy.org/doc/numpy/reference/index.html

    Example
    -------
    >>> p = Point(array([1, 2, 3]), copy=False)
    >>> print p
    Point(coords=array([1, 2, 3]), hash=-378539185)
    >>> print p.coords.flags
    C_CONTIGUOUS : True
    F_CONTIGUOUS : True
    OWNDATA : True
    WRITEABLE : True
    ALIGNED : True
    UPDATEIFCOPY : False
    >>> print p.hash, hash(p)
    -378539185 -378539185
    >>> d = {p:1}
    >>> print d[p]
    1
    >>> print d[(1, 2, 3)]
    1
    '''
    __slots__ = ()

    @staticmethod
    def __new__(cls, coords, copy=True):
        if copy:
            coords = array(coords)
            coords.setflags(write=False)
        assert isinstance(coords, np.ndarray)

        return super(Point, cls).__new__(cls, coords, hash(tuple(coords)))

    def __len__(self):
        return len(self.coords)

    def __ne__(self, other):
        if type(other) != Point:
            return not np.array_equal(self.coords, other)
        return not np.array_equal(self.coords, other.coords)

    def __eq__(self, other):
        if type(other) == types.IntType: # equality by hash value
            return self.hash == other
        if isinstance(other, Point): # equality to a Point
            return np.array_equal(self.coords, other.coords)
        return np.array_equal(self.coords, other) # equality to an iterator type

    def __lt__(self, other):
        return self.hash < other.hash
    def __leq__(self, other):
        return self.hash <= other.hash
    def __gt__(self, other):
        return self.hash > other.hash
    def __geq__(self, other):
        return self.hash >= other.hash

    def __repr__(self):
        return 'Point(coords={})'.format(tuple(self.coords))

    def __hash__(self):
        return self.hash


# Aliases
State = Point
'''Representation of an n-dimensional state vector.'''
Configuration = Point
'''Representation of an n-dimensional configuration.'''


class Boundary(object):
    '''Defines a boundary for a metric space.'''
    def __init__(self):
        self._hash = None

    def intersects(self, src, dest=None):
        raise NotImplementedError

    def volume(self):
        raise NotImplementedError

    def boundingBox(self):
        raise NotImplementedError

    def sample(self):
        raise NotImplementedError

    def __hash__(self):
        return self._hash


class Region(Boundary):
    '''Defines a labeled region.'''
    def __init__(self, symbols):
        Boundary.__init__(self)
        self.symbols = set(symbols)

    def outputWord(self, traj):
        raise NotImplementedError

    def __eq__(self):
        raise NotImplementedError

    def __str__(self):
        return 'Region(boundary={boundary}, symbols={symbols})'.format(
                boundary=None, symbols=tuple(self.symbols))

    __repr__ = __str__


class RealMetricSpace(object):
    '''Defines an n-dimensional real metric space. The metric should be given as
    a distance function.
    '''

    def __init__(self, dimension=2, boundary=None, metric=euclidean):
        '''Constructor'''
        # set boundary (may be undefined)
        self.boundary = boundary
        # set dimension
        self.dimension = dimension
        # set metric
        self.metric = metric
        self.dist = lambda x, y: self.metric(x.coords, y.coords)
        self.norm = lambda x: self.metric(x.coords, 0)

    def getDimensions(self):
        '''Returns the dimensions of the configuration space.

        Examples:
        ---------
        >>> space = RealMetricSpace()
        >>> space.getDimensions()
        2
        '''
        return self.dimension

    def getSample(self):
        if self.boundary:
            return self.boundary.sample()
        raise Exception('The space has no boundary!')

    def getVolume(self):
        '''Returns the volume delimited by the boundary.'''
        if self.boundary:
            return self.boundary.volume()
        return -1

    def __str__(self):
        return 'RealMetricSpace(dimension={dim})'.format(dim=self.dimension)


class Workspace(RealMetricSpace):
    '''Defines an n-dimensional real metric space with labeled regions. The
    metric should be given as a distance function.
    '''

    def __init__(self, dimension=2, boundary=None, metric=euclidean,
                 regions=None):
        '''Constructor'''
        RealMetricSpace.__init__(self, dimension, boundary, metric)

        # set regions
        if regions: # regions are static (immmutable)
            self.regions = set(regions)
        else:
            self.regions = set()

        # extract global symbols set
        symbols = (region.symbols for region in self.regions)
        self.symbols = set(itertools.chain.from_iterable(symbols))

    def getSample(self):
        return self.boundary.sample()

    def getSymbols(self, position=None):
        '''Returns the set of symbols of all regions that overlap the given
        position. If position is None, it returns all symbols from all regions.
        '''
        if not position:
            return self.symbols
        regions = (region.symbols for region in self.regions
                                                 if region.intersects(position))
        return set(itertools.chain.from_iterable(regions))

    def addRegion(self, region):
        '''Adds a region of interest to the workspace.'''
        if not isinstance(region, Region):
            raise TypeError('Expected Region variable!')
        self.regions.add(region)
        self.symbols |= region.symbols

    def removeRegion(self, region, update=False):
        '''Removes a region of interest from the workspace.'''
        if not isinstance(region, Region):
            raise TypeError('Expected Region variable!')
        self.regions.discard(region)
        if update:
            symbols = (region.symbols for region in self.regions)
            self.symbols = set(itertools.chain.from_iterable(symbols))

    def intersectingRegions(self, src, dest=None):
        '''Returns the regions which intersect the point or line.'''
        return [r for r in self.regions if r.intersects(src, dest)]

    def __str__(self):
        return 'Workspace(boundary={boundary}, regions={regions})'.format(
                    boundary=self.boundary, regions=self.regions)

    __repr__ = __str__


# Aliases for now
StateSpace = RealMetricSpace
#TODO: Configuration space must have sampling methods
'''Define a n-dimensional state space.'''
ConfigurationSpace = RealMetricSpace
'''Define a n-dimensional configurations space.'''
ControlSpace = RealMetricSpace
'''Define a n-dimensional control space.'''


def line_path(a, b, step, PointCls=Point):
    '''Generate straight path from point a to point b.'''
    assert issubclass(PointCls, Point)
    a = np.array(a.coords)
    b = np.array(b.coords)
    u = b - a
    dist = norm(u)
    points = [PointCls(a + k * u) for k in np.arange(0, 1, step/dist)]
    points.append(PointCls(b))
    return points

def line_translate(a, b, step):
    '''Generate straight path from point a to point b as sequence of translation
    vectors.
    '''
    a = np.asarray(a)
    b = np.asarray(b)
    u = b - a
    dist = norm(u)
    u = u / dist
    points = [step * u] * int(dist/step)
    points.append(u * (dist - step * len(points)))
    return points
