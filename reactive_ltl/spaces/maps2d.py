'''
.. module:: maps2d
   :synopsis: Module defining the classes for handling 2-dimensional Euclidean
   spaces.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    The module defines classes for handling 2-dimensional Euclidean spaces.
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

import numpy as np
from numpy import array, dot, ones, zeros
from numpy.random import uniform
from scipy.spatial.distance import euclidean, sqeuclidean

import shapely.geometry as geom
import shapely.affinity as affine

from base import Boundary, Point, Region


__all__ = ['Point2D', 'BoxBoundary2D', 'BoxRegion2D',
           'BallBoundary2D', 'BallRegion2D',
           'PolygonBoundary2D', 'PolygonRegion2D']


class Point2D(Point):
    '''Defines a 2D point.'''
    __slots__ = ()

    @property
    def x(self):
        '''x coordinate.'''
        return self.coords[0]

    @property
    def y(self):
        '''y coordinate.'''
        return self.coords[1]

    def __len__(self):
        return 2

    def __repr__(self):
        return 'Point2D(({0}, {1}))'.format(self.x, self.y)


class BoxBoundary2D(Boundary):
    '''Defines a 2D rectangle boundary.

    Note: Assumes Euclidean space.
    '''

    def __init__(self, ranges):
        Boundary.__init__(self)

        self.ranges = array(ranges, dtype=np.double)
        assert self.ranges.shape == (2, 2)
        assert all(self.ranges[:, 0] <= self.ranges[:, 1])
        self.dimension = 2

        self.polygon = None

        self._hash = hash(tuple(self.ranges.flat))

    def volume(self):
        return np.prod(self.ranges[:, 1] - self.ranges[:, 0])

    def boundingBox(self):
        return self.ranges

    def sample(self):
        low, high = self.ranges.T
        return Point2D(low + uniform(size=2)* (high - low))

    def intersects(self, src, dest=None):
        '''It dest is None then it returns true if src is inside the region,
        otherwise it returns true if the line segment intersects the region.

        .. math ::

            p = s + \lambda (d - s), p, s, d \in \mathbb{R}^n,
            \lambda \in [0, 1]

            p \in Box(l, h) \equiv p_i \in [l_i, h_i], i \in {1,\ldots, n}

            \equiv \lambda (d_i-s_i) \in [l_i - s_i, h_i - s_i],
            i \in {1,\ldots, n}

            \equiv \lambda \in [\lambda^{l}_i, \lambda^{u}_i],
            i \in {1,\ldots, n}

            s_i \neq d_i \rightarrow
            \lambda^{l}_i = \min(\frac{l_i - s_i}{d_i - s_i},
                                 \frac{h_i - s_i}{d_i - s_i}),
            \lambda^{u}_i = \max(\frac{l_i - s_i}{d_i - s_i},
                                 \frac{h_i - s_i}{d_i - s_i}),

            s_i = d_i \wedge s_i \in [l_i, h_i] \rightarrow
            \lambda^{l}_i = 0, \lambda^{u}_i = 1

            s_i = d_i \wedge s_i \notin [l_i, h_i] \rightarrow
            \lambda^{l}_i = 1, \lambda^{u}_i = 0

            \equiv \lambda \in
            \Lambda = (\bigcap_{i=1}^{n} [\lambda^{l}_i, \lambda^{u}_i]) \cap
            [0, 1]

            \Lambda \neq \emptyset \equiv
            \max(0, \max_{i=1,\ldots,n}(\lambda^{l}_i)) \leq
            \min(1, \min_{i=1,\ldots,n}(\lambda^{u}_i))
        '''
        if dest:
            diff = dest.coords - src.coords
            low, high = self.ranges.T
            # add extra dimension for the [0, 1] constraint
            u = zeros((3,))
            v = ones((3,))

            if abs(diff[0]) < np.finfo(float).eps: # constant along the x-axis
                if not (self.ranges[0, 0] <= src.x <= self.ranges[0, 1]):
                    return False
            else:
                u[0] = (low[0] - src.x)/diff[0]
                v[0] = (high[0] - src.x)/diff[0]
                if diff[0] < 0:
                    u[0], v[0] = v[0], u[0]

            if abs(diff[1]) < np.finfo(float).eps: # constant along the y-axis
                if not (self.ranges[1, 0] <= src.y <= self.ranges[1, 1]):
                    return False
            else:
                u[1] = (low[1] - src.y)/diff[1]
                v[1] = (high[1] - src.y)/diff[1]
                if diff[1] < 0:
                    u[1], v[1] = v[1], u[1]

            return np.max(u) <= np.min(v)

        return (self.ranges[0, 0] <= src.x <= self.ranges[0, 1]
                and self.ranges[1, 0] <= src.y <= self.ranges[1, 1])

    def contains(self, src, dest):
        '''Returns True if the line segment from src to dest in contained in the
        box boundary.
        '''
        return self.intersects(src) and self.intersects(dest)

    def toPolygon(self):
        '''Convert to list of points forming a polygon.'''
        if self.polygon is None:
            low, high = self.ranges.T
            self.polygon = geom.box(low[0], low[1], high[0], high[1])
        return self.polygon

    def translate(self, v):
        '''Translates the object.'''
        v = np.asarray(v)
        self.ranges[0] += v[0]
        self.ranges[1] += v[1]
        self.polygon = None

    def xrange(self):
        '''Returns the range of the x coordinate.'''
        return self.ranges[0]

    def yrange(self):
        '''Returns the range of the y coordinate.'''
        return self.ranges[1]

    def __eq__(self, other):
        if not isinstance(other, BoxBoundary2D):
            return False
        return np.all(self.ranges == other.ranges)

#     def __str__(self):
#         return 'BoxBoundary2D(x={0}, y={1})'.format(*map(list, self.ranges))

    def __repr__(self):
        return 'BoxBoundary2D(ranges={0})'.format(
                                    [list(self.xrange()), list(self.yrange())])

class BoxRegion2D(BoxBoundary2D, Region):
    '''Defines a labeled box region in a 2-dimensional workspace.

    Note: Assumes Euclidean space.
    '''
    def __init__(self, ranges, symbols):
        Region.__init__(self, symbols)
        BoxBoundary2D.__init__(self, ranges)

    def outputWord(self, traj):
        raise NotImplementedError

    def __repr__(self):
        return 'BoxRegion2D(ranges={0}, symbols={1})'.format(
                    [list(self.xrange()), list(self.yrange())], self.symbols)

    __str__ = __repr__


class BallBoundary2D(Boundary):
    '''Defines a ball boundary in a 2-dimensional workspace.

    Note: Assumes Euclidean space.
    '''

    def __init__(self, center, radius):
        Boundary.__init__(self)

        if len(center) != 2:
            raise ValueError("Center dimension does not match the space's dimension!")
        assert radius > 0

        self.center = array(center, dtype=np.double).flatten()
        self.radius = float(radius)
        self.dimension = 2

        self.polygon = None

        self._hash =  self._hash = hash(tuple(self.center) + (self.radius,))

    def volume(self):
        return np.pi*(self.radius**2)

    def boundingBox(self):
        return array([self.center - self.radius, self.center + self.radius]).T

    def sample(self):
        r = uniform(size=3)
        rr = r[0] + r[1]
        if rr > 1:
            rr = 2 - rr
        rad = self.radius * rr
        theta = 2 * np.pi * r[2]
        p = np.array([rad*np.cos(theta), rad*np.sin(theta)])
        return Point2D(self.center + p)

    def intersects(self, src, dest=None):
        '''It dest is None then it returns true if src is inside the region,
        otherwise it returns true if the line segment intersects the region.

        .. math ::

            w = x_{center} - x_0

            u = (x_1 - x_0)/norm(x_1 - x_0)

            d = norm(w - (w \cdot u) u)

            return \ d <= radius

        '''
        if isinstance(src, Point):
            src = src.coords

        if dest:
            if isinstance(dest, Point):
                dest = dest.coords

            w = self.center - src
            u = dest - src
            lambd = dot(w, u)/sqeuclidean(u, 0)
            lambd = min(max(lambd, 0), 1)
            dist = euclidean(w - lambd*u, 0)
            return dist <= self.radius
        return euclidean(self.center, src) <= self.radius

    def contains(self, src, dest):
        '''Returns True if the line segment from src to dest in contained in the
        ball boundary.
        '''
        return self.intersects(src) and self.intersects(dest)

    def toPolygon(self):
        '''Convert to list of points forming a polygon.'''
        if self.polygon is None:
            N = 36
            points = np.empty((N, 2))
            theta = np.linspace(0, 2*np.pi, N)
            points[:, 0] = self.center[0] + self.radius * np.cos(theta)
            points[:, 1] = self.center[1] + self.radius * np.sin(theta)
            self.polygon = geom.Polygon(points)
        return self.polygon

    def translate(self, v):
        '''Translates the object.'''
        self.center += np.asarray(v)
        if self.polygon:
            self.polygon = affine.translate(self.polygon, xoff=v[0], yoff=v[1])

    def __eq__(self, other):
        if not isinstance(other, BallBoundary2D):
            return False
        return np.all(self.center == other.center) and (self.radius == other.radius)

    def __repr__(self):
        return 'BallBoundary2D(center={0}, radius={1})'.format(
                list(self.center), self.radius)


class BallRegion2D(BallBoundary2D, Region):
    '''Defines a labeled ball region in a 2-dimensional workspace.

    Note: Assumes Euclidean space.
    '''

    def __init__(self, center, radius, symbols):
        Region.__init__(self, symbols)
        BallBoundary2D.__init__(self, center, radius)

    def outputWord(self, traj):
        raise NotImplementedError

    def __repr__(self):
        return 'BallRegion2D(center={0}, radius={1}, symbols={2})'.format(
                list(self.center), self.radius, self.symbols)

    __str__ = __repr__


class PolygonBoundary2D(Boundary):
    '''Defines a polygon boundary in a 2-dimensional workspace.

    Note: Assumes Euclidean space.
    '''

    def __init__(self, polygon):
        Boundary.__init__(self)

        self.polygon = geom.Polygon(polygon)
        self.dimension = 2
        self._hash = hash(tuple(array(self.polygon.exterior.coords).flat))

    def volume(self):
        return self.polygon.area()

    def boundingBox(self):
        return array(self.polygon.bounds()).reshape((2, 2), order='F')

    def intersects(self, src, dest=None):
        if dest:
            return self.polygon.intersects(geom.LineString((src.coords, dest.coords)))
        return self.polygon.intersects(geom.Point(src.coords))

    def contains(self, src, dest):
        '''Returns True if the line segment from src to dest in contained in the
        polygon boundary.
        '''
        return self.polygon.contains(geom.LineString(src, dest))

    def toPolygon(self):
        '''Convert to list of points forming a polygon.'''
        return self.polygon

    def translate(self, v):
        '''Translates the object.'''
        self.polygon = affine.translate(self.polygon, xoff=v[0], yoff=v[1])

    def __eq__(self, other):
        if not isinstance(other, PolygonBoundary2D):
            return False
        return self.polygon == other.polygon

    def __repr__(self):
        return 'PolygonBoundary2D(polygon={0})'.format(map(list,
                                                  self.polygon.exterior.coords))

class PolygonRegion2D(PolygonBoundary2D, Region):
    '''Defines a labeled polygon region in a 2-dimensional workspace.

    Note: Assumes Euclidean space.
    '''

    def __init__(self, polygon, symbols):
        Region.__init__(self, symbols)
        PolygonBoundary2D.__init__(self, polygon)

    def outputWord(self, traj):
        raise NotImplementedError

    def __repr__(self):
        return 'PolygonRegion2D(polygon={0}, symbols={1})'.format(
                map(list, self.polygon.exterior.coords), self.symbols)

    __str__ = __repr__


def expandRegion(region, epsilon, tolerance=0.01):
    if isinstance(region, BallRegion2D):
        return BallRegion2D(region.center, region.radius+epsilon, region.symbols)
    elif isinstance(region, (BoxRegion2D, PolygonRegion2D)):
        if isinstance(region, BoxRegion2D):
            low, high = region.ranges.T
            polygon = geom.Polygon([low, (low[0], high[1]), high, (high[0], low[1])])
        else:
            polygon = region.polygon
        polygon = polygon.buffer(epsilon).simplify(tolerance)
        return PolygonRegion2D(polygon.exterior.coords, region.symbols)
    else:
        raise TypeError(str(region))

def intersection(reg1, reg2):
    poly1 = reg1.toPolygon()
    poly2 = reg2.toPolygon()
    poly3 = poly1.intersection(poly2)

    if not poly3:
        return None
    return PolygonBoundary2D(poly3.exterior.coords)

if __name__ == '__main__':
    import doctest
    doctest.testmod()
