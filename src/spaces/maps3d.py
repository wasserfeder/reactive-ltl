'''
.. module:: maps3d
   :synopsis: Module defining the classes for handling 3-dimensional Euclidean
   spaces.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    The module defines classes for handling 3-dimensional Euclidean spaces.
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

import numpy as np
from numpy import array, dot, ones, zeros
from scipy.spatial.distance import euclidean

from base import Point, Region


__all__ = ['Point3D', 'BoxRegion3D', 'BallRegion3D']


class Point3D(Point):
    '''
    Defines a 2D point.
    '''
    __slots__ = ()

    @property
    def x(self):
        '''x coordinate.'''
        return self.coords[0]
    @property
    def y(self):
        '''y coordinate.'''
        return self.coords[1]
    @property
    def z(self):
        '''z coordinate.'''
        return self.coords[2]

    def __len__(self):
        return 3


class BoxRegion3D(Region):
    '''
    Defines a labeled box region in a 3-dimensional workspace.

    Note: Assumes Euclidean space.
    '''
    def __init__(self, ranges, symbols):
        super().__init__(self, symbols)

        self.ranges = array(ranges)
        assert self.ranges.shape == (3, 2)
        self.dimension = 3

        self._hash = hash(tuple(self.ranges.flat))

    def intersects(self, src, dest=None):
        '''
        It dest is None then it returns true if src is inside the region,
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
            diff = dest - src
            low, high = self.ranges.T
            u = zeros((4,))
            v = ones((4,))

            if abs(diff[0]) < np.finfo(float).eps: # constant along the x-axis
                if not (self.ranges[0, 0] <= src.x <= self.ranges[0, 1]):
                    return False
            else:
                u[0] = (low[0] - src[0])/diff[0]
                v[0] = (high[0] - src[0])/diff[0]

            if abs(diff[1]) < np.finfo(float).eps: # constant along the y-axis
                if not (self.ranges[1, 0] <= src.y <= self.ranges[1, 1]):
                    return False
            else:
                u[1] = (low[1] - src[1])/diff[1]
                v[1] = (high[1] - src[1])/diff[1]

            if abs(diff[2]) < np.finfo(float).eps: # constant along the z-axis
                if not (self.ranges[2, 0] <= src.z <= self.ranges[2, 1]):
                    return False
            else:
                u[2] = (low[2] - src[2])/diff[2]
                v[2] = (high[2] - src[2])/diff[2]

            return np.max(u) <= np.min(v)

        return (self.ranges[0, 0] <= src.x <= self.ranges[0, 1]
                and self.ranges[1, 0] <= src.y <= self.ranges[1, 1]
                and self.ranges[2, 0] <= src.z <= self.ranges[2, 1])

    def outputWord(self, traj):
        raise NotImplementedError

    def xrange(self):
        '''Returns the range of the x coordinate.'''
        return self.ranges[0]

    def yrange(self):
        '''Returns the range of the y coordinate.'''
        return self.ranges[1]

    def zrange(self):
        '''Returns the range of the z coordinate.'''
        return self.ranges[2]

    def __eq__(self, other):
        return self.ranges == other.ranges

    def __repr__(self):
        return 'BoxRegion(x={0}, y={1}, z={2})'.format(*map(list, self.ranges))


class BallRegion3D(Region):
    '''
    Defines a labeled ball region in a 3-dimensional workspace.

    Note: Assumes Euclidean space.
    '''

    def __init__(self, center, radius, symbols):
        super().__init__(self, symbols)

        if len(center) != 3:
            raise ValueError("Center dimension does not match the space's dimension!")

        self.center = array(center).flatten()
        self.radius = float(self.radius)
        self.dimension = 3

        self._hash =  self._hash = hash(tuple(self.center) + (self.radius))

    def intersects(self, src, dest=None):
        '''
        It dest is None then it returns true if src is inside the region,
        otherwise it returns true if the line segment intersects the region.

        .. math ::

            w = x_{center} - x_0

            u = (x_1 - x_0)/norm(x_1 - x_0)

            d = abs(w - (w \cdot u) u)

            return \ d <= radius

        '''
        if dest:
            w = self.center - src.coords
            u = (dest.coords - src.coords)/euclidean(src.coords, dest.coords)
            dist = abs(w - dot(w, u)*u)
            return dist <= self.radius
        return euclidean(self.center, src.coords) <= self.radius

    def outputWord(self, traj):
        raise NotImplementedError

    def __eq__(self, other):
        return (self.center == other.center) and (self.radius == other.radius)

    def __repr__(self):
        return 'BallRegion(center={0}, radius={1})'.format(list(self.center),
                                                           self.radius)


if __name__ == '__main__':
    import doctest
    doctest.testmod()
