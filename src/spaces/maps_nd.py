'''
.. module:: maps_nd
   :synopsis: Module defining the classes for handling n-dimensional Euclidean
   spaces.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    The module defines classes for handling n-dimensional Euclidean spaces.
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
from numpy.random import uniform, normal
from scipy.spatial.distance import euclidean, sqeuclidean

from base import Boundary, Point, Region


__all__ = ['BallBoundary', 'BoxBoundary', 'BallRegion', 'BoxRegion']


class BoxBoundary(Boundary):
    '''Defines an n-dimensional hyper-rectangular boundary.

    Note: Assumes Euclidean space.
    '''
    def __init__(self, ranges):
        Boundary.__init__(self)

        self.ranges = array(ranges, dtype=np.double)
        assert self.ranges.shape[1] == 2
        assert all(self.ranges[:, 0] <= self.ranges[:, 1])
        self.dimension = self.ranges.shape[0]

        self._hash = hash(tuple(self.ranges.flat))

    def volume(self):
        return np.prod(self.ranges[:, 1] - self.ranges[:, 0])

    def boundingBox(self):
        return self.ranges

    def sample(self):
        low, high = self.ranges.T
        return Point(low + uniform(size=self.dimension) * (high - low))

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
            u = zeros((self.dimension,))
            v = ones((self.dimension,))

            # indices i where src[i] == dest[i]
            indices = np.abs(diff) < np.finfo(float).eps
            if (np.any(self.ranges[indices, 0] > src.coords[indices])
                      or np.any(src.coords[indices] > self.ranges[indices, 1])):
                return False

            indices = ~indices # indices i where src[i] != dest[i]
            u[indices] = (low[indices] - src.coords[indices])/diff[indices]
            v[indices] = (high[indices] - src.coords[indices])/diff[indices]
            indices = indices & (diff < 0) # indices that require sign change
            u[indices], v[indices] = v[indices], u[indices]

            # add extra constraint to [0, 1]
            return max(np.max(u), 0) <= min(np.min(v), 1)

        return (np.all(self.ranges[:, 0] <= src.coords)
                and np.all(src.coords <= self.ranges[:, 1]))

    def contains(self, src, dest):
        '''Returns True if the line segment from src to dest in contained in the
        box boundary.
        '''
        return self.intersects(src) and self.intersects(dest)

    def coordRange(self, i):
        '''Returns the range of the i-th coordinate.'''
        return self.ranges[i]

    def translate(self, v):
        '''Translates the object.'''
        v = np.asarray(v)
        self.ranges[:, :] = (self.ranges.T + v).T

    def __eq__(self, other):
        return np.all(self.ranges == other.ranges)

    def __repr__(self):
        return 'BoxBoundary(ranges={0})'.format(map(list, self.ranges))


class BoxRegion(BoxBoundary, Region):
    '''Defines a labeled box region in a n-dimensional workspace.

    Note: Assumes Euclidean space.
    '''
    def __init__(self, ranges, symbols):
        Region.__init__(self, symbols)
        BoxBoundary.__init__(self, ranges)

    def outputWord(self, traj):
        raise NotImplementedError

    def __repr__(self):
        return 'BoxRegion(ranges={0}, symbols={1})'.format(
                                        map(list, self.ranges), self.symbols)

    __str__ = __repr__


class BallBoundary(Boundary):
    '''
    Defines a labeled ball region in a n-dimensional workspace.

    Note: Assumes Euclidean space.
    '''

    def __init__(self, center, radius):
        Boundary.__init__(self)

        self.center = array(center, dtype=np.double).flatten()
        self.radius = float(radius)
        self.dimension = len(self.center)

        self._hash =  self._hash = hash(tuple(self.center) + (self.radius, ))

    def volume(self):
        raise NotImplementedError

    def boundingBox(self):
        return array([self.center - self.radius, self.center + self.radius]).T

    def sample(self):
        r = self.radius * uniform()
        x = normal(size=self.dimension)
        return Point(r * x / np.linalg.norm(x))

    def intersects(self, src, dest=None):
        '''
        It dest is None then it returns true if src is inside the region,
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
            u = (dest - src)
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

    def translate(self, v):
        '''Translates the object.'''
        self.center += np.asarray(v)

    def __eq__(self, other):
        return np.all(self.center == other.center) and (self.radius == other.radius)

    def __repr__(self):
        return 'BallBoundary(center={0}, radius={1})'.format(list(self.center),
                                                             self.radius)


class BallRegion(BallBoundary, Region):
    '''
    Defines a labeled ball region in a n-dimensional workspace.

    Note: Assumes Euclidean space.
    '''

    def __init__(self, center, radius, symbols):
        Region.__init__(self, symbols)
        BallBoundary.__init__(self, center, radius)

    def outputWord(self, traj):
        raise NotImplementedError

    def __repr__(self):
        return 'BallRegion(center={0}, radius={1}, symbols={2})'.format(
                list(self.center), self.radius, self.symbols)

    __str__ = __repr__


if __name__ == '__main__':
    import doctest
    doctest.testmod()
