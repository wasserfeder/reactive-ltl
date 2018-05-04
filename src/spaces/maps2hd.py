'''
.. module:: maps2hd
   :synopsis: Module defining the classes for handling elevation maps,
   2.5-dimensional Euclidean spaces.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    The module defines classes for handling elevation maps.
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
from numpy import array, ones, zeros

import shapely.geometry as geom

from base import Region


__all__ = ['BoxRegion2hD', 'PolygonRegion2hD']


class BoxRegion2hD(Region):
    '''
    Defines a labeled box region in a 2.5-dimensional (elevation) workspace.

    Note: Assumes Euclidean space.
    '''
    def __init__(self, ranges, elevation, symbols):
        super().__init__(self, symbols)

        self.ranges = array(ranges)
        assert self.ranges.shape == (2, 2)
        self.elevation = elevation
        self.dimension = 2.5

        self._hash = hash(tuple(self.ranges.flat) + (elevation, ))

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
            low = array(tuple(self.ranges[:, 0]) + (0,))
            high = array(tuple(self.ranges[:, 1]) + (self.elevation,))
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
                and src.z <= self.elevation)

    def outputWord(self, traj):
        raise NotImplementedError

    def xrange(self):
        '''Returns the range of the x coordinate.'''
        return self.ranges[0]

    def yrange(self):
        '''Returns the range of the y coordinate.'''
        return self.ranges[1]

    def elevation(self):
        '''Returns the elevation.'''
        return self.elevation

    def __eq__(self, other):
        return self.ranges == other.ranges and self.elevation == other.elevation

    def __repr__(self):
        return 'BoxRegion(x={0}, y={1}, h={2})'.format(*(map(list, self.ranges)+
                                                       [self.elevation]))


class PolygonRegion2hD(Region):
    '''
    Defines a labeled polygon region in a 2.5-dimensional (elevation) workspace.

    Note: Assumes Euclidean space.
    '''
    def __init__(self, polygon, elevation, symbols):
        super().__init__(self, symbols)

        self.polygon = geom.Polygon(polygon)
        self.dimension = 2.5
        self._hash = hash(tuple(array(self.polygon.exterior.coords).flat) +
                          (elevation,))
        self.elevation = elevation

    def intersects(self, src, dest=None):
        '''
        It dest is None then it returns true if src is inside the region,
        otherwise it returns true if the line segment intersects the region.

        .. math ::

            s.h > \wedge d.h > h \rightarrow \emptyset

            s.h \leq \wedge d.h \leq h \rightarrow [s, d] \cap poly

            s.h \leq h \wedge d.h > h \rightarow d' \leftarrow P_h \cap [s, d]
            \rightarrow [s, d'] \cap poly

            s.h < h \wedge d.h \leq h \rightarow s' \leftarrow P_h \cap [s, d]
            \rightarrow [s', d] \cap poly

            p = s + \lambda (d-s) = [x, y, h]^T, \lambda \in [0, 1]

            p_3 = h \rightarrow \lambda = \frac{h - s_3}{d_3 - s_3}

            x = s_1 + \lambda (d_1 - s_1), y = s_2 + \lambda (d_2 - s_2)
        '''
        if dest:
            if src.z > self.elevation and dest.z > self.elevation:
                return False

            s = src.coords[:2]
            d = dest.coords[:2]

            if src.z > self.elevation:
                t = (self.elevation - src.z) / (dest.z - src.z)
                s = s + t * (d - s)
            if dest.z > self.elevation:
                t = (self.elevation - src.z) / (dest.z - src.z)
                d = s + t * (d - s)

            return self.polygon.intersect(geom.LineString(s, d))

        return (self.polygon.intersect(src.coords[:2])
                and src.z <= self.elevation)

    def outputWord(self, traj):
        raise NotImplementedError

    def __eq__(self, other):
        return self.polygon == other.polygon

    def __repr__(self):
        return 'PolygonRegion(polygon={0})'.format(map(list,
                                                  self.polygon.exterior.coords))


if __name__ == '__main__':
    import doctest
    doctest.testmod()
