'''
.. module:: maps_nd
   :synopsis: Module defining the classes for handling n-dimensional Euclidean
   spaces.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    The module defines classes for handling n-dimensional Euclidean spaces.
    Copyright (C) 2014  Cristian Ioan Vasile <cvasile@bu.edu>
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
from numpy import array, dot, ones, zeros
from scipy.spatial.distance import euclidean

from base import Region


__all__ = ['BoxRegion', 'BallRegion']


class BoxRegion(Region):
    '''
    Defines a labeled box region in a n-dimensional workspace.
    
    Note: Assumes Euclidean space.
    '''
    def __init__(self, ranges, symbols):
        super().__init__(self, symbols)
        
        self.ranges = array(ranges)
        assert self.ranges.shape[1] == 2
        
        self.dimension = self.ranges.shape[0]
        
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
            u = zeros((self.dimension + 1,))
            v = ones((self.dimension + 1,))
            
            # indices i where src[i] == dest[i] 
            indices = np.abs(diff) < np.finfo(float).eps
            if not np.all(self.ranges[indices, 0] <= src[indices] <= self.ranges[indices, 1]):
                return False
            
            indices = ~indices # indices i where src[i] != dest[i]
            u[indices] = (low[indices] - src[indices])/diff[indices]
            v[indices] = (high[indices] - src[indices])/diff[indices]
            
            return np.max(u) <= np.min(v)
                    
        return np.all(self.ranges[:, 0] <= src <= self.ranges[:, 1])
    
    def outputWord(self, traj):
        raise NotImplementedError
    
    def coordRange(self, i):
        '''Returns the range of the i-th coordinate.'''
        return self.ranges[i]
    
    def __eq__(self, other):
        return self.ranges == other.ranges
    
    def __repr__(self):
        return 'BoxRegion(ranges={0})'.format(map(list, self.ranges))


class BallRegion(Region):
    '''
    Defines a labeled ball region in a n-dimensional workspace.
    
    Note: Assumes Euclidean space.
    '''
    
    def __init__(self, center, radius, symbols):
        super().__init__(self, symbols)
        
        self.center = array(center).flatten()
        self.radius = float(self.radius)
        self.dimension = len(self.center)
        
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
            dist = euclidean(w - dot(w, u)*u, 0)
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