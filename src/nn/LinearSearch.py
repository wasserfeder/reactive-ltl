'''
.. module:: TODO:
   :synopsis: Module defining the real Euclidean vector state space.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''
'''
    TODO:
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

__all__ = ['LinearSearch']

#from numpy import array, zeros, argmin
from scipy.spatial.distance import sqeuclidean


# FIXME:
class LinearSearch(object):
    
    def __init__(self, ts, points=None, dimension=2, metric=sqeuclidean):
        '''
        Note
        ----
#         Silently ignores dimension parameter if points are given.
        '''
        self.ts = ts
        
#         if points:
#             self.__points = array(points)
#             self.dimension = self.__points.shape[1]
#             self.dimension = 
#         else:
#             self.dimension = dimension
#             self.__points = zeros((0, dimension))
        
        self.dimension = dimension
        self.metric = metric
    
    def add(self, point):
        '''Adds a point to the structure.'''
#         self.__points = vstack(self.__points, point)
        pass
    
    def kNearestQuery(self, x, k=1):
        '''Query the structure for k nearest neighbors.'''
        pass
#         distances = array([norm(p-x, ord=self.norm) for p in self.__points])
#         distances = array([self.metric(p,x) for p in self.ts.g.nodes_iter()])
#         nearest = zeros((k, self.dimension))
#         for i in range(k):
#             idx = argmin(distances)
#             distances[idx] = float('inf')
#             nearest[i] = self.__points[idx]
#         
#         return nearest
    
    def ballTest(self, p, eta):
        '''
        Tests whether there are points within a ball of radius eta around p.
        '''
        pass
#         return any([norm(p-x, ord=self.norm) < eta
#                                                for x in self.ts.g.nodes.iter()])
#         return any([norm(p-x, ord=self.norm) < eta for x in self.__points])
    
    def ballQuery(self, p, eta):
        '''
        Return the set of points which fall within the the ball or radius eta
        around p.
        '''
        pass
#         return array([x for x in self.ts.g.nodes_iter()
#                                              if norm(p-x, ord=self.norm) < eta])
#         return array([x for x in self.__points if norm(p-x, ord=self.norm) < eta])

if __name__ == '__main__':
    pass