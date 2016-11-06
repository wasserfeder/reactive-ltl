'''
.. module:: samplers
   :synopsis: Module defining metric space sampling algorithms.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''
'''
    Module defining metric space sampling algorithms.
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

__all__ = ['box_uniform_sampler']

from numpy import tile
from numpy.random import uniform

#-------------
def sample():
    '''
    Samples uniformly inside the space's boundary.
    # TODO: consider other pdf for sampling. abstract out functionality
      
    Examples: TODO:
    ---------
    >>> world = PolygonMap(Boundary(Polygon([(-1, -1), (1, -1), (1, 1), (-1, 1)]), (0, 0, 0)))
    >>> world.limits
    (-1.0, -1.0, 1.0, 1.0)
    >>> all([world.boundary.polygon.intersects(world.sample()) for _ in range(100)])
    True
    >>> world = PolygonMap(Boundary(Polygon([(0, -1), (1, 0), (0, 1), (-1, 0)]), (1, 1, 1)))
    >>> world.limits
    (-1.0, -1.0, 1.0, 1.0)
    >>> world.boundary.color
    (1, 1, 1)
    >>> all([world.boundary.polygon.intersects(world.sample()) for _ in range(100)])
    True
    '''
    space = []
    return Point(map(lambda lw, hg: uniform(lw, hg), *space.boundary))
#--------------


def box_uniform_sampler_iter(low, high, N=10000):
    '''TODO: Experimental -- should be more efficient???'''
    samples = iter([])
    dimension = len(low)
    tlow = tile(low, (N, 1))
    tdiff = tile(high - low, (N, 1))
     
    while True:
        try:
            yield next(samples)
        except StopIteration:
            samples = iter(tlow + uniform(size=(N, dimension)) * tdiff)

def box_uniform_sampler(low, high):
    '''Returns a sample from a uniform distribution over a box (hypercube).'''
    return low + uniform(size=len(low)) * (high - low)

def ball_uniform_sampler(center, radius):
    '''Returns a sample from a uniform distribution over a ball.'''
    return None # TODO: 

if __name__ == '__main__':
    # TODO: test samplers
    pass