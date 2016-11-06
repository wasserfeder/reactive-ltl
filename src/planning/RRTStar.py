'''
.. module:: RRTStar
   :synopsis: # TODO:

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''
'''
    #TODO: RRT*
    Copyright (C) 2013  Cristian Ioan Vasile <cvasile@bu.edu>
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

from planning import planner


class RRTStar(planner):
    '''
    classdocs # TODO: implement on-line planner
    '''

    def __init__(self):
        '''
        Constructor
        '''
        planner.__init__(self, 'RRTStar')
        
    def solve(self):
        raise NotImplementedError
    
    def clear(self):
        raise NotImplementedError
    
    def setup(self):
        raise NotImplementedError
    
    def check(self): # TODO: this method may not be useful
        raise NotImplementedError
    
    def status(self):
        raise NotImplementedError
    
    def __repr__(self): # TODO: pprint
        pass