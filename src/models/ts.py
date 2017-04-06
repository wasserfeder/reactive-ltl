'''
Class implementing a (un)weighted finite transition system.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    Class implementing a (un)weighted finite transition system. 
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

import collections

from model import Model

# TODO: add nearest neighbor search if possible
class TS(Model):
    '''
    Weighted Transition System.
    '''
    
    def __init__(self, nnAlgorithm):
        '''
        Constructor
        '''
        Model.__init__(self)
        
#         self.nn = nnAlgorithm(self)
    
    def statesWithProp(self, prop):
        '''
        Returns the states from the model with a given proposition set.
        
        Args:
            prop : proposition or set of proposition
        '''
        if not isinstance(prop, collections.Set):
            prop = set([prop])
        
        return set((state
                for state, data in self.g.nodes_iter(data=True)
                    if prop <= data.get('prop', set())))
    
#     def addState(self, key, propositions=None, **kwargs):
#         '''
#         Adds a state to the transition system.
#         '''
#         print '[TS.addState]', key
#         Model.addState(self, key, propositions=propositions, **kwargs)
#         self.nn.add(key)

if __name__ == '__main__':
    class NNTest(object):
        def __init__(self, ts): pass
        def add(self, key):pass

    ts = TS(NNTest)
    ts.addState(0, propositions=set(['a', 'b']))
    ts.addState(1, propositions=set(['a', 'c']), init=True)
    ts.addStates([(2, {'propositions' : set(['a', 'b', 'c'])}),
                  (3, {'propositions' : set()})])
    ts.addState(4, propositions=set(), final=True)
    print ts.init, ts.final
    