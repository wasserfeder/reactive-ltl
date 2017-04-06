'''
Base class for finite abstraction models.

.. moduleauthor:: Alphan Ulusoy <alphan@bu.edu>
                  Cristian Ioan Vasile <cvasile@bu.edu>
'''
'''
    Abstract class describing a graphical model.
    Copyright (C) 2014-2016  Alphan Ulusoy <alphan@bu.edu>
                             Cristian Ioan Vasile <cvasile@bu.edu>
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


import itertools as it

import networkx as nx

# always use safe load
from yaml import safe_load as load, dump, YAMLObject
try: # try using the libyaml if installed
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError: # else use default PyYAML loader and dumper
    from yaml import Loader, Dumper


class Model(YAMLObject):
    '''
    Base class for finite abstraction system models.
    '''

    def __init__(self, name='Unnamed system model'):
        '''
        Empty Model object constructor
        '''
        self.name = name
        self.init = set()
        self.final = set()
        self.g = nx.DiGraph()
        self.cnt = it.count()
    
    def size(self):
        '''
        Returns the number of states and transitions of the model.
        '''
        nedges = sum(it.imap(len, self.g.adj.itervalues()))
        return self.g.number_of_nodes(), nedges
        
    def addState(self, key, propositions=None, init=False, final=False, **kwargs):
        '''TODO:
        Adds a state to the model.
        Adds an initial state to the model.
        Adds a final state to the model.
        
        Note
        ----
        If there is more than one initial state then the model is
        non-deterministic.
        '''
        # create custom label
        label = kwargs.get('label', '\n'.join([str(key), str(propositions)]))
        self.g.add_node(key, prop=propositions, label=label,
                        order=self.cnt.next(), **kwargs)
        if init:
            self.init.add(key)
        if final:
            self.final.add(key)
    
    def addStates(self, states):
        '''
        Adds the given states to the model.
        
        Note: states is a sequence type of 2-tuples of the form
        (key, kwargs), where kwargs are keyword arguments, which must include
        the propositions parameter.
        '''
        [self.addState(key, **kwargs) for key, kwargs in states]
    
    def addTransitions(self, transitions):
        '''
        Adds the given transitions to the model.
        '''
        self.g.add_edges_from(transitions)
    
    def visualize(self):
        '''
        Visualizes a model.
        # TODO: improve matplotlib visualization feature
        '''
        #nx.view_pygraphviz(self.g)
        #nx.view_pygraphviz(self.g, 'weight')
        nx.draw_networkx(self.g)
    
    @classmethod
    def load(cls, filename):
        '''
        TODO: create representer and constructor; test
        '''
        with open(filename, 'r') as fin:
            return load(fin, Loader=Loader)
    
    def save(self, filename):
        '''
        TODO: create representer and constructor; test
        '''
        with open(filename, 'w') as fout:
            dump(self, fout, Dumper=Dumper)
