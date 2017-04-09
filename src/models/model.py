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

# TODO: always use safe load
from yaml import load, dump #, safe_load as load
try: # try using the libyaml if installed
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError: # else use default PyYAML loader and dumper
    from yaml import Loader, Dumper


class Model(object):
    '''Base class for finite abstraction system models.'''
    
    yaml_tag = u'!Model'

    def __init__(self, name='Unnamed system model'):
        '''Empty Model object constructor.'''
        self.name = name
        self.init = set()
        self.final = set()
        self.g = nx.DiGraph()
        self.cnt = it.count()
    
    def size(self):
        '''Returns the number of states and transitions of the model.'''
        nedges = sum(it.imap(len, self.g.adj.itervalues()))
        return self.g.number_of_nodes(), nedges
    
    def add_state(self, key, propositions=None, init=False, final=False,
                  **kwargs):
        '''Adds a state to the model, and marks it as initial and final as
        specified.
        
        Note
        ----
        If there is more than one initial state then the model is
        non-deterministic.
        '''
        self.g.add_node(key, prop=propositions,
                        order=self.cnt.next(), **kwargs)
        if init:
            self.init.add(key)
        if final:
            self.final.add(key)
    
    def add_states(self, states):
        '''
        Adds the given states to the model.
        
        Note: states is a sequence type of 2-tuples of the form
        (key, kwargs), where kwargs are keyword arguments, which must include
        the propositions parameter.
        '''
        [self.add_state(key, **kwargs) for key, kwargs in states]
    
    def add_transitions(self, transitions):
        '''Adds the given transitions to the model.'''
        self.g.add_edges_from(transitions)
    
    def visualize(self):
        '''Visualizes a model.'''
        #nx.view_pygraphviz(self.g)
        #nx.view_pygraphviz(self.g, 'weight')
        nx.draw_networkx(self.g)
    
    @classmethod
    def load(cls, filename):
        '''Load model from file in YAML format.'''
        with open(filename, 'r') as fin:
            return load(fin, Loader=Loader)
    
    def save(self, filename):
        '''Save the model to file in YAML format.'''
        with open(filename, 'w') as fout:
            dump(self, fout, Dumper=Dumper)


def model_representer(dumper, model):
    '''YAML representer for a model object.
    Note: it uses the object's yaml_tag attribute as its YAML tag.
    '''
    return dumper.represent_mapping(tag=model.yaml_tag, mapping={
        'name'  : model.name,
        'init'  : list(model.init),
        'final' : list(model.final),
        'graph' : {
            'nodes' : dict(model.g.nodes(data=True)),
            'edges' : model.g.edges(data=True)
            }
        })

def model_constructor(loader, node, ModelClass):
    '''Model constructor from YAML document.
    Note: Creates an object of class ModelClass.
    '''
    data = loader.construct_mapping(node, deep=True)
    name = data.get('name', 'Unnamed')
    
    model = ModelClass(name)
    model.init = set(data.get('init', []))
    model.final = set(data.get('final', []))
    model.g.add_nodes_from(data['graph'].get('nodes', dict()).iteritems())
    model.g.add_edges_from(data['graph'].get('edges', []))
    
    if model.g.number_of_nodes():
        max_order = max([d['order'] for _, d in model.g.nodes_iter(data=True)])
    else:
        max_order = -1
    model.cnt = it.count(start=max_order + 1)
