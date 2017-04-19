'''
.. module:: incremental_product
   :synopsis: Module implements incremental product automata maintainace and
   incremental solution checking.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''
'''
    Module implements incremental product automata maintainace and incremental
    solution checking.
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

import networkx as nx

from lomap.algorithms.dijkstra import source_to_target_dijkstra

from models import Model, Buchi
from util.scc import scc, initNodes


class IncrementalProduct(Model):
    '''Product automaton between a weighted transition system and a static Buchi
    automaton with incremental updating. It maintains both reachability of final
    states and strongly connected components for quick model checking. 
    '''
    
    def __init__(self, globalSpec):
        '''Constructor'''
        Model.__init__(self)
        
        self.globalSpec = globalSpec # global specification
        # construct Buchi automaton from global LTL specification
        self.buchi = Buchi()
        self.buchi.from_formula(self.globalSpec)
        self.proj_ts = {} # TS states to Buchi states map
        # initialize SCC algorithm
        self.initSCC()
    
    def addInitialState(self, node, symbols):
        ''' Adds an initial node to the product automaton.
        
        Note: based on lomap.product.ts_times_buchi by
              Alphan Ulusoy <alphan@bu.edu>
        '''
        # Iterate over the initial states of the Buchi automaton
        init_buchi_states = [s for init_buchi in self.buchi.init
                        for s in self.buchi.next_states(init_buchi, symbols)]
        init_pa_states = [(node, s) for s in init_buchi_states]
        self.g.add_nodes_from(init_pa_states) # add states to PA
        self.init.update(init_pa_states) # mark states as initial
        # mark final states in Buchi as final states in PA 
        self.final.update([p for p in init_pa_states if p[1] in self.buchi.final])
        self.proj_ts[node] = set(init_buchi_states) # update projection of PA onto K
        # incremental update algorithm for SCC
        self.updateSCC()
    
    def update(self, E):
        '''Incrementally updates the global structure.
        Takes a PA edges and adds them to self.g, updates the self.proj_ts. 
        '''
        if not E:
            return
        
        # add edges to product automaton, automatically adds missing nodes
        self.g.add_edges_from(E, weight=1)
        
        # update projection of PA onto K
        _, states  = zip(*E)
        for nodeK, nodeB in states:
            self.proj_ts[nodeK] = self.proj_ts.get(nodeK, set()) | {nodeB}
            
            # Mark as final if final in buchi
            if nodeB in self.buchi.final:
                self.final.add((nodeK, nodeB))
        
        # incremental update algorithm for SCC
        self.updateSCC(E)
    
    def check(self, ts, nodeS, nodeD, propD, forward=False):
        ''' Checks if new node will correspond to a blocking state in the
        product automaton. If it is the case, the given edge is rejected.
        Otherwise, it returns the edges of the product automaton that are
        determined by the given edge.
        
        Note: It does not add the edges to the product automaton.
        '''
        assert nodeS in self.proj_ts # nodeS is in product automaton
        E = set()
        statesD = set()
        for nodeSBuchi in self.proj_ts.get(nodeS):
            stateS = (nodeS, nodeSBuchi)
            # get next Buchi nodes from the Buchi source node using propD
            nodesDBuchi = self.buchi.next_states(nodeSBuchi, propD)
            # the new reachable states
            newStatesD = [(nodeD, nodeDBuchi) for nodeDBuchi in nodesDBuchi]
            statesD.update(newStatesD)
            # append corresponding product automaton edges
            E.update(((stateS, stateD) for stateD in newStatesD))
        
        if forward:
            return E
        
        if E:
            assert not forward
            ts.g.add_edge(nodeS, nodeD)
            # Add edges to product automaton, automatically adds missing nodes
            self.g.add_edges_from(E, weight=1)
                        
            stack = []
            stack.extend(statesD)
             
            while stack:
                current_state = stack.pop()
                ts_state, buchi_state = current_state
                for _, ts_next_state in ts.g.out_edges_iter([ts_state]):
                    ts_next_prop = ts.g.node[ts_next_state].get('prop')
                    for buchi_next_state in self.buchi.next_states(buchi_state,
                                                                  ts_next_prop):
                        next_state = (ts_next_state, buchi_next_state)
                        if next_state not in self.g:
                            # Add transition w/ weight
                            self.g.add_edge(current_state, next_state, weight=1)
                            E.add((current_state, next_state))
                            # Continue search from next state
                            stack.append(next_state)
                        elif next_state not in self.g[current_state]:
                            self.g.add_edge(current_state, next_state, weight=1)
                            E.add((current_state, next_state))
        return E
    
    def foundPolicy(self):
        '''Checks if there is a satisfying path in the product automaton.'''
        return any([(x in scc) for x in self.final
                                   for scc in self.scc if len(scc) > 1])
        # incremental check
        return any([self.sccG.node[self.g.node[x]['canonical']]['size'] > 1
                       for x in self.final]) 
    
    def updateSCC(self, E=None):
        '''Incrementally updates the SCCs of PA.'''
        self.scc = list(nx.strongly_connected_components(self.g))
        
        return
        # incremental computation
        if E:
            nodesS, nodesD = zip(*E)
            Q = [x for x in set(nodesS + nodesD)
                       if not self.g.node[x].get('canonical', False)]
            initNodes(self.g, self.sccG, self.order, Q)
            self.order = scc(self.g, E, self.sccG, self.order)
    
    def initSCC(self):
        '''Initializes the SCC computation.'''
        self.scc = None
        
        # incremental computation
        self.sccG = nx.DiGraph()
        self.order = []
    
    def globalPolicy(self, ts=None):
        '''Computes the global policy.'''
        paths = nx.shortest_path(self.g)
        
        policy = None
        policy_len = None
        for init in self.init:
            for final in self.final:
                prefix = paths[init][final]
                _, suffix = source_to_target_dijkstra(self.g, final, final)
                if prefix and len(suffix) > 2:
                    comp_policy = (list(prefix), list(suffix))
                    comp_len = len(comp_policy[0]) + len(comp_policy[1])
                    if not policy or policy_len > comp_len:
                        policy = comp_policy
                        policy_len = comp_len
        
        prefix, suffix = policy
        return [v[0] for v in prefix], [v[0] for v in suffix]
