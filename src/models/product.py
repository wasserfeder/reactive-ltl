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

import itertools as it

import networkx as nx

from models import Model, Buchi
from lomap import Timer
from util.scc import scc, initNodes


# TODO: make SCC maintenance modular -- abstract it out of the PA implementation

class IncrementalProduct(Model):
    '''
    Product automaton between a weighted transition system and a static Buchi
    automaton with incremental updating. It maintains both reachability of final
    states and strongly connected components for quick model checking. 
    '''
    
    def __init__(self, globalSpec, specFile=None):
        '''
        Constructor
        '''
        Model.__init__(self)
        
        self.globalSpec = globalSpec # global specification
        # construct Buchi automaton from global LTL specification
        self.buchi = Buchi()
        if specFile: # FIXME: not implemented
            self.buchi.buchi_from_file(self.globalSpec, specFile)
        else:
            self.buchi.from_formula(self.globalSpec)
        self.proj_ts = {} # TS states to Buchi states map
        # initialize SCC algorithm
        self.initSCC()
    
    def addInitialState(self, node, symbols):
        ''' Adds an initial node to the product automaton.
        propositions ('prop').
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
        '''
        Incrementally updates the global structure.
        TODO: refactor into a function which takes a PA edge and adds it to
        self.g, updates the self.proj_ts 
        '''
        if not E:
            return
        
        # Add edges to product automaton, automatically adds missing nodes
        self.g.add_edges_from(E, weight=1)
        
        # Update projection of PA onto K
        _, states  = zip(*E)
        for nodeK, nodeB in states:
            self.proj_ts[nodeK] = self.proj_ts.get(nodeK, set()) | {nodeB}
            
            # Mark as final if final in buchi
            if nodeB in self.buchi.final:
                self.final.add((nodeK, nodeB))
        
        #TODO: incremental update algorithm for SCC
#        with Timer('Update_w_lin_scc'):
#            self._updateSCC(E)
        
#        with Timer('Update_w_lin_scc_batch'):
#            self._updateSCC_batch(E)
        
#         with Timer('Update_w_inc_scc'):
#             self._updateSCC2(E)
        
        self.updateSCC(E)
    
    def check(self, ts, nodeS, nodeD, propD, forward=False):
        '''
        Checks if new node will correspond to a blocking state in the product
        automaton. If it is the case, the given edge is rejected. Otherwise, it
        returns the edges of the product automaton that are determined by the
        given edge.
        
        Note: It does not add the edges to the product automaton.
        FIXME: Correct and Update the docs.
        '''
#         if ts.g.number_of_nodes() in [32, 33]:
#             print 'begin checker', 'forward' if forward else 'backward'
        assert nodeS in self.proj_ts # nodeS is in product automaton
        E = set()
        statesD = set()
#         if ts.g.number_of_nodes() in [32, 33]:
#             print 'check src:', nodeS.coords, self.proj_ts.get(nodeS)
#             print 'check dest:', nodeD.coords, propD
        for nodeSBuchi in self.proj_ts.get(nodeS):
            stateS = (nodeS, nodeSBuchi)
            # get next Buchi nodes from the Buchi source node using propositions propD
            nodesDBuchi = self.buchi.next_states(nodeSBuchi, propD)
#             print 'next buchi:', nodesDBuchi, nodeSBuchi, propD
            # the new reachable states
            newStatesD = [(nodeD, nodeDBuchi) for nodeDBuchi in nodesDBuchi]
            statesD.update(newStatesD)
#             if ts.g.number_of_nodes() in [32, 33]:
#                 print 'buchi src:', nodeSBuchi, 'prop:', propD, 'dest:', nodesDBuchi
#                 print 'pa states:', statesD
            # append corresponding product automaton edges
            E.update(((stateS, stateD) for stateD in newStatesD))
        
        if forward:
#             if ts.g.number_of_nodes() in [32, 33]:
#                 print 'end checker', 'forward' if forward else 'backward'
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
#         if ts.g.number_of_nodes() in [32, 33]:
#             print 'end checker', 'forward' if forward else 'backward'
        return E
    
    def foundPolicy(self):
        '''
        Returns True if there is a satisfying path in the product automaton.
        TODO: Dummy - not incremental
        '''
#         print '[found policy] begin'
#         for scc in self.scc:
#             print scc
#         print
#         print self.final
#         print '[found policy] end'
#         print
        return any([(x in scc) for x in self.final for scc in self.scc if len(scc) > 1])
        
#        with lomap.Timer('Check_w_empty'):
#            result_empty = not self.has_empty_language()
#        with lomap.Timer('Check_w_lin_scc'):
#            result_scc = any(map(lambda x: len(self.getSCC(x)) > 1, self.final))
        with Timer('Check_w_inc_scc'):
            result_inc_scc = any(map(lambda x: self.sccG.node[self.g.node[x]['canonical']]['size'] > 1, self.final))
#        if result_empty == result_scc and result_empty == result_inc_scc:
#            print '[PASS] Found policy:', result_empty
#        else:
#            print '[FAIL] Found policy:', 'empty:', result_empty, 'scc:', result_scc, 'inc_scc:', result_inc_scc
#        
#        if True: # delete after test
#            return result_empty
#        return any(map(lambda x: len(self.getSCC(x)) > 1, self.final))
        return result_inc_scc 
    
    def getSCC(self, x):
        '''
        TODO: Dummy -- to implement
        '''
        return filter(lambda scc: x in scc, self.scc)[0]
    
    def updateSCC(self, E=None):
        self.scc = nx.strongly_connected_components(self.g)
    
    def _updateSCC(self, E=None):
        '''
        TODO: to implement
        '''
        if not E:
            self.scc = nx.strongly_connected_components(self.g)
        else:
            for _ in E:
                self.scc = nx.strongly_connected_components(self.g)
    
    def _updateSCC_batch(self, E=None):
        '''
        TODO: to implement
        '''
        self.scc = nx.strongly_connected_components(self.g)
    
    def _updateSCC2(self, E):
        '''
        TODO: to implement
        '''
#        print 'E:', E
        nodesS, nodesD = zip(*E)
        Q = filter(lambda x: not self.g.node[x].get('canonical', None), set(nodesS + nodesD))
#        print 'Q:', Q
#        print 'order:', self.order
#        print [(x, data) for x, data in self.g.nodes_iter(data=True)]
        initNodes(self.g, self.sccG, self.order, Q)
#        print 'order:', self.order
#        print [(x, data) for x, data in self.g.nodes_iter(data=True)]
#        with lomap.Timer('call_scc'):
        self.order = scc(self.g, E, self.sccG, self.order)
#        print 'order:', self.order
#        print [(x, data) for x, data in self.g.nodes_iter(data=True)]
    
    def initSCC(self):
        '''
        TODO:
        '''
        self.scc = None
#         self.sccG = nx.DiGraph()
#         self.order = []
        
#         initNodes(self.g, self.sccG, self.order, Q) # TODO: move this to updateSCC 
    
    def globalPolicy(self, ts=None):
        """
        FIXME: very bad implementation - should use BFS or DFS
        """
#         #FIXME: we shouldn't need to do this
#         self.g = nx.MultiDiGraph(self.g)
        from lomap.algorithms.dijkstra import source_to_target_dijkstra
        paths = nx.shortest_path(self.g)
        
        policy = None
        for init in self.init:
            for final in self.final:
                prefix = paths[init][final]
                _, suffix = source_to_target_dijkstra(self.g, final, final)
#                print 'prefix:', prefix, 'suffix:', suffix
                if prefix and len(suffix) > 2:
                    comp_policy = (list(prefix), list(suffix))
                    if not policy:
                        policy = comp_policy
                    elif len(policy[0] + policy[1]) > len(comp_policy[0] + comp_policy[1]):
                        policy = comp_policy
        
        prefix, suffix = policy
        
#        K print '[globalPolicy] prefix'
#         for x, s in prefix:
#             print tuple(x.coords), s, ts.g.node[x]['prop']
#         print '[globalPolicy] suffix'
#         for x, s in suffix:
#             print tuple(x.coords), s, ts.g.node[x]['prop']
        
        return map(lambda v: v[0], prefix), map(lambda v: v[0], suffix)

if __name__ == '__main__':
    pass