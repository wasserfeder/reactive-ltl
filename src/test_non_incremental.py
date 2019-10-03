'''
.. module:: test_non_incremental.py
   :synopsis: Defines a drop-in non-incremental product automata based checker.
    It is only useful to compare the performance against the proposed
    algorithms.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

license_text='''
    Defines a drop-in non-incremental product automata based checker.
    Copyright (C) 2019  Cristian Ioan Vasile <cvasile@bu.edu>
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


from lomap import ts_times_buchi, no_data


class NonIncrementalChecker(Model):
    '''Drop-in non-incremental checker that computes the product automaton
    between the weighted transition system and the static Buchi automaton for
    every satisfaction check.
    '''

    def __init__(self, globalSpec):
        '''Constructor'''
        Model.__init__(self, directed=True, multi=False)

        self.globalSpec = globalSpec # global specification
        # construct Buchi automaton from global LTL specification
        self.buchi = Buchi()
        self.buchi.from_formula(self.globalSpec)
        self.ts = None

    def addInitialState(self, node, symbols): pass

    def update(self, E): pass

    def check(self, ts, nodeS, nodeD, propD, forward=False):
        self.ts = ts
        return {True}

    def foundPolicy(self):
        '''Checks if there is a satisfying path in the product automaton.
        It generates the automaton from scratch.
        '''
        if self.ts is None:
            return False
        self.pa = ts_times_buchi(self.ts, self.buchi, False, no_data, no_data)
        self.g = self.pa.g
        self.init = self.pa.init
        self.final = self.pa.final
        return len(self.pa.final) > 0

    def globalPolicy(self, ts=None):
        '''Computes the global policy.'''
        self.g = self.pa.g
        self.init = self.pa.init
        self.final = self.pa.final
        nx.set_edge_attributes(self.g, 'weight', 1)
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
