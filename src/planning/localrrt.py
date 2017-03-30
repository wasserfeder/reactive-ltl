'''
    Local RRT
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

from collections import namedtuple

import numpy as np
from numpy.linalg import norm
import networkx as nx

from lomap import Ts, Timer

from spaces import Point2D


def get_actual_potential(x, B, pa):
    '''Compute potential of a TS node x with Buchi set B from product
    automaton pa.
    '''
    pa_states = [(x, buchi_state) for buchi_state in pa.proj_ts[x] & B]
    if not pa_states:
        return float('Inf')
    pa_state =  min(pa_states, key=lambda state: pa.g.node[state]['potential'])
    return pa.g.node[pa_state]['potential']

def monitor(start, buchi, start_prop, stop_prop=None):
    '''
    start -- is the vector of Buchi states corresponding to the start
             configuration from the local transition system
    buchi -- Buchi automaton corresponding to the specification LTL formula
    start_prop -- proposition of the start configuration
    stop_prop -- proposition of the stop configuration
            if not given, it will force the computation of the next Buchi states
    
    returns stop  -- is the vector of Buchi states corresponding to the stop
             configuration from the local transition system
    
    Note:
    Assumes that the line segment [start; stop] crosses the boundary of a single
    region once.
    '''
    if start_prop == stop_prop:
        return set(start)
    
    stop = set()
    for buchi_state in start:
        stop |= set(buchi.next_states(buchi_state, stop_prop))
    return stop

def nearest(lts, random_sample):
    '''
    Returns nearest node in lts to the random sample. Uses linear search.
    '''
    random_sample = random_sample.coords
    return min(lts.g.nodes_iter(), key=lambda x: norm(x.coords - random_sample))


Request = namedtuple('Request', ('region', 'name', 'priority'))


class LocalPlanner(object):
    '''Local planner generating a random tree within the sensing area of the
    robot.
    '''
    
    def __init__(self, pa, ts, robot, local_spec, eta=0.1):
        '''Constructor'''
        self.pa = pa
        self.ts = ts
        self.robot = robot
        self.priority = local_spec
        self.lts = None
        
        self.PointCls = Point2D
        
        self.local_plan = None
        self.tracking_req = None
        self.requests = []
        self.obstacles = []
        
        self.trajectory = [self.robot.currentConf]
        self.buchi_states = [set([s for _, s in self.pa.init])]
        self.potential = [get_actual_potential(self.robot.initConf,
                                               self.buchi_states[0], self.pa)]
        self.global_target_state = self.robot.initConf
        
        # set max steer step size eta and sensing radius
        self.eta = eta
#         self.sensing_radius = sensing_radius

        self.durations = []
        self.sizes = []

#     def update_local_info(self):
#         '''Sets information about locally sensed requests and obstacles.'''
#         self.requests, self.obstacles = self.sense(
#                                                 self.robot.cspace.localRegions)
#     
#         requests, obstacles = [], []
#         for reg in self.robot.cspace.localRegions:
#             if self.robot.localObst in reg.symbols:
#                 if (self.robot.cspace.dist(self.robot.currentConf, reg.center)
#                                 < self.robot.sensingShape.radius + reg.radius):
#                     obstacles.append(reg)
#             else:
#                 # test if it is within sensing area
#                 if self.robot.sensingShape.intersects(reg.center):
#                     name = next(iter(reg.symbols))
#                     requests.append(Request(reg, self.priority[name]))
#         self.requests = requests
#         self.obstacles = obstacles

    def execute(self, requests, obstacles):
        '''Plan locally.'''
        assert not requests and not obstacles # TODO: delete after debug
        # update local information
        self.requests = requests
        self.obstacles = obstacles
        # update local plan
        local_planning_timer = Timer('Generate Local Plan')
        with local_planning_timer:
            nr_nodes = -1
            if not self.check_local_plan():
                print 'Local check: False'
                self.local_plan, nr_nodes = self.generate_local_plan()
            else:
                print 'Local check: True'
        # update meta data
        self.durations.append(local_planning_timer.duration)
        self.sizes.append(nr_nodes)
        self.update()
        # return next point
        return self.local_plan.pop(0)

    def check_local_plan(self):
        '''
        NOTE: Assumes the path is satisfying w.r.t. the global spec.
        It does not check collision w/ global regions.
        NOTE: Sets/Resets the local target request
        1. Checks if the target needs to be modified
        2. Checks if the local plan is collision free
        '''
        requests = self.requests
        
#         if self.local_plan: # TODO: uncomment
#             print '[check local plan]', [(p.x, p.y) for p in self.local_plan]
#         else:
#             print '[check local plan]', self.local_plan
#         print '[check local plan]', self.requests
        
        # check if target is needs to be modified
        if requests:
            highest_priority_req = min(requests, key=lambda req: req.priority)
            if (self.tracking_req is None or
                    self.tracking_req.priority > highest_priority_req.priority):
                self.tracking_req = highest_priority_req
                return False
        # there is a target, but it disappeared
        if self.tracking_req and self.tracking_req not in requests:
            if requests:
                self.tracking_req = highest_priority_req # switch target
            else:
                self.tracking_req = None # reset target
            return False
#         # target moved # TODO: is this necessary
#         if robot.local_target_request == robot.current_position:
#     #         if __verbose__:
#     #             print 'Target moved!!!'
#             robot.local_target_request = None # reset target
#             return False
        
        if not self.local_plan: # no plan
            return False
        
        # 2. check that path is collision free
        return self.robot.collision_free(self.local_plan, self.obstacles)

    def update(self):
        '''#TODO:
        '''
        new_conf = self.local_plan[0]
        prop = self.robot.getSymbols(new_conf)
        prev_prop = self.robot.getSymbols(self.robot.currentConf)
        B = monitor(self.buchi_states[-1], self.pa.buchi, prev_prop, prop)
        self.trajectory.append(new_conf)
        self.buchi_states.append(B)
        
        if new_conf in self.ts.g:
            self.potential.append(get_actual_potential(new_conf, B, self.pa))

    def min_potetial_global_state(self, state):
        '''#TODO:
        '''
#         print 'buchi:', self.buchi_states
#         print 'buchi prev:', self.buchi_states[-1]
        
        pa_states = [(state, buchi_state)
                                    for buchi_state in self.buchi_states[-1]]
        
#         print 'pa states:', pa_states
        
        _, pa_next_states = zip(*self.pa.g.out_edges_iter(pa_states))
        opt_pa_next_state = min(pa_next_states,
                           key=lambda x: self.pa.g.node[x]['potential'])
        
#         print 'next pa states:',
#         for p in pa_next_states:
#             print p, '->', self.pa.g.node[p]['potential']
            
        if (self.potential[-1] == 0 and
            self.pa.g.node[opt_pa_next_state]['potential'] == 0):
            pa_next_states.remove(opt_pa_next_state)
            opt_pa_next_state = min(pa_next_states,
                           key=lambda x: self.pa.g.node[x]['potential'])
            assert self.pa.g.node[opt_pa_next_state]['potential'] > 0
        opt_next_state, _ = opt_pa_next_state
        return opt_next_state
        
#         buchi_states = self.buchi_states[-1]
#         prop = self.ts.g.node[state]['prop']
#         opt_next_state = None
#         min_potential = float('Inf')
#         for next_state in self.ts.g.neighbors_iter(state):
#             B = monitor(buchi_states, self.pa.buchi, prop)
#             potential = get_actual_potential(next_state, B, self.pa)
#             
#             print '[min_potential_global_state] B', B
#             print '[min_potential_global_state]', 'n_state', next_state,
#             print 'potential', potential
#             
#             if potential < min_potential:
#                 min_potential = potential
#                 opt_next_state = next_state
#         
#         return opt_next_state

    def free_movement(self, current_state=None):
        '''Assumes that it can find a solution from the current position to the
        stored target global ts state.
        Return an empty list if the line segment to target global ts state is
        not ``simple''.
        '''
        # get current state
        if not current_state:
            current_state = self.robot.currentConf
        
        # if global state => switch to next global state
        if current_state in self.ts.g: # TODO: make this distance based
#             pa_states = [(current_state, buchi_state)
#                          for buchi_state in self.buchi_states[-1]]
#             _, pa_next_states = zip(*self.pa.g.out_edges_iter(pa_states))
#             next_states, _ = zip(*pa_next_states)
#             
#             print
#             print '[free_movement] current state:', current_state
#             print '[free_movement] buchi states:', self.buchi_states[-1]
#             for p in pa_next_states:
#                 print '[free_movement] pa next states:', p,
#                 print 'potential', self.pa.g.node[p]['potential']
# #             print '[free_movement] pa next states:', pa_next_states
#             
#             
#             B = set(self.pa.buchi.g.nodes())
#             final_state = min(next_states,
#                               key=lambda x: get_actual_potential(x, B, self.pa))
#             
#             print '[free_movement] final_state:', final_state
#             print '[free_movement] min_potential_state:', \
#                                 self.min_potetial_global_state(current_state)
#             print
#             
#             final_state = self.min_potetial_global_state(current_state)
#             
#             assert get_actual_potential(final_state, B, self.pa) < float('Inf')
            
            final_state = self.min_potetial_global_state(current_state)
            self.global_target_state = final_state
        else: # local state
            assert False # TODO: remote after debug
            
            final_state = self.global_target_state
            # check if it can be connected
            if not self.robot.isSimpleSegment(current_state, final_state):
                return []
        
        # TODO: remove after debug
        final_state = np.array(final_state.coords)
        return [self.PointCls(final_state)]
        
        # generate straight path to the node
        current_state = np.array(current_state.coords)
        final_state = np.array(final_state.coords)
        dist = norm(final_state - current_state)
        u = final_state - current_state
        points = [self.PointCls(current_state + a * u)
                        for a in np.arange(0, 1, self.eta/dist)][1:]
        points.append(self.PointCls(final_state))
        return points

    def test(self, state):
        ''' Test if target was hit and can be connected to global ts.
        NOTE: state must not be in the global ts.
        '''
        if not state: # the state is invalid
            return False
        if state in self.ts.g: # local nodes may not be in the global ts
            return False
        
        # test if the target was hit
        target = self.tracking_req
        if target:
            if not self.lts.g.node[state]['hit']:
                return False
        
        # test if the node can be connected to the global ts
        return (self.robot.isSimpleSegment(state, self.global_target_state)
         and self.robot.collision_free_segment(state, self.global_target_state,
                                               self.obstacles))

    def generate_local_plan(self):
        # NOTE: Assumes that it is called either because there is no local
        # policy or there are new events => re-planning is needed
        #NOTE: exclude current position as next target position
        #NOTE: free movement if there are no local requests
        
        local_requests, local_obstacles = self.requests, self.obstacles
        
        # 0. no local requests => move towards global state of minimum potential
#         print '[generate_local_plan] local reqs:', local_requests
        if not local_requests and not local_obstacles:
#             print '[generate_local_plan]', local_requests
            local_plan = self.free_movement()
            if local_plan:
                return local_plan, -1
        
        assert False # TODO: delete after debug
        
        print '[generate_local_plan]', self.tracking_req
        
        target = self.tracking_req
        
        # 1. initialize local ts
        self.lts = Ts()
        # add current state to local ts
        global_prop = self.robot.getSymbols(self.robot.currentConf)
        local_prop = self.robot.getSymbols(self.robot.currentConf, local=True)
        self.lts.g.add_node(self.robot.currentConf, global_prop=global_prop,
                       buchi_states=self.buchi_states[-1],
                       hit=(target.name in local_prop) if target else False)
        
        dest_state = self.robot.currentConf
        # test if solution was found
        while not self.test(dest_state):
            # 3. generate sample
            random_sample = self.robot.sample(local=True)
            print '[generate_local_plan] random_sample:', random_sample
            
            # 4. get nearest neighbor in local ts
            source_state = nearest(self.lts, random_sample)
            # 5. steer robot towards random sample
            dest_state = self.robot.steer(source_state, random_sample)
            # 6. label new sample
            dest_global_prop = self.robot.getSymbols(dest_state)
            dest_local_prop = self.robot.getSymbols(dest_state, local=True)
            
            if self.robot.isSimpleSegment(source_state, dest_state):
                # 7. compute Buchi states for new sample
                source_data = self.lts.g.node[source_state]
                B = monitor(source_data['buchi_states'], self.pa.buchi,
                            source_data['global_prop'], dest_global_prop)
                Bp = monitor(B, self.pa.buchi, dest_global_prop)
                if B and Bp:
                    if self.robot.collision_free_segment(source_state,
                                                  dest_state, local_obstacles):
                        # 8. update local transition system
                        hit = False
                        if target:
                            hit = ((target.name in dest_local_prop)
                                                        or source_data['hit'])
                        
                        self.lts.g.add_node(dest_state,
                                            global_prop=dest_global_prop,
                                            buchi_states=B, hit=hit)
                        self.lts.g.add_edge(source_state, dest_state)
            
            if dest_state not in self.lts.g:
                dest_state = None
            
            if dest_state:
                print '[generate_local_plan] local tree'
                print '[generate_local_plan] hit', hit
                print '[generate_local_plan] dest_local_prop', dest_local_prop
                print '[generate_local_plan] target.name', target.name
                print '[generate_local_plan] test', self.test(dest_state)
                self.sim.display(expanded=True, localinfo='tree')
        
        # 11. return local plan
        plan_to_leaf = nx.shortest_path(self.lts.g, self.robot.currentConf,
                                        dest_state)[1:]
        return (plan_to_leaf + self.free_movement(dest_state),
                self.lts.g.number_of_nodes())
