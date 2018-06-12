'''
.. module:: localrrt
   :synopsis: Local planner generating a random tree within the sensing area of
   the robot.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    Local planner generating a random tree within the sensing area of the robot.
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

import logging
import itertools as it
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
    '''Half-monitor for violation of LTL specification.
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
    '''Returns nearest node in lts to the random sample. Uses linear search.'''
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
        self.buchi_states = None
        self.potential = None
        if self.pa is not None:
            self.buchi_states = set([s for _, s in self.pa.init])
            self.potential = get_actual_potential(self.robot.initConf,
                                                  self.buchi_states, self.pa)
        self.global_target_state = self.robot.initConf

        # set max steer step size eta and sensing radius
        self.eta = eta

        self.detailed_logging = False
        logging.info('"initialize local planner": True')
        self.log()

    def log(self, gen_tree=False):
        '''Saves information about the current planner status to log.'''
        logging.info('"Buchi states": %s', self.buchi_states)
        logging.info('"potential": %f', self.potential)
        logging.info('"tree size": %d',
                     self.lts.g.number_of_nodes() if gen_tree else 0)
        logging.info('"tree": %s', self.lts.g.edges() if gen_tree else [])
        if self.local_plan is None:
            logging.info('"new configuration": %s', self.robot.currentConf)
            logging.info('"local plan": %s', [])
        else:
            logging.info('"new configuration": %s', self.local_plan[0])
            logging.info('"local plan": %s', self.local_plan)
        logging.info('"requests": %s', self.robot.sensor.requests)

    def execute(self, requests, obstacles):
        '''Plan locally.'''
        # update local information
        self.requests = requests
        self.obstacles = obstacles
        # update local plan
        with Timer(op_name='local planning', template='"%s runtime": %f'):
            gen_tree=False
            # if global state => switch to next global state
            current_state = self.robot.currentConf
            if current_state in self.ts.g:
                final_state = self.min_potential_global_state(current_state)
                self.global_target_state = final_state

            if not self.check_local_plan():
                self.local_plan, gen_tree = self.generate_local_plan()

        self.update()

        self.log(gen_tree)

        # return next point
        return self.local_plan.pop(0)

    def local_plan_hit(self):
        '''Checks if the local plan hits the tracked request.'''
        if not self.local_plan:
            return False
        assert self.local_plan[-1] in self.ts.g # last state is global
        target = self.tracking_req.name
        for conf in self.local_plan:
            if target in self.robot.getSymbols(conf, local=True):
                return True
        return False

    def check_local_plan(self):
        '''Checks if the local plan is still satisfying with respect to the
        local specification.

        NOTE: Assumes the path is satisfying w.r.t. the global specification.
        It does not check collision with global regions.
        NOTE: Sets/Resets the local target request
        1. Checks if the target needs to be modified
        2. Checks if the local plan is collision free
        '''
        requests = self.requests
        # check if target needs to be modified
        if requests:
            highest_priority_req = min(requests, key=lambda req: req.priority)
            if (self.tracking_req is None or
                    self.tracking_req.priority > highest_priority_req.priority):
                self.tracking_req = highest_priority_req
            return self.local_plan_hit()
        # there is a target, but it disappeared
        if self.tracking_req and self.tracking_req not in requests:
            if requests:
                self.tracking_req = highest_priority_req # switch target
                return self.local_plan_hit()
            else:
                self.tracking_req = None # reset target
        # no plan
        if not self.local_plan:
            return False
        # 2. check that path is collision free
        assert self.local_plan[-1] in self.ts.g # last state is global
        r = self.robot.collision_free(self.local_plan, self.obstacles)
        return r

    def update(self):
        '''Updates the information for the next configuration in the local plan.
        '''
        new_conf = self.local_plan[0]
        prop = self.robot.getSymbols(new_conf)
        prev_prop = self.robot.getSymbols(self.robot.currentConf)
        B = monitor(self.buchi_states, self.pa.buchi, prev_prop, prop)
        self.trajectory.append(new_conf)
        self.buchi_states = B

        if new_conf in self.ts.g:
            self.potential = get_actual_potential(new_conf, B, self.pa)

    def min_potential_global_state(self, state):
        '''Computes the minimum potential global state to connect to.'''
        pa_states = [(state, buchi_state) for buchi_state in self.buchi_states]
        _, pa_next_states = zip(*self.pa.g.out_edges_iter(pa_states))
        opt_pa_next_state = min(pa_next_states,
                           key=lambda x: self.pa.g.node[x]['potential'])
        if (self.potential == 0 and
            self.pa.g.node[opt_pa_next_state]['potential'] == 0):
            pa_next_states.remove(opt_pa_next_state)
            opt_pa_next_state = min(pa_next_states,
                           key=lambda x: self.pa.g.node[x]['potential'])
            assert self.pa.g.node[opt_pa_next_state]['potential'] > 0
        opt_next_state, _ = opt_pa_next_state
        return opt_next_state

    def connection_global_state(self, state, B):
        '''Computes the global state the state of the local TS should connect
        to. The choice is made to minimize potential first, and then distance.
        Search radius is given by the currently stored global target state.
        '''
        if state is None or not B:
            return None

        r = max(1, norm(state.coords - self.global_target_state.coords))
        near_ts_states = [x for x in self.ts.g.nodes_iter()
                                        if norm(state.coords - x.coords) <= r]
        assert self.global_target_state in near_ts_states

        global_state = None
        min_potential = float('inf')
        min_dist = float('inf')
        src_global_prop = self.robot.getSymbols(state)
        for x in near_ts_states:
            if self.robot.isSimpleSegment(state, x):
                dist = norm(state.coords - x.coords)
                dest_global_prop = self.robot.getSymbols(x)
                B_next = monitor(B, self.pa.buchi,
                                 src_global_prop, dest_global_prop)
                potential = get_actual_potential(x, B_next, self.pa)
                if (global_state is None or potential < min_potential
                        or (potential == min_potential and dist < min_dist)):
                    global_state = x
                    min_potential = potential
                    min_dist = dist

        # check for progress with respect to global mission specification
        if min_potential >= self.potential > 0:
            global_state = None

        if global_state: # check for collision w.r.t. local obstacles 
            if not self.robot.collision_free_segment(state, global_state,
                                                 self.obstacles):
                global_state = None
        return global_state

    def free_movement(self, current_state=None):
        '''Assumes that it can find a solution from the current position to the
        stored target global ts state.
        Return an empty list if the line segment to target global ts state is
        not ``simple''.
        '''
        # get current state
        if not current_state:
            current_state = self.robot.currentConf

        final_state = self.global_target_state
        # check if it can be connected
        if not self.robot.isSimpleSegment(current_state, final_state):
            return []

        # generate straight path to the node
        current_state = np.array(current_state.coords)
        final_state = np.array(final_state.coords)
        dist = norm(final_state - current_state)
        u = final_state - current_state
        points = [self.PointCls(current_state + a * u)
                        for a in np.arange(0, 1, self.eta/dist)][1:]
        points.append(self.PointCls(final_state))
        return points

    def generate_local_plan(self):
        '''Generates the local plan from the current configuration.
        NOTE: Assumes that it is called either because there is no local
        policy or there are new events => re-planning is needed
        NOTE: exclude current position as next target position
        NOTE: free movement if there are no local requests
        '''

        local_requests, local_obstacles = self.requests, self.obstacles
        # 0. no local requests => move towards global state of minimum potential
        if not local_requests:
            if not local_obstacles:
                collision_free = True
            else:
                state = self.robot.currentConf
                assert self.robot.isSimpleSegment(state, self.global_target_state)
                collision_free = self.robot.collision_free_segment(state,
                                     self.global_target_state, local_obstacles)
            if collision_free:
                local_plan = self.free_movement()
                if local_plan:
                    return local_plan, False

        target = self.tracking_req

        # 1. initialize local ts
        self.lts = Ts(multi=False)
        # add current state to local ts
        global_prop = self.robot.getSymbols(self.robot.currentConf)
        local_prop = self.robot.getSymbols(self.robot.currentConf, local=True)
        self.lts.g.add_node(self.robot.currentConf, global_prop=global_prop,
                       buchi_states=self.buchi_states,
                       hit=(target.name in local_prop) if target else False)

        done = False
        cnt = it.count()
        # test if solution was found
        while not done:
            iteration = cnt.next()
            # 3. generate sample
            random_sample = self.robot.sample(local=True)
            # 4. get nearest neighbor in local ts
            source_state = nearest(self.lts, random_sample)
            # 5. steer robot towards random sample
            dest_state = self.robot.steer(source_state, random_sample)
            # 6. label new sample
            dest_global_prop = self.robot.getSymbols(dest_state)
            dest_local_prop = self.robot.getSymbols(dest_state, local=True)

            simple_segment = self.robot.isSimpleSegment(source_state, dest_state)
            empty_buchi = collision_free = hit = global_state = None
            if simple_segment:
                # 7. compute Buchi states for new sample
                source_data = self.lts.g.node[source_state]
                B = monitor(source_data['buchi_states'], self.pa.buchi,
                            source_data['global_prop'], dest_global_prop)
                empty_buchi = len(B) > 0

                if B:
                    collision_free = self.robot.collision_free_segment(
                                    source_state, dest_state, local_obstacles)
                    if collision_free:
                        # 8. update local transition system
                        hit = True # all samples hit when there are not targets
                        if target:
                            hit = ((target.name in dest_local_prop)
                                                        or source_data['hit'])

                        self.lts.g.add_node(dest_state,
                                            global_prop=dest_global_prop,
                                            buchi_states=B, hit=hit)
                        self.lts.g.add_edge(source_state, dest_state)

                        if hit:
                            # test if the node can be connected to the global ts
                            global_state = self.connection_global_state(
                                                                dest_state, B)
                            if global_state:
                                self.global_target_state = global_state
                                done = True

            if self.detailed_logging:
                logging.info('"lts iteration %d" : '
                             '(%d, %s, %s, %s, %s, %s, %s, %s, %s)',
                             iteration, iteration,
                             random_sample, source_state, dest_state,
                             simple_segment, empty_buchi, collision_free, hit,
                             global_state)

        # 11. return local plan
        plan_to_leaf = nx.shortest_path(self.lts.g, self.robot.currentConf,
                                        dest_state)[1:]

        return (plan_to_leaf + self.free_movement(dest_state), True)
