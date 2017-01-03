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

import time
import os.path
from collections import namedtuple

import numpy as np
from numpy.random import uniform
from numpy.linalg import norm
import networkx as nx

import lomap
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
#     if start_prop == stop_prop:
#         return set(start)
    
    stop = set()
    for buchi_state in start:
        stop |= set(buchi.next_states(buchi_state, start_prop))
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
        self.buchi_states = [set(pa.proj_ts[self.robot.initConf])]
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
                print 'Local check: False'
        # update meta data
        self.durations.append(local_planning_timer.duration)
        self.sizes.append(nr_nodes)
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
        if not self.local_plan: # no plan
            return False
        requests = self.requests
        
        print '[check local plan]', self.local_plan
        print '[check local plan]', self.requests
        
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
        
        # 2. check that path is collision free
        return self.robot.collision_free(self.local_plan, self.obstacles)

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
            pa_states = [(current_state, buchi_state)
                         for buchi_state in self.buchi_states[-1]]
            _, pa_next_states = zip(*self.pa.g.out_edges_iter(pa_states))
            next_states, _ = zip(*pa_next_states)
            
            B = set(self.pa.buchi.g.nodes())
            final_state = min(next_states,
                              key=lambda x: get_actual_potential(x, B, self.pa))
            
            assert get_actual_potential(final_state, B, self.pa) < float('Inf')
            
            self.global_target_state = final_state
        else: # local state
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
        if not local_requests and not local_obstacles:
            print '[generate_local_plan]', local_requests
            local_plan = self.free_movement()
            if local_plan:
                return local_plan, -1
        
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


class Environment(object):
    boundary_color = (0, 0, 0)
    
    def __init__(self, filename):
        self.global_regions = {} # NOTE: list of (LowerLeft, UpperRight) tuples
        self.obstacles = []  # NOTE: list of (LowerLeft, UpperRight) tuples
        self.requests = [] # NOTE: list of (Name, Position) tuples
        self.requests_trajectories = []
    
    def get_requests(self, position, radius, prior):
        # square
        requests = filter(lambda r: (position[0]-radius <= r[1][0] <= position[0]+radius) and
                                    (position[1]-radius <= r[1][1] <= position[1]+radius), self.requests)
        obstacles = []
        for obstacle in self.obstacles:
            if max(position[0]-radius, obstacle[0][0]) <= min(position[0]+radius, obstacle[1][0]) and \
               max(position[1]-radius, obstacle[0][1]) <= min(position[1]+radius, obstacle[1][1]):
                obstacles.append(obstacle)
        
        # order local requests based on their priority
        requests.sort(key=lambda r: prior[r[0]])
        
        return requests, obstacles
    
    def update_requests(self, robot):
#         # 1. remove local target request if it was serviced
#         position = np.array(position)
#         target_position = np.array(target_request[1])
#         if norm(position - target_position) <= self.local_requests[target_request[0]][0]:
#             self.requests.remove(target_request)
        
        robot_position, target_request = robot.current_position, robot.local_target_request
        robot_position = np.array(robot_position)
        
#         if __verbose__:
#             print '[UPDATE]', target_request, robot_position
#             print '[UPDATE]', self.requests_trajectories
        deactivated = False
        self.requests = []
        for req in self.requests_trajectories:
            label, position, active, traj = req
            
            # 1. remove local target request if it was serviced
            if target_request == (label, position):
                target_position = np.array(target_request[1])
                if norm(robot_position - target_position) <= self.local_requests[target_request[0]][0]:
                    active = False
                    req[2] = False
                    deactivated = True
#                     if __verbose__:
#                         print '[UPDATE] deactivate', target_request
                    
                # update robot local target
                robot.local_target_request = (label, traj[(traj.index(position)+1) % len(traj)])
            
            # 2. update requests positions
            req[1] = traj[(traj.index(position)+1) % len(traj)]
            
            if active:
                self.requests.append(tuple(req[:2]))
        
        return deactivated     
        
    def sample(self, position, radius):
        x = uniform(max(self.boundary[0][0], position[0]-radius), min(self.boundary[1][0], position[0]+radius), 1)
        y = uniform(max(self.boundary[0][1], position[1]-radius), min(self.boundary[1][1], position[1]+radius), 1)
        return (float(x), float(y))
#         return tuple(np.array(position) + uniform(-radius, radius, 2))


class Robot(object): # NOTE: does also buchi tracking and maintains trajectory
    
    def __init__(self, ts, buchi, pa, env, eta=1, sensing_radius=2.5):
        self.ts = ts
        self.buchi = buchi
        self.pa = pa
        self.env = env
        
        # assume deterministic transition system
        assert len(self.ts.init.keys()) == 1
        
        self.initial_position = self.ts.init.keys()[0]
        self.current_position = self.initial_position
        
        # initialize trajectory and buchi tracking
        self.trajectory = [self.current_position]
        self.buchi_states = [set(pa.proj_ts[self.initial_position])]
        self.potential = [get_actual_potential(self.initial_position, self.buchi_states[0], self.pa)]
        self.global_target_state = self.initial_position
#         self.local_target_requests = []
        self.local_target_request = None
        
        # set max steer step size eta and sensing radius
        self.eta = eta
        self.sensing_radius = sensing_radius
    
    def in_sensing_range(self, position):
        return -self.sensing_radius <= position[0] - self.current_position[0] <= self.sensing_radius and \
               -self.sensing_radius <= position[1] - self.current_position[1] <= self.sensing_radius
    
    def move(self, new_position):
        prop, _ = self.env.label(new_position)
        prev_prop, _ = self.env.label(self.current_position)
        B = monitor(self.buchi_states[-1], self.buchi, prev_prop, prop)
        
        self.current_position = new_position
        self.trajectory.append(self.current_position)
        self.buchi_states.append(B)
        
        if new_position in self.ts.g:
            self.potential.append(get_actual_potential(new_position, B, self.pa))
            
            current_state = self.current_position
            pa_states = [(current_state, buchi_state) for buchi_state in self.buchi_states[-1]]
            _, pa_next_states = zip(*self.pa.g.out_edges_iter(pa_states))
            next_states, _ = zip(*pa_next_states)
            
            B = set(self.buchi.g.nodes())
            final_state = min(next_states, key=lambda x: get_actual_potential(x, B, self.pa))
            
            assert get_actual_potential(final_state, B, self.pa) < float('Inf')
            
            self.global_target_state = final_state


def lin_space(x1, x2, step):
    x1 = np.array(x1)
    x2 = np.array(x2)
    dist = norm(x2 - x1)
    u = x2 - x1
    return map(lambda a: tuple(x1 + a * u), np.arange(0, 1, step/dist))

def run_experiment():
    '''
    LTL local control
    '''    
    
    # 4. compute potential for each state of PA
    with lomap.Timer():
        compute_potentials(pa)
           
    # 5. initialize environment
    env = Environment('environment.txt')
    
    # 6. initialize plan, robot and trajectory
    local_plan = []
    robot = Robot(ts, buchi, pa, env)
    
    prev_info = set([])

#     # add on-line requests and their trajectories
#     env.requests.append(('survivor', (8, 8)))
#     env.requests.append(('survivor', (1.5, 4.7)))
#     env.requests.append(('fire', (1.5, 6.2)))
    
#     env.requests_trajectories.append(['survivor', (8, 8), True, [(8, 8), (4, 5), (9, 5)], 0.05])
#     env.requests_trajectories.append(['survivor', (1.5, 4.7), True, [(1.5, 4.7), (-4, 5), (-8. -6.5), (-1, 7)], 0.07])
#     env.requests_trajectories.append(['fire', (1.5, 6.2), True, [(1.5, 6.2), (-5, 7.5), (-7.5, 0), (-5.5, 4.5)], 0.03])
    
    
    # 7. define priorities of local requests (local specification)
    prior = {'survivor':0, 'fire':1}
    
    plan_durations = []
    cycle_duration = []
    cycle = 0
    no_cycles = 100
    plan_nr_nodes = []
    cycle_info = []
#     no_hit = 0
#     detected = set()
    
    # Real-time algorithm start
    runtime = 100
    for _ in range(runtime):
        # 8. get sensor data
        info = env.get_requests(robot.current_position, robot.sensing_radius, prior)
        
        with Timer() as local_planning_timer:
            if not check_local_plan(info, prev_info, local_plan, env, robot, prior):
                # 9. plan locally
#                 if __verbose__:
#                     print 'Computing local plan'
                local_plan, nr_nodes = generate_local_plan(env, info, robot)
        
        plan_durations.append(local_planning_timer.duration)
        plan_nr_nodes.append(nr_nodes)
        
          
        # 10. move to next point
        robot.move(local_plan.pop(0))
        
        # 11. Update local requests
        env.update_requests(robot)
        
        #HACK: add on-line requests back to the list of active requests
        if (robot.current_position == (-7, 0)):
            for req in env.requests_trajectories:
                env.requests.append(tuple(req[:2]))
                req[2] = True
            
            cycle_duration.append(plan_durations)
            print 'Cycle:', cycle, 'Cycle duration:', sum(plan_durations)#, (no_hit, len(detected))
#             print 'Cycle:', cycle
#             cycle_info.append((no_hit, len(detected)))

#             detected = set()
#             no_hit = 0
            plan_durations = []
            cycle += 1
            
            if __reset_traj__:
                robot.trajectory = [robot.trajectory[-1]]
            
            if cycle > no_cycles: break
        
        prev_info = info
        
#         if __verbose__:
#             print env.requests
#             for req in env.requests_trajectories:
#                 print req[:3]
#             
#             
#             print
#             print
    
#     year, mon, day, hour, mi, sec, _, _, _ = time.gmtime()
#     log_timestamp = '{Y}-{M}-{D}.{h}-{m}-{s}'.format(Y=year, M=mon, D=day, h=hour, m=mi, s=sec)
#     with open('results.' + log_timestamp + '.txt', 'w') as fout:
#         print>>fout, cycle_duration
#         print>>fout, plan_nr_nodes
#         print>>fout, [sum(d) for d in cycle_duration[1:]]
#         print>>fout, np.mean(np.array([sum(d) for d in cycle_duration[1:]]))
#         print>>fout, np.mean([n for n in plan_nr_nodes if n>=0])

if __name__ == '__main__':
    run_experiment()
    