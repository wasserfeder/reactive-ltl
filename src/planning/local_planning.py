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

import re
import time
import os.path

import numpy as np
from numpy import dot
from numpy.random import uniform
from numpy.linalg import norm
import networkx as nx

import matplotlib.pyplot as plt

from shapely.geometry import Polygon, LineString


__verbose__ = False
__save_fig__ = False
__step__ = False
__reset_traj__ = False
fig_counter = 0

class Ts(object):
    def __init__(self):
        self.init = dict()

def load_ts(): #TODO: load the ts from file
    ts = Ts()        
    ts.init[(-9, -9)] = 1
    return ts

def draw_ts(ts, viewport, ts_color=(0, 0, 0), local=False):
    # draw states
    x, y = zip(*ts.g.nodes_iter())
    viewport.plot(x, y, color=ts_color, marker='o', linestyle='')
    
    if local:
        for start, stop in ts.g.edges_iter():
            viewport.plot([start[0], stop[0]], [start[1], stop[1]],
                          color=ts_color, linestyle='-')
    else:
        li, ls = viewport.get_xlim()
        # draw transitions
        for start, stop in ts.g.edges_iter():
            viewport.arrow(start[0], start[1], stop[0] - start[0], stop[1] - start[1],
                       length_includes_head=True, head_width=(ls-li)*0.5/40,
                       color=ts_color)

class Environment(object):
    boundary_color = (0, 0, 0)
    
    def __init__(self, filename):
        self.global_regions = {} # NOTE: list of (LowerLeft, UpperRight) tuples
        self.obstacles = []  # NOTE: list of (LowerLeft, UpperRight) tuples
        self.requests = [] # NOTE: list of (Name, Position) tuples
        self.requests_trajectories = []
        
        with open(filename, 'r') as fin:
            m = re.search('boundary (.*$)', fin.readline().strip())
            self.boundary = eval(m.group(1))
#             if __verbose__:
#                 print 'boundary:', self.boundary
            
            m = re.search('global_regions (.*$)', fin.readline().strip())
            n_global_regions = int(m.group(1))
#             if __verbose__:
#                 print '# global reg:', n_global_regions
            
            m = re.search('global_colors (.*$)', fin.readline().strip())
            self.global_colors = eval(m.group(1))
#             if __verbose__:
#                 print 'global colors:', self.global_colors
            
            for _ in range(n_global_regions):
                m = re.search('(\S*) (.*)$', fin.readline().strip())
                self.global_regions[m.group(1)] = eval(m.group(2))
#             if __verbose__:
#                 print 'global regions:', self.global_regions
            
            m = re.search('local_requests (.*$)', fin.readline().strip())
            self.local_requests = eval(m.group(1))
#             if __verbose__:
#                 print 'local requests:', self.local_requests
            
            m = re.search('local_obstacles (.*$)', fin.readline().strip())
            n_local_obstacles = int(m.group(1))
#             if __verbose__:
#                 print '# local obs:', n_local_obstacles
            
            m = re.search('local_obstacle_label (.*$)', fin.readline().strip())
            self.local_obstacle_label = m.group(1)
#             if __verbose__:
#                 print 'local obs label:', self.local_obstacle_label
            
            m = re.search('local_obstacle_color (.*$)', fin.readline().strip())
            self.local_obstacle_color = eval(m.group(1))
#             if __verbose__:
#                 print 'local obs color:', self.local_obstacle_color
            
            for _ in range(n_local_obstacles):
                m = re.search('(.*)$', fin.readline().strip())
                self.obstacles.append(eval(m.group(1)))
            self.obstacles = map(tuple, self.obstacles)
#             if __verbose__:
#                 print 'obstacles:', self.obstacles
    
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
    
    def intersecting_regions(self, u, v, local=False): 
        if local:
            regions = []
            for coords in self.obstacles: # local obstacles
                box = Polygon([coords[0], (coords[1][0], coords[0][1]),
                               coords[1], (coords[0][0], coords[1][1])])
                if box.intersects(LineString([u, v])):
                    regions.append(self.local_obstacle_label)
            
            unit = np.array(v) - np.array(u)
            unit = unit / norm(unit)
            for label, coords in self.requests: # local requests
                w = np.array(coords) - np.array(u)
                if norm(w - dot(w, unit)*unit) <= self.local_requests[label][0]:
                    regions.append(label)
        else: # global regions
            regions = []
            for label, coords, in self.global_regions.items():
                box = Polygon([coords[0], (coords[1][0], coords[0][1]),
                               coords[1], (coords[0][0], coords[1][1])])
                if box.intersects(LineString([u, v])):
                    regions.append(label)
        return regions
    
    def collision_free(self, u, v, robot, obstacles=None):
        '''
        Returns True if the edge (u, v) does not intersect local obstacle regions.
        '''
        if not obstacles:
            obstacles = self.obstacles
        
        coords = ((robot.current_position[0] - robot.sensing_radius, robot.current_position[1] - robot.sensing_radius),
                  (robot.current_position[0] + robot.sensing_radius, robot.current_position[1] + robot.sensing_radius))
        robot_box = Polygon([coords[0], (coords[1][0], coords[0][1]),
                               coords[1], (coords[0][0], coords[1][1])])
            
        for coords in obstacles:
            box = Polygon([coords[0], (coords[1][0], coords[0][1]),
                       coords[1], (coords[0][0], coords[1][1])]).intersection(robot_box)
            if box.intersects(LineString([u, v])):
                return False
        return True
    
    def is_simple_segment(self, u, v, local=False):
        '''
        Returns True if the edge (x, y) does not intersect other regions than
        the ones x and y belong to.
        '''
        u_reg, u_lreg = self.label(u)
        v_reg, v_lreg = self.label(v)
        
        if local:
            u_reg = u_lreg
            v_reg = v_lreg
        
        nr_reg = len(self.intersecting_regions(u, v, local))
        
        if (not u_reg) and (not v_reg):
            return nr_reg == 0
        
        if (not u_reg) or (not v_reg):
            return nr_reg == 1
        
        if u_reg == v_reg:
            return nr_reg == 1
        
        return nr_reg == 2
    
    def label(self, point):
        '''
        returns set of global propositions, set of local propositions 
        '''
        global_prop = set()
        for label, coords, in self.global_regions.items():
            if coords[0][0] <= point[0] <= coords[1][0] and \
               coords[0][1] <= point[1] <= coords[1][1]:
                global_prop.add(label)
        
        local_prop = set()
        for coords in self.obstacles: # local obstacles
            if coords[0][0] <= point[0] <= coords[1][0] and \
               coords[0][1] <= point[1] <= coords[1][1]:
                local_prop.add(self.local_obstacle_label)
                break
        
        p = np.array(point)
        for label, coords in self.requests: # local requests
            if norm(p - np.array(coords)) <= self.local_requests[label][0]:
                local_prop.add(label)
        
        return global_prop, local_prop        

    def test(self, state, lts, robot, obstacles):
        '''
        Test if target was hit and can be connected to global ts.
        NOTE: state must not be in the global ts.
        '''
        if not state: # the state is invalid
            return False
        if state in robot.ts.g: # local nodes may not correspond to global nodes
            return False
        
        # test if the target was hit
        target = robot.local_target_request
        if target:
            if not lts.g.node[state]['hit']:
                return False
        
        # test if the node can be connected to the global ts
        return self.is_simple_segment(state, robot.global_target_state) and \
               self.collision_free(state, robot.global_target_state, robot, obstacles)
    
    def draw(self, viewport, figure):
        # draw boundary
        x, y = zip(*[self.boundary[0], (self.boundary[1][0], self.boundary[0][1]),
                     self.boundary[1], (self.boundary[0][0], self.boundary[1][1]),
                     self.boundary[0]]) 
        viewport.plot(x, y, color=self.boundary_color)
        
        # draw global regions
        for label, coords, in self.global_regions.items():
            x, y = zip(*[coords[0], (coords[1][0], coords[0][1]),
                         coords[1], (coords[0][0], coords[1][1])])
            viewport.fill(x, y, color=self.global_colors[label])
            viewport.text(np.mean(x), np.mean(y), label, fontsize=50,
                          horizontalalignment='center', verticalalignment='center')
        
#         # draw local obstacles
#         for coords in self.obstacles:
#             x, y = zip(*[coords[0], (coords[1][0], coords[0][1]),
#                          coords[1], (coords[0][0], coords[1][1])])
#             viewport.fill(x, y, color=self.local_obstacle_color)
#             viewport.text(np.mean(x), np.mean(y), self.local_obstacle_label, fontsize=40,
#                           horizontalalignment='center', verticalalignment='center')
#         
#         # draw local requests
#         for label, coords in self.requests:
#             x, y = coords
#             viewport.plot(x, y, color=self.local_requests[label][1], marker='o', linestyle='')
#             c = plt.Circle((x, y), self.local_requests[label][0],
#                             color=self.local_requests[label][1], fill=False)
# #             viewport.text(np.mean(x), np.mean(y), label, fontsize=40,
# #                           horizontalalignment='right', verticalalignment='center',
# #                           color=self.local_requests[label][1])
# #             print self.local_requests[label][0]
#             figure.gca().add_artist(c)


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
        
#         # initialize trajectory and buchi tracking
#         self.trajectory = [self.current_position]
#         self.buchi_states = [set(pa.proj_ts[self.initial_position])]
#         self.potential = [get_actual_potential(self.initial_position, self.buchi_states[0], self.pa)]
#         self.global_target_state = self.initial_position
#         self.local_target_requests = []
        self.local_target_request = None
        
        # set max steer step size eta and sensing radius
        self.eta = eta
        self.sensing_radius = sensing_radius
    
    def in_sensing_range(self, position):
        return -self.sensing_radius <= position[0] - self.current_position[0] <= self.sensing_radius and \
               -self.sensing_radius <= position[1] - self.current_position[1] <= self.sensing_radius
    
    def draw_robot(self, viewport):
        x, y = self.current_position
        viewport.plot([x], [y], color='blue', marker='o', markersize=24)
        viewport.fill([x-self.sensing_radius, x+self.sensing_radius, x+self.sensing_radius, x-self.sensing_radius],
                      [y-self.sensing_radius, y-self.sensing_radius, y+self.sensing_radius, y+self.sensing_radius],
                      color=(0, 0, 1, 0.3))
    
    def draw_trajectory(self, viewport, first=0):
        # draw states
        x, y = zip(*self.trajectory[first:])
        viewport.plot(x, y, color='r', marker='o', linestyle='')
        
        # draw transitions
        for start, stop in zip(self.trajectory[first:-1], self.trajectory[first+1:]):
            viewport.arrow(start[0], start[1], stop[0] - start[0], stop[1] - start[1],
                       length_includes_head=True, head_width=0.2,
                       color='r')


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
    global __verbose__, __step__, __reset_traj__, __save_fig__, fig_counter
    __verbose__ = True
    __step__ = True
    __reset_traj__ = False
    __save_fig__ = False
    
    # Offline Computation
 
    # 1. Load transition system
    ts = load_ts() 
#     ts.visualize()
               
    # 5. initialize environment
    env = Environment('environment.txt')
    
    # 6. initialize plan, robot and trajectory
    robot = Robot(ts, None, None, env)
    
    # add on-line requests and their trajectories
    env.requests.append(('survivor', (8, 8)))
    env.requests.append(('survivor', (1.5, 4.7)))
    env.requests.append(('fire', (1.5, 6.2)))
    
#     env.requests_trajectories.append(['survivor', (8, 8), True, [(8, 8), (4, 5), (9, 5)], 0.05])
#     env.requests_trajectories.append(['survivor', (1.5, 4.7), True, [(1.5, 4.7), (-4, 5), (-8. -6.5), (-1, 7)], 0.07])
#     env.requests_trajectories.append(['fire', (1.5, 6.2), True, [(1.5, 6.2), (-5, 7.5), (-7.5, 0), (-5.5, 4.5)], 0.03])
    
    traj = [(8, 8), (4, 5), (9, 5)]
    traj_ext = []
    for segment in map(lambda x1, x2: lin_space(x1, x2, 0.05), traj[:-1], traj[1:]):
        traj_ext.extend(segment)
    traj_ext.append(traj[-1])
    env.requests_trajectories.append(['survivor', (8, 8), True, traj_ext])
    
    traj = [(1.5, 4.7), (-4, 5), (-8, 6.5), (-1, 7)]
    traj_ext = []
    for segment in map(lambda x1, x2: lin_space(x1, x2, 0.07), traj[:-1], traj[1:]):
        traj_ext.extend(segment)
    traj_ext.append(traj[-1])
    env.requests_trajectories.append(['survivor', (1.5, 4.7), True, traj_ext])
    
    traj = [(1.5, 6.2), (-5, 7.5), (-7.5, 0), (-5.5, 4.5)]
    traj_ext = []
    for segment in map(lambda x1, x2: lin_space(x1, x2, 0.03), traj[:-1], traj[1:]):
        traj_ext.extend(segment)
    traj_ext.append(traj[-1])
    env.requests_trajectories.append(['fire', (1.5, 6.2), True, traj_ext])
    
    
    if __verbose__:
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')
        fig.subplots_adjust(left=0.03, right=0.97, bottom=0.03, top=0.97)
        env.draw(ax, fig)
#         draw_ts(ts, ax)
        robot.draw_robot(ax)
        plt.xlim([env.boundary[0][0], env.boundary[1][0]])
        plt.ylim([env.boundary[0][1], env.boundary[1][1]])
        
#         for label, coords in env.requests:
#             x, y = coords
#             ax.text(np.mean(x), np.mean(y), label, fontsize=40,
#                           horizontalalignment='right', verticalalignment='center',
#                           color=env.local_requests[label][1])
        
        if __save_fig__:
            filename = 'figure-{0}.png'.format(format(fig_counter, '0>4d'))
            plt.savefig(os.path.join('gen_fig', filename), bbox_inches=0)
            fig_counter += 1
            print 'saved:',  filename
            time.sleep(0.25)
        else:
            fig.set_size_inches(9, 9, forward=True)
            plt.show()
    

if __name__ == '__main__':
    run_experiment()
