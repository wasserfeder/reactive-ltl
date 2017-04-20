'''
.. module:: planar
   :synopsis: Module defining the classes for displaying planar environments.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''
'''
    The module defines classes for displaying planar environments.
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

import os
import itertools as it
import logging

import numpy as np
from numpy import mean

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import ColorConverter

from spaces.maps2d import BallRegion2D, BoxRegion2D, PolygonRegion2D, \
                          BallBoundary2D, BoxBoundary2D, PolygonBoundary2D, \
                          Point2D
#                           expandRegion


__all__ = []

colconv = ColorConverter()
to_rgba = colconv.to_rgba


def drawPoint2D(viewport, point, color, style=None):
    if style:
        viewport.plot(point.x, point.y, color=color, **style)
    else:
        viewport.plot(point.x, point.y, color=color)

def drawBoxRegion2D(viewport, region, style, text='', textStyle=None):
    x = [region.ranges[0, 0], region.ranges[0, 0],
         region.ranges[0, 1], region.ranges[0, 1]]
    y = [region.ranges[1, 0], region.ranges[1, 1],
         region.ranges[1, 1], region.ranges[1, 0]]
    viewport.fill(x, y, **style)
    if text:
        if textStyle:
            viewport.text(mean(x), mean(y), text, **textStyle)
        else:
            viewport.text(mean(x), mean(y), text)

def drawBallRegion2D(viewport, region, style, text='', textStyle=None):
    x, y = region.center
    c = plt.Circle(region.center, region.radius, **style)
    viewport.add_artist(c)
    if text:
        if textStyle:
            viewport.text(x, y, text, **textStyle)
        else:
            viewport.text(x, y, text)

def drawPolygonRegion2D(viewport, region, style, text='', textStyle=None):
    x, y = zip(*region.polygon.exterior.coords)
    viewport.fill(x, y, **style)
    if text:
        x, y = region.polygon.centroid.coords[0]
        if textStyle:
            viewport.text(x, y, text, **textStyle)
        else:
            viewport.text(x, y, text)

def drawRegion2D(viewport, region, style, text='', textStyle=None):
    if isinstance(region, BoxRegion2D):
        drawBoxRegion2D(viewport, region, style, text, textStyle)
    elif isinstance(region, BallRegion2D):
        drawBallRegion2D(viewport, region, style, text, textStyle)
    elif isinstance(region, PolygonRegion2D):
        drawPolygonRegion2D(viewport, region, style, text, textStyle)
    else:
        raise TypeError

def drawBoundary2D(viewport, boundary, style):
    if isinstance(boundary, BoxBoundary2D):
        x, y = zip(*((0, 0), (0, 1), (1, 1), (1, 0), (0, 0)))
        x, y = boundary.xrange()[list(x)], boundary.yrange()[list(y)]
        viewport.plot(x, y, **style)
    elif isinstance(boundary, BallBoundary2D):
        c = plt.Circle(boundary.center, boundary.radius, **style)
        viewport.add_artist(c)
    elif isinstance(boundary, PolygonBoundary2D):
        x, y = zip(*boundary.polygon.exterior.coords)
        viewport.plot(x, y, **style)

def addStyle(region, style=None, withText=True, text=None, textStyle=None):
    if style:
        region.style = style
        region.style['facecolor'] = style.get('facecolor', 'white')
        region.style['edgecolor'] = style.get('edgecolor', 'black')
    else:
        region.style = {'facecolor': 'white', 'edgecolor':'black'}
    
    if withText:
        if text:
            region.text = text
        else:
            region.text = ' '.join(region.symbols)
        if textStyle:
            region.textStyle = textStyle
            region.textStyle['horizontalalignment'] = \
                                  textStyle.get('horizontalalignment', 'center')
            region.textStyle['verticalalignment'] = \
                                    textStyle.get('verticalalignment', 'center')
            region.textStyle['fontsize'] = textStyle.get('fontsize', 12)
        else:
            region.textStyle = {'horizontalalignment' : 'center',
                               'verticalalignment' : 'center',
                               'fontsize' : 12}
    else:
        region.text = None
        region.textStyle = None

def drawRobot2D(viewport, robot, size, text='', textStyle=None):
    x, y = robot.currentConf.coords
    #TODO: make this general
    sensingShape = BallBoundary2D([x, y], robot.sensor.sensingShape.radius)
    style = {'facecolor': (0, 0, 1, 0.2), 'edgecolor': (0, 0, 1, 0.2), 'fill': True}
    drawBoundary2D(viewport, sensingShape, style)
    
    style = {'facecolor': 'blue', 'edgecolor':'black'}
    if size == 0:
        plt.plot(x, y, 'o', color=style['facecolor'], zorder=2)
    else:
        c = plt.Circle(robot.currentConf.coords, size, **style)
        viewport.add_artist(c)
    if text:
        if textStyle:
            viewport.text(x, y, text, **textStyle)
        else:
            viewport.text(x, y, text)
    

def drawGraph(viewport, g):
    
    for u in g.nodes_iter():
        plt.plot(u.x, u.y, 'o', color='blue')
    
    for u, v in g.edges_iter():
        x = (u.x, v.x)
        y = (u.y, v.y)
        plt.plot(x, y, color='black')

def drawPolicy(viewport, solution, color='black'):
    for u, v in it.izip(solution, solution[1:]):
        dx, dy = v.x - u.x, v.y - u.y
        plt.arrow(u.x, u.y, dx, dy, hold=True, color=color,
                  length_includes_head=True, head_width=0.08)


class Simulate2D(object):
    '''Management class for planar systems and environments. It implements
    visualization of the workspace, saves images or videos, and support playback
    of computed trajectories.
    
    It also handles the simulation of local requests.
    '''
    
    def __init__(self, workspace, robot, expandedWorkspace=None, config=None):
        self.workspace = workspace
        self.expandedWorkspace = expandedWorkspace
        self.robot = robot
        self.requests = tuple()
        
        if config:
            self.config = config
        else:
            self.config = dict()
        self.__defaultConfiguration()
        
        self.offline = None
        self.online = None
        
        self.play_global = True
    
    def __defaultConfiguration(self):
        self.config['x-padding'] = self.config.get('x-padding',
                                                   self.robot.diameter/2)
        self.config['y-padding'] = self.config.get('y-padding',
                                                   self.robot.diameter/2)
        self.config['grid-on'] = self.config.get('grid-on', True)
        self.config['sim-step'] = self.config.get('sim-step', 0.1) # TODO: for simulation video 0.01)
        self.config['video-file'] = self.config.get('video-file', 'video.mp4')
        self.config['video-interval'] = self.config.get('video-interval', 2000)
        self.config['video-writer'] = self.config.get('video-writer', None)
        self.config['output-dir'] = self.config.get('output-dir', '.')
    
    def loadConfiguration(self, filename):
        pass
    
    def saveConfiguration(self, filename):
        pass
    
    def loadTimeline(self, filename):
        pass
    
    def reset(self):
        pass
    
    def makeRequestsRecurrent(self):
        '''TODO:
        '''
        self.requests = self.robot.sensor.requests
    
    def display(self, expanded=False, solution=None, localinfo=None):
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')
        
        if expanded == 'both':
            self.render(ax, expanded=True, solution=[], localinfo=localinfo)
            self.render(ax, expanded=False, solution=solution,
                        localinfo=localinfo)
        else:
            self.render(ax, expanded, solution, localinfo=localinfo)
        
        plt.show()
    
    def render(self, viewport, expanded=False, solution=None, withtext=True,
               localinfo=None):
        if expanded:
            wp = self.expandedWorkspace
        else:
            wp = self.workspace
        
        limits = wp.boundary.boundingBox()
        plt.xlim(limits[0] + np.array([-1, 1]) * self.config['x-padding'])
        plt.ylim(limits[1] + np.array([-1, 1]) * self.config['y-padding'])
        
        if self.config.get('grid-on'):
            plt.grid()
        
        if self.config.get('background', None):
            img = plt.imread(self.config['background'])
            img = np.flipud(img)
            plt.imshow(img, origin='lower', extent=limits.flatten(), zorder=0,
#                        alpha=0.5
                       )
        
        # draw regions
        for r in wp.regions:
            text = None
            if withtext:
                text = r.text
            drawRegion2D(viewport, r, r.style, text, r.textStyle)
        
        # draw boundary
        drawBoundary2D(viewport, wp.boundary, wp.boundary.style)
        
        # draw robot
        r = 0 if expanded else self.robot.diameter/2 
        drawRobot2D(viewport, self.robot, size=r)
        
        if solution is not None:
            drawPolicy(viewport, solution)
        else:
            if self.offline is not None:
                # draw transition system
                drawGraph(viewport, self.offline.ts.g)
        
        # draw local regions/plans/data
        if self.online is not None:
            for req in self.robot.sensor.requests:
                r = req.region
                text = None
                if withtext:
                    text = req.region.text
                if req in self.online.requests: #FIXME: remove hack
                    r.style['facecolor'] = r.style['facecolor'][:3] + (0.2,)
                else:
                    r.style['facecolor'] = r.style['facecolor'][:3] + (0.5,)
                
                drawRegion2D(viewport, r, r.style, text, r.textStyle)
            
            for r in self.robot.sensor.obstacles:
                text = None
                if withtext:
                    text = r.text
                drawRegion2D(viewport, r, r.style, text, r.textStyle)
            
            if 'tree' in localinfo and self.online.lts:
                drawGraph(viewport, self.online.lts.g)
            if 'trajectory' in localinfo:
                drawPolicy(viewport, self.online.trajectory)
            if 'plan' in localinfo:
                local_plan = [self.robot.currentConf] + self.online.local_plan
                drawPolicy(viewport, local_plan, 'blue')
    
    def update(self):
        '''Removes requests serviced by the robot, resets all requests at the
        end of each cycle, and moves them on their paths.
        '''
        conf = self.robot.currentConf
        # remove serviced requests
        self.robot.sensor.requests = [r for r in self.robot.sensor.requests
                                       if not r.region.intersects(conf)]
        # move requests on their paths
        for r in self.robot.sensor.requests:
            v = next(r.region.path)
            r.region.translate(v)
        # reset requests at the start of a cycle, i.e., reaching a final state
        if (self.online.trajectory[-1] in self.online.ts.g
                and self.online.potential[-1] == 0):
            self.robot.sensor.requests = self.requests
            return True
        return False
    
    #--------------------------------------------------------------------------
    
    def simulate(self, loops=2, offline=True):
        '''Simulates the system along the trajectory induced by the policy 
        satisfying the global specification or the trajectory induced by the
        local planner.
        '''
        assert self.offline.checker.foundPolicy()
        if offline:
            prefix, suffix = self.offline.checker.globalPolicy()
            self.solution = (prefix, suffix[1:])
            trajectory = it.chain(prefix, *it.repeat(suffix[1:], times=loops))
        else:
            trajectory = iter(self.online.trajectory)
        
        self.path = [trajectory.next()]
        prevConf = self.path[0]
        stepSize = self.config['sim-step']
        for conf in trajectory:
            d = self.robot.wspace.dist(prevConf, conf)
            n = int(np.ceil(d/float(stepSize)))
            u = (conf.coords - prevConf.coords) * stepSize / d
            print conf.coords, d, n, u
            self.path.extend([Point2D(prevConf.coords + k*u) for k in range(n)])
            prevConf = conf
        self.path.append(conf)
    
    def play(self, output='video'):
        '''Playbacks the stored path.'''
        assert self.path is not None, 'Run simulation first!'
        
        # TODO: delete after debugging
        from lomap import Timer
        print 'Starting playback...'
        
        if output == 'video':
            self.cstep = 0
            prefix, suffix = self.solution
            policy = prefix + suffix
            
            fig = plt.figure()
            ax = fig.add_subplot(111, aspect='equal')
            
            def init_anim():
                self.render(ax, expanded=True, solution=[])
                self.render(ax, expanded=False, solution=policy)
                return []
            
            def run_anim(frame, *args):
    #             logging.info('Processing frame %s!', frame)
    #             self.draw_time_label(frame, loops_iter.next(), d_fsa.next())
                print frame, '/', len(self.path)
                with Timer('Overall'):
                    with Timer('Step'):
                        self.step()
                    with Timer('Clear plot'):
                        plt.cla()
                    with Timer('render expanded workspace'):
                        self.render(ax, expanded=True, solution=[])
                    with Timer('render workspace'):
                        self.render(ax, expanded=False, solution=policy)
                
                return []
              
            N = len(self.path)
             
            self.vehicle_animation = animation.FuncAnimation(fig, run_anim,
                                init_func=init_anim, save_count=N,
                                interval=self.config['video-interval'],
                                blit=False)
        elif output == 'plots':
            pass
#         fname = 'frames/frame_{frame:04d}.png' #TODO: fix path
#         
#         fig = plt.figure()
#         ax = fig.add_subplot(111, aspect='equal')
#         
#         for frame, _ in enumerate(self.path):
# #             self.display(expanded='both', solution= policy)
#             print frame, '/', len(self.path)
#             plt.cla()
#             self.step()
#             self.render(ax, expanded=True, solution=[], withtext=False)
#             self.render(ax, expanded=False, solution=policy)
#             
#             plt.savefig(fname.format(frame=frame))

        else:
            raise NotImplementedError('Unknown output format!')
    
#     def execute(self, loops=2):
#         self.robot.setup()
#         assert self.offline.checker.foundPolicy()
#         prefix, suffix = self.offline.checker.globalPolicy()
#         self.solution = (prefix, suffix[1:])
#         self.path = list(it.chain(prefix, *it.repeat(suffix[1:], times=loops)))
#         
#         self.cstep = -1
#         for c in self.path:
#             print 'Move to:', c.coords
#             self.step()
    
    def save(self, output='video'):
        '''TODO:
        '''
        if output == 'video':
            filename = os.path.join(self.config['output-dir'],
                                    self.config['video-file'])
            logging.info('Saving video to file: %s !', filename)
            self.vehicle_animation.save(filename,
                writer=self.config['video-writer'],
                metadata={'artist': 'Cristian-Ioan Vasile'})
        else:
            raise NotImplementedError('Unknown output format!')
    
    def step(self, steps=1):
        '''TODO:
        '''
        assert self.path is not None, 'Run simulation first!'
        if self.cstep < len(self.path)-1:
            self.cstep += 1
            self.robot.move(self.path[self.cstep])
    
    def rewind(self, steps=1):
        # TODO:
        pass

if __name__ == '__main__':
    import doctest
    doctest.testmod()