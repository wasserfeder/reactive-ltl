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
    '''Draws a point in the planar environment.'''
    if style:
        viewport.plot(point.x, point.y, color=color, **style)
    else:
        viewport.plot(point.x, point.y, color=color)

def drawBoxRegion2D(viewport, region, style, text='', textStyle=None):
    '''Draws a box region in a planar environment.'''
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
    '''Draws a ball region in a planar environment.'''
    x, y = region.center
    c = plt.Circle(region.center, region.radius, **style)
    viewport.add_artist(c)
    if text:
        if textStyle:
            viewport.text(x, y, text, **textStyle)
        else:
            viewport.text(x, y, text)

def drawPolygonRegion2D(viewport, region, style, text='', textStyle=None):
    '''Draws a polygonal region in a planar environment.'''
    x, y = zip(*region.polygon.exterior.coords)
    viewport.fill(x, y, **style)
    if text:
        x, y = region.polygon.centroid.coords[0]
        if textStyle:
            viewport.text(x, y, text, **textStyle)
        else:
            viewport.text(x, y, text)

def drawRegion2D(viewport, region, style, text='', textStyle=None):
    '''Draws a region in a planar environment.'''
    if isinstance(region, BoxRegion2D):
        drawBoxRegion2D(viewport, region, style, text, textStyle)
    elif isinstance(region, BallRegion2D):
        drawBallRegion2D(viewport, region, style, text, textStyle)
    elif isinstance(region, PolygonRegion2D):
        drawPolygonRegion2D(viewport, region, style, text, textStyle)
    else:
        raise TypeError

def drawBoundary2D(viewport, boundary, style):
    '''Draws a boundary in a planar environment.'''
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
    '''Adds style information to the region for rendering.'''
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

def drawRobot2D(viewport, robot, size, text='', textStyle=None,
                sensing_area=True, zorder=2):
    '''Draws the robot with its sensing area in the viewport.'''
    x, y = robot.currentConf.coords
    #TODO: make this general
    if sensing_area:
        sensingShape = BallBoundary2D([x, y], robot.sensor.sensingShape.radius)
        style = {'facecolor': (0, 0, 1, 0.2), 'edgecolor': (0, 0, 1, 0.2),
                 'fill': True, 'zorder': zorder}
        drawBoundary2D(viewport, sensingShape, style)

    style = {'facecolor': 'blue', 'edgecolor':'black', 'zorder': zorder}
    if size == 0:
        plt.plot(x, y, 'o', color=style['facecolor'], zorder=zorder)
    else:
        c = plt.Circle(robot.currentConf.coords, size, **style)
        viewport.add_artist(c)
    if text:
        if textStyle:
            viewport.text(x, y, text, **textStyle)
        else:
            viewport.text(x, y, text)

def drawGraph(viewport, g, node_color='blue', edge_color='black', zorder=2):
    '''Plots the given graph in the viewport.'''
    for u in g.nodes_iter():
        plt.plot(u.x, u.y, 'o', color=node_color, zorder=zorder+1)

    for u, v in g.edges_iter():
        x = (u.x, v.x)
        y = (u.y, v.y)
        plt.plot(x, y, color=edge_color, zorder=zorder)

def drawPolicy(viewport, solution, color='black', alpha_min=1.0, zorder=2):
    '''Draws the a solution path with a fading effect.'''
    if alpha_min == 1.0:
        transparency = it.repeat(1.0)
    else:
        transparency = np.linspace(alpha_min, 1.0, len(solution)-1)
    
    for u, v, a in it.izip(solution, solution[1:], transparency):
        dx, dy = v.x - u.x, v.y - u.y
        plt.arrow(u.x, u.y, dx, dy, hold=True, color=color, alpha=a,
                  length_includes_head=True, head_width=0.08, zorder=zorder)


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

        self.solution = None

        self.play_global = True

    def __defaultConfiguration(self):
        self.config['x-padding'] = self.config.get('x-padding',
                                                   self.robot.diameter/2)
        self.config['y-padding'] = self.config.get('y-padding',
                                                   self.robot.diameter/2)
        self.config['grid-on'] = self.config.get('grid-on', True)
        self.config['sim-step'] = self.config.get('sim-step', 0.1)

        self.config['detected-request-transparency'] = \
                        self.config.get('local-obstacle-transparency', 0.2)
        self.config['request-transparency'] = \
                        self.config.get('request-transparency', 0.5)

        self.config['trajectory-history-length'] = 20
        self.config['trajectory-color'] = 'black'
        self.config['trajectory-min-transparency'] = 0.2
        self.config['local-plan-color'] = 'green'

        self.config['background-transparency'] = 1.0

        self.config['video-file'] = self.config.get('video-file', 'video.mp4')
        self.config['video-interval'] = self.config.get('video-interval', 1000)
        self.config['video-writer'] = self.config.get('video-writer', None)
        self.config['video-codec'] = self.config.get('video-codec', 'libx264')
        self.config['image-file-template'] = \
          self.config.get('image-file-template', 'frames/frame_{frame:04d}.png')
        self.config['output-dir'] = self.config.get('output-dir', '.')

        self.config['z-order'] = {
            'background'          : 0,
            'boundary'            : 1,
            'global region'       : 2,
            'global region text'  : 3,
            'local obstacle'      : 4,
            'local obstacle text' : 5,
            'request'             : 6,
            'request text'        : 7,
            'global ts'           : 8,
            'global plan'         : 10,
            'local ts'            : 11,
            'trajectory'          : 13,
            'local plan'          : 14,
            'robot'               : 15,
            }

    def loadConfiguration(self, filename):
        pass

    def saveConfiguration(self, filename):
        pass

    def loadTimeline(self, filename):
        pass

    def reset(self):
        pass

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
               localinfo=None, sensing_area=True):
        zorder = self.config['z-order']

        if expanded:
            wp = self.expandedWorkspace
        else:
            wp = self.workspace
        wp.boundary.style['zorder'] = zorder['boundary']

        limits = wp.boundary.boundingBox()
        plt.xlim(limits[0] + np.array([-1, 1]) * self.config['x-padding'])
        plt.ylim(limits[1] + np.array([-1, 1]) * self.config['y-padding'])

        if self.config.get('grid-on'):
            plt.grid()

        if self.config.get('background', None):
            img = plt.imread(self.config['background'])
            img = np.flipud(img)
            plt.imshow(img, origin='lower', extent=limits.flatten(),
                       zorder=zorder['background'],
                       alpha=self.config['background-transparency'])

        # draw regions
        for r in wp.regions:
            r.style['zorder'] = zorder['global region']
            text = None
            if withtext:
                text = r.text
                r.textStyle['zorder'] = zorder['global region text']
            drawRegion2D(viewport, r, r.style, text, r.textStyle)

        # draw boundary
        drawBoundary2D(viewport, wp.boundary, wp.boundary.style)

        # draw robot
        r = 0 if expanded else self.robot.diameter/2
        drawRobot2D(viewport, self.robot, size=r, sensing_area=sensing_area,
                    zorder=zorder['robot'])

        if solution is not None:
            drawGraph(viewport, self.offline.ts.g, zorder=zorder['global ts'],
                      node_color='gray', edge_color='gray')
            drawPolicy(viewport, solution, zorder=zorder['global plan'])
        else:
            if self.offline is not None: # draw transition system
                drawGraph(viewport, self.offline.ts.g, zorder=zorder['global ts'])

        # draw local regions/plans/data
        if self.online is not None:
            for req in self.robot.sensor.requests:
                r = req.region
                r.style['zorder'] = zorder['request']
                text = None
                if withtext:
                    text = r.text
                    r.textStyle['zorder'] = zorder['request text']
                if req in self.online.requests:
                    r.style['facecolor'] = r.style['facecolor'][:3] \
                            + (self.config['request-transparency'],)
                else:
                    r.style['facecolor'] = r.style['facecolor'][:3] \
                            + (self.config['detected-request-transparency'],)

                drawRegion2D(viewport, r, r.style, text, r.textStyle)

            for r in self.robot.sensor.obstacles:
                r.style['zorder'] = zorder['local obstacle']
                text = None
                if withtext:
                    text = r.text
                    r.textStyle['zorder'] = zorder['local obstacle text']
                drawRegion2D(viewport, r, r.style, text, r.textStyle)

            if 'tree' in localinfo and self.online.lts:
                drawGraph(viewport, self.online.lts.g, zorder=zorder['local ts'])
            if 'trajectory' in localinfo:
                # compute history 
                history = self.config['trajectory-history-length']
                start = max(0, len(self.online.trajectory) - history)
                drawPolicy(viewport, self.online.trajectory[start:],
                           color=self.config['trajectory-color'],
                           alpha_min=self.config['trajectory-min-transparency'],
                           zorder=zorder['trajectory'])
            if 'plan' in localinfo:
                local_plan = self.online.local_plan
                drawPolicy(viewport, local_plan, zorder=zorder['local plan'],
                           color=self.config['local-plan-color'])

    def update(self):
        '''Removes requests serviced by the robot, resets all requests at the
        end of each cycle, and moves them on their paths.
        '''

        # update requests and local obstacles
        self.robot.sensor.update()
        # reset requests at the start of a cycle, i.e., reaching a final state
        if (self.online.trajectory[-1] in self.online.ts.g
                and self.online.potential == 0):
            self.robot.sensor.reset()
            return True
        return False

    def simulate(self, loops=2, offline=True):
        '''Simulates the system along the trajectory induced by the policy
        satisfying the global specification or the trajectory induced by the
        local planner.
        '''
        self.cstep = 0

        if self.solution is None:
            assert self.offline.checker.foundPolicy()
            prefix, suffix = self.offline.checker.globalPolicy()
            self.solution = (prefix, suffix[1:])

        prefix, suffix = self.solution
        if offline:
            trajectory = it.chain(prefix, *it.repeat(suffix, times=loops))
        else:
            trajectory = iter(self.online.trajectory)

        self.path = [trajectory.next()]
        prevConf = self.path[0]
        stepSize = self.config['sim-step']
        for conf in trajectory:
            d = self.robot.wspace.dist(prevConf, conf)
            n = int(np.ceil(d/float(stepSize)))
            u = (conf.coords - prevConf.coords) * stepSize / d
            self.path.extend([Point2D(prevConf.coords + k*u) for k in range(n)])
            prevConf = conf
        self.path.append(conf)

    def play(self, output='video', show=True, localinfo=dict()):
        '''Playbacks the stored path.'''
        assert self.path is not None, 'Run simulation first!'

        logging.info('Starting playback...')

        self.cstep = 0
        prefix, suffix = self.solution
        policy = prefix + suffix

        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')

        local = 'trajectory' in localinfo
        rendering_options = ()

        if local:
            self.traj_step = 0
            trajectory = localinfo['trajectory']
            potential = localinfo['potential']
            rendering_options = ('trajectory', 'plan')

        def init_anim():
            self.render(ax, expanded=True, solution=[],
                        localinfo=('trajectory',))
            self.render(ax, expanded=False, solution=policy,
                        localinfo=('trajectory',))
            return []

        def run_anim(frame, *args):

            if local:
                if self.path[self.cstep] == trajectory[self.traj_step]:
                    self.traj_step += 1
                    self.online.potential = potential[self.traj_step]
                    self.update()
                    current_requests = localinfo['requests'][self.traj_step]
                    self.robot.sensor.requests = [
                                    r for r in self.robot.sensor.all_requests
                                        if r in current_requests]
                    print self.traj_step

                self.online.trajectory = trajectory[:self.traj_step+1]
                self.online.local_plan = localinfo['plans'][self.traj_step]
            
            self.step()
            plt.cla()
            self.render(ax, expanded=True, solution=policy,
                        localinfo=rendering_options)
            return []

        if output == 'video':
            self.vehicle_animation = animation.FuncAnimation(fig, run_anim,
                            init_func=init_anim, save_count=len(self.path),
                            interval=self.config['video-interval'], blit=False)
        elif output == 'plots':
            self.draw_plot = run_anim
        else:
            raise NotImplementedError('Unknown output format!')

        if show:
            plt.show()

    def save(self, output='video'):
        '''Saves the playback data in the given output format.'''
        if output == 'video':
            filename = os.path.join(self.config['output-dir'],
                                    self.config['video-file'])
            logging.info('Saving video to file: %s !', filename)
            self.vehicle_animation.save(filename,
                writer=self.config['video-writer'],
                codec=self.config['video-codec'],
                metadata={'artist': 'Cristian-Ioan Vasile'})
        elif output == 'plots':
            filename = os.path.join(self.config['output-dir'],
                                    self.config['image-file-template'])
            basedir = os.path.dirname(filename)
            if not os.path.isdir(basedir):
                os.makedirs(basedir)

            for frame, _ in enumerate(self.path):
                self.draw_plot(frame)
                logging.info('Saving frame %d to file: %s !', frame, filename)
                plt.savefig(filename.format(frame=frame))
        else:
            raise NotImplementedError('Unknown output format!')

    def step(self, steps=1):
        '''Moves the robot along the stored path.'''
        assert self.path is not None, 'Run simulation first!'
        if self.cstep < len(self.path)-1:
            self.cstep += 1
            self.robot.move(self.path[self.cstep])
            return True
        return False

    def save_rrg_process(self, rrg_data, markersize=12,
                         max_detailed_iteration = 20, endframe_hold=4):
        '''Saves a video with generation of the RRG transition system.'''
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')

        def init_rrg_anim():
            self.render(ax, expanded=True)
            return []

        rrg_display_data = iter(rrg_data + [rrg_data[-1]]*endframe_hold)
        class LocalContext(object): pass
        ctx = LocalContext()
        ctx.phases = ('sampling', 'nearest', 'steering', 'forward', 'backward')
        ctx.phases_remaning = iter([])
        ctx.display_data = None
        ctx.max_detailed_iteration = max_detailed_iteration
        def run_rrg_anim(frame, *args):
            plt.cla()

            try:
                phase = next(ctx.phases_remaning)
            except StopIteration:
                ctx.display_data = next(rrg_display_data)
                ctx.phases_remaning = iter(ctx.phases)
                phase = next(ctx.phases_remaning)
            display_data = ctx.display_data

            print 'Iteration:', display_data['iteration'],
            print 'Frame:', frame, '/', nframes

            if display_data['iteration'] > ctx.max_detailed_iteration:
                plt.title('iteration: {}'.format(display_data['iteration']))
                sample = display_data['random configuration']
                plt.plot(sample.x, sample.y, 'o', color='orange', ms=markersize)
                self.offline.ts.g.add_edges_from(
                                            display_data['forward edges added'])
                self.offline.ts.g.add_edges_from(
                                        display_data['backward edges added'])
                self.render(ax, expanded=True, sensing_area=False)
                ctx.phases_remaning = iter([])
                return []

            self.render(ax, expanded=True, sensing_area=False)
            plt.title('iteration: {}, phase: {}'.format(
                                            display_data['iteration'], phase))
            if phase in ('sampling', 'nearest', 'steering'):
                sample = display_data['random configuration']
                plt.plot(sample.x, sample.y, 'o', color='orange', ms=markersize)
            if phase in ('nearest', 'steering'):
                nearest = display_data['nearest state']
                plt.plot(nearest.x, nearest.y, 'o', color='gray', alpha=.4,
                         ms=markersize)
            if phase == 'steering':
                new_conf = display_data['new configuration']
                plt.plot(new_conf.x, new_conf.y, 'o', color='green',
                         ms=markersize)
                plt.plot([nearest.x, new_conf.x], [nearest.y, new_conf.y],
                         color='black', linestyle='dashed')

            if phase == 'forward':
                new_conf = display_data['new configuration']
                eta = display_data['far'][1:]
                drawBallRegion2D(ax, BallRegion2D(new_conf.coords, eta[1], set()),
                                 style={'color':'gray', 'alpha':.2})
                drawBallRegion2D(ax, BallRegion2D(new_conf.coords, eta[0], set()),
                                 style={'color':'gray', 'alpha':.2})
                if display_data['forward state added'] is not None:
                    plt.plot(new_conf.x, new_conf.y, 'o', color='green',
                             ms=markersize)
                    for u, v in display_data['forward edges added']:
                        dx, dy = v.x - u.x, v.y - u.y
                        plt.arrow(u.x, u.y, dx, dy, color='black',
                                  length_includes_head=True, head_width=0.08)
                else:
                    plt.plot(new_conf.x, new_conf.y, 'o', color='red',
                             ms=markersize)
                self.offline.ts.g.add_edges_from(
                                            display_data['forward edges added'])

            if phase == 'backward':
                new_conf = display_data['new configuration']
                if display_data['forward state added'] is not None:
                    plt.plot(new_conf.x, new_conf.y, 'o', color='green',
                             ms=markersize)
                else:
                    plt.plot(new_conf.x, new_conf.y, 'o', color='red',
                             ms=markersize)
                for u, v in display_data['backward edges added']:
                    dx, dy = v.x - u.x, v.y - u.y
                    plt.arrow(u.x, u.y, dx, dy, color='black',
                              length_includes_head=True, head_width=0.08)
                self.offline.ts.g.add_edges_from(
                                        display_data['backward edges added'])
            return []

        nframes = ((len(ctx.phases)-1) * ctx.max_detailed_iteration
                    + len(rrg_data) + endframe_hold)
        rrg_animation = animation.FuncAnimation(fig, run_rrg_anim,
                            init_func=init_rrg_anim, save_count=nframes,
                            interval=self.config['video-interval'], blit=False)
        filename = os.path.join(self.config['output-dir'],
                                self.config['video-file'])
        rrg_animation.save(filename, writer=self.config['video-writer'],
                           codec=self.config['video-codec'],
                           metadata={'artist': 'Cristian-Ioan Vasile'})

    def save_lts_process(self, rrt_data, markersize=10,
                         max_detailed_iteration = 20, endframe_hold=4):
        '''Saves a video with generation of the local RRT transition system.'''
        # move robot to current configuration
        conf = iter(self.online.lts.init).next()
        self.robot.move(conf)
        # reset local transition system
        self.online.lts.g.clear()
        self.online.lts.g.add_node(conf)

        zorder = self.config['z-order']

        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')

        def init_lts_anim():
            self.render(ax, expanded=True, localinfo=('tree',))
            return []

        rrt_display_data = iter(rrt_data + [rrt_data[-1]]*endframe_hold)
        class LocalContext(object): pass
        ctx = LocalContext()
        ctx.phases = ('sampling', 'nearest', 'steering', 'check')
        ctx.phases_remaning = iter([])
        ctx.display_data = None
        ctx.max_detailed_iteration = max_detailed_iteration

        def run_lts_anim(frame, *args):
            plt.cla()

            try:
                phase = next(ctx.phases_remaning)
            except StopIteration:
                ctx.display_data = next(rrt_display_data)
                ctx.phases_remaning = iter(ctx.phases)
                phase = next(ctx.phases_remaning)
            (iteration, sample, nearest, new_conf, simple_segment, empty_buchi,
             collision_free, hit, global_target_state) = ctx.display_data

            print 'Iteration:', iteration,
            print 'Frame:', frame, '/', nframes

            if iteration > ctx.max_detailed_iteration:
                plt.title('iteration: {}'.format(iteration))
                if hit and frame > nframes-endframe_hold:
                    u, v = new_conf, global_target_state
                    dx, dy = v.x - u.x, v.y - u.y
                    plt.arrow(u.x, u.y, dx, dy, color='yellow',
                              length_includes_head=True, head_width=0.02,
                              zorder=zorder['local ts'])
                else:
                    plt.plot(sample.x, sample.y, 'o', color='orange',
                             ms=markersize, zorder=zorder['local ts'])
                if collision_free:
                    self.online.ts.g.add_edge(nearest, new_conf)
                self.render(ax, expanded=True, localinfo=('tree',))
                ctx.phases_remaning = iter([])
                return []

            self.render(ax, expanded=True, localinfo=('tree',))
            plt.title('iteration: {}, phase: {}'.format(iteration, phase))
            if phase in ('sampling', 'nearest', 'steering'):
                plt.plot(sample.x, sample.y, 'o', color='orange', ms=markersize,
                         zorder=zorder['local ts'])
            if phase in ('nearest', 'steering'):
                plt.plot(nearest.x, nearest.y, 'o', color='gray', alpha=.4,
                         ms=markersize, zorder=zorder['local ts'])
            if phase == 'steering':
                plt.plot(new_conf.x, new_conf.y, 'o', color='green',
                         ms=markersize)
                plt.plot([nearest.x, new_conf.x], [nearest.y, new_conf.y],
                         color='black', linestyle='dashed',
                         zorder=zorder['local ts'])

            if phase == 'check':
                if collision_free:
                    if hit:
                        color = 'yellow'
                    else:
                        color = 'green'
                    plt.plot(new_conf.x, new_conf.y, 'o', color=color,
                             ms=markersize, zorder=zorder['local ts'])
                    arrows = [(nearest, new_conf, 'black')]
                    if hit and frame >= nframes-endframe_hold:
                        arrows.append((new_conf, global_target_state, 'yellow'))
                    for u, v, c in arrows:
                        dx, dy = v.x - u.x, v.y - u.y
                        plt.arrow(u.x, u.y, dx, dy, color=c,
                                  length_includes_head=True, head_width=0.02,
                                  zorder=zorder['local ts'])
                    self.online.ts.g.add_edge(nearest, new_conf)
                else:
                    if not simple_segment:
                        c = 'violet'
                    elif not empty_buchi:
                        c = 'magenta'
                    else:
                        c = 'red'
                    plt.plot(new_conf.x, new_conf.y, 'o', color=c,
                             ms=markersize, zorder=zorder['local ts'])

            return []

        nframes = ((len(ctx.phases)-1) * ctx.max_detailed_iteration
                    + len(rrt_data) + endframe_hold)
        rrt_animation = animation.FuncAnimation(fig, run_lts_anim,
                            init_func=init_lts_anim, save_count=nframes,
                            interval=self.config['video-interval'], blit=False)
        filename = os.path.join(self.config['output-dir'],
                                self.config['video-file'])
        rrt_animation.save(filename, writer=self.config['video-writer'],
                           codec=self.config['video-codec'],
                           metadata={'artist': 'Cristian-Ioan Vasile'})


if __name__ == '__main__':
    import doctest
    doctest.testmod()
