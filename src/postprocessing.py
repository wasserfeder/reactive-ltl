'''
.. module:: postprocessing.py
   :synopsis: TODO:

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

license_text='''
    Post-processing module of logged simulation data.
    Copyright (C) 2018  Cristian Ioan Vasile <cvasile@bu.edu>
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

import matplotlib.pyplot as plt
import matplotlib.animation as animation

from spaces.base import Workspace
from spaces.maps2d import BallRegion2D, BoxRegion2D, PolygonRegion2D, \
                          expandRegion, BoxBoundary2D, BallBoundary2D, Point2D
from robots import FullyActuatedRobot, SimulatedSensor
from planning import RRGPlanner
from graphics.planar import addStyle, Simulate2D, to_rgba, drawBallRegion2D


def postprocessing(logfilename, outdir):
    '''Parses log file and generate statistics and figures.'''

    if not os.path.isdir(outdir):
        os.makedirs(outdir)

    with open(logfilename, 'r') as logfile:
        # first process general data
        data = dict()
        line = ''
        for line in logfile:
            prefix, line_data = line.split('--')
            if prefix.lower().rfind('info') >= 0:
                data.update(eval(line_data))
                if line_data.lower().find('start global planning') >= 0:
                    break
        print 'general data:', len(data)

        # second process data on global planner
        rrg_data = []
        iteration_data = None
        for line in logfile:
            prefix, line_data = line.split('--')
            if prefix.lower().rfind('info') >= 0:
                if line_data.lower().rfind('found solution') >=0:
                    rrg_data.append(iteration_data)
                    break
                if line_data.lower().find('iteration') >= 0:
                    if iteration_data is not None:
                        rrg_data.append(iteration_data)
                    iteration_data = dict()
                iteration_data.update(eval(line_data))

        print 'rrg data:', len(rrg_data)
        rrg_stat_data = eval(line_data)
        for line in logfile:
            prefix, line_data = line.split('--')
            if prefix.lower().rfind('info') >= 0:
                if line_data.lower().find('starting playback') >= 0:
                    break
                rrg_stat_data.update(eval(line_data))

        assert rrg_stat_data['Iterations'] == len(rrg_data)

        # third process data on local planner
        pass #TODO:

    # get workspace
    wspace, style = data['Workspace']
    wspace.boundary.style = style
    ewspace, style = data['Expanded workspace']
    ewspace.boundary.style = style

    # get robot
    robot_name = data['Robot name']
    initConf = data['Robot initial configuration']
    stepsize = data['Robot step size']
    robot = eval(data['Robot constructor'])
    robot.diameter = data['Robot diameter']
    robot.localObst = data['Local obstacle label']

    # create simulation object
    sim = Simulate2D(wspace, robot, ewspace)
    sim.config['output-dir'] = outdir
    sim.config['video-interval'] = 500

    # add regions to workspace
    for key, value in data.iteritems():
        if isinstance(key, tuple) and key[0] == "Global region":
            r, style = value
            # add styles to region
            addStyle(r, style=style)
            # add region to workspace
            sim.workspace.addRegion(r)
            # create expanded region
            er = expandRegion(r, robot.diameter/2)
            # add style to the expanded region
            addStyle(er, style=style)
            # add expanded region to the expanded workspace
            sim.expandedWorkspace.addRegion(er)

    #TODO: get local requests and obstacles
    requests = []
    obstacles = []

    # get robot sensor
    robot.sensor = eval(data['Robot sensor constructor'])

    # display workspace
    sim.display()

    # display expanded workspace
    sim.display(expanded=True)

    # display solution for off-line problem
    prefix, suffix = rrg_stat_data['global policy']
    sim.solution = (prefix, suffix[1:])
    sim.display(expanded='both', solution=prefix+suffix[1:])

    # show construction of rrg
    sim.offline = RRGPlanner(robot, None, 1)
    sim.offline.ts.init[robot.initConf] = 1

    fig = plt.figure()
    ax = fig.add_subplot(111, aspect='equal')

    def init_rrg_anim():
        sim.render(ax, expanded='both')
        return []

    endframe_hold = 4
    rrg_display_data = iter(rrg_data + [rrg_data[-1]]*endframe_hold)
    class LocalContext(object): pass
    ctx = LocalContext()
    ctx.phases = ('sampling', 'nearest', 'steering', 'forward', 'backward')
    ctx.phases_remaning = iter([])
    ctx.display_data = None
    ctx.max_detailed_iteration = 20
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
            plt.plot(sample.x, sample.y, 'o', color='orange', markersize=12)
            sim.offline.ts.g.add_edges_from(display_data['forward edges added'])
            sim.offline.ts.g.add_edges_from(display_data['backward edges added'])
            sim.render(ax, expanded=True, sensing_area=False)
            ctx.phases_remaning = iter([])
            return []

        sim.render(ax, expanded=True, sensing_area=False)
        plt.title('iteration: {}, phase: {}'.format(display_data['iteration'],
                                                    phase))
        if phase in ('sampling', 'nearest', 'steering'):
            sample = display_data['random configuration']
            plt.plot(sample.x, sample.y, 'o', color='orange', markersize=12)
        if phase in ('nearest', 'steering'):
            nearest = display_data['nearest state']
            plt.plot(nearest.x, nearest.y, 'o', color='gray', alpha=.4,
                     markersize=12)
        if phase == 'steering':
            new_conf = display_data['new configuration']
            plt.plot(new_conf.x, new_conf.y, 'o', color='green', markersize=12)
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
                         markersize=12)
                for u, v in display_data['forward edges added']:
                    dx, dy = v.x - u.x, v.y - u.y
                    plt.arrow(u.x, u.y, dx, dy, color='black',
                              length_includes_head=True, head_width=0.08)
            else:
                plt.plot(new_conf.x, new_conf.y, 'o', color='red',
                         markersize=12)
            sim.offline.ts.g.add_edges_from(display_data['forward edges added'])

        if phase == 'backward':
            new_conf = display_data['new configuration']
            if display_data['forward state added'] is not None:
                plt.plot(new_conf.x, new_conf.y, 'o', color='green',
                         markersize=12)
            else:
                plt.plot(new_conf.x, new_conf.y, 'o', color='red',
                         markersize=12)
            for u, v in display_data['backward edges added']:
                dx, dy = v.x - u.x, v.y - u.y
                plt.arrow(u.x, u.y, dx, dy, color='black',
                          length_includes_head=True, head_width=0.08)
            sim.offline.ts.g.add_edges_from(display_data['backward edges added'])

        return []

    nframes = ((len(ctx.phases)-1) * ctx.max_detailed_iteration
                + len(rrg_data) + endframe_hold)
    rrg_animation = animation.FuncAnimation(fig, run_rrg_anim,
                            init_func=init_rrg_anim, save_count=nframes,
                            interval=sim.config['video-interval'], blit=False)
    filename = os.path.join(sim.config['output-dir'], 'rrg_construction.mp4')
    rrg_animation.save(filename, writer=sim.config['video-writer'],
                       codec=sim.config['video-codec'],
                       metadata={'artist': 'Cristian-Ioan Vasile'})

    # set to global and to save animation
    sim.config['video-interval'] = 30
    sim.config['sim-step'] = 0.01
    sim.simulate(loops=2, offline=True)
    sim.play(output='video', show=False)
    sim.save()

if __name__ == '__main__':
    postprocessing(logfilename='../data_ijrr/example1/ijrr_example_1.log',
                   outdir='../data_ijrr/example1')
