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
import itertools as it

from lomap import Ts

from spaces.base import Workspace
from spaces.maps2d import BallRegion2D, BoxRegion2D, PolygonRegion2D, \
                          expandRegion, BoxBoundary2D, BallBoundary2D, \
                          PolygonBoundary2D, Point2D
from robots import FullyActuatedRobot, SimulatedSensor
from planning import RRGPlanner, Request, LocalPlanner
from graphics.planar import addStyle, Simulate2D, drawBallRegion2D


def postprocessing(logfilename, ts_filename, outdir):
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
                if line_data.lower().find('end global planning') >= 0:
                    break
                rrg_stat_data.update(eval(line_data))

        assert rrg_stat_data['Iterations'] == len(rrg_data)

        # third process data on local planner
        rrt_stat_data = dict()
        for line in logfile:
            prefix, line_data = line.split('--')
            if prefix.lower().rfind('info') >= 0:
                if line_data.lower().find('initialize local planner') >= 0:
                    break
                rrt_stat_data.update(eval(line_data))

        rrt_data = []
        execution_data = dict()
        for line in logfile:
            prefix, line_data = line.split('--')
            if prefix.lower().rfind('info') >= 0:
                if line_data.lower().find('local online planning finished') >=0:
                    break
                if line_data.lower().find('start local planning step') >= 0:
                    if execution_data is not None:
                        rrt_data.append(execution_data)
                    execution_data = dict()
                execution_data.update(eval(line_data))
        #TODO: parse tree construction data?

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

    # get local requests and obstacles
    localSpec = data['Local specification']
    requests = []
    obstacles = []
    for key, value in data.iteritems():
        if isinstance(key, tuple) and key[0] == "Local region":
            r, style, path, is_request = value
            # add styles to region
            addStyle(r, style=style)
            # add path
            if path:
                r.path = it.cycle(path)
            # add to requests or obstacles lists
            if is_request:
                name = next(iter(r.symbols))
                requests.append(Request(r, name, localSpec[name]))
            else:
                obstacles.append(r)

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
    sim.offline.ts.init[initConf] = 1
    sim.config['video-interval'] = 500
    sim.config['video-file'] = 'rrg_construction.mp4'
    sim.save_rrg_process(rrg_data)

    # set to global and to save animation
    sim.config['video-interval'] = 30
    sim.config['sim-step'] = 0.01
    sim.config['video-file'] = 'global_plan.mp4'
    sim.simulate(loops=2, offline=True)
    sim.play(output='video', show=False)
    sim.save()

    # get online trajectory
    sim.offline.ts = Ts.load(ts_filename)
    trajectory = [d['new configuration'] for d in rrt_data]
    local_plans = [d['local plan'] for d in rrt_data] + [[]]
    potential = [d['potential'] for d in rrt_data] + [0]
    requests = [d['requests'] for d in rrt_data] + [[]]
    print len(trajectory), len(local_plans)

    # local plan visualization
    sim.online = LocalPlanner(None, sim.offline.ts, robot, localSpec)
    sim.online.trajectory = trajectory
    sim.config['video-interval'] = 30
    sim.config['sim-step'] = 0.01
    sim.config['video-file'] = 'local_plan.mp4'
    sim.simulate(offline=False)
    sim.play(output='video', show=False,
             localinfo={'trajectory': trajectory, 'plans': local_plans,
                        'potential': potential, 'requests': requests})
    sim.save()
    # TODO: local tree visualization
#         if logging.getLogger().isEnabledFor(logging.DEBUG):
#             ll = self.local_plan
#
#             logging.debug('plan to leaf: %s', plan_to_leaf)
#             logging.debug('dest state: %s', dest_state)
#
#             logging.debug('free movement: %s', self.free_movement(dest_state))
#
#             self.local_plan = plan_to_leaf + self.free_movement(dest_state)
#             self.sim.display(expanded=True, localinfo=('plan', 'trajectory'))
#             self.local_plan = ll

#                     self.sim.display(expanded=True, localinfo=('tree',))

if __name__ == '__main__':
    postprocessing(logfilename='../data_ijrr/example1/ijrr_example_1.log',
                   ts_filename='../data_ijrr/example1/ts.yaml',
                   outdir='../data_ijrr/example1')
