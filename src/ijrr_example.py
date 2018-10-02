#!/usr/bin/env python
'''
.. module:: ijrr_example
   :synopsis: Defines the case study presented in the IJRR journal paper.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    Defines the case study presented in the IJRR journal paper.
    Copyright (C) 2014-2018  Cristian Ioan Vasile <cvasile@bu.edu>
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

import os, sys
import logging
import itertools as it

import numpy as np

from spaces.base import Workspace, line_translate
from spaces.maps2d import BallRegion2D, BoxRegion2D, PolygonRegion2D, \
                          expandRegion, BoxBoundary2D, BallBoundary2D, Point2D
from robots import FullyActuatedRobot, SimulatedSensor

from planning import RRGPlanner, LocalPlanner, Request
from models import IncrementalProduct
from lomap import compute_potentials
from graphics.planar import addStyle, Simulate2D, to_rgba
from lomap import Timer


def setup(outputdir='.', logfilename='example.log'):
    '''Setup logging, and set output and debug options.'''
    if not os.path.isdir(outputdir):
        os.makedirs(outputdir)

    # configure logging
    fs = '%(asctime)s [%(levelname)s] -- { %(message)s }'
    dfs = '%m/%d/%Y %I:%M:%S %p'
    loglevel = logging.DEBUG
    logfile = os.path.join(outputdir, logfilename)
    verbose = True
    logging.basicConfig(filename=logfile, filemode='w', level=loglevel,
                        format=fs, datefmt=dfs)
    if verbose:
        root = logging.getLogger()
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(loglevel)
        ch.setFormatter(logging.Formatter(fs, dfs))
        root.addHandler(ch)

def define_problem(outputdir='.'):
    '''Define case study setup (robot parameters, workspace, etc.).'''

    # define robot diameter (used to compute the expanded workspace)
    robotDiameter = 0.36

    # define boundary
    boundary = BoxBoundary2D([[0, 4.8], [0, 3.6]])
    # define boundary style
    boundary.style = {'color' : 'black'}
    # create expanded region
    eboundary = BoxBoundary2D(boundary.ranges +
                                np.array([[1, -1], [1, -1]]) * robotDiameter/2)
    eboundary.style = {'color' : 'black'}

    # create robot's workspace and expanded workspace
    wspace = Workspace(boundary=boundary)
    ewspace = Workspace(boundary=eboundary)

    # create robot object
    robot = FullyActuatedRobot('Cozmo', init=Point2D((2, 2)), wspace=ewspace,
                               stepsize=0.999)
    robot.diameter = robotDiameter
    robot.localObst = 'local_obstacle'

    logging.info('"Workspace": (%s, %s)', wspace, boundary.style)
    logging.info('"Expanded workspace": (%s, %s)', ewspace, eboundary.style)
    logging.info('"Robot name": "%s"', robot.name)
    logging.info('"Robot initial configuration": %s', robot.initConf)
    logging.info('"Robot step size": %f', robot.controlspace)
    logging.info('"Robot diameter": %f', robot.diameter)
    logging.info('"Robot constructor": "%s"',
        'FullyActuatedRobot(robot_name, init=initConf, wspace=ewspace, '
        'stepsize=stepsize)')
    logging.info('"Local obstacle label": "%s"', robot.localObst)

    # create simulation object
    sim = Simulate2D(wspace, robot, ewspace)
    sim.config['output-dir'] = outputdir

    # regions of interest
    R1 = (BoxRegion2D([[0.45, 0.75], [0.45, 0.6]], ['r1']), 'brown')
    R2 = (BallRegion2D([3.9, 1.2], 0.3, ['r2']), 'green')
    R3 = (BoxRegion2D([[3.75, 4.05], [2.7, 3]], ['r3']), 'red')
    R4 = (PolygonRegion2D([[1.2 , 2.85], [0.9 , 3.15], [0.6 , 2.85],
                           [0.9 , 2.55], [1.2 , 2.55]], ['r4']), 'magenta')
    # global obstacles
    O1 = (BallRegion2D([2.4, 1.5], 0.36, ['o1']), 'gray')
    O2 = (PolygonRegion2D([[0.45, 1.2], [1.05, 1.5], [0.45, 1.8]], ['o2']),
          'gray')
    O3 = (PolygonRegion2D([[2.1, 2.7], [2.4, 3], [2.7, 2.7]], ['o3']), 'gray')
    O4 = (BoxRegion2D([[1.65, 1.95], [0.45, 0.6]], ['o4']), 'gray')

    # add all regions
    regions = [R1, R2, R3, R4, O1, O2, O3, O4]

    # add regions to workspace
    for k, (r, c) in enumerate(regions):
        # add styles to region
        addStyle(r, style={'facecolor': c})
        # add region to workspace
        sim.workspace.addRegion(r)
        # create expanded region
        er = expandRegion(r, robot.diameter/2)
        # add style to the expanded region
        addStyle(er, style={'facecolor': c})
        # add expanded region to the expanded workspace
        sim.expandedWorkspace.addRegion(er)

        logging.info('("Global region", %d): (%s, %s)', k, r, r.style)

    # local  requests
    F1 = (BallRegion2D([3.24, 1.98], 0.3, ['fire']),
          ([[3.24, 1.98], [2.54, 2.28], [3.5, 3], [4.02, 2.28]], 0.05),
          ('orange', 0.5))
    F2 = (BallRegion2D([1.26, 0.48], 0.18, ['fire']),
          ([[1.26, 0.48], [1.1, 1.1], [1.74, 0.92], [0.6, 0.6]], 0.05),
          ('orange', 0.5))
    S2 = (BallRegion2D([4.32, 1.48], 0.27, ['survivor']),
          ([[4.32, 1.48], [3.6, 1.2], [4, 2]], 0.05),
          ('yellow', 0.5))
    requests = [F1, F2, S2]
    # define local specification as a priority function
    localSpec = {'survivor': 0, 'fire': 1}
    logging.info('"Local specification": %s', localSpec)
    # local obstacles #FIXME: defined in expanded workspace not workspace
    obstacles = [(BoxRegion2D([[3, 3.5], [2, 2.5]], ['LO']), None,
                  ('gray', 0.8)),
                 (PolygonRegion2D([[3.2, 1.4], [3, 0.8], [3.4, 0.7]], ['LO']),
                  None, ('gray', 0.8)),
                 (BallRegion2D([1.6, 2.1], 0.15, ['LO']), None, ('gray', 0.8))]

    # add style to local requests and obstacles
    for k, (r, path, c) in enumerate(requests + obstacles):
        # add styles to region
        addStyle(r, style={'facecolor': to_rgba(*c)}) #FIXME: HACK
        # create path
        r_path = path
        if path:
            wps, step = path
            wps = wps + [wps[0]]
            r.path = []
            for a, b in it.izip(wps[:-1], wps[1:]):
                r.path += line_translate(a, b, step)
            r_path = map(list, r.path)
            r.path = it.cycle(r.path)

        logging.info('("Local region", %d): (%s, %s, %s, %d)', k, r, r.style,
                     r_path, k < len(requests))

    # create request objects
    reqs = []
    for r, _, _ in requests:
        name = next(iter(r.symbols))
        reqs.append(Request(r, name, localSpec[name]))
    requests = reqs
    obstacles = [o for o, _, _ in obstacles]

    # set the robot's sensor
    sensingShape = BallBoundary2D([0, 0], robot.diameter*2.5)
    robot.sensor = SimulatedSensor(robot, sensingShape, requests, obstacles)

    logging.info('"Robot sensor constructor": "%s"',
        'SimulatedSensor(robot, BallBoundary2D([0, 0], robot.diameter*2.5),'
        'requests, obstacles)')

    # display workspace
    sim.display()

    # display expanded workspace
    sim.display(expanded=True)

    globalSpec = ('[] ( (<> r1) && (<> r2) && (<> r3) && (<> r4)'
                  + ' && !(o1 || o2 || o3 || o4 ))')
    logging.info('"Global specification": "%s"', globalSpec)


    return robot, sim, globalSpec, localSpec

def generate_global_ts(globalSpec, sim, robot, eta=(0.5, 1.0),
                       ts_file='ts.yaml', outputdir='.', show=True):
    '''Generate global transition system and off-line control policy.'''

    # initialize incremental product automaton
    checker = IncrementalProduct(globalSpec) #, specFile='ijrr_globalSpec.txt')
    logging.info('"Buchi size": (%d, %d)', checker.buchi.g.number_of_nodes(),
                                           checker.buchi.g.number_of_edges())

    # initialize global off-line RRG planner
    sim.offline = RRGPlanner(robot, checker, iterations=1000)
    sim.offline.eta = eta

    logging.info('"Start global planning": True')
    with Timer(op_name='global planning', template='"%s runtime": %f'):
        found = sim.offline.solve()
        logging.info('"Found solution": %s', found)

    logging.info('"Iterations": %d', sim.offline.iteration)
    logging.info('"Size of TS": %s', sim.offline.ts.size())
    logging.info('"Size of PA": %s', sim.offline.checker.size())

    # save global transition system and control policy
    if ts_file is not None:
        sim.offline.ts.save(os.path.join(outputdir, ts_file))

    ############################################################################
    ### Display the global transition system and the off-line control policy ###
    ############################################################################

    # display workspace and global transition system
    prefix, suffix = [], []
    if found:
        prefix, suffix = sim.offline.checker.globalPolicy(sim.offline.ts)
        if show:
            sim.display(expanded='both', solution=prefix+suffix[1:])
    logging.info('"global policy": (%s, %s)', prefix, suffix)
    logging.info('"End global planning": True')

    return found

def plan_online(localSpec, sim, robot, iterations=2):
    '''Execute on-line path planning algorithm.'''

    # compute potential for each state of PA
    with Timer(op_name='Computing potential function',
               template='"%s runtime": %f'):
        if not compute_potentials(sim.offline.checker):
            return

    # set the step size for the local controller
    robot.controlspace = 0.101

    # initialize local on-line RRT planner
    sim.online = LocalPlanner(sim.offline.checker, sim.offline.ts, robot,
                              localSpec)
    sim.online.detailed_logging = True

    # define number of surveillance cycles to run
    cycles = iterations
    # execute controller
    cycle = -1 # number of completed cycles, -1 accounts for the prefix
    while cycle < cycles:
        logging.info('"Start local planning step": True')
        # update the locally sensed requests and obstacles
        requests, obstacles = robot.sensor.sense()
        with Timer(op_name='local planning', template='"%s execution": %f'):
            # feed data to planner and get next control input
            nextConf = sim.online.execute(requests, obstacles)
        # enforce movement
        robot.move(nextConf)
        # if completed cycle increment cycle
        if sim.update():
            cycle += 1

    logging.info('"Local online planning finished": True')

def global_performance(outputdir, logfilename, trials=20):
    setup(outputdir, logfilename)
    robot, sim, globalSpec, _ = define_problem(outputdir)
    for k in range(trials):
        np.random.seed(1001 + 100 * k)
        generate_global_ts(globalSpec, sim, robot, eta=(0.3, 1.0), ts_file=None,
                           show=False)

def caseStudy(outputdir, logfilename, iterations):
    setup(outputdir, logfilename)
    robot, sim, globalSpec, localSpec = define_problem(outputdir)
    if not generate_global_ts(globalSpec, sim, robot, outputdir=outputdir):
        return
    plan_online(localSpec, sim, robot, iterations)

if __name__ == '__main__':
    outputdir = os.path.abspath('../data_ijrr/example1')

#     global_performance(outputdir, logfilename='ijrr_example_1_global.log',
#                        trials=100)

#     np.random.seed(1001)
#     caseStudy(outputdir, logfilename='ijrr_example_1.log', iterations=2)

    np.random.seed(1001)
    caseStudy(outputdir, logfilename='ijrr_example_1_long.log', iterations=100)

