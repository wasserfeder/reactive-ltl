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

from spaces.base import Point, Workspace, line_translate
from spaces.maps_nd import BallBoundary, BoxBoundary, BallRegion, BoxRegion
from robots import FullyActuatedRobot, BoundingBoxSimulatedSensor

from planning import RRGPlanner, LocalPlanner, Request
from models import IncrementalProduct
from lomap import compute_potentials
from lomap import Timer


def setup(outputdir='.', logfilename='example.log'):
    '''Setup logging, and set output and debug options.'''
    if not os.path.isdir(outputdir):
        os.makedirs(outputdir)

    # clear root logger
    logging.shutdown()
    del logging.getLogger().handlers[:]

    # configure logging
    fs = '%(asctime)s [%(levelname)s] -- { %(message)s }'
    dfs = '%m/%d/%Y %I:%M:%S %p'
    loglevel = logging.INFO
    logfile = os.path.join(outputdir, logfilename)
    verbose = False
    logging.basicConfig(filename=logfile, filemode='w', level=loglevel,
                        format=fs, datefmt=dfs)
    if verbose:
        root = logging.getLogger()
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(loglevel)
        ch.setFormatter(logging.Formatter(fs, dfs))
        root.addHandler(ch)

def define_problem(dim=3):
    '''Define case study setup (robot parameters, workspace, etc.).'''

    # define robot diameter (used to compute the expanded workspace)
    robotDiameter = 0.0

    # define boundary: unit hypercube
    boundary = BoxBoundary([[0, 1]]*dim)

    # create robot's workspace
    wspace = Workspace(boundary=boundary)

    # create robot object
    initConf = Point((0.1,)*dim) # start close to the origin 
    robot = FullyActuatedRobot('Robot-highdim', init=initConf, wspace=wspace,
                               stepsize=0.999)
    robot.diameter = robotDiameter
    robot.localObst = 'local_obstacle'

    logging.info('"Workspace": %s', wspace)
    logging.info('"Robot name": "%s"', robot.name)
    logging.info('"Robot initial configuration": %s', robot.initConf)
    logging.info('"Robot step size": %f', robot.controlspace)
    logging.info('"Robot diameter": %f', robot.diameter)
    logging.info('"Robot constructor": "%s"',
        'FullyActuatedRobot(robot_name, init=initConf, wspace=ewspace, '
        'stepsize=stepsize)')
    logging.info('"Local obstacle label": "%s"', robot.localObst)

    # regions of interest
    R1 = BoxRegion([[0.00, 0.20], [0.00, 0.20]] + [[0, 1]]*(dim-2), ['r1'])
    R2 = BoxRegion([[0.25, 0.40], [0.40, 0.55]] + [[0, 1]]*(dim-2), ['r2'])
    R3 = BoxRegion([[0.70, 1.00], [0.40, 0.60]] + [[0, 1]]*(dim-2), ['r3'])
    R4 = BoxRegion([[0.00, 0.50], [0.90, 1.00]] + [[0, 1]]*(dim-2), ['r4'])
    # global obstacles
    O1 = BoxRegion([[0.20, 0.30], [0.30, 0.35]] + [[0, 1]]*(dim-2), ['o1'])
    O2 = BoxRegion([[0.15, 0.20], [0.40, 0.60]] + [[0, 1]]*(dim-2), ['o2'])
    O3 = BoxRegion([[0.50, 0.55], [0.30, 0.80]] + [[0, 1]]*(dim-2), ['o3'])

    # add all regions
    regions = [R1, R2, R3, R4, O1, O2, O3]

    # add regions to workspace
    for k, r in enumerate(regions):
        wspace.addRegion(r)
        logging.info('("Global region", %d): (%s, %s)', k, r, None)

#     # local  requests
    F1 = (BallRegion([0.25, 0.10] + [0.50]*(dim-2), 0.30, ['event1']),
          ([[0.25, 0.10] + [0.50]*(dim-2), [0.80, 0.10] + [0.00]*(dim-2),
            [0.50, 0.25] + [1.00]*(dim-2)], 0.02))
    F2 = (BallRegion([0.10, 0.60] + [0.50]*(dim-2), 0.25, ['event1']),
          ([[0.10, 0.60] + [0.50]*(dim-2), [0.25, 0.50] + [0.00]*(dim-2),
            [0.25, 0.85] + [1.00]*(dim-2)], 0.02))
    S2 = (BallRegion([0.50, 0.90] + [0.50]*(dim-2), 0.27, ['event2']),
          ([[0.50, 0.90] + [0.50]*(dim-2), [0.80, 0.90] + [0.00]*(dim-2),
            [0.80, 0.60] + [1.00]*(dim-2)], 0.02))
    requests = [F1, F2, S2]
    # define local specification as a priority function
    localSpec = {'event1': 0, 'event2': 1}
    logging.info('"Local specification": %s', localSpec)
    # local obstacles
    obstacles = [
     (BoxRegion([[0.45, 0.50], [0.75, 0.80]] + [[0, 1]]*(dim-2), ['LO']), None),
     (BoxRegion([[0.90, 1.00], [0.50, 0.55]] + [[0, 1]]*(dim-2), ['LO']), None),
     (BoxRegion([[0.75, 0.80], [0.20, 0.25]] + [[0, 1]]*(dim-2), ['LO']), None)]
    # add style to local requests and obstacles
    for k, (r, path) in enumerate(requests + obstacles):
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
 
        logging.info('("Local region", %d): (%s, %s, %s, %d)', k, r, None,
                     r_path, k < len(requests))

    # create request objects
    reqs = []
    for r, _ in requests:
        name = next(iter(r.symbols))
        reqs.append(Request(r, name, localSpec[name]))
    requests = reqs
    obstacles = [o for o, _ in obstacles]

    # set the robot's sensor
    radius = 0.25 # define sensor's radius
    sensingShape = BallBoundary([0]*dim, radius)
    robot.sensor = BoundingBoxSimulatedSensor(robot, sensingShape, requests,
                                              obstacles)

    logging.info('"Robot sensor constructor": "%s"',
        'SimulatedSensor(robot, BallBoundary([0]*{dim}, {radius}),'
        'requests, obstacles)'.format(dim=dim, radius=radius))

    globalSpec = ('[] ( (<> r1) && (<> r2) && (<> r3) && (<> r4)'
                  + ' && !(o1 || o2 || o3 || o4 ))')
    logging.info('"Global specification": "%s"', globalSpec)

    return robot, globalSpec, localSpec

def generate_global_ts(globalSpec, robot, eta=(0.5, 1.0), ts_file='ts.yaml',
                       outputdir='.'):
    '''Generate global transition system and off-line control policy.'''

    # initialize incremental product automaton
    checker = IncrementalProduct(globalSpec) #, specFile='ijrr_globalSpec.txt')
    logging.info('"Buchi size": %s', checker.buchi.size())

    # initialize global off-line RRG planner
    offline = RRGPlanner(robot, checker, iterations=100000) #TODO: iterations
    offline.eta = eta
    offline.detailed_logging = False

    logging.info('"Start global planning": True')
    with Timer(op_name='global planning', template='"%s runtime": %f'):
        found = offline.solve()
        logging.info('"Found solution": %s', found)

    logging.info('"Iterations": %d', offline.iteration)
    logging.info('"Size of TS": %s', offline.ts.size())
    logging.info('"Size of PA": %s', offline.checker.size())

    # save global transition system and control policy
    if ts_file is not None:
        offline.ts.save(os.path.join(outputdir, ts_file))

    ############################################################################
    ### Display the global transition system and the off-line control policy ###
    ############################################################################

    # display workspace and global transition system
    prefix, suffix = [], []
    if found:
        prefix, suffix = offline.checker.globalPolicy(offline.ts)
    logging.info('"global policy": (%s, %s)', prefix, suffix)
    logging.info('"End global planning": True')

    return found, offline

def update(robot, online):
    '''Removes requests serviced by the robot, resets all requests at the end of
    each cycle, and moves them on their paths.
    '''
    # update requests and local obstacles
    robot.sensor.update()
    # reset requests at the start of a cycle, i.e., reaching a final state
    if (online.trajectory[-1] in online.ts.g and online.potential == 0):
        robot.sensor.reset()
        return True
    return False

def plan_online(localSpec, offline, robot, iterations=2):
    '''Execute on-line path planning algorithm.'''

    # compute potential for each state of PA
    with Timer(op_name='Computing potential function',
               template='"%s runtime": %f'):
        if not compute_potentials(offline.checker):
            return

    # set the step size for the local controller
    robot.controlspace = 0.101

    # initialize local on-line RRT planner
    online = LocalPlanner(offline.checker, offline.ts, robot, localSpec)
    online.PointCls = Point
    online.detailed_logging = False

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
            nextConf = online.execute(requests, obstacles)
        # enforce movement
        robot.move(nextConf)
        # if completed cycle increment cycle
        if update(robot, online):
            print 'End of cycle:', cycle
            cycle += 1

    logging.info('"Local online planning finished": True')

def global_performance(outputdir, logfilename, trials=20, dims=(3,)):
    for dim in dims:
        setup(outputdir, logfilename.format(dim))
        logging.info('"Processing dimension": %d', dim)
        robot, globalSpec, _ = define_problem(dim)
        for k in range(trials):
            np.random.seed(1001 + 100 * k)
            generate_global_ts(globalSpec, robot, eta=(0.1, 0.4), ts_file=None)

def caseStudy(outputdir, logfilename, iterations, dims=(3,)):
    for dim in dims:
        setup(outputdir, logfilename.format(dim))
        logging.info('"Processing dimension": %d', dim)
        robot, globalSpec, localSpec = define_problem(dim)
        found, offline = generate_global_ts(globalSpec, robot, eta=(0.1, 0.4),
                   ts_file='ts_dim{:02d}.yaml'.format(dim), outputdir=outputdir)
        if not found:
            return
        plan_online(localSpec, offline, robot, iterations)

if __name__ == '__main__':
    outputdir = os.path.abspath('../data_ijrr/example4')

#     global_performance(outputdir, trials=100, dims=range(7, 11), # (3, 21)
#                        logfilename='ijrr_example_4_global_dim{:02d}.log')

#     np.random.seed(1001)
#     caseStudy(outputdir, logfilename='ijrr_example_1.log', iterations=2)

    np.random.seed(1001)
    caseStudy(outputdir, logfilename='ijrr_example_4_long_dim{:02d}.log',
              iterations=100, dims=range(3,4))
