#!/usr/bin/env python
'''
.. module:: ijrr_example_cozmo
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

import os, sys, time
import logging

import numpy as np

from spaces.base import Workspace
from spaces.maps2d import BallRegion2D, BoxRegion2D, PolygonRegion2D, \
                          expandRegion, BoxBoundary2D, BallBoundary2D, Point2D
from robots import Cozmo, CozmoSensor

from planning import RRGPlanner, LocalPlanner, Request
from models import IncrementalProduct
from lomap import compute_potentials
from graphics.planar import addStyle, Simulate2D, to_rgba
from lomap import Timer


def caseStudy():
    ############################################################################
    ### Output and debug options ###############################################
    ############################################################################
    outputdir = os.path.abspath('../data_ijrr/example2')
    if not os.path.isdir(outputdir):
        os.makedirs(outputdir)

    # configure logging
    fs = '%(asctime)s [%(levelname)s] -- { %(message)s }'
    dfs = '%m/%d/%Y %I:%M:%S %p'
    loglevel = logging.DEBUG
    logfile = os.path.join(outputdir, 'ijrr_example_2.log')
    verbose = True
    logging.basicConfig(filename=logfile, level=loglevel, format=fs,
                        datefmt=dfs)
    if verbose:
        root = logging.getLogger()
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(loglevel)
        ch.setFormatter(logging.Formatter(fs, dfs))
        root.addHandler(ch)


    ############################################################################
    ### Define case study setup (robot parameters, workspace, etc.) ############
    ############################################################################

    # define robot diameter (used to compute the expanded workspace)
    robotDiameter = 0.036

    # define boundary
    nrow, ncol = 5, 6
    cell_size = 0.591
    boundary = BoxBoundary2D([[0, ncol * cell_size], [0, nrow * cell_size]])
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
    robot = Cozmo('Cozmo', init=Point2D((1.5 * cell_size, 2.5 * cell_size)),
                  wspace=ewspace, stepsize=0.999)
    robot.diameter = robotDiameter
    robot.localObst = 'local_obstacle'

    logging.info('"Workspace": (%s, %s)', wspace, boundary.style)
    logging.info('"Expanded workspace": (%s, %s)', ewspace, eboundary.style)
    logging.info('"Robot name": "%s"', robot.name)
    logging.info('"Robot initial configuration": %s', robot.initConf)
    logging.info('"Robot step size": %f', robot.controlspace)
    logging.info('"Robot diameter": %f', robot.diameter)
    logging.info('"Robot constructor": "%s"',
        'Cozmo(robot_name, init=initConf, wspace=ewspace, stepsize=stepsize)')
    logging.info('"Local obstacle label": "%s"', robot.localObst)

    # create simulation object
    sim = Simulate2D(wspace, robot, ewspace)
    sim.config['output-dir'] = outputdir
#     sim.config['background'] = os.path.abspath('../data_ijrr/imMap.png')

    # regions of interest
    R1 = (BoxRegion2D(np.array([[1.,2.],[4.,5.]])*cell_size, ['r1']), 'brown')
    R2 = (BoxRegion2D(np.array([[4.,5.],[1.,2.]])*cell_size, ['r2']), 'green')
    R3 = (BoxRegion2D(np.array([[0.,1.],[0.,1.]])*cell_size, ['r3']), 'red')
    R4 = (BoxRegion2D(np.array([[4.,6.],[4.,5.]])*cell_size, ['r4']), 'blue')
    # global obstacles
    O1 = (BoxRegion2D(np.array([[1.,2.],[1.,2.]])*cell_size, ['o1']), 'gray')
    O2 = (BoxRegion2D(np.array([[4.,5.],[2.,3.]])*cell_size, ['o2']), 'gray')
    O3 = (PolygonRegion2D(np.array([[1.,3.],[3.,3.],[3.,5.],
                                    [2.,5.],[2.,4.],[1.,4.]])*cell_size,
                          ['o3']), 'gray')

    # add all regions
    regions = [R1, R2, R3, R4, O1, O2, O3]

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
    F1 = (BallRegion2D([3.24, 1.98], 0.3, ['fire']), ('orange', 0.3))
    F2 = (BallRegion2D([1.26, 0.48], 0.3, ['fire']), ('orange', 0.3))
    S2 = (BallRegion2D([4.32, 1.48], 0.3, ['survivor']), ('yellow', 0.3))
    requests = [F1, F2, S2]
    # define local specification as a priority function
    localSpec = {'survivor': 0, 'fire': 1}
    logging.info('"Local specification": %s', localSpec)
    localSpec_cube_color = {'survivor': 3, 'fire': 2}
    # local obstacles
    obstacles = []

    # add style to local requests and obstacles
    for k, (r, c) in enumerate(requests + obstacles):
        # add styles to region
        addStyle(r, style={'facecolor': to_rgba(*c)}) #FIMXE: HACK
        r.cube_color = localSpec_cube_color[next(iter(r.symbols))]

        logging.info('("Local region", %d): (%s, %s, %s, %d)', k, r, r.style,
                     None, k < len(requests))

    # create request objects
    reqs = []
    for r, _ in requests:
        name = next(iter(r.symbols))
        reqs.append(Request(r, name, localSpec[name]))
    requests = reqs
    obstacles = [o for o, _, _ in obstacles]

    # set the robot's sensor
    sensingShape = BallBoundary2D([0, 0], 1.0)
    robot.sensor = CozmoSensor(robot, sensingShape, requests, obstacles)
    robot.sensor.reset()

    logging.info('"Robot sensor constructor": "%s"',
        'CozmoSensor(robot, BallBoundary2D([0, 0], 0.5), requests, obstacles)')

    # display workspace
    sim.display()

    # display expanded workspace
    sim.display(expanded=True)

    ############################################################################
    ### Generate global transition system and off-line control policy ##########
    ############################################################################

    globalSpec = ('[] ( (<> r1) && (<> r2) && (<> r3) && (<> r4)'
                  ' && !(o1 || o2 || o3))')
    logging.info('"Global specification": "%s"', globalSpec)

    # initialize incremental product automaton
    checker = IncrementalProduct(globalSpec) #, specFile='ijrr_globalSpec.txt')
    logging.info('"Buchi size": (%d, %d)', checker.buchi.g.number_of_nodes(),
                                         checker.buchi.g.number_of_edges())

    # initialize global off-line RRG planner
    sim.offline = RRGPlanner(robot, checker, iterations=1000)
    sim.offline.eta = [0.5, 1.0] # good bounds for the planar case study

    logging.info('"Start global planning": True')
    with Timer(op_name='global planning', template='"%s runtime": %f'):
        found = sim.offline.solve()
        logging.info('"Found solution": %s', found)
        if not found:
            return

    logging.info('"Iterations": %d', sim.offline.iteration)
    logging.info('"Size of TS": %s', sim.offline.ts.size())
    logging.info('"Size of PA": %s', sim.offline.checker.size())

    # save global transition system and control policy
    sim.offline.ts.save(os.path.join(outputdir, 'ts.yaml'))

    ############################################################################
    ### Display the global transition system and the off-line control policy ###
    ############################################################################

    # display workspace and global transition system
    prefix, suffix = sim.offline.checker.globalPolicy(sim.offline.ts)
    sim.display(expanded=False, solution=prefix)
    sim.display(expanded=False, solution=suffix)
    sim.display(expanded=False, solution=prefix+suffix[1:])
    logging.info('"global policy": (%s, %s)', prefix, suffix)
    logging.info('"End global planning": True')

    # move to start position
    logging.debug('Moving to start configuration: %s', robot.initConf)
    sim.robot.move([robot.initConf])

    ############################################################################
    ### Execute on-line path planning algorithm ################################
    ############################################################################

    # compute potential for each state of PA
    with Timer(op_name='Computing potential function',
               template='"%s runtime": %f'):
        if not compute_potentials(sim.offline.checker):
            return

    # set the step size for the local controller controlspace
    robot.controlspace = 0.20

    # initialize local on-line RRT planner
    sim.online = LocalPlanner(sim.offline.checker, sim.offline.ts, robot,
                              localSpec, eta=robot.controlspace)
    sim.online.detailed_logging = True

    # define number of surveillance cycles to run
    cycles = 2
    # execute controller
    cycle = -1 # number of completed cycles, -1 accounts for the prefix
    while cycle < cycles:
        # update the locally sensed requests and obstacles
        requests, obstacles = robot.sensor.sense()
        # TODO: sense robot location
        # TODO: update index in local plan
        # TODO: add logging marker for start time for planning
        logging.info('"plan start time": %f', time.time())
        with Timer(op_name='local planning', template='"%s runtime": %f'):
            # feed data to planner and get next control input
            nextConf = sim.online.execute(requests, obstacles)
        # TODO: add logging marker for stop time for planning
        logging.info('"plan stop time": %f', time.time())

#         sim.display(expanded=True, localinfo=('plan', 'trajectory'))

        # enforce movement along plan
        # FIXME: should pass only plan
        robot.move([nextConf])# + sim.online.local_plan)
        # if completed cycle increment cycle
        if sim.update():
            cycle += 1

    logging.info('"Local online planning finished": True')

if __name__ == '__main__':
    np.random.seed(1001)
    caseStudy()
