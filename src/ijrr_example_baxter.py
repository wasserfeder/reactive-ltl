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

#TODO: remove back to add lomap to python path
if os.path.pathsep.join(sys.path).find('lomap') < 0:
    print 'importing lomap'
    sys.path.append(
            '/home/cristi/Dropbox/work/workspace_linux_precision5520/lomap/')

from spaces.base import ConfigurationSpace, line_translate, Point
from spaces.maps_nd import BoxBoundary, BoxRegion
from robots import BaxterRobot

from planning import RRGPlanner, LocalPlanner, Request
from models import IncrementalProduct

from lomap import compute_potentials
from lomap import Timer


def caseStudy():
    ############################################################################
    ### Output and debug options ###############################################
    ############################################################################
    outputdir = os.path.abspath('../data_ijrr/example3')
    if not os.path.isdir(outputdir):
        os.makedirs(outputdir)

    # configure logging
    fs = '%(asctime)s [%(levelname)s] -- { %(message)s }'
    dfs = '%m/%d/%Y %I:%M:%S %p'
    loglevel = logging.DEBUG
    logfile = os.path.join(outputdir, 'ijrr_example_3.log')
    verbose = True
    logging.basicConfig(filename=logfile, filemode='w', level=loglevel,
                        format=fs, datefmt=dfs)
    if verbose:
        root = logging.getLogger()
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(loglevel)
        ch.setFormatter(logging.Formatter(fs, dfs))
        root.addHandler(ch)


    ############################################################################
    ### Define case study setup (robot parameters, workspace, etc.) ############
    ############################################################################

    # define boundary in 7-dimensional configuration space of the Baxter's arm
    boundary = BoxBoundary([[-np.pi, np.pi]]*6)
    # define boundary style
    boundary.style = {'color' : 'black'}

    # create robot's configuration space
    cspace = ConfigurationSpace(boundary=boundary)

    # initial configuration
    init_conf = Point([0, 0, 0, 0, 0, 0])
    # create robot object
    robot = BaxterRobot(name="baxter", init=init_conf, cspace=cspace,
                        stepsize=.99, config={'json-filename':
        '/home/cristi/Dropbox/work/workspace_linux_precision5520/reactive-ltl/'
        'src/robots/baxter_robot/env_config.json'})
    robot.localObst = 'local_obstacle'

    logging.info('"C-space": (%s, %s)', cspace, boundary.style)
    logging.info('"Robot name": "%s"', robot.name)
    logging.info('"Robot initial configuration": %s', robot.initConf)
    logging.info('"Robot step size": %f', robot.controlspace)
    logging.info('"Robot constructor": "%s"',
                 'BaxterRobot(robot_name, init=initConf, cpace=cspace, '
                 'stepsize=stepsize)')
    logging.info('"Local obstacle label": "%s"', robot.localObst)

#     # create simulation object
#     sim = Simulate2D(wspace, robot, ewspace)
#     sim.config['output-dir'] = outputdir

#     # regions of interest
#     R1 = (BoxRegion2D([[0.45, 0.75], [0.45, 0.6]], ['r1']), 'brown')
#     R2 = (BallRegion2D([3.9, 1.2], 0.3, ['r2']), 'green')
#     R3 = (BoxRegion2D([[3.75, 4.05], [2.7, 3]], ['r3']), 'red')
#     R4 = (PolygonRegion2D([[1.2 , 2.85], [0.9 , 3.15], [0.6 , 2.85],
#                            [0.9 , 2.55], [1.2 , 2.55]], ['r4']), 'magenta')
#     # global obstacles
#     O1 = (BallRegion2D([2.4, 1.5], 0.36, ['o1']), 'gray')
#     O2 = (PolygonRegion2D([[0.45, 1.2], [1.05, 1.5], [0.45, 1.8]], ['o2']),
#           'gray')
#     O3 = (PolygonRegion2D([[2.1, 2.7], [2.4, 3], [2.7, 2.7]], ['o3']), 'gray')
#     O4 = (BoxRegion2D([[1.65, 1.95], [0.45, 0.6]], ['o4']), 'gray')
# 
#     # add all regions
#     regions = [R1, R2, R3, R4, O1, O2, O3, O4]
# 
#     # add regions to workspace
#     for k, (r, c) in enumerate(regions):
#         # add styles to region
#         addStyle(r, style={'facecolor': c})
#         # add region to workspace
#         sim.workspace.addRegion(r)
#         # create expanded region
#         er = expandRegion(r, robot.diameter/2)
#         # add style to the expanded region
#         addStyle(er, style={'facecolor': c})
#         # add expanded region to the expanded workspace
#         sim.expandedWorkspace.addRegion(er)
# 
#         logging.info('("Global region", %d): (%s, %s)', k, r, r.style)
# 
#     # local  requests
#     F1 = (BallRegion2D([3.24, 1.98], 0.3, ['fire']),
#           ([[3.24, 1.98], [2.54, 2.28], [3.5, 3], [4.02, 2.28]], 0.05),
#           ('orange', 0.5))
#     F2 = (BallRegion2D([1.26, 0.48], 0.18, ['fire']),
#           ([[1.26, 0.48], [1.1, 1.1], [1.74, 0.92], [0.6, 0.6]], 0.05),
#           ('orange', 0.5))
#     S2 = (BallRegion2D([4.32, 1.48], 0.27, ['survivor']),
#           ([[4.32, 1.48], [3.6, 1.2], [4, 2]], 0.05),
#           ('yellow', 0.5))
#     requests = [F1, F2, S2]

    # define local specification as a priority function
    localSpec = {'plate_red': 0, 'plate_blue': 1}
    logging.info('"Local specification": %s', localSpec)
    #TODO: create local requests objects and robot sensor

#     # local obstacles #FIXME: defined in expanded workspace not workspace
#     obstacles = [(BoxRegion2D([[3, 3.5], [2, 2.5]], ['LO']), None,
#                   ('gray', 0.8)),
#                  (PolygonRegion2D([[3.2, 1.4], [3, 0.8], [3.4, 0.7]], ['LO']),
#                   None, ('gray', 0.8)),
#                  (BallRegion2D([1.6, 2.1], 0.15, ['LO']), None, ('gray', 0.8))]
# 
#     # add style to local requests and obstacles
#     for k, (r, path, c) in enumerate(requests + obstacles):
#         # add styles to region
#         addStyle(r, style={'facecolor': to_rgba(*c)}) #FIXME: HACK
#         # create path
#         r_path = path
#         if path:
#             wps, step = path
#             wps = wps + [wps[0]]
#             r.path = []
#             for a, b in it.izip(wps[:-1], wps[1:]):
#                 r.path += line_translate(a, b, step)
#             r_path = map(list, r.path)
#             r.path = it.cycle(r.path)
# 
#         logging.info('("Local region", %d): (%s, %s, %s, %d)', k, r, r.style,
#                      r_path, k < len(requests))
# 
#     # create request objects
#     reqs = []
#     for r, _, _ in requests:
#         name = next(iter(r.symbols))
#         reqs.append(Request(r, name, localSpec[name]))
#     requests = reqs
#     obstacles = [o for o, _, _ in obstacles]
# 
#     # set the robot's sensor
#     sensingShape = BallBoundary2D([0, 0], robot.diameter*2.5)
#     robot.sensor = SimulatedSensor(robot, sensingShape, requests, obstacles)
# 
#     logging.info('"Robot sensor constructor": "%s"',
#         'SimulatedSensor(robot, BallBoundary2D([0, 0], robot.diameter*2.5),'
#         'requests, obstacles)')
# 
#     # display workspace
#     sim.display()
# 
#     # display expanded workspace
#     sim.display(expanded=True)

    ############################################################################
    ### Generate global transition system and off-line control policy ##########
    ############################################################################

    globalSpec = ('[] ( (<> region1) && (<> region2) && (<> region3)'
                  '&& table )')
    logging.info('"Global specification": "%s"', globalSpec)

    # initialize incremental product automaton
    checker = IncrementalProduct(globalSpec)
    logging.info('"Buchi size": (%d, %d)', checker.buchi.g.number_of_nodes(),
                                           checker.buchi.g.number_of_edges())

    # initialize global off-line RRG planner
    offline = RRGPlanner(robot, checker, iterations=100)
    offline.eta = [0.02, 1.0] # good bounds for the planar case study

    logging.info('"Start global planning": True')
    with Timer(op_name='global planning', template='"%s runtime": %f'):
        found = offline.solve()
        logging.info('"Found solution": %s', found)
        if not found:
            return

    logging.info('"Iterations": %d', offline.iteration)
    logging.info('"Size of TS": %s', offline.ts.size())
    logging.info('"Size of PA": %s', offline.checker.size())

    # save global transition system and control policy
    offline.ts.save(os.path.join(outputdir, 'ts.yaml'))

    ############################################################################
    ### Display the global transition system and the off-line control policy ###
    ############################################################################

    # display workspace and global transition system
    prefix, suffix = offline.checker.globalPolicy(offline.ts)
    logging.info('"global policy": (%s, %s)', prefix, suffix)
    logging.info('"End global planning": True')

    return #TODO: delete after tests

    ############################################################################
    ### Execute on-line path planning algorithm ################################
    ############################################################################

    # compute potential for each state of PA
    with Timer(op_name='Computing potential function',
               template='"%s runtime": %f'):
        if not compute_potentials(offline.checker):
            return

    # set the step size for the local controller
    robot.controlspace = 0.1

    # initialize local on-line RRT planner
    online = LocalPlanner(offline.checker, offline.ts, robot, localSpec)
    online.detailed_logging = True

    def update(robot, online):
        '''Removes requests serviced by the robot, resets all requests at the
        end of each cycle, and moves them on their paths.
        '''

        # update requests and local obstacles
        robot.sensor.update()
        # reset requests at the start of a cycle, i.e., reaching a final state
        if (online.trajectory[-1] in online.ts.g and online.potential == 0):
            robot.sensor.reset()
            return True
        return False

    # define number of surveillance cycles to run
    cycles = 2
    # execute controller
    cycle = -1 # number of completed cycles, -1 accounts for the prefix
    while cycle < cycles:
        logging.info('"Start local planning step": True')
        # update the locally sensed requests and obstacles
        requests, obstacles = robot.sensor.sense()
        with Timer(op_name='local planning', template='"%s runtime": %f'):
            # feed data to planner and get next control input
            nextConf = online.execute(requests, obstacles)
        # enforce movement
        robot.move(nextConf)

        # if completed cycle increment cycle
        if update():
            cycle += 1

    logging.info('"Local online planning finished": True')


if __name__ == '__main__':
    np.random.seed(1001)
    caseStudy()
