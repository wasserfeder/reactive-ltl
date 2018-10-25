#!/usr/bin/env python
'''
.. module:: ijrr_example_baxter
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

import numpy as np

try:
    import lomap
except ImportError:
    print 'adding lomap to python path'
    lomap_path = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                              '..', '..', 'lomap'))
    assert os.path.isdir(lomap_path), 'lomap package not found!'
    sys.path.append(lomap_path)

from lomap import compute_potentials
from lomap import Timer, Ts
    
from spaces.base import ConfigurationSpace, Point
from spaces.maps_nd import BoxBoundary, BallRegion
from robots import BaxterRobot
from planning import RRGPlanner, LocalPlanner, Request
from models import IncrementalProduct


def setup(outputdir='.', logfilename='example.log'):
    '''Setup logging, and set output and debug options.'''
    if not os.path.isdir(outputdir):
        os.makedirs(outputdir)

    # configure logging
    fs = '%(asctime)s [%(levelname)s] -- { %(message)s }'
    dfs = '%m/%d/%Y %I:%M:%S %p'
    loglevel = logging.DEBUG
    logfile = os.path.join(outputdir, logfilename)
    verbose = False
    # create logger
    logger = logging.getLogger('reactive_ltl')
    logger.setLevel(loglevel)
    # create formatter
    fmt = logging.Formatter(fs, dfs)
    # create handlers and add them to logger
    handlers = [logging.FileHandler(logfile, mode='w')]
    if verbose:
        handlers.append(logging.StreamHandler(sys.stdout))
    for handler in handlers:
        handler.setLevel(loglevel)
        handler.setFormatter(fmt)
        logger.addHandler(handler)
    return logger

def define_problem(logger, outputdir='.'):
    '''Define case study setup (robot parameters, workspace, etc.).'''

    # define boundary in 6-dimensional configuration space of the Baxter's arm
    # the joint limits are taken from the Baxter's manual
    boundary = BoxBoundary([[-3., 3.], [0., 2.6], [-1.69, 1.69], [-2.1, 1.],
                            [-3., 3.], [-1.5, 2.]])
    # define boundary style
    boundary.style = {'color' : 'black'}

    # create robot's configuration space
    cspace = ConfigurationSpace(boundary=boundary)

    # initial configuration
    init_conf = Point([0, 0, 0, 0, 0, 0])
    # create robot object
    filename = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                            'robots', 'baxter_robot',
                            'env_config_with_interactive_marker.json')
    assert os.path.isfile(filename), 'Json environment file not found!'
    robot = BaxterRobot(name="baxter", init=init_conf, cspace=cspace,
                        stepsize=.99, config={'json-filename': filename})
    robot.localObst = 'local_obstacle'

    logger.info('"C-space": (%s, %s)', cspace, boundary.style)
    logger.info('"Robot name": "%s"', robot.name)
    logger.info('"Robot initial configuration": %s', robot.initConf)
    logger.info('"Robot step size": %f', robot.controlspace)
    logger.info('"Robot config": %s', robot.config)
    logger.info('"Robot constructor": "%s"',
                 'BaxterRobot(robot_name, init=initConf, cpace=cspace, '
                 'stepsize=stepsize, config=config)')
    logger.info('"Local obstacle label": "%s"', robot.localObst)

    # local  requests
    requests = [BallRegion([0, 0, 0], 0.3, ['interactive'])]

    # define local specification as a priority function
    localSpec = {'interactive': 0}
    logger.info('"Local specification": %s', localSpec)

    for k, r in enumerate(requests):
        logger.info('("Local region", %d): (%s, %s, %s, %d)', k, r, None, None,
                    k < len(requests))

    # create request objects
    reqs = []
    for r in requests:
        name = next(iter(r.symbols))
        reqs.append(Request(r, name, localSpec[name]))
    requests = reqs

    # set requests to look for
    robot.request = requests[0]

    globalSpec = ('[] ( (<> region1) && (<> region2) && (<> region3)'
                  '&& table )')
    logger.info('"Global specification": "%s"', globalSpec)

    return robot, globalSpec, localSpec

def generate_global_ts(logger, globalSpec, robot, eta=(0.02, 1.0), load=True):
    '''Generate global transition system and off-line control policy.'''
    ts_file = os.path.join(outputdir, 'ts.yaml')
    pa_file = os.path.join(outputdir, 'pa.yaml')
    if load and os.path.isfile(ts_file) and os.path.isfile(pa_file):
        checker = IncrementalProduct.load(pa_file)
        logger.info('"Buchi size": %s', checker.buchi.size())
        offline = RRGPlanner(robot, checker, iterations=1000)
        offline.eta = eta
        offline.ts = Ts.load(ts_file)
        prefix, suffix = offline.checker.globalPolicy(offline.ts)
        logger.info('"global policy": (%s, %s)', prefix, suffix)
        logger.info('"global policy length": (%d, %d)', len(prefix), len(suffix))
        logger.info('"End global planning": True')
        return True, offline

    # initialize incremental product automaton
    checker = IncrementalProduct(globalSpec)
    logger.info('"Buchi size": %s', checker.buchi.size())

    # initialize global off-line RRG planner
    offline = RRGPlanner(robot, checker, iterations=1000)
    offline.eta = eta

    logger.info('"Start global planning": True')
    with Timer(op_name='global planning', template='"%s runtime": %f'):
        found = offline.solve()
        logger.info('"Found solution": %s', found)
        if not found:
            return

    logger.info('"Iterations": %d', offline.iteration)
    logger.info('"Size of TS": %s', offline.ts.size())
    logger.info('"Size of PA": %s', offline.checker.size())

    # save global transition system and control policy
    offline.ts.save(ts_file)
    # save product automaton
    offline.checker.save(pa_file)

    ############################################################################
    ### Display the global transition system and the off-line control policy ###
    ############################################################################

    # display workspace and global transition system
    prefix, suffix = [], []
    if found:
        prefix, suffix = offline.checker.globalPolicy(offline.ts)
    logger.info('"global policy": (%s, %s)', prefix, suffix)
    logger.info('"global policy length": (%d, %d)', len(prefix), len(suffix))
    logger.info('"End global planning": True')

    return found, offline

def update(robot, online):
    '''Removes requests serviced by the robot, resets all requests at the end of
    each cycle, and moves them on their paths.
    '''
    # update requests and local obstacles
    robot.sensor_update()
    # reset requests at the start of a cycle, i.e., reaching a final state
    if (online.trajectory[-1] in online.ts.g and online.potential == 0):
        robot.sensor_reset()
        return True
    return False

def plan_online(logger, localSpec, offline, robot, iterations=2):
    '''Execute on-line path planning algorithm.'''

    # compute potential for each state of PA
    with Timer(op_name='Computing potential function',
               template='"%s runtime": %f'):
        if not compute_potentials(offline.checker):
            return

    # set the step size for the local controller
    robot.controlspace = 0.1

    # initialize local on-line RRT planner
    online = LocalPlanner(offline.checker, offline.ts, robot, localSpec)
    online.PointCls = Point
    online.detailed_logging = True

    # define number of surveillance cycles to run
    cycles = iterations
    # execute controller
    cycle = -1 # number of completed cycles, -1 accounts for the prefix
    while cycle < cycles:
        logger.info('"Start local planning step": True')
        # update the locally sensed requests and obstacles
        requests, obstacles = robot.sensor_sense()
        with Timer(op_name='local planning', template='"%s execution": %f'):
            # feed data to planner and get next control input
            nextConf = online.execute(requests, obstacles)
        # enforce movement
        robot.move(nextConf)
        # if completed cycle increment cycle
        if update(robot, online):
            cycle += 1

    logger.info('"Local online planning finished": True')

def caseStudy(outputdir, logfilename, iterations):
    logger = setup(outputdir, logfilename)
    robot, globalSpec, localSpec = define_problem(logger, outputdir)
    found, offline = generate_global_ts(logger, globalSpec, robot)
    if not found:
        return
    plan_online(logger, localSpec, offline, robot, iterations)

if __name__ == '__main__':
    np.random.seed(1001)
    outputdir=os.path.abspath('../data_ijrr/example3')
    caseStudy(outputdir, logfilename='ijrr_example_3.log', iterations=2)
