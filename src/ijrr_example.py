#!/usr/bin/env python
'''
.. module:: ijrr_example
   :synopsis: Defined the case study presented in the IJRR journal paper.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    Defined the case study presented in the IJRR journal paper.
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

import numpy as np

from spaces.base import Workspace
from spaces.maps2d import BallRegion2D, BoxRegion2D, PolygonRegion2D, \
                          expandRegion, BoxBoundary2D, BallBoundary2D, Point2D
from robots import FullyActuatedRobot, SimulatedSensor

from planning import RRGPlanner, LocalPlanner, Request
from models import IncrementalProduct, compute_potentials
from graphics.planar import addStyle, Simulate2D, to_rgba
from lomap import Timer


def caseStudy():
    ############################################################################
    ### Define case study setup (robot parameters, workspace, etc.) ############
    ############################################################################
    
    # define robot diameter (used to compute the expanded workspace)
    robotDiameter = 0.36
    
    L = 0.6
    
    # define boundary
    boundary = BoxBoundary2D(L*np.array([[0, 8], [0, 6]]))
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
    print 'Conf space:', robot.cspace
    
    # create simulation object
    sim = Simulate2D(wspace, robot, ewspace)
    
    # regions of interest
    R1 = (BoxRegion2D(L*np.array([[0.75, 1.25], [0.75, 1]]), ['r1']), 'blue')
    R2 = (BallRegion2D(L*np.array([6.5, 2]), L*0.5, ['r2']), 'green')
    R3 = (BoxRegion2D(L*np.array([[6.25, 6.75], [4.5, 5]]), ['r3']), 'red')
    R4 = (PolygonRegion2D(L*np.array([[2.0, 4.75], [1.5, 5.25], [1, 4.75],
                      [1.5, 4.25], [2.0, 4.25]]), ['r4']), 'magenta')
    # global obstacles
    O1 = (BallRegion2D(L*np.array([4, 2.5]), L*0.6, ['o1']), 'gray')
    O2 = (PolygonRegion2D(L*np.array([[0.75, 2], [1.75, 2.5], [0.75, 3]]),
                          ['o2']),
          'gray')
    O3 = (PolygonRegion2D(L*np.array([[3.5, 4.5], [4, 5], [4.5, 4.5]]), ['o3']),
          'gray')
    O4 = (BoxRegion2D(L*np.array([[2.75, 3.25], [0.75, 1]]), ['o4']), 'gray')
#     O4 = (BallRegion2D(L*np.array([3, 0.875], 0.5, ['o4']), 'gray')
    
    # add all regions
    regions = [R1, R2, R3, R4, O1, O2, O3, O4]
    
    # add regions to workspace
    for r, c in regions:
        # add styles to region
        addStyle(r, style={'facecolor':c})
        # add region to workspace
        sim.workspace.addRegion(r)
        # create expanded region
        er = expandRegion(r, robot.diameter/2)
        # add style to the expanded region
        addStyle(er, style={'facecolor':c})
        # add expanded region to the expanded workspace
        sim.expandedWorkspace.addRegion(er)
    
    # local  requests
    F1 = (BallRegion2D(L*np.array([5.4, 3.3]), L*0.5, ['fire']), 'orange')
    F2 = (BallRegion2D(L*np.array([2.1, 0.8]), L*0.3, ['fire']), 'orange')
    S2 = (BallRegion2D(L*np.array([7.2, 0.8]), L*0.45, ['survivor']), 'yellow')
    requests = [F1, F2, S2]
    # define local specification as a priority function
    localSpec = {'survivor': 0, 'fire': 1}
    print 'Local specification:', localSpec
    # local obstacles
    obstacles = []
    
    # add style to local requests
    for r, c in requests:
        # add styles to region
        addStyle(r, style={'facecolor': to_rgba(c, 0.5)}) #FIMXE: HACK
    # create request objects
    reqs = []
    for r, _ in requests:
        name = next(iter(r.symbols))
        reqs.append(Request(r, name, localSpec[name]))
    requests = reqs
    
    # set the robot's sensor
    sensingShape = BallBoundary2D([0, 0], robot.diameter*2.5)
    robot.sensor = SimulatedSensor(robot, sensingShape, requests, obstacles)
    
    # resets requests in each surveillance cycle
    sim.makeRequestsRecurrent()
    
    # display workspace
    sim.display()
     
    # display expanded workspace
    sim.display(expanded=True)
    
    ############################################################################
    ### Generate global transition system and off-line control policy ##########
    ############################################################################
    
#     globalSpec = '[] ( <> r1 )' + \
#                  '&&' + \
#                  '[] ( r1 -> ( ( <> ( r2 || r3 ) ) ' +\
#                          '&& ( ! o1 U ( r2 || r3 ) ) ) )' +\
#                  '&&' + \
#                  '[] ( r2 -> ( ( <> r4 ) && ( ! r3 U r1 ) ) )' + \
#                  '&& ' + \
#                  '[] ( ! ( o2 || o3 || o4 ) )'
#     globalSpec = '[] ( <> o2 && <> o4 )'
#     globalSpec = '[] ( <> o2 && ! o4 )'
#     globalSpec = '[] ( (<> r1) && (<> r2) && (<> r3) && (<> r4) )'
    globalSpec = ('[] ( (<> r1) && (<> r2) && (<> r3) && (<> r4)'
                  + ' && !(o1 || o2 || o3 || o4 || o5))')
    print globalSpec
    
    # initialize incremental product automaton
    checker = IncrementalProduct(globalSpec) #, specFile='ijrr_globalSpec.txt')
    print 'Buchi size:', (checker.buchi.g.number_of_nodes(),
                          checker.buchi.g.number_of_edges())
    print
    
    # TODO: delete
#     print checker.buchi
#     pset = set(checker.buchi.props.itervalues())
#     pset.add(0)
#     for u, v, d in checker.buchi.g.edges(data=True):
#         print u, v, d
#         d['input'] &= pset
#         if not d['input']:
#             checker.buchi.g.remove_edge(u, v)
#     print checker.buchi
#     for u, v, d in checker.buchi.g.edges(data=True):
#         print u, v, d
    
    # initialize global off-line RRG planner
    sim.offline = RRGPlanner(robot, checker, None, iterations=1000)
    
    with Timer():
        if sim.offline.solve():
            print 'Found solution!'
        else:
            print 'No solution found!'
        print
    
#     print '\n\n\n\n\n\n'
#     for p in sorted(sim.offline.checker.g.nodes()):
#         print ((round(p[0].x, 2), round(p[0].y, 2)), p[1]), ':',
#         for q in sim.offline.checker.g.neighbors(p):
#             print ((round(q[0].x, 2), round(q[0].y, 2)), q[1]),
#         print
#     print '\n\n\n\n\n\n'
    
    print 'Finished in', sim.offline.iteration, 'iterations!'
    print 'Size of TS:', sim.offline.ts.size()
    print 'Size of PA:', sim.offline.checker.size()
    
#     # save global transition system and control policy
#     planner.ts.save('ts.yaml')
#     save -> planner.policy()
    
    ############################################################################
    ### Display the global transition system and the off-line control policy ###
    ############################################################################
    
    # display workspace and global transition system
#     sim.display()
#     sim.display(expanded='both')
# #     return
    prefix, suffix = sim.offline.checker.globalPolicy(sim.offline.ts)
#     sim.display(expanded='both', solution=prefix+suffix[1:])
#     sim.display(expanded=True, solution=prefix)
#     sim.display(expanded=True, solution=suffix)
    
#     # FIXME: set to global and to save animation
#     sim.simulate()
#     sim.play()
# #     sim.execute(2)
#     sim.save()
    
    ############################################################################
    ### Execute on-line path planning algorithm ################################
    ############################################################################
    
    sim.display(expanded='both', solution=prefix+suffix[1:])
    
    # compute potential for each state of PA
    with Timer():
        print compute_potentials(sim.offline.checker)
    
#     print 'PA final states:', sim.offline.checker.final
#     for u, d in sim.offline.checker.g.nodes_iter(data=True):
#         print 'node:', u, 'data:', d
    
    # FIXME: HACK
    robot.controlspace = 0.1
    
    # initialize local on-line RRT planner
    sim.online = LocalPlanner(sim.offline.checker, sim.offline.ts, robot,
                              localSpec)
    
    # TODO: debug code, delete after use
    sim.online.sim = sim
    
    # define number of surveillance cycles to run
    cycles = 4
    # execute controller
    cycle = -1 # number of completed cycles, -1 accounts for the prefix 
    while cycle < cycles:
        # update the locally sensed requests and obstacles
        requests, obstacles = robot.sensor.sense()
        with Timer('local planning'):
            # feed data to planner and get next control input
            nextConf = sim.online.execute(requests, obstacles)
        
        print 'local plan'
#         sim.display(expanded=True, localinfo='plan')
        
        # enforce movement
        robot.move(nextConf)
        # if completed cycle increment cycle
        if sim.update():
            cycle += 1
    
    ############################################################################
    ### Display the local transition systems and the on-line control policy ####
    ############################################################################
    
#     # FIXME: set to local and to save animation 
#     sim.simulate()
#     sim.save()
    

if __name__ == '__main__':
    np.random.seed(1001)
    caseStudy()