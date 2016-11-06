#!/usr/bin/env python
'''
.. module:: ijrr_example
   :synopsis: Defined the case study presented in the IJRR journal paper.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    Defined the case study presented in the IJRR journal paper.
    Copyright (C) 2014  Cristian Ioan Vasile <cvasile@bu.edu>
    Hybrid and Networked Systems (HyNeSs) Laboratory, Boston University

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
import networkx as nx

from spaces.base import Workspace
from spaces.maps2d import BallRegion2D, BoxRegion2D, PolygonRegion2D, \
                          expandRegion, BoxBoundary2D, BallBoundary2D, Point2D
from robots import FullyActuatedRobot

from planning import RRGPlanner
from models import IncrementalProduct
from graphics.planar import addStyle, Simulate2D
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
    robot = FullyActuatedRobot('DrRobot', init=Point2D((2, 2)), wspace=ewspace,
                               stepsize=0.999)
    robot.diameter = robotDiameter
    robot.sensingShape = BallBoundary2D([0, 0], robot.diameter*2.5)
#     robot.origin = Point2D([0.7, -0.7])
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
    O2 = (PolygonRegion2D(L*np.array([[0.75, 2], [1.75, 2.5], [0.75, 3]]), ['o2']), 'gray')
    O3 = (PolygonRegion2D(L*np.array([[3.5, 4.5], [4, 5], [4.5, 4.5]]), ['o3']), 'gray')
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
    
    # display workspace
#     sim.display()
    
    # display expanded workspace
#     sim.display(expanded=True)
    
    ############################################################################
    ### Generate global transition system and off-line control policy ##########
    ############################################################################
    
#     globalSpec = '[] ( <> r1 )' + \
#                  '&&' + \
#                  '[] ( r1 -> ( ( <> ( r2 || r3 ) ) && ( ! o1 U ( r2 || r3 ) ) ) )' +\
#                  '&&' + \
#                  '[] ( r2 -> ( ( <> r4 ) && ( ! r3 U r1 ) ) )' + \
#                  '&& ' + \
#                  '[] ( ! ( o2 || o3 || o4 ) )'
#     globalSpec = '[] ( <> o2 && <> o4 )'
#     globalSpec = '[] ( <> o2 && ! o4 )'
#     globalSpec = '[] ( (<> r1) && (<> r2) && (<> r3) && (<> r4) )' # && !(o1 || o2 || o3 || o4 || o5))'
    globalSpec = '[] ( (<> r1) && (<> r2) && (<> r3) && (<> r4) && !(o1 || o2 || o3 || o4 || o5))'
    print globalSpec
    
    # initialize incremental product automaton
    checker = IncrementalProduct(globalSpec) #, specFile='ijrr_globalSpec.txt')
    print 'Buchi size:', (checker.buchi.g.number_of_nodes(),
                          checker.buchi.g.number_of_edges())
    print
    
#     print checker.buchi.g.nodes()
#     for s in checker.buchi.g.nodes():
#         for sym in ['r1', 'r2', 'r3', 'r4', 'o1', 'o2', 'o3', 'o4', 'o5']:
#             print s, '-', sym, '->', checker.buchi.next_states_of_buchi(s, set([sym]))
#     
#     return
#     import matplotlib.pyplot as plt
#     checker.buchi.visualize()
#     plt.show()
    
#     nx.write_adjlist(checker.buchi.g, 'buchi.test.txt')
    
    # initialize global off-line RRG planner
    planner = RRGPlanner(robot, checker, None, iterations=1000)
    sim.offline = planner
    
    with Timer():
        if planner.solve():
            print 'Found solution!'
        else:
            print 'No solution found!'
        print
        
    print 'Finished in', planner.iteration, 'iterations!'
    print 'Size of TS:', planner.ts.size()
    print 'Size of PA:', planner.checker.size()
    
#     print 'TS'
#     ids = dict()
#     for k, x in enumerate(sorted(sim.offline.ts.g.nodes_iter(), key=lambda a: a.coords[0])):
#         ids[x] = k
    
#     for x, d in sorted(sim.offline.ts.g.nodes_iter(data=True), key=lambda a: a[1]['order']):
#         print d['order'], tuple(x.coords), d['prop'], [sim.offline.ts.g.node[nb]['order'] for nb in sim.offline.ts.g.neighbors(x)]
#     
#     print
#     print
#     
#     print 'PA'
#     for (x, s), d in sorted(sim.offline.checker.g.nodes_iter(data=True), key=lambda a: sim.offline.ts.g.node[a[0][0]]['order']):
#         print sim.offline.ts.g.node[x]['order'], tuple(x.coords), s, [(sim.offline.ts.g.node[nb]['order'], nbs) for nb, nbs in sim.offline.checker.g.neighbors((x, s))]
#     return
    # TODO: display input box to define output filename
#     # save global transition system and control policy
#     planner.ts.save('ts.yaml')
#     save -> planner.policy()
    
    ############################################################################
    ### Display the global transition system and the off-line control policy ###
    ############################################################################
    
    # display workspace and global transition system
#     sim.display()
    sim.display(expanded='both')
#     return
    prefix, suffix = planner.checker.globalPolicy(sim.offline.ts)
    sim.display(expanded='both', solution=prefix+suffix[1:])
#     sim.display(expanded=True, solution=prefix)
#     sim.display(expanded=True, solution=suffix)
    
    # FIXME: set to global and to save animation
    sim.simulate()
    sim.play()
#     sim.execute(2)
    sim.save()
    return
    ############################################################################
    ### Execute on-line path planning algorithm ################################
    ############################################################################
    
    # FIXME: define local specification as a priority function
    localSpec = None
    print localSpec
    
    # FIXME: initialize local on-line RRT planner
    planner = None
    sim.online = planner
    
    # define number of surveillance cycles to run
    cycles = 4
    
    # execute controller
    cycle = 0 # number of completed cycles
    while cycle < cycles:
        # TODO: sense
        with Timer():
            pass
            # TODO: feed data to planner and get next control input
        # TODO: enforce movement
        # TODO: if completed cycle increment cycle
        # FIXME: delete this hack
        cycle = 4
    
    ############################################################################
    ### Display the local transition systems and the on-line control policy ####
    ############################################################################
    
    # FIXME: set to local and to save animation 
    sim.simulate()
    sim.save()
    

if __name__ == '__main__':
    np.random.seed(1001)
    caseStudy()