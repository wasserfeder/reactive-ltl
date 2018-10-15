'''
.. module:: ltlrrg
   :synopsis: The module implements the RRG based path planner with LTL constraints.
    The algorithm represents the (off-line) global component of the proposed
    framework.

.. moduleauthor:: Cristian Ioan Vasile <cvasile@bu.edu>
'''

'''
    The module implements the RRG based path planner with LTL constraints.
    The algorithm represents the (off-line) global component of the proposed
    framework.
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

import logging
from itertools import ifilter, imap, tee

from lomap import Ts, Timer


# FIXME: HACK to get around ROS logging capture
logger = logging.getLogger('reactive_ltl.' + __name__)
logger.addHandler(logging.NullHandler())


class RRGPlanner(object):
    '''
    Class implementing the RRG based path planner with LTL constraints.
    The algorithm represents the (off-line) global component of the proposed
    framework.
    Planning is performed in the configuration space of the robot. However, this
    is transparent for this class and all calls to the underlying spaces
    (workspace and configuration) are wrapped as methods of the robot class.
    Thus the planner does not need to know how the samples are generated or
    mapped between the two spaces.
    '''

    def __init__(self, robot, checker, iterations):
        '''Constructor'''
        self.name = 'RRG-based LTL path planner'
        self.maxIterations = iterations
        self.iteration = 0

        self.robot = robot
        symbols = self.robot.getSymbols(robot.initConf)

        # initialize transition system
        self.ts = Ts(name='global transition system', multi=False)
        self.ts.g.add_node(robot.initConf, prop=symbols)
        self.ts.init[robot.initConf] = 1.0

        # initialize checker
        self.checker = checker
        if checker:
            self.checker.addInitialState(robot.initConf, symbols)

        # initialize bounds for far and nearest checks
        # should be set based on specific problem
        self.eta = [0.5, 1.0]

        self.detailed_logging = True

    def solve(self):
        '''Try to solve the problem.'''
        for self.iteration in range(1, self.maxIterations+1):
            logger.info('"iteration": %d', self.iteration)
            if self.iterate():
                return True
        return False

    def iterate(self):
        '''Execute one step of the off-line global planner.'''
        if not self.checker.foundPolicy():
            ### First phase - Forward
            # initialize update sets
            newState, Delta, E = None, set(), set()
            # sample new configuration
            randomConf = self.robot.sample()
            nearestState = self.nearest(randomConf)
            # steer towards the random configuration
            newConf = self.robot.steer(nearestState, randomConf)
            # set propositions
            newProp = self.robot.getSymbols(newConf)

            if self.detailed_logging:
                logger.info('"random configuration": %s', randomConf)
                logger.info('"nearest state": %s', nearestState)
                logger.info('"new configuration": %s', newConf)

            for state in self.far(newConf):
                # check if the new state satisfies the global specification
                if self.robot.isSimpleSegment(state, newConf):
                    with Timer(op_name='PA check forward',
                               template='"%s runtime": %f'):
                        Ep = self.checker.check(self.ts, state, newConf, newProp,
                                                forward=True)
                    if Ep:
                        newState = newConf
                        Delta.add((state, newConf))
                        E.update(Ep)

            if newState:
                self.ts.g.add_node(newState, prop=newProp)
                self.ts.g.add_edges_from(Delta)
                with Timer(op_name='PA update forward',
                           template='"%s runtime": %f'):
                    self.checker.update(E)

            if self.detailed_logging:
                logger.info('"forward state added": %s', newState)
                logger.info('"forward edges added": %s', Delta)

            ### Second phase - Backward
            Delta = set()
            E = set()
            if newState: # for newly added state
#                 logger.info('"near states": %s', list(self.near(newState)))
                for state in self.near(newState):
                    st = self.robot.steer(newState, state, atol=1e-8)
                    # if the robot can steer from a new state to another state
                    if (state == st):
                            #FIXME: should be symmetric
#                             and self.robot.isSimpleSegment(newState, state):
                        # check if the new state satisfies the global
                        # specification
                        with Timer(op_name='PA check backward',
                               template='"%s runtime": %f'):
                            Ep = self.checker.check(self.ts, newState, state,
                                   self.ts.g.node[state]['prop'], forward=False)
                        if Ep:
                            Delta.add((newState, state))
                            E.update(Ep)
#                             logger.info('("backward edge added", %s): %s',
#                                          state, (newState, state))

            self.ts.g.add_edges_from(Delta)
            with Timer(op_name='PA update backward',
                               template='"%s runtime": %f'):
                self.checker.update(E)
            if self.detailed_logging:
                logger.info('"backward edges added": %s', Delta)

            # uncomment assertion for debugging
#             assert all([self.checker.buchi.g.has_edge(u[1], v[1])
#                         for u, v in self.checker.g.edges_iter()]), \
#                         '{} {}'.format(self.iteration,
#                     [((u[0].coords, u[1]), (v[0].coords, v[1]))
#                         for u, v in self.checker.g.edges_iter()
#                             if not self.checker.buchi.g.has_edge(u[1], v[1])])

            return self.checker.foundPolicy()

        return True

    def nearest(self, p):
        '''Returns the nearest configuration in the transition system.'''
        dist = self.robot.cspace.dist
        return min(self.ts.g.nodes_iter(), key=lambda x: dist(p, x))

    def far(self, p):
        '''Return all states in the transition system that fall at distance d,
        d < self.eta[1], away from the given configuration p. If there is a
        state which is closer to the given configuration p than self.eta[0]
        then the function returns an empty list.
        '''
        if self.detailed_logging:
            logger.info('"far": (%s, %f, %f)', p, self.eta[0], self.eta[1])
        metric = self.robot.cspace.dist
        ret, test = tee(ifilter(lambda v: metric(v, p) < self.eta[1],
                                self.ts.g.nodes_iter()))
        if any(imap(lambda v: metric(v, p) <= self.eta[0], test)):
            return iter([])
        return ret

    def near(self, p):
        '''Return all states in the transition system that fall at distance d,
        0 < d < self.eta[1], away from the given configuration p.
        '''
        if self.detailed_logging:
            logger.info('("near", %s): %f', p, self.eta[1])
        metric = self.robot.cspace.dist
        return ifilter(lambda v: 0 < metric(v, p) < self.eta[1],
                       self.ts.g.nodes_iter())
