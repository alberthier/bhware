#!/usr/bin/env python
# encoding: utf-8

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

import math

import statemachine
import packets
import trajectory
import logger
import math
import tools

from collections import deque

from definitions import *

terrain_width=3.0



# homologation_trajectory = [ list(Cell(0,0).center_right()) + [ANGLE_S,DIRECTION_FORWARD]
#                           , list(Cell(0,4).center_right()) + [ANGLE_S,DIRECTION_FORWARD]
#                           , list(Cell(0,5).center_middle()) + [ANGLE_S,DIRECTION_FORWARD]
#                           ]

#homologation_trajectory = [ list(Cell(0,0).center_left()) #+ [ ANGLE_SE ]
#                          , [ANGLE_SE]
#                          , list(Cell(0,0).down_right()) + [ ANGLE_SE ]
#                          , [ ANGLE_S ]
                          # , Cell(0,4).center_right()
                          # , [ ANGLE_SW ]
#                          , list(Cell(0,4).down_right()) + [ ANGLE_SW ]
#                          , [ ANGLE_SW ] # TODO : remove, robot can rotate and move
                          # , [ ANGLE_S ]
#1                          , Cell(0,5).center_middle()
#                          ]

#homologation_trajectory = [ [ 0.1685, 0.5, ANGLE_E ]
#                          , [ANGLE_SE]
#                          , list(Cell(0,0).down_right()) + [ ANGLE_SE ]
#                          , [ ANGLE_S ]
                          # , Cell(0,4).center_right()
                          # , [ ANGLE_SW ]
#                          , list(Cell(0,4).down_right()) + [ ANGLE_SW ]
#                          , [ ANGLE_SW ] # TODO : remove, robot can rotate and move
                          # , [ ANGLE_S ]
#1                          , Cell(0,5).center_middle()
#                          ]

homologation_trajectory = [ list(Cell(0,0).center_left()) + [ ANGLE_SE ]
#                          , [ANGLE_SE]
                          , list(Cell(0,0).down_right()) + [ ANGLE_SE ]
                          , [ ANGLE_S ]
                          # , Cell(0,4).center_right()
                          # , [ ANGLE_SW ]
                          , list(Cell(0,4).down_right()) + [ ANGLE_SW ]
                          , [ ANGLE_SW ] # TODO : remove, robot can rotate and move
                          # , [ ANGLE_S ]
                          , list(Cell(0,5).center_middle()) + [ANGLE_SW]
                          ]


def mirror_angle_y(a):
    return math.atan2(math.sin(a),-math.cos(a))

class Pose(object):
    def __init__(self, x, y, angle=None, direction=DIRECTION_FORWARD) :
    # def __init__(self, x, y, angle=None, direction=DIRECTION_FORWARD, movement = MOVEMENT_FORWARD) :
        self.x=x
        self.y=y
        self.angle=angle
        self.direction=direction
        # self.movement=movement

    def __str__(self):
        return "Pose({0}, {1}, {2}, {3})".format(self.x,self.y, self.angle, self.direction)

    def get_values(self):
        # TODO : this x,y swapping pisses me off
        # return [self.x, self.y, self.angle, self.direction]
        #a = self.angle + math.pi / 2 if self.angle else None
        # return [self.y, self.x, a, self.direction, self.movement]
#return [self.y, self.x, a, self.direction]
        return [self.x, self.y, self.angle, self.direction]

class TeamPose(Pose) :
    """ Pose which is automatically translated for the current team """
    def __init__(self, x, y, angle=None, direction=DIRECTION_FORWARD, original_team = TEAM_RED ) :
        Pose.__init__(self, x,y,angle,direction)
        self.current_team = original_team

    def change_team(self, team):
        if team != self.current_team :
            self.current_team = team
            # self.x = (Cell.total_cell_x_offset + Cell.offset_x) - self.x
            if not tools.quasi_null(self.x) and not tools.quasi_null(self.y) :
                self.y = terrain_width - self.y
            # self.x = Cell.offset_x + (Cell.total_cell_x_offset / 2) + (Cell.total_cell_x_offset / 2 - (self.x - Cell.offset_x) )
            if self.angle is not None : self.angle = mirror_angle_y(self.angle)

class DefaultStateMachine(statemachine.StateMachine):
    """Default state machine"""
    def __init__(self):
        statemachine.StateMachine.__init__(self, WaitDeviceReady())

class WaitDeviceReady(statemachine.State):
    """Waiting for PIC Ready"""
    def on_device_ready(self, team):
        self.switch_to_state(WaitStart())


class WaitStart(statemachine.State):
    """Waiting for start"""
    def on_start(self, team):
        self.switch_to_state(WaitFirstKeepAlive())

class WaitFirstKeepAlive(statemachine.State):
    def on_keep_alive(self, current_pose, match_started, match_time):
        self.switch_to_state(Sequence(Resettle(0.1685,0.0,math.pi/2)
                                        ,Homologation()
                                        )
                            )

class Resettle(statemachine.State):
    def __init__(self,abscissa, ordinate, angle):
        self.resettle_count = 0
        self.abscissa = abscissa
        self.ordinate = ordinate
        self.angle = angle

    def on_enter(self):
        self.robot().resettle( AXIS_X, self.abscissa, 0.0 )

    def on_resettle(self):
        if self.resettle_count == 0 :
            self.robot().resettle( AXIS_Y, self.ordinate, self.angle )
            self.resettle_count+=1
        else:
            self.exit_machine()

class Homologation(statemachine.State):
    def on_enter(self):
        self.switch_to_substate(HomologationStart())

    def on_exit_substate(self, inst):
        self.switch_to_state(HomologationEnd())


class HomologationStart(statemachine.State) :
    """Start of homologation"""
    def __init__(self):
        statemachine.State.__init__(self)

    def on_enter(self):
        self.switch_to_substate(Sequence( TrajectoryWalk(homologation_trajectory[0])
                                        , Deploy()
                                        , TrajectoryWalk(homologation_trajectory[1:])
                                        )
                                )

    def on_exit_substate(self, st):
        self.switch_to_state(HomologationEnd())


class HomologationEnd(statemachine.State) :
    """End of homologation"""
    def __init__(self):
        statemachine.State.__init__(self)

    def on_enter(self):
        self.robot().deploy()
        logger.log("Homologation ended")



if __name__=="__main__" :
    import unittest

    class TeamMirroringTest(unittest.TestCase):
        def setUp(self):
            self.pose = TeamPose(10.0, 10.0, math.pi)
        def test_normal(self):
            self.assertEquals(self.pose.x, 10.0)
        def test_mirror_pos(self):
            self.pose.change_team(TEAM_BLUE)
            self.assertEquals(self.pose.x, Cell.total_cell_x_offset + Cell.offset_x  - 10.0)
            self.assertEquals(self.pose.y, 10.0)
        def test_mirror_angle(self):
            self.pose.change_team(TEAM_BLUE)
            self.assertAlmostEqual(self.pose.angle, 0.0)

    class CellTest(unittest.TestCase):
        def setUp(self):
            self.c = Cell(1,1)
        def test_center_middle(self):
            x,y = self.c.center_middle()
            self.assertEquals(x, Cell.offset_x + Cell.cell_size * 1.5)
            self.assertEquals(y,Cell.cell_size * 1.5)
        def test_center_right(self):
            x,y = self.c.center_right()
            self.assertEquals(x, Cell.offset_x + Cell.cell_size * 2)
            self.assertEquals(y,Cell.cell_size * 1.5)

    class CellMirroringTest(unittest.TestCase):
        def test(self):
            p1 = TeamPose(*Cell(0,0).center_middle())
            p1.change_team(TEAM_BLUE)
            p2 = TeamPose(*Cell(0,5).center_middle())
            self.assertEqual(p1.get_values(),p2.get_values())


    unittest.main()
