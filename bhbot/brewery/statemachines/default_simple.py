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

class Cell(object):
    cell_size = 0.35
    offset_x = 0.450
    total_cell_x = 6
    total_cell_x_offset = total_cell_x * cell_size
    def __init__(self, x, y) :
        self.x = x
        self.y = y

    def center_left(self) :
        x = self.offset_x + self.cell_size * self.x
        y = self.cell_size * (self.y + 0.5)
        return y,x
    
    def center_right(self) :
        x = self.offset_x + self.cell_size * (self.x + 1)
        y = self.cell_size * (self.y + 0.5)
        return y,x

    def down_right(self) :
        x = self.offset_x + self.cell_size * (self.x + 1)
        y = self.cell_size * (self.y + 1)
        return y,x

    def center_middle(self) :
        x = self.offset_x + self.cell_size * (self.x + 0.5)
        y = self.cell_size * (self.y + 0.5)
        return y,x

    def down_middle(self) :
        x = self.offset_x + self.cell_size * (self.x + 0.5)
        y = self.cell_size * (self.y + 1)
        return y,x

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
        statemachine.StateMachine.__init__(self, WaitDeviceReady)

class WaitDeviceReady(statemachine.State):
    """Waiting for PIC Ready"""
    def on_device_ready(self, team):
        self.switch_to_state(WaitStart)


class WaitStart(statemachine.State):
    """Waiting for start"""
    def on_start(self, team):
        self.switch_to_state(WaitFirstKeepAlive)

class WaitFirstKeepAlive(statemachine.State):
    def on_keep_alive(self, current_pose, match_started, match_time):
        self.switch_to_state(Resettle)

class Resettle(statemachine.State):
    def __init__(self):
        self.resettle_count = 0

    def on_enter(self):
        self.robot().resettle( AXIS_ABSCISSA, 0.1685, math.pi/2 ) #todo : improve

    def on_resettle(self):
        if self.resettle_count == 0 :
            self.robot().resettle( AXIS_ORDINATE, 0.0, math.pi/2 )
            self.resettle_count+=1
        else:
            self.switch_to_state(HomologationStart)


class HomologationStart(statemachine.State) :
    """Start of homologation"""
    def __init__(self):
        statemachine.State.__init__(self)
    
    def on_enter(self):
        self.fsm.points = deque()
        lk = as_dict()
        try :
            for vals in homologation_trajectory :
                angle = None
                x,y=0.0,0.0
                dir_=DIRECTION_FORWARD
                if len(vals) > 1 or type(vals) is dict :
                    if type(vals) is dict :
                        x,y = vals.get("pos",(x,y))
                        angle = vals.get("angle",angle)
                        dir_ = vals.get("dir",dir_)
                    else :
                        if len(vals) == 2 : x,y=vals
                        if len(vals) == 3 : x,y,angle=vals
                        if len(vals) == 4 : x,y,angle,dir_=vals
                else : angle = vals[0]
                a=lookup_defs("ANGLE",angle) if angle else None
                d=lookup_defs("DIRECTION",dir_) if dir_ else None
                logger.log("Traj : {0},{1},{2},{3}".format(x,y,a,d))
                self.fsm.points.append(TeamPose(x,y,angle,dir_))
        except Exception, e :
            logger.log( "Error decoding trajectory '{0}' : Exception is {1}".format(str(vals),str(e)))
            logger.exception(e)
        logger.log("Points : {0}".format(", ".join(str(p) for p in self.fsm.points)))
        self.fsm.successful_exit_state = HomologationEnd
        self.switch_to_state(TrajectoryWalk)

class TrajectoryWalk(statemachine.State):
    """Walk a path"""

    def on_enter(self):
        self.robot().goto_pose(self.fsm.points[0])

    def on_goto_finished(self,ignore1,ignore2):
        self.fsm.points.popleft()
        if len(self.fsm.points) > 0 :
            self.robot().goto_pose(self.fsm.points[0])
        else:
            self.switch_to_state(self.fsm.successful_exit_state)

    def on_evit_detected(self):
        self.switch_to_state(self.fsm.successful_exit_state)
    def on_turret_detect(self, detect_angle):
        # self.wait(time=5
        #          ,success=self.continue_path
        #          ,failure=lambda: self.fsm.exit()
        #          ,condition=lambda : angle.is_aligned(vector(self.robot.pose,detect_angle), self.robot.current_direction, constants.90_DEGREES))
        #          )
        if self.robot().is_in_my_way(detect_angle) :
            self.switch_to_state(WaitOpponentMove)

    # def on_opponent_moved(self, position):
    #     if self.robot.is_in_my_way(position) :
    #         self.switch_to_state(WaitOpponentMove)
    
    # def continue_path(self):
    #     self.robot.goto(self.robot.current_destination)

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
