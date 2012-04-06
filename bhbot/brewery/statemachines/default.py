#!/usr/bin/env python
# encoding: utf-8

import math

import statemachine
import packets
import trajectory
import logger
import commonstates
from definitions import *



################################################################################
# Setup states


class Main(statemachine.State):

    def on_device_ready(self, packet):
        self.switch_to_substate(commonstates.DefinePosition())


    #noinspection PyUnusedLocal
    def on_exit_substate(self, substate):
        self.switch_to_state(WaitStart())




class WaitStart(statemachine.State):

    def on_enter(self):
        self.sent = False


    def on_keep_alive(self, packet):
#        pass
        if not self.sent:
            self.sent = True
            import cProfile
            import sys
            cProfile.runctx("self.event_loop.map.route(self.robot().pose.x, self.robot().pose.y, 1.6, 2.0)", None, { "self": self })
            sys.stdout.flush()
            self.event_loop.map.route(self.robot().pose.x, self.robot().pose.y, 0.5, 1.0)
            #self.event_loop.map.route(self.robot().pose.x, self.robot().pose.y, 1.8, 2.8)


    def on_start(self, packet):
        self.switch_to_substate(commonstates.DefinePosition())


    def on_exit_substate(self, substate):
        self.switch_to_state(WaitFirstKeepAlive())




class WaitFirstKeepAlive(statemachine.State):

    def on_keep_alive(self, packet):
        #self.switch_to_state(GotoStartPoint())
        #self.switch_to_state(TestTraj())
        self.switch_to_state(TestCommands())




class TestTraj(statemachine.State):

    def on_enter(self):
        walk = commonstates.TrajectoryWalk(None, TEAM_UNKNOWN)
        points = self.event_loop.map.route(self.robot().pose.x, self.robot().pose.y, 1.6, 2.0)
        x = self.robot().pose.x
        y = self.robot().pose.y
        logger.log("{}".format(self.robot().team))
        for p in points:
            x = p.x
            y = p.y
            walk.look_at_opposite(x, y)
            walk.move_to(x, y, DIRECTION_BACKWARD)
        g = packets.Goto()
        g.direction = DIRECTION_BACKWARD
        g.points = points
        #self.send_packet(g)
        self.switch_to_substate(walk)




class TestCommands(statemachine.State):

    def on_enter(self):
        seq = commonstates.Sequence()

#        seq.add(commonstates.StoreFabric(FABRIC_STORE_LOW))
#        seq.add(commonstates.MapArm(MAP_ARM_OPEN))
#        seq.add(commonstates.MapGripper(MAP_GRIPPER_OPEN))
#        seq.add(commonstates.MapGripper(MAP_GRIPPER_CLOSE))
#        seq.add(commonstates.MapArm(MAP_ARM_CLOSE))
#        seq.add(commonstates.StoreFabric(FABRIC_STORE_HIGH))
#
#        seq.add(commonstates.Gripper(GRIPPER_SIDE_LEFT, GRIPPER_OPEN))
#        seq.add(commonstates.Gripper(GRIPPER_SIDE_RIGHT, GRIPPER_OPEN))
#        seq.add(commonstates.Gripper(GRIPPER_SIDE_LEFT, GRIPPER_CLOSE))
#        seq.add(commonstates.Gripper(GRIPPER_SIDE_RIGHT, GRIPPER_CLOSE))
        seq.add(commonstates.Gripper(GRIPPER_SIDE_BOTH, GRIPPER_OPEN))
        seq.add(commonstates.Gripper(GRIPPER_SIDE_BOTH, GRIPPER_CLOSE))
#        seq.add(commonstates.EmptyTank(TANK_DEPLOY))
#        seq.add(commonstates.EmptyTank(TANK_RETRACT))

#        seq.add(commonstates.Sweeper(SWEEPER_OPEN))
#        seq.add(commonstates.Sweeper(SWEEPER_CLOSE))


        self.switch_to_substate(seq)


    def on_exit_substate(self, substate):
        self.switch_to_state(TestTraj())


class GotoStartPoint(statemachine.State):

    def on_enter(self):
        self.walk = commonstates.TrajectoryWalk()
        self.walk.move_to(PURPLE_START_X, 0.59)
        if self.robot().team == TEAM_PURPLE:
            self.walk.rotate_to(0.0)
            self.walk.move_to(0.96, 0.59)
        else:
            self.walk.rotate_to(math.pi)
            self.walk.move_to(1.04, 0.59)
        self.switch_to_substate(self.walk)


    def on_exit_substate(self, substate):
        self.switch_to_state(LoopAroundPeanutIsland())




class LoopAroundPeanutIsland(statemachine.State):

    def on_enter(self):
        points = [(1.50, 1.06,  math.pi / 2.0),
                  (1.40, 1.46,  math.pi / 2.0),
                  (1.50, 1.86,  math.pi / 2.0),
                  (1.04, 2.41,  math.pi      ),
                  (0.50, 1.86, -math.pi / 2.0),
                  (0.60, 1.46, -math.pi / 2.0),
                  (0.50, 1.06, -math.pi / 2.0),
                  (0.96, 0.59,  0.0          )]

        if self.robot().team == TEAM_RED:
            points.reverse()
            points = points[1:] + points[:1]
            angle_offset = math.pi
        else:
            angle_offset = 0.0

        self.walk = commonstates.TrajectoryWalk()

        for i in xrange(len(points) * 3):
            k = i % len(points)
            (x, y, angle) = points[k]
            self.walk.goto(x, y, angle + angle_offset)

        self.switch_to_substate(self.walk)


    def on_exit_substate(self, substate):
        self.switch_to_state(GotoCaptainRoom())




class GotoCaptainRoom(statemachine.State):

    def on_enter(self):
        self.walk = commonstates.TrajectoryWalk()
        self.walk.move_to(PURPLE_START_X, 0.59)
        self.walk.rotate_to(PURPLE_START_ANGLE)
        self.walk.move_to(PURPLE_START_X, PURPLE_START_Y)
        self.switch_to_substate(self.walk)

class FindNextGoal(statemachine.State):
    """ Find next goal
    """
    def on_enter(self):
        pass

class TakeMap(statemachine.State):
    """ Take the map
    """
    def on_enter(self):
        pass

class GrapMap(statemachine.State):
    """ Physically grab the map
    """
    def on_enter(self):
        pass

class TakeGoldBar(statemachine.State):
    """ Take a gold bar
    """
    def on_enter(self):
        pass

class GradGoldbarAndStuff(statemachine.State):
    """ Grab the gold bar and all the coins
    """
    def on_enter(self):
        pass

class GotoWithPathFinding(statemachine.State):
    """ Goto in the captain's lair
    """
    def on_enter(self):
        pass

class EmptyTank(statemachine.State):
    """ Empty the tank
    """
    def on_enter(self):
        pass

