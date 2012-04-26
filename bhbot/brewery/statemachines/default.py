#!/usr/bin/env python
# encoding: utf-8

import math

import statemachine
import packets
import trajectory
import logger
import commonstates
import goalmanager

from definitions import *



################################################################################
# Setup states
#from bhbot.brewery.definitions import DIRECTION_BACKWARD


class Main(statemachine.State):

    def on_enter(self):
        gm = goalmanager.GoalManager(self.event_loop)
        self.robot().goal_manager = gm
        gm.harvesting_goals.append(goalmanager.Goal("MAP", 1.0, 0.31, 1.37, DIRECTION_BACKWARD, GrabMap))

        x1, y1 = 0.60, 0.86
        x2, y2 = x1, (1.1 - y1) + 1.1
        offset_x = (1.0 - x1) * 2.0
        offset_y = 0.80

        gm.harvesting_goals.append(goalmanager.Goal("SELF_NORTH" , 1.0, x1           , y1           , DIRECTION_BACKWARD, TakeGoldBar))
        gm.harvesting_goals.append(goalmanager.Goal("SELF_NORTH" , 1.0, x2           , y2           , DIRECTION_BACKWARD, TakeGoldBar))
        gm.harvesting_goals.append(goalmanager.Goal("SELF_SOUTH" , 1.0, x1 + offset_x, y1           , DIRECTION_BACKWARD, TakeGoldBar))
        gm.harvesting_goals.append(goalmanager.Goal("SELF_SOUTH" , 1.0, x2 + offset_x, y2           , DIRECTION_BACKWARD, TakeGoldBar))
        gm.harvesting_goals.append(goalmanager.Goal("OTHER_NORTH", 1.0, x1           , y1 + offset_y, DIRECTION_BACKWARD, TakeGoldBar))
        gm.harvesting_goals.append(goalmanager.Goal("OTHER_NORTH", 1.0, x2           , y2 + offset_y, DIRECTION_BACKWARD, TakeGoldBar))
        gm.harvesting_goals.append(goalmanager.Goal("OTHER_SOUTH", 1.0, x1 + offset_x, y1 + offset_y, DIRECTION_BACKWARD, TakeGoldBar))
        gm.harvesting_goals.append(goalmanager.Goal("OTHER_SOUTH", 1.0, x2 + offset_x, y2 + offset_y, DIRECTION_BACKWARD, TakeGoldBar))

        gm.emptying_goals.append(goalmanager.Goal("DEPOSIT_1", 2.0, 0.25, 0.6, DIRECTION_FORWARD, EmptyTank))
        gm.emptying_goals.append(goalmanager.Goal("DEPOSIT_2", 1.0, 0.90, 0.5, DIRECTION_FORWARD, EmptyTank))
        gm.emptying_goals.append(goalmanager.Goal("DEPOSIT_3", 1.0, 0.72, 0.5, DIRECTION_FORWARD, EmptyTank))
        gm.emptying_goals.append(goalmanager.Goal("DEPOSIT_4", 1.0, 1.06, 0.5, DIRECTION_FORWARD, EmptyTank))


    def on_device_ready(self, packet):
        self.switch_to_substate(commonstates.DefinePosition())


    #noinspection PyUnusedLocal
    def on_exit_substate(self, substate):
        self.switch_to_state(WaitStart())




class WaitStart(statemachine.State):

    def on_enter(self):
        self.sent = False

    def on_start(self, packet):
        self.switch_to_substate(commonstates.DefinePosition())


    def on_exit_substate(self, substate):
        self.switch_to_state(WaitFirstKeepAlive())




class WaitFirstKeepAlive(statemachine.State):

    def on_keep_alive(self, packet):
        #self.switch_to_state(GotoStartPoint())
        #self.switch_to_state(TestTraj())
#        self.switch_to_state(TestCommands())
        self.switch_to_state(FindNextGoal())




class TestTraj(statemachine.State):

    def on_enter(self):
        walk = commonstates.TrajectoryWalk(None, TEAM_UNKNOWN)
        points = self.event_loop.map.route(self.robot().pose, trajectory.Pose(1.6, 2.0, None, True))
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


class FindNextGoal(statemachine.State):
    """ Find next goal
    """
    def __init__(self):
        super(FindNextGoal, self).__init__()
        self.current_goal = None

    def on_enter(self):
        self.find_next()

    def switch_to_goal_state(self):
        ctor = self.current_goal.handler_state
        state = None
        if self.current_goal.ctor_parameters :
            logger.log("Ctor parameters {}".format(self.current_goal.ctor_parameters))
            state = ctor(self.current_goal, *self.current_goal.ctor_parameters)
        else :
            state = ctor(self.current_goal)
        self.switch_to_substate(state)

    def find_next(self):
        import time
        logger.log("Calling GoalManager")
        start_time = time.time()
        next_goal = self.robot().goal_manager.next_goal()
        end_time = time.time()
        if next_goal :
            self.current_goal = next_goal
            logger.log("Next goal is " + next_goal.identifier)
            logger.log("Time taken for decision taking {} ms".format((end_time - start_time) * 1000))
            nav = commonstates.Navigate(next_goal.x, next_goal.y, next_goal.direction)
            self.switch_to_substate(nav)
        else :
            self.switch_to_state(EndOfMatch())


    def on_exit_substate(self, state):
        if isinstance(state, commonstates.Navigate ) :
            if state.exit_reason == TRAJECTORY_DESTINATION_REACHED :
                self.switch_to_goal_state()
            elif state.exit_reason == TRAJECTORY_DESTINATION_UNREACHABLE :
                logger.log("Goal {} unreachable, skipping it".format(self.current_goal.identifier))
                self.robot().goal_manager.goal_done(self.current_goal)
                self.switch_to_state(FindNextGoal())
            else :
                self.switch_to_state(FindNextGoal())
        else :
            self.switch_to_state(FindNextGoal())


class GrabMap(statemachine.State):
    """ Physically grab the map
    """
    def __init__(self, goal):
        super(GrabMap, self).__init__()
        self.goal = goal

    def on_enter(self):
        walk = commonstates.TrajectoryWalk()
        walk.rotate_to(0.0)
        walk.wait_for(commonstates.StoreFabric(FABRIC_STORE_LOW))
        walk.wait_for(commonstates.MapArm(MAP_ARM_OPEN))
        walk.wait_for(commonstates.MapGripper(MAP_GRIPPER_OPEN))
        walk.backward(0.10)
        walk.wait_for(commonstates.MapGripper(MAP_GRIPPER_CLOSE))
        walk.wait_for(commonstates.MapArm(MAP_ARM_CLOSE))
        walk.wait_for(commonstates.StoreFabric(FABRIC_STORE_HIGH))
        walk.forward(0.10)
        self.switch_to_substate(walk)

    def on_exit_substate(self, state):
        self.robot().goal_manager.goal_done(self.goal)
        self.exit_substate()


class TakeGoldBar(statemachine.State):
    """ Take a gold bar
    """
    def __init__(self, goal):
        super(TakeGoldBar,self).__init__()
        self.goal = goal


    def on_enter(self):
        walk = commonstates.TrajectoryWalk()
        self.switch_to_substate(walk)

    def on_exit_substate(self, state):
        self.robot().tank_full = True
        self.robot().goal_manager.goal_done(self.goal)
        self.exit_substate()

class GrabGoldbarAndStuff(statemachine.State):
    """ Grab the gold bar and all the coins
    """
    def __init__(self):
        super(GrabGoldbarAndStuff, self).__init__()

    def on_enter(self):
        pass

#class GotoWithPathFinding(statemachine.State):
#    """ Goto in the captain's lair
#    """
#    def __init__(self):
#        super(GotoWithPathFinding, self).__init__()
#
#    def on_enter(self):
#        pass


class EmptyTank(statemachine.State):
    """ Empty the tank
    """
    def __init__(self, goal):
        self.goal = goal

    def on_enter(self):
        self.switch_to_substate(commonstates.EmptyTank(TANK_DEPLOY))


    def on_exit_substate(self, substate):
        packet = packets.EmptyTankControl()
        packet.move = TANK_RETRACT
        self.send_packet(packet)
        # No need to wait for confirmation here
        self.robot().goal_manager.goal_done(self.goal)
        self.exit_substate()

class EndOfMatch(statemachine.State):
    """End of match"""

    def on_enter(self):
        pass
