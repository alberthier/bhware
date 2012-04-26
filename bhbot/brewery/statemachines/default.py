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
        gm.harvesting_goals.append(goalmanager.Goal("MAP", 1.0, 0.1043, 1.3699, GrabMap))
        gm.harvesting_goals.append(goalmanager.Goal("SELF_NORTH", 1.0, 0.60, 0.86, TakeGoldBar))
        gm.harvesting_goals.append(goalmanager.Goal("SELF_NORTH", 1.0, 0.60, 1.30, TakeGoldBar))
        gm.harvesting_goals.append(goalmanager.Goal("SELF_SOUTH", 1.0, 1.35, 0.86, TakeGoldBar))
        gm.harvesting_goals.append(goalmanager.Goal("SELF_SOUTH", 1.0, 1.35, 1.30, TakeGoldBar))

        gm.emptying_goals.append(goalmanager.Goal("DEPOSIT_1", 1.0, 0.7, 0.5, EmptyTank))
        gm.emptying_goals.append(goalmanager.Goal("DEPOSIT_2", 1.0, 0.85, 0.48, EmptyTank))
        gm.emptying_goals.append(goalmanager.Goal("DEPOSIT_3", 1.0, 1.02, 0.48, EmptyTank))
        gm.emptying_goals.append(goalmanager.Goal("DEPOSIT_4", 1.0, 1.21, 0.46, EmptyTank))

#        for gold_bar_start_pos in [(0.60, 0.86, 0.0), (1.35, 0.86, 0.0)] :
#            gm.harvesting_goals.append(goalmanager.Goal(0.0, 1.0, gold_bar_start_pos[0], gold_bar_start_pos[1], TakeGoldBar,
#                gold_bar_start_pos))


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
            cProfile.runctx("import trajectory;self.event_loop.map.route(self.robot().pose, trajectory.Pose(1.6, 2.0, virtual=True))", None, { "self": self })
            sys.stdout.flush()
            self.event_loop.map.route(self.robot().pose, trajectory.Pose(0.5, 1.0, virtual=True))
            #self.event_loop.map.route(self.robot().pose.x, self.robot().pose.y, 1.8, 2.8)


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
            nav = commonstates.Navigate(next_goal.x, next_goal.y)
            self.switch_to_substate(nav)


    def on_exit_substate(self, state):
        if isinstance(state, commonstates.Navigate ) :
            if state.exit_reason == REASON_DESTINATION_REACHED :
                self.switch_to_goal_state()
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
        walk.forward(10.0)
        walk_at_end = commonstates.TrajectoryWalk()
        walk_at_end.backward(20.0)
        #TODO : parallelize
        seq = commonstates.Sequence(commonstates.FabricStore(FABRIC_STORE_LOW),
                                    commonstates.MapArm(MAP_ARM_OPEN),
                                    commonstates.MapGripper(MAP_GRIPPER_OPEN),
                                    walk,
                                    commonstates.MapGripper(MAP_GRIPPER_CLOSE),
                                    commonstates.MapArm(MAP_ARM_CLOSE),
                                    commonstates.FabricStore(FABRIC_STORE_HIGH),
                                    walk_at_end
                                    )
        self.switch_to_substate(seq)

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

