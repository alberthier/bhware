# encoding: utf-8

import math

import statemachine
import packets
import trajectory
import logger
import commonstates
import goalmanager

from definitions import *

class Override(object):
    def __init__(self, state):
        self.state = state




TAKE_GOLDBAR_APPROACH = 0.115

totem_take_positions = { # name           x      y     angle
                         "SELF_NORTH"  : ( 0.878, 1.10, 0.0     ),
                         "SELF_SOUTH"  : ( 1.122, 1.10, math.pi ),
                         "OTHER_NORTH" : ( 0.878, 1.90, 0.0     ),
                         "OTHER_SOUTH" : ( 1.122, 1.90, math.pi )
}

totem_approach_delta = ROBOT_X_SIZE - ROBOT_CENTER_X

totem_approach_start_positions = {}
totem_approach_end_positions = {}

for k, v in list(totem_take_positions.items()) :
    vals = None
    if k.endswith("_NORTH") :
        vals = v[0] - (TAKE_GOLDBAR_APPROACH + totem_approach_delta), v[1], v[2]
        totem_approach_start_positions[k] = vals
        vals = v[0] - totem_approach_delta, v[1], v[2]
        totem_approach_end_positions[k] = (vals[0] + 0.05, vals[1], vals[2])
    else :
        vals = v[0] + (TAKE_GOLDBAR_APPROACH + totem_approach_delta), v[1], v[2]
        totem_approach_start_positions[k] = vals
        vals = v[0] + totem_approach_delta, v[1], v[2]
        totem_approach_end_positions[k] = (vals[0] - 0.05, vals[1], vals[2])

APPROACH_ABSOLUTE_X = totem_approach_start_positions["SELF_NORTH"][0]


class Main(statemachine.State):

    def on_enter(self):
        gm = goalmanager.GoalManager(self.event_loop)
        self.robot().goal_manager = gm
        map_goal = goalmanager.Goal("MAP", 1.0, 0.31, 1.37, DIRECTION_BACKWARD, GrabMap)
        gm.harvesting_goals.append(map_goal)
        gm.emptying_goals.append(map_goal)

        x1, y1 = APPROACH_ABSOLUTE_X, 0.86
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
        #        gm.harvesting_goals.append(goalmanager.Goal("SWIFFER"    , 0.5, 1.37         , 2.0          , DIRECTION_BACKWARD, Swiffer))

        gm.emptying_goals.append(goalmanager.Goal("DEPOSIT_CAPTAIN", 2.0, 0.30, 0.60, DIRECTION_FORWARD, DepositTreasure))
        gm.emptying_goals.append(goalmanager.Goal("DEPOSIT_2",       1.0, 0.90, 0.45, DIRECTION_FORWARD, DepositTreasure))
        gm.emptying_goals.append(goalmanager.Goal("DEPOSIT_3",       1.0, 0.80, 0.45, DIRECTION_FORWARD, DepositTreasure))
        gm.emptying_goals.append(goalmanager.Goal("DEPOSIT_4",       1.0, 1.00, 0.45, DIRECTION_FORWARD, DepositTreasure))


    def on_device_ready(self, packet):
        yield commonstates.DefinePosition()
        yield WaitStart()



class WaitStart(statemachine.State):

    def on_enter(self):
        self.sent = False


    def on_start(self, packet):
        yield commonstates.DefinePosition()
        yield FindNextGoal()



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
        yield state
        yield Override(FindNextGoal())

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
            state = yield nav
            if state.exit_reason == TRAJECTORY_DESTINATION_REACHED :
                self.switch_to_goal_state()
            else :
                logger.log("Goal {} unreachable or blocked, adding a penalty (reason={})".format(self.current_goal
                                                                                                 .identifier, state.exit_reason))
                self.robot().goal_manager.penalize_goal(self.current_goal)
                yield Override(FindNextGoal())
            
        else :
            yield Override(EndOfMatch())





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
        walk.backward(0.25)
        walk.wait_for(commonstates.MapGripper(MAP_GRIPPER_CLOSE))
        walk.forward(0.25)
        walk.wait_for(commonstates.MapArm(MAP_ARM_CLOSE))
        walk.wait_for(commonstates.StoreFabric(FABRIC_STORE_HIGH))
        yield walk
        self.robot().goal_manager.goal_done(self.goal)
        

        




class DepositTreasure(statemachine.State):
    """ Empty the tank
    """
    def __init__(self, goal):
        self.goal = goal


    def on_enter(self):
        walk = commonstates.TrajectoryWalk()
        walk.rotate_to(-math.pi / 2.0)
        walk.wait_for(commonstates.Gripper(GRIPPER_SIDE_BOTH, GRIPPER_OPEN))
        walk.wait_for(commonstates.EmptyTank(TANK_DEPLOY))
        walk.wait_for(commonstates.EmptyTank(TANK_RETRACT))
        walk.wait_for(commonstates.EmptyTank(TANK_DEPLOY))
        walk.backward(0.150)
        walk.wait_for(commonstates.EmptyTank(TANK_RETRACT))
        walk.wait_for(commonstates.Gripper(GRIPPER_SIDE_BOTH, GRIPPER_CLOSE))
        yield walk
        packet = packets.EmptyTankControl()
        packet.move = TANK_RETRACT
        self.send_packet(packet)
        # No need to wait for confirmation here
        self.robot().goal_manager.goal_done(self.goal)
        





class TakeGoldBar(statemachine.State):
    """ Take a gold bar
    """

    def __init__(self, goal):
        """
        @param goal: Goal which led to this state
        @type goal: trajectory.Goal
        """
        super(TakeGoldBar,self).__init__()
        self.goal = goal
        self.start_pos = totem_approach_start_positions[goal.identifier]

    def on_enter(self):
        walk = commonstates.TrajectoryWalk()

        walk.look_at_opposite(self.start_pos[0], self.start_pos[1])
        walk.move_to(self.start_pos[0], self.start_pos[1], DIRECTION_BACKWARD)

        walk.wait_for(commonstates.Antiblocking(True))
        walk.rotate_to(self.start_pos[2])
        walk.wait_for(commonstates.Antiblocking(False))

        state = yield walk
        if state.exit_reason != REASON_DESTINATION_REACHED:
            self.robot().goal_manager.penalize_goal(self.goal)
            yield Override(EscapeTotem())
        else:
            yield DetectAndTakeGoldbar(self.goal)
            
            




class EscapeTotem(statemachine.State):

    def on_enter(self):
        self.count = 0
        self.try_escape()


    def try_escape(self):
        self.count += 1
        walk = commonstates.TrajectoryWalk()
        walk.wait_for(commonstates.Antiblocking(True))
        walk.backward(0.15)
        walk.wait_for(commonstates.Antiblocking(False))
        state = yield walk
        if state.exit_reason != REASON_DESTINATION_REACHED and self.count <= 5:
            self.try_escape()
        else:
            self.send_packet(packets.DisableAntiBlocking())
            





class DetectAndTakeGoldbar(statemachine.State):
    def __init__(self, goal):
        """
        @type goal: Goal
        """
        self.goal = goal

    def on_enter(self):
        state = yield commonstates.GetGoldBarStatus()
        if self.goal.identifier.startswith("SELF") or state.status == GOLD_BAR_PRESENT :
            logger.log("Goldbar was present")
            walk = commonstates.TrajectoryWalk()
            #open gripper
            walk.wait_for(commonstates.Gripper(GRIPPER_SIDE_BOTH, GRIPPER_OPEN))
            #TODO : speed up by opening gripper while moving
            #look at totem
            totem_take_pose = totem_approach_end_positions[self.goal.identifier][0:2]
            walk.look_at(*totem_take_pose)
            #bump again totem
            walk.move_to(*totem_take_pose,direction=DIRECTION_FORWARD)
            #close gripper
            walk.wait_for(commonstates.Gripper(GRIPPER_SIDE_BOTH, GRIPPER_CLOSE))
            #go backwards
            walk.move_to(*totem_approach_start_positions[self.goal.identifier][0:2],direction=DIRECTION_BACKWARD)
            yield walk
            self.robot().tank_full = True
            return
        else:
            logger.log("Goldbar was not detected")
        
        self.robot().goal_manager.goal_done(self.goal)
                    
        




class Swiffer(statemachine.State):
    """Try to find some pieces to grab"""
    def __init__(self, goal):
        """
        @param goal: goal which led to this state
        @type goal: trajectory.Goal
        """
        self.goal = goal

    def on_enter(self):
        walk = commonstates.TrajectoryWalk()
        walk.rotate_to(math.pi)
        walk.wait_for(commonstates.Sweeper(SWEEPER_OPEN))
        walk.move_to(1.65, 2.05, direction=DIRECTION_BACKWARD)
        walk.rotate_to(math.pi / 2)
        walk.move_to(1.65, 0.67, direction=DIRECTION_BACKWARD)
        walk.rotate_to(0.0)
        walk.backward(0.7)
        walk.rotate_to(math.pi/2)
        walk.backward(0.35)
        yield walk
        self.robot().goal_manager.goal_done(self.goal)
        



class EndOfMatch(statemachine.State):
    """End of match"""

    def on_enter(self):
        pass
