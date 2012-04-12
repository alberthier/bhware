#!/usr/bin/env python
# encoding: utf-8


import datetime
import math
from collections import deque

import statemachine
import packets
import trajectory
import logger
from definitions import *




class Timer(statemachine.State):

    TIMEOUT = 0

    def __init__(self, miliseconds):
        logger.log("Enter timer {}".format(datetime.datetime.now()))
        statemachine.State.__init__(self)
        self.end_time = datetime.datetime.now() + datetime.timedelta(0, 0, 0, miliseconds)


    def on_timer_tick(self):
        if datetime.datetime.now() > self.end_time:
            logger.log("End timer {}".format(datetime.datetime.now()))
            self.exit_substate(self.TIMEOUT)




class WaitForOpponentLeave(Timer):

    OPPONENT_LEFT = 1

    def __init__(self, opponent, miliseconds, current_move_direction):
        Timer.__init__(self, miliseconds)
        self.opponent = opponent
        self.exit_reason = self.TIMEOUT


    def on_enter(self):
        if self.current_move_direction == DIRECTION_FORWARD:
            self.robot().backward(0.100)
        else:
            self.robot().forward(0.100)


    def on_opponent_disapeared(self, opponent):
        if opponent == self.opponent:
            self.exit_reason = self.OPPONENT_LEFT
            logger.log("WaitForOpponentLeave : exit on opponent leave")
            self.exit_substate(self.exit_reason)




class Sequence(statemachine.State):

    def __init__(self, *args):
        statemachine.State.__init__(self)
        self.substates = deque(args)


    def add(self, substate):
        self.substates.append(substate)


    def on_enter(self):
        self.process_next_substate()


    def on_exit_substate(self, finished_substate):
        self.process_next_substate()


    def process_next_substate(self):
        if len(self.substates) == 0:
            self.exit_substate()
        else:
            self.switch_to_substate(self.substates.popleft())




class SetupPositionControl(statemachine.State):

    def __init__(self, t_acc = 0.0, f_va_max = 0.0):
        statemachine.State.__init__(self)
        self.packet = packets.PositionControlConfig()
        self.packet.t_acc = t_acc
        self.packet.f_va_max = f_va_max


    def on_enter(self):
        self.send_packet(self.packet)


    def on_position_control_configured(self, packet):
        self.exit_substate()




class DefinePosition(statemachine.State):

    def __init__(self, x = None, y = None, angle = None):
        statemachine.State.__init__(self)
        if x is None or y is None or angle is None:
            self.pose = None
        else:
            self.pose = trajectory.Pose(x, y, angle, True)
        self.x_sent = False
        self.y_sent = False


    def on_enter(self):
        if self.pose is None:
            self.pose = trajectory.Pose(PURPLE_START_X, PURPLE_START_Y, PURPLE_START_ANGLE, True)
        self.process()


    def on_resettle(self, packet):
        self.process()


    def process(self):
        if not self.x_sent:
            packet = packets.Resettle()
            packet.axis = AXIS_X
            packet.position = self.pose.x
            packet.angle = self.pose.angle
            self.send_packet(packet)
            self.x_sent = True
        elif not self.y_sent:
            packet = packets.Resettle()
            packet.axis = AXIS_Y
            packet.position = self.pose.y
            packet.angle = self.pose.angle
            self.send_packet(packet)
            self.y_sent = True
        else:
            self.exit_substate()




class Gripper(statemachine.State):

    def __init__(self, which, move):
        statemachine.State.__init__(self)
        self.packet = packets.GripperControl()
        self.packet.which = which
        self.packet.move = move


    def on_enter(self):
        self.send_packet(self.packet)


    def on_gripper_control(self, packet):
        self.exit_substate()




class Sweeper(statemachine.State):

    def __init__(self, move):
        statemachine.State.__init__(self)
        self.packet = packets.SweeperControl()
        self.packet.move = move


    def on_enter(self):
        self.send_packet(self.packet)


    def on_sweeper_control(self, packet):
        self.exit_substate()




class MapArm(statemachine.State):

    def __init__(self, move):
        statemachine.State.__init__(self)
        self.packet = packets.MapArmControl()
        self.packet.move = move


    def on_enter(self):
        self.send_packet(self.packet)


    def on_map_arm_control(self, packet):
        self.exit_substate()




class MapGripper(statemachine.State):

    def __init__(self, move):
        statemachine.State.__init__(self)
        self.packet = packets.MapGripperControl()
        self.packet.move = move


    def on_enter(self):
        self.send_packet(self.packet)


    def on_map_gripper_control(self, packet):
        self.exit_substate()




class EmptyTank(statemachine.State):

    def __init__(self, move):
        statemachine.State.__init__(self)
        self.packet = packets.EmptyTankControl()
        self.packet.move = move


    def on_enter(self):
        self.send_packet(self.packet)


    def on_empty_tank_control(self, packet):
        self.exit_substate()




class GetGoldBarStatus(statemachine.State):

    def __init__(self):
        statemachine.State.__init__(self)
        self.status = GOLD_BAR_PRESENT


    def on_enter(self):
        self.send_packet(packets.GoldBarDetection())


    def on_gold_bar_detection(self, packet):
        self.status = packet.status
        self.exit_substate()




class StoreFabric(statemachine.State):

    def __init__(self, move):
        statemachine.State.__init__(self)
        self.packet = packets.FabricStoreControl()
        self.packet.move = move


    def on_enter(self):
        self.send_packet(self.packet)


    def on_fabric_store_control(self, packet):
        self.exit_substate()




class TrajectoryWalk(statemachine.State):
    """Walk a path"""

    def __init__(self, points = None, reference_team = TEAM_PURPLE):
        statemachine.State.__init__(self)
        self.jobs = deque()
        self.current_goto_packet = None
        self.opponent_wait_time = DEFAULT_OPPONENT_WAIT_MS
        self.opponent_blocking_max_retries = DEFAULT_OPPONENT_MAX_RETRIES
        self.opponent_blocking_current_retries = 0
        self.exit_reason = TRAJECTORY_WALK_DESTINATION_REACHED
        self.reference_team = reference_team
        if points is not None:
            self.load_points(points)


    def move(self, dx, dy, direction = DIRECTION_FORWARD):
        current_pose = self.robot().pose
        self.move_to(current_pose.virt.x + dx, current_pose.virt.y + dy, direction)


    def move_to(self, x, y, direction = DIRECTION_FORWARD):
        packet = packets.Goto()
        packet.movement = MOVEMENT_MOVE
        packet.direction = direction
        packet.points = [ trajectory.Pose(x, y, None, True) ]
        self.jobs.append(packet)


    def forward(self, distance):
        current_pose = self.robot().pose
        dx = math.cos(current_pose.virt.angle) * distance
        dy = math.sin(current_pose.virt.angle) * distance
        self.move(dx, dy)


    def backward(self, distance):
        current_pose = self.robot().pose
        dx = -math.cos(current_pose.virt.angle) * distance
        dy = -math.sin(current_pose.virt.angle) * distance
        self.move(dx, dy, DIRECTION_BACKWARD)


    def look_at(self, x, y):
        current_pose = self.robot().pose
        dest = trajectory.Pose(x, y, None, True)
        dx = dest.virt.x - current_pose.virt.x
        dy = dest.virt.y - current_pose.virt.y
        self.rotate_to(math.atan2(dy, dx))


    def look_at_opposite(self, x, y):
        current_pose = self.robot().pose
        dest = trajectory.Pose(x, y, None, True)
        dx = dest.virt.x - current_pose.virt.x
        dy = dest.virt.y - current_pose.virt.y
        self.rotate_to(math.atan2(dy, dx) + math.pi)


    def rotate(self, da):
        current_pose = self.robot().pose
        self.rotate_to(current_pose.virt.angle + da)


    def rotate_to(self, angle):
        dest = trajectory.Pose(0.0, 0.0, angle, True)
        packet = packets.Goto()
        packet.movement = MOVEMENT_ROTATE
        packet.angle = dest.angle
        self.jobs.append(packet)


    def goto(self, x, y, angle, direction = DIRECTION_FORWARD):
        dest = trajectory.Pose(x, y, angle, True)
        packet = packets.Goto()
        packet.movement = MOVEMENT_MOVE
        packet.direction = direction
        packet.angle = dest.angle
        packet.points = [ dest ]
        self.jobs.append(packet)


    def goto_looking_at(self, x, y, look_at_x, look_at_y, direction = DIRECTION_FORWARD):
        current_pose = self.robot().pose
        dest = trajectory.Pose(look_at_x, look_at_y, None, True)
        dx = dest.virt.x - current_pose.virt.x
        dy = dest.virt.y - current_pose.virt.y
        self.goto(x, y, math.atan2(dy, dx))


    def goto_looking_at_opposite(self, x, y, look_at_x, look_at_y, direction = DIRECTION_FORWARD):
        current_pose = self.robot().pose
        dest = trajectory.Pose(look_at_x, look_at_y, None, True)
        dx = dest.virt.x - current_pose.virt.x
        dy = dest.virt.y - current_pose.virt.y
        self.goto(x, y, math.atan2(dy, dx) + math.pi)


    def follow(self, points, angle = None, direction = DIRECTION_FORWARD):
        dest = trajectory.Pose(0.0, 0.0, angle, True)
        packet = packets.Goto()
        packet.movement = MOVEMENT_MOVE
        packet.direction = direction
        packet.angle = dest.angle
        packet.points = [ points ]
        self.jobs.append(packet)


    def wait_for(self, substate):
        self.jobs.append(substate)


    def load_points(self, points):
        for vals in points :
            try :
                angle = None
                x, y = 0.0, 0.0
                direction = DIRECTION_FORWARD
                if len(vals) > 1 or type(vals) is dict:
                    if type(vals) is dict:
                        x, y = vals.get("pos", (x, y))
                        angle = vals.get("angle", angle)
                        direction = vals.get("dir", direction)
                    else :
                        if len(vals) == 2:
                            x, y = vals
                        elif len(vals) == 3:
                            x, y, angle = vals
                        elif len(vals) == 4:
                            x, y, angle, direction = vals
                else:
                    angle = vals[0]
                    a = lookup_defs("ANGLE", angle) if angle else None
                    d = lookup_defs("DIRECTION", direction) if direction else None
                    self.goto(x, y, angle, direction)
            except Exception, e :
                logger.log("Error decoding trajectory '{}' : Exception is {}".format(str(vals), str(e)))
                logger.log_exception(e)
                # TODO : exit loop if exception occurred ?


    def on_enter(self):
        self.process_next_job()


    def on_goto_finished(self, packet):
        if packet.reason == REASON_DESTINATION_REACHED:
            self.exit_reason = TRAJECTORY_WALK_DESTINATION_REACHED
            self.current_goto_packet = None
            self.process_next_job()
        elif packet.reason == REASON_BLOCKED_FRONT or packet.reason == REASON_BLOCKED_BACK:
            self.exit_reason = TRAJECTORY_WALK_BLOCKED
            self.exit_substate()


    def process_next_job(self):
        if self.current_goto_packet is None:
            if len(self.jobs) == 0:
                self.exit_reason = TRAJECTORY_WALK_DESTINATION_REACHED
                self.exit_substate()
            else:
                job = self.jobs.popleft()
                if isinstance(job, statemachine.State):
                    self.switch_to_substate(job)
                else:
                    self.current_goto_packet = job
                    self.send_packet(job)
        else:
            self.send_packet(self.current_goto_packet)


    def on_exit_substate(self, substate):
        if self.current_goto_packet is not None and isinstance(substate, WaitForOpponentLeave):
            if substate.exit_reason == WaitForOpponentLeave.OPPONENT_LEFT:
                self.opponent_blocking_current_retries = 0
                self.send_packet(self.current_goto_packet)
            else:
                self.on_opponent_detected(0.0)
        else:
            self.process_next_job()


    def on_opponent_in_front(self, packet):
        self.on_opponent_detected(packet, DIRECTION_FORWARD)


    def on_opponent_in_back(self, packet):
        self.on_opponent_detected(packet, DIRECTION_BACKWARD)


    def on_opponent_detected(self, packet, opponent_direction):
        if self.current_goto_packet is not None:
            direction = self.current_goto_packet.direction
            if self.current_goto_packet.movement != MOVEMENT_ROTATE and self.current_goto_packet.direction == opponent_direction:
                logger.log("Opponent detected. direction = {}".format(opponent_direction))
                self.robot().stop()
                logger.log("TrajectoryWalk robot stopped")
                if self.opponent_blocking_current_retries < self.opponent_blocking_max_retries :
                    logger.log("TrajectoryWalk retries ({}/{})".format(self.opponent_blocking_current_retries, self.opponent_blocking_max_retries))
                    self.opponent_blocking_current_retries += 1
                    self.switch_to_substate(WaitForOpponentLeave(packet.robot, self.opponent_wait_time, direction))
                else :
                    logger.log("TrajectoryWalk failed")
                    self.exit_reason = TRAJECTORY_WALK_OPPONENT_DETECTED
                    self.exit_substate()




class Navigate(statemachine.State):

    def __init__(self, x, y, direction=DIRECTION_FORWARD, reference_team=TEAM_PURPLE):
        super(Navigate, self).__init__()
        self.x = x
        self.y = y
        self.direction = direction
        self.reference_team = reference_team
        self.path = None
        self.rotation_packet = None
        self.exit_reason = NAVIGATE_DESTINATION_REACHED


    def on_enter(self):
        self.path = self.event_loop.map.route(self.robot().pose.x, self.robot().pose.y, self.x, self.y)
        if len(path) == 0:
            self.exit_reason = NAVIGATE_DESTINATION_UNREACHABLE
            self.exit_substate()
        else:
            if direction == DIRECTION_FORWARD:
                if not self.robot().is_looking_at(path[0].x, path[0].y, self.reference_team):
                    self.rotation_packet = self.robot().look_at(path[0].x, path[0].y, self.reference_team)
            else:
                if not self.robot().is_looking_at_opposite(path[0].x, path[0].y, self.reference_team):
                    self.rotation_packet = self.robot().look_at_opposite(path[0].x, path[0].y, self.reference_team)
            if self.rotation_packet is None:
                self.send_path()


    def on_goto_finished(self, packet):
        if self.rotation_packet is not None:
            self.rotation_packet = None
            self.send_path()
        elif packet.reason == REASON_DESTINATION_REACHED:
            self.exit_reason = NAVIGATE_DESTINATION_REACHED
            self.exit_substate()
        else:
            self.exit_reason = NAVIGATE_BLOCKED
            self.exit_substate()


    def on_opponent_in_front(self, packet):
        if self.direction == DIRECTION_FORWARD:
            self.robot().stop()
            self.exit_reason = NAVIGATE_OPPONENT_IN_THE_WAY
            self.exit_substate()


    def on_opponent_in_back(self, packet):
        if self.direction == DIRECTION_BACKWARD:
            self.robot().stop()
            self.exit_reason = NAVIGATE_OPPONENT_IN_THE_WAY
            self.exit_substate()


    def send_path(self):
        self.robot().follow(self.path, self.direction, self.reference_team)




class GotoHome(statemachine.State):

    def on_enter(self):
        current_pose = self.event_loop.robot.pose
        walk = TrajectoryWalk()
        if current_pose.x < 1.0:
            if math.cos(current_pose.angle) < 0.0:
                walk.goto(PURPLE_START_X, 0.70, math.pi / 2.0)
            else:
                walk.goto(PURPLE_START_X, 0.70, -math.pi / 2.0, DIRECTION_BACKWARD)
        else:
            if math.cos(current_pose.angle) < 0.0:
                walk.goto(1.5, 0.70, math.pi / 2.0, DIRECTION_BACKWARD)
            else:
                walk.goto(1.5, 0.70, -math.pi / 2.0)
            #walk.rotate_to(math.pi)
            #walk.move_to(PURPLE_START_X, 0.70)
        #walk.rotate_to(PURPLE_START_ANGLE)
        #walk.move_to(PURPLE_START_X, PURPLE_START_Y)
        self.switch_to_substate(walk)


    def on_exit_substate(self, substate):
        self.exit_substate()
