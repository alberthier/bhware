#!/usr/bin/env python
# encoding: utf-8


import datetime
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

    def __init__(self, miliseconds, current_move_direction):
        Timer.__init__(self,miliseconds)
        self.exit_reason=self.TIMEOUT
        self.current_move_direction = current_move_direction


    def on_enter(self):
        if self.current_move_direction == DIRECTION_FORWARD:
            self.robot().backward(0.100)
        else:
            self.robot().forward(0.100)


    def on_opponent_left(self):
        self.exit_reason=self.OPPONENT_LEFT
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


    def on_exit_substate(self,finished_substate):
        self.process_next_substate()


    def process_next_substate(self):
        if len(self.substates) == 0:
            self.exit_substate()
        else:
            self.switch_to_substate(self.substates.popleft())




class SetupPositionControl(statemachine.State):

    def __init__(self):
        statemachine.State.__init__(self)
        self.packet = packets.PositionControlConfig()


    def on_enter(self):
        self.send_packet(self.packet)


    def on_position_control_configured(self):
        self.exit_substate()




class DefinePosition(statemachine.State):

    def __init__(self, x = None, y = None, angle = None):
        statemachine.State.__init__(self)
        if x == None or y == None or angle == None:
            self.pose = None
        else:
            self.pose = trajectory.Pose(x, y, angle)
        self.x_sent = False
        self.y_sent = False


    def on_enter(self):
        if self.pose == None:
            if self.robot().team == TEAM_RED:
                self.pose = trajectory.Pose(RED_START_X, RED_START_Y, RED_START_ANGLE)
            else:
                self.pose = trajectory.Pose(PURPLE_START_X, PURPLE_START_Y, PURPLE_START_ANGLE)
        self.process()


    def on_resettled(self, axis, position, angle):
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
        if points != None:
            self.load_points(points)


    def move(self, dx, dy, direction = DIRECTION_FORWARD):
        self.jobs.append(('move', (dx, dy, direction)))


    def move_to(self, x, y, direction = DIRECTION_FORWARD):
        self.jobs.append(('move_to', (x, y, direction)))


    def forward(self, distance):
        self.jobs.append(('forward', (distance,)))


    def backward(self, distance):
        self.jobs.append(('backward', (distance,)))


    def look_at(self, x, y):
        self.jobs.append(('look_at', (x, y)))


    def look_at_opposite(self, x, y):
        self.jobs.append(('look_at_opposite', (x, y)))


    def rotate(self, da):
        self.jobs.append(('rotate', (da,)))


    def rotate_to(self, angle):
        self.jobs.append(('rotate_to', (angle,)))


    def goto(self, x, y, angle, direction = DIRECTION_FORWARD):
        self.jobs.append(('goto', (x, y, angle, direction)))


    def goto_looking_at(self, x, y, look_at_x, look_at_y, direction = DIRECTION_FORWARD):
        self.jobs.append(('goto_looking_at', (x, y, look_at_x, look_at_y, direction)))


    def follow(self, points, direction = DIRECTION_FORWARD):
        self.jobs.append(('follow', (points, direction)))


    def wait_for(self, substate):
        self.jobs.append(('wait_for', (substate,)))


    def load_points(self, points):
        try :
            for vals in points :
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


    def on_enter(self):
        self.process_next_job()


    def on_goto_finished(self, reason, current_pose, current_point_index):
        if reason == REASON_DESTINATION_REACHED:
            self.exit_reason = TRAJECTORY_WALK_DESTINATION_REACHED
            self.current_goto_packet = None
            self.process_next_job()
        elif reason == REASON_PIECE_FOUND:
            if self.current_goto_packet.direction == DIRECTION_BACKWARD:
                self.process_next_job()
            else:
                self.exit_reason = TRAJECTORY_WALK_PIECE_FOUND
                self.exit_substate()


    def process_next_job(self):
        if self.current_goto_packet == None:
            if len(self.jobs) == 0:
                self.exit_reason = TRAJECTORY_WALK_DESTINATION_REACHED
                self.exit_substate()
            else:
                (method, args) = self.jobs.popleft()
                if method == 'wait_for':
                    self.switch_to_substate(*args)
                else:
                    if hasattr(self.robot(), method):
                        self.current_goto_packet = getattr(self.robot(), method)(*(args + (self.reference_team,)))
                    else:
                        logger.log("Unknown move method: {}{}".format(method, args))
                        self.exit_reason = TRAJECTORY_WALK_DESTINATION_REACHED
                        self.exit_substate()
        else:
            self.send_packet(self.current_goto_packet)


    def on_blocked(self, side):
        self.exit_reason = TRAJECTORY_WALK_BLOCKED
        self.exit_substate()


    def on_exit_substate(self, substate):
        if self.current_goto_packet != None and isinstance(substate, WaitForOpponentLeave):
            if substate.exit_reason == WaitForOpponentLeave.OPPONENT_LEFT:
                self.opponent_blocking_current_retries = 0
                self.send_packet(self.current_goto_packet)
            else:
                self.on_opponent_detected(0.0)
        else:
            self.process_next_job()


    def on_opponent_detected(self, angle):
        logger.log("TrajectoryWalk call on_opponent_entered")
        if self.current_goto_packet != None:
            direction = self.current_goto_packet.direction
            is_in_front = self.event_loop.opponent_detector.is_opponent_in_front(angle)
            is_in_back = self.event_loop.opponent_detector.is_opponent_in_back(angle)
            logger.log("TrajectoryWalk self.current_goto_packet != None - is_in_front={} - is_in_back={}".format(is_in_front, is_in_back))
            if direction == DIRECTION_FORWARD and is_in_front or \
                direction == DIRECTION_BACKWARD and is_in_back:
                self.robot().stop()
                logger.log("TrajectoryWalk robot stopped")
                if self.opponent_blocking_current_retries < self.opponent_blocking_max_retries :
                    logger.log("TrajectoryWalk self.opponent_blocking_current_retries={} < self.opponent_blocking_max_retries={}".format(self.opponent_blocking_current_retries, self.opponent_blocking_max_retries))
                    self.opponent_blocking_current_retries += 1
                    self.switch_to_substate(WaitForOpponentLeave(self.opponent_wait_time, direction))
                else :
                    logger.log("TrajectoryWalk TRAJECTORY_WALK_OPPONENT_DETECTED")
                    self.exit_reason = TRAJECTORY_WALK_OPPONENT_DETECTED
                    self.exit_substate()
