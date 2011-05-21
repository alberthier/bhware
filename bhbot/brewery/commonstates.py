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

    def __init__(self, miliseconds):
        statemachine.State.__init__(self)
        self.end_time = datetime.datetime.now() + datetime.timedelta(0, 0, 0, miliseconds)


    def on_keep_alive(self, current_pose, match_started, match_time):
        if datetime.datetime.now() > self.end_time:
            self.exit_substate()




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
                self.pose = trajectory.Pose(BLUE_START_X, BLUE_START_Y, BLUE_START_ANGLE)
        self.process()


    def on_resettled(self, axis, position, angle):
        self.process()


    def process(self):
        if not self.x_sent:
            packet = packets.Resettle()
            packet.axis = AXIS_ABSCISSA
            packet.position = self.pose.x
            packet.angle = self.pose.angle
            self.send_packet(packet)
            self.x_sent = True
        elif not self.y_sent:
            packet = packets.Resettle()
            packet.axis = AXIS_ORDINATE
            packet.position = self.pose.y
            packet.angle = self.pose.angle
            self.send_packet(packet)
            self.y_sent = True
        else:
            self.exit_substate()




class Deploy(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.Deployment())


    def on_deployed(self):
        self.exit_substate()




class StorePiece1(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.StorePiece1())


    def on_piece_stored1(self, piece_count):
        self.exit_substate()




class StorePiece2(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.StorePiece2())


    def on_piece_stored2(self, piece_count):
        self.exit_substate()




class StorePiece3(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.StorePiece3())


    def on_piece_stored3(self, piece_count):
        self.exit_substate()




class DirectStorePiece(Sequence):

    def __init__(self):
        Sequence.__init__(self)
        self.add(StorePiece1())
        self.add(StorePiece2())
        self.add(StorePiece3())




class ReleasePiece(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.ReleasePiece())


    def on_piece_released(self):
        self.exit_substate()




class OpenNippers(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.OpenNippers())


    def on_nippers_opened(self):
        self.exit_substate()




class CloseNippers(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.CloseNippers())


    def on_nippers_closed(self):
        self.exit_substate()




class EnableLateralSensors(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.EnableLateralSensors())


    def on_lateral_sensors_enabled(self):
        self.exit_substate()




class DisableLateralSensors(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.DisableLateralSensors())


    def on_lateral_sensors_disabled(self):
        self.exit_substate()




class TrajectoryWalk(statemachine.State):
    """Walk a path"""

    def __init__(self, points = None, reference_team = TEAM_RED):
        statemachine.State.__init__(self)
        self.moves = deque()
        self.current_packet = None
        self.exit_reason = TRAJECTORY_WALK_DESTINATION_REACHED
        self.reference_team = reference_team
        if points != None:
            self.load_points(points)


    def move(self, dx, dy, direction = DIRECTION_FORWARD):
        self.moves.append(('move', (dx, dy, direction)))


    def move_to(self, x, y, direction = DIRECTION_FORWARD):
        self.moves.append(('move_to', (x, y, direction)))


    def forward(self, distance):
        self.moves.append(('forward', (distance,)))


    def backward(self, distance):
        self.moves.append(('backward', (distance,)))


    def look_at(self, x, y):
        self.moves.append(('look_at', (x, y)))


    def look_at_opposite(self, x, y):
        self.moves.append(('look_at_opposite', (x, y)))


    def rotate(self, da):
        self.moves.append(('rotate', (da,)))


    def rotate_to(self, angle):
        self.moves.append(('rotate_to', (angle,)))


    def goto(self, x, y, angle, direction):
        self.moves.append(('goto', (x, y, angle, direction)))


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
                    logger.log("Traj : {0}, {1}, {2}, {3}".format(x, y, a, d))
                    self.goto(x, y, angle, direction)
        except Exception, e :
            logger.log( "Error decoding trajectory '{0}' : Exception is {1}".format(str(vals),str(e)))
            logger.exception(e)


    def on_enter(self):
        self.process_next_move()


    def on_goto_finished(self, reason, current_pose):
        if reason == REASON_DESTINATION_REACHED:
            self.exit_reason = TRAJECTORY_WALK_DESTINATION_REACHED
            self.current_packet = None
            self.process_next_move()
        elif reason == REASON_PIECE_FOUND:
            self.exit_reason = TRAJECTORY_WALK_PIECE_FOUND
            self.exit_substate()


    def process_next_move(self):
        if self.current_packet == None:
            if len(self.moves) == 0:
                self.exit_reason = TRAJECTORY_WALK_DESTINATION_REACHED
                self.exit_substate()
            else:
                (method, args) = self.moves.popleft()
                if hasattr(self.robot(), method):
                    self.current_packet = getattr(self.robot(), method)(*(args + (self.reference_team,)))
                else:
                    logger.log("Unknown move method: {O}{1}".format(method, args))
                    self.exit_reason = TRAJECTORY_WALK_DESTINATION_REACHED
                    self.exit_substate()
        else:
            self.send_packet(self.current_packet)


    def on_blocked(self, side):
        self.exit_reason = TRAJECTORY_WALK_BLOCKED
        self.exit_substate()


    def on_opponent_detected(self, angle):
        self.exit_reason = TRAJECTORY_WALK_OPPONENT_DETECTED
        self.exit_substate()
