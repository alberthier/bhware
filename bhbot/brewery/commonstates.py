# encoding: utf-8


import datetime
import math
from collections import deque

import statemachine
import packets
import position
import logger
import eventloop
import tools
from definitions import *




class Timer(statemachine.State):

    TIMEOUT = 0

    def __init__(self, miliseconds):
        statemachine.State.__init__(self)
        """
        @param miliseconds: Time to wait in miliseconds
        @type miliseconds: float
        """
        self.miliseconds = miliseconds


    def on_enter(self):
        self.end_time = datetime.datetime.now() + datetime.timedelta(0, 0, 0, self.miliseconds)


    def on_timer_tick(self):
        if datetime.datetime.now() > self.end_time:
            self.return_value = Timer.TIMEOUT
            return self.on_timeout()


    def on_timeout(self):
        yield None




class SetupPositionControl(statemachine.State):

    def __init__(self, t_acc = 0.0, f_va_max = 0.0):
        statemachine.State.__init__(self)
        self.packet = packets.PositionControlConfig()
        self.packet.t_acc = t_acc
        self.packet.f_va_max = f_va_max


    def on_enter(self):
        self.send_packet(self.packet)


    def on_position_control_config(self, packet):
        yield None




class WaitPacket(statemachine.State):
    def __init__(self, packet_class):
        logger.log('Waiting for packet type {}'.format(packet_class.__name__))
        self.packet_class = packet_class

    def on_packet(self, packet):
        if isinstance(packet, self.packet_class):
            yield None




class DefinePosition(statemachine.State):

    def __init__(self, x = None, y = None, angle = None):
        statemachine.State.__init__(self)
        if x is None or y is None or angle is None:
            self.pose = None
        else:
            self.pose = position.Pose(x, y, angle, True)
        self.x_sent = False
        self.y_sent = False


    def on_enter(self):
        if self.pose is None:
            self.pose = position.Pose(BLUE_START_X, BLUE_START_Y, BLUE_START_ANGLE, True)

        packet = packets.Resettle()
        packet.axis = AXIS_X
        packet.position = self.pose.x
        packet.angle = self.pose.angle
        self.send_packet(packet)

        yield WaitPacket(packets.Resettle)

        packet = packets.Resettle()
        packet.axis = AXIS_Y
        packet.position = self.pose.y
        packet.angle = self.pose.angle
        self.send_packet(packet)

        yield WaitPacket(packets.Resettle)
        yield None




class Antiblocking(statemachine.State):

    def __init__(self, desired_status):
        self.status = desired_status
        if desired_status :
            self.packet = packets.EnableAntiBlocking()
        else :
            self.packet = packets.DisableAntiBlocking()


    def on_enter(self):
        self.send_packet(self.packet)


    def on_enable_anti_blocking(self, packet):
        yield None


    def on_disable_anti_blocking(self, packet):
        yield None




class WaitForOpponentLeave(Timer):

    TIMEOUT       = 0
    OPPONENT_LEFT = 1

    def __init__(self, opponent, miliseconds, move_direction):
        Timer.__init__(self, miliseconds)
        self.opponent = opponent
        self.move_direction = move_direction


    def on_enter(self):
        Timer.on_enter(self)
        self.goto_finished = False
        self.opponent_disappeared = False
        self.timer_expired = False
        self.exit_reason = None
        if self.move_direction == DIRECTION_FORWARDS:
            direction = DIRECTION_BACKWARDS
            distance = -0.100
        else:
            direction = DIRECTION_FORWARDS
            distance = 0.100

        current_pose = self.robot().pose
        x = current_pose.virt.x + math.cos(current_pose.virt.angle) * distance
        y = current_pose.virt.y + math.sin(current_pose.virt.angle) * distance
        packet = packets.MoveLine()
        packet.direction = direction
        packet.points = [ position.Pose(x, y, None, True) ]
        self.send_packet(packet)


    def on_timeout(self):
        self.timer_expired = True
        if self.exit_reason is None:
            self.exit_reason = self.TIMEOUT
        return self.try_leave()


    def on_goto_finished(self, packet):
        self.goto_finished = True
        return self.try_leave()


    def on_opponent_disapeared(self, opponent, opponent_direction):
        if opponent == self.opponent:
            self.exit_reason = self.OPPONENT_LEFT
            self.opponent_disappeared = True
            if not self.goto_finished:
                self.send_packet(packets.Stop())
            return self.try_leave()


    def try_leave(self):
        if self.goto_finished and (self.timer_expired or self.opponent_disappeared):
            logger.log("WaitForOpponentLeave : exit reason={}".format(self.exit_reason))
            yield None




class AbstractMove(statemachine.State):

    def __init__(self, chained, wait_opponent_leave):
        self.current_opponent = None
        self.chained = chained
        self.wait_opponent_leave = wait_opponent_leave


    def on_enter(self):
        if self.chained is not None and self.chained.exit_reason != REASON_DESTINATION_REACHED:
            self.exit_reason = self.chained.exit_reason
            yield None
        else:
            self.send_packet(self.packet)


    def on_opponent_detected(self, packet, opponent_direction, x, y):
        if not isinstance(self, Rotate) and self.packet.direction == opponent_direction and self.current_opponent is None:
            logger.log("Opponent detected. direction = {}. Stop robot".format(opponent_direction))
            self.send_packet(packets.Stop())
            self.current_opponent = packet.robot


    def on_goto_finished(self, packet):
        if packet.reason == REASON_DESTINATION_REACHED:
            self.exit_reason = TRAJECTORY_DESTINATION_REACHED
            yield None
        elif packet.reason == REASON_BLOCKED_FRONT or packet.reason == REASON_BLOCKED_BACK:
            self.exit_reason = TRAJECTORY_BLOCKED
            yield None
        elif self.current_opponent is not None:
            if self.wait_opponent_leave:
                reason = (yield WaitForOpponentLeave(self.current_opponent, 2000, self.packet.direction)).exit_reason
                if reason == WaitForOpponentLeave.TIMEOUT:
                    self.exit_reason = TRAJECTORY_OPPONENT_DETECTED
                    yield None
                else:
                    # Continue the current move
                    if hasattr(self.packet, "points"):
                        self.packet.points = self.packet.points[packet.current_point_index:]
                    self.send_packet(self.packet)
            else:
                self.exit_reason = TRAJECTORY_OPPONENT_DETECTED
                yield None




class Rotate(AbstractMove):

    def __init__(self, direction, angle, chained = None):
        AbstractMove.__init__(self, chained, False)
        self.packet = packets.Rotate(direction = direction, angle = angle)




class MoveCurve(AbstractMove):

    def __init__(self, direction, angle, points, chained = None):
        AbstractMove.__init__(self, chained, True)
        self.packet = packets.MoveCurve(direction = direction, angle = angle, points = points)




class MoveLine(AbstractMove):

    def __init__(self, direction, points, chained = None):
        AbstractMove.__init__(self, chained, True)
        self.packet = packets.MoveLine(direction = direction, points = points)




class MoveArc(AbstractMove):

    def __init__(self, direction, center, radius, points, chained = None):
        AbstractMove.__init__(self, chained, True)
        self.packet = packets.MoveArc(direction = direction, center = center, radius = radius, points = points)




class BottomHolder(statemachine.State):

    def __init__(self, side, move):
        self.packet = packets.BottomHolder(side = side, move = move)


    def on_enter(self):
        self.send_packet(self.packet)


    def on_bottom_holder(self, packet):
        yield None




class Lifter(statemachine.State):

    def __init__(self, side, move):
        self.packet = packets.Lifter(side = side, move = move)


    def on_enter(self):
        self.send_packet(self.packet)


    def on_lifter(self, packet):
        yield None




class Gripper(statemachine.State):

    def __init__(self, side, move):
        self.packet = packets.Gripper(side = side, move = move)


    def on_enter(self):
        self.send_packet(self.packet)


    def on_gripper(self, packet):
        yield None




class TopHolder(statemachine.State):

    def __init__(self, side, move):
        self.packet = packets.TopHolder(side = side, move = move)


    def on_enter(self):
        self.send_packet(self.packet)


    def on_top_holder(self, packet):
        yield None




class CandleKicker(statemachine.State):

    def __init__(self, side, which, position):
        self.packet = packets.CandleKicker(side = side, which = which, position = position)


    def on_enter(self):
        self.send_packet(self.packet)


    def on_candle_kicker(self, packet):
        yield None




class GiftOpener(statemachine.State):

    def __init__(self, position):
        self.packet = packets.GiftOpener(position = position)


    def on_enter(self):
        self.send_packet(self.packet)


    def on_gift_opener(self, packet):
        yield None




class Pump(statemachine.State):

    def __init__(self, action):
        self.packet = packets.Pump(action = action)


    def on_enter(self):
        self.send_packet(self.packet)


    def on_pump(self, packet):
        yield None




class FetchCandleColors(statemachine.State):

    def on_enter(self):
        if IS_HOST_DEVICE_ARM:
            self.colors = self.event_loop.colordetector.invoke("fetch")
            yield None
        else:
            self.send_packet(packets.SimulatorFetchColors())


    def on_simulator_fetch_colors(self, packet):
        self.colors = packet.colors
        yield None



########################################################

# States to port to new FSM




class Navigate(statemachine.State):

    def __init__(self, x, y, direction = DIRECTION_FORWARDS):
        statemachine.State.__init__(self)
        self.destination = position.Pose(x, y, None, True)
        self.direction = direction
        self.walk = TrajectoryWalk()
        self.walk.opponent_wait_time = 0
        self.exit_reason = TRAJECTORY_DESTINATION_REACHED


    def do_walk(self):
        seq = Sequence( Antiblocking(True),
                        self.walk,
                        Antiblocking(False)
                      )
        self.switch_to_substate(seq)


    def on_enter(self):
        path = self.event_loop.map.route(self.robot().pose, self.destination)
        if len(path) == 0:
            self.exit_reason = TRAJECTORY_DESTINATION_UNREACHABLE
            self.exit_substate()
        elif NAVIGATION_USES_MULTIPOINT:
            self.multipoint_walk(path)
        else:
            self.monopoint_walk(path)



    def multipoint_walk(self, path):
        for sub_path in path:
            first_point = sub_path[0]
            if self.direction == DIRECTION_FORWARDS:
                if not self.robot().is_looking_at(first_point):
                    self.walk.look_at(first_point.virt.x, first_point.virt.y)
            else:
                if not self.robot().is_looking_at_opposite(first_point):
                    self.walk.look_at_opposite(first_point.virt.x, first_point.virt.y)
            self.walk.follow(sub_path, None, self.direction)
        self.do_walk()


    def monopoint_walk(self, path):
        for sub_path in path:
            for point in sub_path:
                if self.direction == DIRECTION_FORWARDS:
                    if not self.robot().is_looking_at(point):
                        self.walk.look_at(point.virt.x, point.virt.y)
                else:
                    if not self.robot().is_looking_at_opposite(point):
                        self.walk.look_at_opposite(point.virt.x, point.virt.y)
                self.walk.move_to(point.virt.x, point.virt.y, self.direction)
        self.do_walk()


    def on_exit_substate(self, substate):
        self.exit_reason = self.walk.exit_reason
        self.exit_substate()




class GotoHome(statemachine.State):

    def on_enter(self):
        seq = Sequence()
        seq.add(Navigate(BLUE_START_X, 0.60))
        walk = TrajectoryWalk()
        walk.look_at(BLUE_START_X, BLUE_START_Y + 0.03)
        walk.move_to(BLUE_START_X, BLUE_START_Y + 0.03)
        seq.add(walk)
        self.switch_to_substate(seq)


    def on_exit_substate(self, substate):
        self.exit_substate()
