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
        """
        @param miliseconds: Time to wait in miliseconds
        @type miliseconds: float
        """
        self.miliseconds = miliseconds


    def on_enter(self):
        statemachine.State.__init__(self)
        self.end_time = datetime.datetime.now() + datetime.timedelta(0, 0, 0, self.miliseconds)


    def on_timer_tick(self):
        if datetime.datetime.now() > self.end_time:
            self.return_value = Timer.TIMEOUT
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




class AbstractMove(statemachine.State):

    def __init__(self):
        self.current_opponent = None


    def on_enter(self):
        self.send_packet(self.packet)


    def on_opponent_in_front(self, packet):
        self.on_opponent_detected(packet, DIRECTION_FORWARD)


    def on_opponent_in_back(self, packet):
        self.on_opponent_detected(packet, DIRECTION_BACKWARD)


    def on_opponent_detected(self, packet, opponent_direction):
        if not isinstance(self, Rotate) and self.direction == opponent_direction and self.current_opponent is None:
            logger.log("Opponent detected. direction = {}. Robot stopped".format(opponent_direction))
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
            self.exit_reason = TRAJECTORY_OPPONENT_DETECTED
            yield None
            #reason = (yield WaitForOpponentLeave(self.current_opponent, self.opponent_wait_time, self.current_goto_packet.direction)).exit_reason
            #if reason = WaitForOpponentLeave.TIMEOUT:
                #yield None
            #else:
                #return self.continue_when_opponent_left(packet.current_point_index)


    def continue_when_opponent_left(self, index):
        self.send_packet(self.packet)




class Rotate(AbstractMove):

    def __init__(self, direction, angle):
        AbstractMove.__init__(self)
        self.packet = packets.Rotate(direction = direction, angle = angle)




class MoveCurve(AbstractMove):

    def __init__(self, direction, angle, points):
        AbstractMove.__init__(self)
        self.packet = packets.Rotate(direction = direction, angle = angle, points = points)




class MoveLine(AbstractMove):

    def __init__(self, direction, angle):
        AbstractMove.__init__(self)
        self.packet = packets.Rotate(direction = direction, points = points)




class MoveArc(AbstractMove):

    def __init__(self, direction, center, radius, points):
        AbstractMove.__init__(self)
        self.packet = packets.Rotate(direction = direction, center = center, radius = radius, points = points)




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

#TODO : Créer un State générique "Movement" qui gère le goto_finished
class Move(statemachine.State):
    def __init__(self, x, y, direction = DIRECTION_FORWARD):
        statemachine.State.__init__(self)
        self.x = x
        self.y = y
        self.direction = direction

    def on_enter(self):
        packet = packets.MoveLine()
        packet.direction = self.direction
        packet.points = [ position.Pose(self.x, self.y, None, True) ]

        self.send_packet(packet)

    def on_goto_finished(self, reason):
        #TODO : à implémenter
        yield None






class WaitForOpponentLeave(statemachine.State):

    TIMEOUT       = 0
    OPPONENT_LEFT = 1

    def __init__(self, opponent, miliseconds, current_move_direction):
        statemachine.State.__init__(self)
        self.current_move_direction = current_move_direction
        self.opponent = opponent
        self.miliseconds = miliseconds
        self.goto_finished = False
        self.opponent_disappeared = False
        self.timer_expired = False
        self.exit_reason = None
        self.timer = eventloop.Timer(self.event_loop, miliseconds, self.on_timeout)


    def on_enter(self):
        self.goto_finished = False
        self.opponent_disappeared = False
        self.timer_expired = False
        self.exit_reason = None
        if self.current_move_direction == DIRECTION_FORWARD:
            direction = DIRECTION_BACKWARD
            distance = -0.100
        else:
            direction = DIRECTION_FORWARD
            distance = 0.100

        current_pose = self.robot().pose
        x = current_pose.virt.x + math.cos(current_pose.virt.angle) * distance
        y = current_pose.virt.y + math.sin(current_pose.virt.angle) * distance
        packet = packets.Goto()
        packet.movement = MOVEMENT_MOVE
        packet.direction = direction
        packet.points = [ position.Pose(x, y, None, True) ]
        self.send_packet(packet)


    def on_timeout(self):
        self.timer_expired = True
        if self.exit_reason is None:
            self.exit_reason = self.TIMEOUT
        self.try_leave()


    def on_goto_finished(self, packet):
        self.goto_finished = True
        self.try_leave()


    def on_opponent_disapeared(self, opponent, is_in_front):
        if opponent == self.opponent:
            self.exit_reason = self.OPPONENT_LEFT
            self.opponent_disappeared = True
            if not self.goto_finished:
                self.send_packet(packets.Stop())
            self.try_leave()


    def try_leave(self):
        if self.goto_finished and ((self.timer_expired or self.miliseconds == 0) or self.opponent_disappeared):
            logger.log("WaitForOpponentLeave : exit on opponent leave reason={}".format(self.exit_reason))
            yield None




class TrajectoryWalk(statemachine.State):
    """Walk a path"""

    def __init__(self):
        statemachine.State.__init__(self)
        self.jobs = deque()
        self.current_goto_packet = None
        self.opponent_wait_time = DEFAULT_OPPONENT_WAIT_MS
        self.exit_reason = TRAJECTORY_DESTINATION_REACHED
        self.current_opponent = None


    def move(self, dx, dy, direction = DIRECTION_FORWARD):
        self.jobs.append(('create_move_packet', (dx, dy, direction)))


    def move_to(self, x, y, direction = DIRECTION_FORWARD):
        self.jobs.append(('create_move_to_packet', (x, y, direction)))


    def forward(self, distance):
        self.jobs.append(('create_forward_packet', (distance,)))


    def backward(self, distance):
        self.jobs.append(('create_backward_packet', (distance,)))


    def look_at(self, x, y):
        self.jobs.append(('create_look_at_packet', (x, y)))


    def look_at_opposite(self, x, y):
        self.jobs.append(('create_look_at_opposite_packet', (x, y)))


    def rotate(self, da):
        self.jobs.append(('create_rotate_packet', (da,)))


    def rotate_to(self, angle):
        self.jobs.append(('create_rotate_to_packet', (angle,)))


    def goto(self, x, y, angle, direction = DIRECTION_FORWARD):
        self.jobs.append(('create_goto_packet', (x, y, angle, direction)))


    def goto_looking_at(self, x, y, look_at_x, look_at_y, direction = DIRECTION_FORWARD):
        self.jobs.append(('create_goto_looking_at_packet', (dx, dy, look_at_x, look_at_y, direction)))


    def goto_looking_at_opposite(self, x, y, look_at_x, look_at_y, direction = DIRECTION_FORWARD):
        self.jobs.append(('create_goto_looking_at_opposite_packet', (dx, dy, look_at_x, look_at_y, direction)))


    def follow(self, points, angle = None, direction = DIRECTION_FORWARD):
        self.jobs.append(('create_follow_packet', (points, angle, direction)))


    def follow_arc(self, direction, center, radius, *args):
        self.jobs.append(('create_follow_arc_packet', (direction, center, radius, args)))


    def wait_for(self, substate):
        self.jobs.append(substate)


    def create_move_packet(self, dx, dy, direction = DIRECTION_FORWARD):
        current_pose = self.robot().pose
        return self.create_move_to_packet(current_pose.virt.x + dx, current_pose.virt.y + dy, direction)


    def create_move_to_packet(self, x, y, direction = DIRECTION_FORWARD):
        packet = packets.MoveLine()
        packet.direction = direction
        packet.points = [ position.Pose(x, y, None, True) ]
        return packet


    def create_forward_packet(self, distance):
        current_pose = self.robot().pose
        dx = math.cos(current_pose.virt.angle) * distance
        dy = math.sin(current_pose.virt.angle) * distance
        return self.create_move_packet(dx, dy)


    def create_backward_packet(self, distance):
        current_pose = self.robot().pose
        dx = -math.cos(current_pose.virt.angle) * distance
        dy = -math.sin(current_pose.virt.angle) * distance
        return self.create_move_packet(dx, dy, DIRECTION_BACKWARD)


    def create_look_at_packet(self, x, y):
        current_pose = self.robot().pose
        dest = position.Pose(x, y, None, True)
        dx = dest.virt.x - current_pose.virt.x
        dy = dest.virt.y - current_pose.virt.y
        return self.create_rotate_to_packet(math.atan2(dy, dx))


    def create_look_at_opposite_packet(self, x, y):
        current_pose = self.robot().pose
        dest = position.Pose(x, y, None, True)
        dx = dest.virt.x - current_pose.virt.x
        dy = dest.virt.y - current_pose.virt.y
        angle = math.atan2(dy, dx) + math.pi
        if angle > math.pi:
            angle -= 2.0 * math.pi
        return self.create_rotate_to_packet(angle)


    def create_rotate_packet(self, da):
        current_pose = self.robot().pose
        return self.create_rotate_to_packet(current_pose.virt.angle + da)


    def create_rotate_to_packet(self, angle):
        angle = position.Pose(0.0, 0.0, angle, True).angle
        packet = packets.Rotate()
        packet.angle = tools.normalize_angle(angle)
        return packet


    def create_goto_packet(self, x, y, angle, direction = DIRECTION_FORWARD):
        dest = position.Pose(x, y, angle, True)
        packet = packets.MoveCurve()
        packet.direction = direction
        packet.angle = dest.angle
        packet.points = [ dest ]
        return packet


    def create_goto_looking_at_packet(self, x, y, look_at_x, look_at_y, direction = DIRECTION_FORWARD):
        current_pose = self.robot().pose
        dest = position.Pose(look_at_x, look_at_y, None, True)
        dx = dest.virt.x - current_pose.virt.x
        dy = dest.virt.y - current_pose.virt.y
        return self.create_goto_packet(x, y, math.atan2(dy, dx))


    def create_goto_looking_at_opposite_packet(self, x, y, look_at_x, look_at_y, direction = DIRECTION_FORWARD):
        current_pose = self.robot().pose
        dest = position.Pose(look_at_x, look_at_y, None, True)
        dx = dest.virt.x - current_pose.virt.x
        dy = dest.virt.y - current_pose.virt.y
        return self.create_goto_packet(x, y, math.atan2(dy, dx) + math.pi)


    def create_follow_packet(self, points, angle = None, direction = DIRECTION_FORWARD):
        dest = position.Pose(0.0, 0.0, angle, True)
        packet = packets.MoveCurve()
        packet.direction = direction
        packet.angle = dest.angle
        packet.points = points
        return packet


    def create_follow_arc_packet(self, direction, center, radius, *args):
        angles = []
        for angle in args[0]:
            logger.log(angle)
            dest = position.Pose(0.0, 0.0, angle, True)
            angles.append(dest.angle)
        packet = packets.MoveArc()
        packet.direction = direction
        packet.center = center
        packet.radius = radius
        packet.points = angles
        return packet


    def on_enter(self):
        self.process_next_job()


    def on_goto_finished(self, packet):
        if packet.reason == REASON_DESTINATION_REACHED:
            self.exit_reason = TRAJECTORY_DESTINATION_REACHED
            self.current_goto_packet = None
            self.process_next_job()
        elif packet.reason == REASON_BLOCKED_FRONT or packet.reason == REASON_BLOCKED_BACK:
            self.exit_reason = TRAJECTORY_BLOCKED
            self.exit_substate()
        elif self.current_opponent is not None:
            self.switch_to_substate(WaitForOpponentLeave(self.current_opponent, self.opponent_wait_time, self.current_goto_packet.direction))
            self.current_opponent = None



    def process_next_job(self):
        if self.current_goto_packet is None:
            if len(self.jobs) == 0:
                self.exit_reason = TRAJECTORY_DESTINATION_REACHED
                self.exit_substate()
            else:
                job = self.jobs.popleft()
                if isinstance(job, statemachine.State):
                    self.switch_to_substate(job)
                else:
                    self.current_goto_packet = getattr(self, job[0])(*job[1])
                    self.send_packet(self.current_goto_packet)
        else:
            self.send_packet(self.current_goto_packet)


    def on_exit_substate(self, substate):
        if isinstance(substate, WaitForOpponentLeave):
            if self.current_goto_packet is not None and self.opponent_wait_time != 0 and substate.exit_reason == WaitForOpponentLeave.OPPONENT_LEFT:
                self.current_opponent = None
                self.send_packet(self.current_goto_packet)
            else:
                logger.log("TrajectoryWalk failed")
                self.exit_reason = TRAJECTORY_OPPONENT_DETECTED
                self.exit_substate()
        else:
            self.process_next_job()


    def on_opponent_in_front(self, packet):
        self.on_opponent_detected(packet, DIRECTION_FORWARD)


    def on_opponent_in_back(self, packet):
        self.on_opponent_detected(packet, DIRECTION_BACKWARD)


    def on_opponent_detected(self, packet, opponent_direction):
        if self.current_goto_packet is not None:
            if self.current_goto_packet.movement != MOVEMENT_ROTATE and self.current_goto_packet.direction == opponent_direction and self.current_opponent is None:
                logger.log("Opponent detected. direction = {}. Robot stopped".format(opponent_direction))
                self.send_packet(packets.Stop())
                self.current_opponent = packet.robot





class Navigate(statemachine.State):

    def __init__(self, x, y, direction = DIRECTION_FORWARD):
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
            if self.direction == DIRECTION_FORWARD:
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
                if self.direction == DIRECTION_FORWARD:
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
