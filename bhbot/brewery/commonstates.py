# encoding: utf-8


import datetime
import functools
import math
import random

import eventloop
import logger
import packets
import position
import statemachine
import tools

from definitions import *




class StateChain(statemachine.State):

    def __init__(self, *args):
        self.states = args
        self.dbg('StateChain : {}'.format(self.states))


    def __getattr__(self, item):
        if item.startswith('on_'):
            method = functools.partial(self.handle_event, item)
            setattr(self, item, method)
            return method
        raise AttributeError()


    def on_enter(self, *args, **kwargs):
        for s in self.states :
            self.fsm.init_state(s)

        real_ret = None
        for state in self.states :
            ret = state.on_enter(*args, **kwargs)
            if ret :
                real_ret = ret
        return real_ret


    def handle_event(self, event_name, *args, **kwargs):
        self.dbg('StateChain.handle_event : {}'.format(event_name))
        for state in self.states :
            method = getattr(state, event_name, None)
            if method :
                ret = method(*args, **kwargs)
                if ret :
                    return ret
        return None




class Timer(statemachine.State):

    TIMEOUT = 0

    def __init__(self, miliseconds):
        statemachine.State.__init__(self)
        """
        @param miliseconds: Time to wait in miliseconds
        @type miliseconds: float
        """
        self.miliseconds = miliseconds
        self.end_time = None
        self.start_time = None


    def on_enter(self):
        self.restart()


    def restart(self):
        self.start_time = datetime.datetime.now()
        self.end_time = self.start_time + datetime.timedelta(0, 0, 0, self.miliseconds)


    def stop(self):
        self.end_time = None


    def on_timer_tick(self):
        if self.end_time is not None and datetime.datetime.now() > self.end_time:
            self.return_value = Timer.TIMEOUT
            self.restart()
            return self.on_timeout()

    def get_current_wait_time(self):
        return (datetime.datetime.now() - self.start_time).total_seconds()


    def on_timeout(self):
        yield None




class WaitPackets(statemachine.State):

    def __init__(self, *args):
        self.packet_classes = [ pc for pc in args ]


    def on_packet(self, packet):
        for i in range(len(self.packet_classes)):
            if isinstance(packet, self.packet_classes[i]):
                del self.packet_classes[i]
                if len(self.packet_classes) == 0:
                    yield None
                    return
                break




class SendPacketAndWait(statemachine.State):

    def __init__(self, packet_to_send, packet_class_to_wait = None):
        super().__init__()
        self.packet_to_send = packet_to_send
        if packet_class_to_wait is None:
            self.packet_class_to_wait = type(packet_to_send)
        else:
            self.packet_class_to_wait = packet_class_to_wait


    def on_enter(self):
        self.dbg("Sending packet {}".format(self.packet_to_send))
        self.send_packet(self.packet_to_send)
        self.dbg('Waiting for packet type {}'.format(self.packet_class_to_wait.__name__))


    def on_packet(self, packet):
        if isinstance(packet, self.packet_class_to_wait):
            self.dbg("Got expected packet, exiting state")
            yield None




class SendPacketsAndWaitAnswer(statemachine.State):

    def __init__(self, *packets):
        super().__init__()
        self.packets = set(packets)
        """ :type self.packets: set of packets.BasePacket"""


    def on_enter(self):
        for p in self.packets :
            self.dbg("Sending packet {}".format(p))
            self.send_packet(p)
            self.dbg('Waiting for packet type {}'.format(p.name))


    def on_packet(self, packet):
        for p in self.packets :
            if type(p) == type(packet):
                self.dbg("Got expected packet {}".format(packet.name))
                self.packets.remove(p)
                break
        if not self.packets :
            self.dbg('No more packets to wait, exiting state')
            yield None




class DefinePosition(statemachine.State):

    def __init__(self, x = None, y = None, angle = None):
        statemachine.State.__init__(self)
        self.has_x = x is not None
        self.has_y = y is not None
        px = x if self.has_x else 0.0
        py = y if self.has_y else 0.0
        pangle = angle if angle is not None else 0.0
        self.pose = position.Pose(px, py, pangle, True)


    def on_enter(self):
        if self.has_x:
            packet = packets.Resettle()
            packet.axis = AXIS_X
            packet.position = self.pose.x
            packet.angle = self.pose.angle
            yield SendPacketAndWait(packet, packets.Resettle)
            self.robot.pose.x = self.pose.x
            self.robot.pose.angle = self.pose.angle

        if self.has_y:
            packet = packets.Resettle()
            packet.axis = AXIS_Y
            packet.position = self.pose.y
            packet.angle = self.pose.angle
            yield SendPacketAndWait(packet, packets.Resettle)
            self.robot.pose.y = self.pose.y
            self.robot.pose.angle = self.pose.angle

        yield None




class AntiBlocking(statemachine.State):

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




class SpeedControl(statemachine.State):

    def __init__(self, speed = None):
        statemachine.State.__init__(self)
        self.packet = packets.PositionControlConfig()
        if speed is not None:
            self.packet.vmax_limit = speed
        else:
            self.packet.vmax_limit = ROBOT_VMAX_LIMIT


    def on_enter(self):
        self.send_packet(self.packet)


    def on_position_control_config(self, packet):
        yield None




class WaitForOpponentLeave(Timer):

    TIMEOUT       = 0
    OPPONENT_LEFT = 1

    def __init__(self, opponent, miliseconds, move_direction, retries):
        if miliseconds is None :
            miliseconds = DEFAULT_OPPONENT_WAIT_MS
        Timer.__init__(self, miliseconds)
        self.opponent = opponent
        self.move_direction = move_direction
        self.retries = retries
        if not self.retries :
            self.retries = DEFAULT_OPPONENT_DISAPPEAR_RETRIES


    def on_enter(self):
        logger.log('WaitForOpponentLeave : time={}, retries={}'.format(self.miliseconds, self.retries))
        Timer.on_enter(self)
        self.goto_finished = False
        self.opponent_disappeared = False
        self.timer_expired = False
        self.exit_reason = None
        if self.move_direction == DIRECTION_FORWARD:
            direction = DIRECTION_BACKWARDS
            distance = -0.150
        else:
            direction = DIRECTION_FORWARD
            distance = 0.150

        current_pose = self.robot.pose
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


    def on_opponent_disappeared(self, packet):
        if packet.robot == self.opponent:
            self.exit_reason = self.OPPONENT_LEFT
            self.opponent_disappeared = True
            if not self.goto_finished:
                self.send_packet(packets.Stop())
            return self.try_leave()


    def try_leave(self):
        if self.goto_finished and (self.timer_expired or self.opponent_disappeared):
            yield None
        if self.retries >= 0 :
            if self.timer_expired :
                self.retries-=1
                if self.retries < 0 :
                    logger.log('WaitForOpponentLeave : retries exceeded')
                    self.exit_reason = TRAJECTORY_BLOCKED
                    yield None
                else :
                    logger.log('WaitForOpponentLeave : retries remaining = {}'.format(self.retries))

    def on_exit(self):
        self.log("WaitForOpponentLeave : exit reason={}".format(TRAJECTORY.lookup_by_value[self.exit_reason]))




class OpponentHandlingConfig:
    def __init__(self, stop, backout, retries: int or None=None, wait_delay: float or None=None):
        self.stop = stop
        self.backout = backout
        self.retries_count = retries
        self.wait_delay = wait_delay


NO_OPPONENT_HANDLING = OpponentHandlingConfig(False, False, None, None)
#OPPONENT_HANDLING = OpponentHandlingConfig(True, True, 0, 0)
OPPONENT_HANDLING = OpponentHandlingConfig(True, False, 0, 0)




class AbstractMove(statemachine.State):

    def __init__(self, chained, opponent_handling_config: OpponentHandlingConfig):
        self.current_opponent = None
        self.chained = chained
        self.opponent_handling_config = opponent_handling_config


    def on_enter(self):
        if self.chained is not None and self.chained.exit_reason != REASON_DESTINATION_REACHED:
            self.exit_reason = self.chained.exit_reason
            yield None
        elif self.opponent_handling_config.stop and self.packet.direction in [ self.robot.main_opponent_direction, self.robot.secondary_opponent_direction ]:
            if self.robot.main_opponent_direction is not None:
                self.current_opponent = OPPONENT_ROBOT_MAIN
            else:
                self.current_opponent = OPPONENT_ROBOT_SECONDARY
            yield from self.handle_opponent_detected(None)
        else:
            self.send_packet(self.packet)


    def on_opponent_detected(self, packet):
        if self.opponent_handling_config.stop and self.packet.direction == packet.direction and self.current_opponent is None:
            self.log("Opponent detected. direction = {}. Stop robot".format(packet.direction))
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
            if hasattr(self.packet, "points"):
                index = packet.current_point_index
            else:
                index = None
            yield from self.handle_opponent_detected(index)
        else:
            self.exit_reason = TRAJECTORY_OPPONENT_DETECTED
            yield None


    def handle_opponent_detected(self, point_index):
        if self.opponent_handling_config.backout:
            leave_state = yield WaitForOpponentLeave(self.current_opponent, self.opponent_handling_config.wait_delay,
                                                     self.packet.direction, self.opponent_handling_config.retries_count)
            reason = leave_state.exit_reason
            self.current_opponent = None
            if reason in (WaitForOpponentLeave.TIMEOUT, TRAJECTORY_BLOCKED) :
                self.exit_reason = TRAJECTORY_OPPONENT_DETECTED
                yield None
            else:
                if point_index is not None:
                    self.packet.points = self.packet.points[point_index:]
                self.send_packet(self.packet)
        else:
            self.log("Opponent detected, cancelling log")
            self.exit_reason = TRAJECTORY_OPPONENT_DETECTED
            yield None




class RotateTo(AbstractMove):

    def __init__(self, angle, direction = DIRECTION_FORWARD, chained = None, virtual = True):
        super().__init__(chained, NO_OPPONENT_HANDLING)
        pose = position.Pose(0.0, 0.0, angle, virtual)
        self.packet = packets.Rotate(direction = direction, angle = pose.angle)




class LookAt(AbstractMove):

    def __init__(self, x, y, direction = DIRECTION_FORWARD, chained = None, virtual = True):
        super().__init__(chained, NO_OPPONENT_HANDLING)
        self.pose = position.Pose(x, y, None, virtual)
        self.direction = direction


    def on_enter(self):
        current_pose = self.robot.pose
        dx = self.pose.x - current_pose.x
        dy = self.pose.y - current_pose.y
        angle = math.atan2(dy, dx)
        self.packet = packets.Rotate(direction = self.direction, angle = angle)
        return AbstractMove.on_enter(self)




class LookAtOpposite(AbstractMove):

    def __init__(self, x, y, direction = DIRECTION_FORWARD, chained = None, virtual = True):
        super().__init__(chained, NO_OPPONENT_HANDLING)
        self.pose = position.Pose(x, y, None, virtual)
        self.direction = direction


    def on_enter(self):
        current_pose = self.robot.pose
        dx = self.pose.x - current_pose.x
        dy = self.pose.y - current_pose.y
        angle = math.atan2(dy, dx) + math.pi
        self.packet = packets.Rotate(direction = self.direction, angle = angle)
        return AbstractMove.on_enter(self)




class MoveCurve(AbstractMove):

    def __init__(self, angle, points, direction = DIRECTION_FORWARD, chained = None, virtual = True, opponent_handling_config = OPPONENT_HANDLING):
        super().__init__(chained, opponent_handling_config)
        apose = position.Pose(0.0, 0.0, angle, virtual)
        poses = []
        for pt in points:
            if isinstance(pt, tuple):
                poses.append(position.Pose(pt[0], pt[1], None, virtual))
            else:
                poses.append(pt)
        self.packet = packets.MoveCurve(direction = direction, angle = apose.angle, points = poses)




class MoveCurveTo(MoveCurve):

    def __init__(self, angle, pose, direction = DIRECTION_FORWARD, chained = None, virtual = True, opponent_handling_config = OPPONENT_HANDLING):
        super().__init__(angle, [pose], direction, chained, virtual, opponent_handling)




class MoveLine(AbstractMove):

    def __init__(self, points, direction = DIRECTION_FORWARD, chained = None, virtual = True, opponent_handling_config = OPPONENT_HANDLING):
        super().__init__(chained, opponent_handling_config)
        poses = []
        for pt in points:
            if type(pt) == tuple:
                poses.append(position.Pose(pt[0], pt[1], None, virtual))
            else:
                poses.append(pt)
        self.packet = packets.MoveLine(direction = direction, points = poses)




class MoveLineTo(MoveLine):

    def __init__(self, x, y, direction = DIRECTION_FORWARD, chained = None, virtual = True, opponent_handling_config = OPPONENT_HANDLING):
        super().__init__([position.Pose(x, y, None, virtual)], direction, chained, virtual, opponent_handling_config)




class MoveLineRelative(statemachine.State):

    def __init__(self, distance, direction = DIRECTION_FORWARD, chained = None, opponent_handling_config = OPPONENT_HANDLING):
        self.opponent_handling_config = opponent_handling_config
        self.distance = distance * direction
        self.direction = direction
        self.chained = chained


    def on_enter(self):
        current_pose = self.robot.pose
        x = current_pose.virt.x + math.cos(current_pose.virt.angle) * self.distance
        y = current_pose.virt.y + math.sin(current_pose.virt.angle) * self.distance
        move = yield MoveLineTo(x, y, self.direction, self.chained, self.opponent_handling_config)
        self.exit_reason = move.exit_reason
        yield None




class RotateRelative(statemachine.State):

    def __init__(self, relative_angle, chained = None):
        self.relative_angle = relative_angle
        self.chained = chained


    def on_enter(self):
        current_pose = self.robot.pose
        move = yield RotateTo(current_pose.angle + self.relative_angle, self.chained)
        self.exit_reason = move.exit_reason
        yield None




class MoveArc(AbstractMove):

    def __init__(self, center_x, center_y, radius, points, direction = DIRECTION_FORWARD, chained = None, virtual = True, opponent_handling_config = OPPONENT_HANDLING):
        AbstractMove.__init__(self, chained, opponent_handling_config)
        cpose = position.Pose(center_x, center_y, None, virtual)
        angles = []
        for a in points:
            apose = position.Pose(0.0, 0.0, a, virtual)
            angles.append(apose.angle)
        self.packet = packets.MoveArc(direction = direction, center = cpose, radius = radius, points = angles)




class FollowPath(statemachine.State):

    def __init__(self, path, direction = DIRECTION_FORWARD, chained = None):
        super().__init__()
        self.path = path
        self.direction = direction
        self.exit_reason = None
        self.chained = chained


    def on_enter(self):
        move = self.chained
        dest = self.path[-1]
        self.robot.destination = position.Pose(dest.x, dest.y, 0.0)
        for pose in self.path:
            if self.direction == DIRECTION_FORWARD:
                if not self.robot.is_looking_at(pose):
                    move = yield LookAt(pose.virt.x, pose.virt.y, DIRECTION_FORWARD, move)
            else:
                if not self.robot.is_looking_at_opposite(pose):
                    move = yield LookAtOpposite(pose.virt.x, pose.virt.y, DIRECTION_FORWARD, move)
            move = yield MoveLine([pose], self.direction, move)
        self.robot.destination = None
        self.exit_reason = move.exit_reason
        yield None




class StopAll(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.StopAll())


    def on_stop_all(self, packet):
        yield None




class Navigate(statemachine.State):

    def __init__(self, x, y, direction = DIRECTION_FORWARD):
        statemachine.State.__init__(self)
        self.destination = position.Pose(x, y, None, True)
        self.direction = direction
        self.exit_reason = TRAJECTORY_DESTINATION_REACHED


    def create_path(self):
        (cost, path) = self.event_loop.map.route(self.robot.pose, self.destination)
        return path


    def on_enter(self):
        path = self.create_path()
        self.log(str(path))
        if len(path) == 0:
            self.exit_reason = TRAJECTORY_DESTINATION_UNREACHABLE
            yield None
            return
        move = yield FollowPath(path, self.direction)
        self.exit_reason = move.exit_reason
        yield None




class GotoHome(Navigate):

    def __init__(self):
        Navigate.__init__(self, YELLOW_START_X, YELLOW_START_Y)




class Trigger(statemachine.State):

    TYPE               = 0
    ID                 = 1
    SERVO_COMMAND      = 2
    SERVO_VALUE        = 3
    SERVO_TIMEOUT      = 4
    RELAY_ACTION       = 2
    RELAY_TOGGLE_COUNT = 3
    PWM_VALUE          = 2

    def __init__(self, *args):
        """
            args can be 3 or 4 arguments: actuator_type, id, args... for a actuator control or
            a list of tuples (actuator_type, id, args...) for multiple motor control
            ex:
                Trigger(ACTUATOR_TYPE_SERVO_AX, 1, 154, 1000)
                Trigger((ACTUATOR_TYPE_SERVO_AX, 1, 154, 1000), (ACTUATOR_TYPE_RELAY, 2, ACTION_ON))
        """
        if len(args) > 0:
            if type(args[0]) == tuple:
                self.commands = list(args)
            elif len(args) == 3 or len(args) == 4:
                self.commands = [ args ]
            else:
                raise TypeError("Invalid arguments")
        else:
            raise TypeError("Invalid arguments")

        self.exit_reason = None
        self.statuses = {}


    def on_enter(self):
        for cmd in self.commands:
            actuator_type = cmd[self.TYPE]
            if actuator_type in [ ACTUATOR_TYPE_SERVO_AX, ACTUATOR_TYPE_SERVO_RX ]:
                self.send_packet(packets.ServoControl(*cmd))
            elif actuator_type == ACTUATOR_TYPE_RELAY:
                self.send_packet(packets.RelayToggle(id = cmd[self.ID], action = cmd[self.RELAY_ACTION], toggle_count = cmd[self.RELAY_TOGGLE_COUNT]))
            elif actuator_type == ACTUATOR_TYPE_PWM:
                self.send_packet(packets.PwmControl(id = cmd[self.ID], value = cmd[self.PWM_VALUE]))
            else:
                self.log("Unknown actuator type in command: {}".format(cmd))


    def on_servo_control(self, packet):
        if packet.status != SERVO_STATUS_SUCCESS:
            self.log("Servo #{} timed out".format(packet.id))
        self.exit_reason = packet.status
        self.statuses[packet.id] = packet.status
        yield from self.cleanup(packet.type, packet.id, packet.command)


    def on_relay_toggle(self, packet):
        self.exit_reason = SERVO_STATUS_SUCCESS
        self.statuses[packet.id] = SERVO_STATUS_SUCCESS
        yield from self.cleanup(ACTUATOR_TYPE_RELAY, packet.id)


    def on_pwm_control(self, packet):
        self.exit_reason = SERVO_STATUS_SUCCESS
        self.statuses[packet.id] = SERVO_STATUS_SUCCESS
        yield from self.cleanup(ACTUATOR_TYPE_PWM, packet.id)


    def cleanup(self, actuator_type, id, subcommand = None):
        for i, cmd in enumerate(self.commands):
            if cmd[self.TYPE] == actuator_type and cmd[self.ID] == id:
                if subcommand is None or subcommand == cmd[self.SERVO_COMMAND]:
                    del self.commands[i]
                break
        if len(self.commands) == 0:
            yield None


##################################################
# GOAL MANAGEMENT
##################################################


class ExecuteGoals(statemachine.State):

    def on_enter(self):
        gm = self.robot.goal_manager

        navigation_failures = 0

        while True:

            if navigation_failures < 10:
                logger.log("Choosing the best goal")
                goal = gm.get_next_goal()
            else:
                logger.log("Escaping to anywhere !!")
                yield EscapeToAnywhere()
                gm.whitelist_all()
                navigation_failures = 0
                continue

            if goal:
                logger.log('Next goal is {}'.format(goal.identifier))

                goal.doing()

                current_navigation_succeeded = True
                if goal.navigate :
                    logger.log('Navigating to goal')
                    move = yield Navigate(goal.x, goal.y, goal.direction)
                    logger.log('End of navigation : {}'.format(TRAJECTORY.lookup_by_value[move.exit_reason]))

                    current_navigation_succeeded = move.exit_reason == TRAJECTORY_DESTINATION_REACHED
                    if current_navigation_succeeded:
                        navigation_failures = 0
                        logger.log('Navigation was successful')
                    else:
                        navigation_failures += 1
                        logger.log('Cannot navigate to goal -> picking another')
                        goal.is_blacklisted = True
                        goal.available()
                    if move.exit_reason == TRAJECTORY_BLOCKED:
                        direction = DIRECTION_BACKWARDS if goal.direction == DIRECTION_FORWARD else DIRECTION_FORWARD
                        dist = ROBOT_GYRATION_RADIUS - ROBOT_CENTER_X + 0.02
                        yield MoveLineRelative(dist, direction)
                        self.event_loop.map.robot_blocked(goal.direction)

                if current_navigation_succeeded:
                    gm.whitelist_all()
                    state = goal.get_state()
                    state.goal = goal
                    state.exit_reason = GOAL_FAILED

                    yield state

                    logger.log('State exit reason : {}'.format(GOAL_STATUS.lookup_by_value[state.exit_reason]))

                    if state.exit_reason == GOAL_DONE :
                        goal.done()
                    else :
                        goal.increment_trials()
                        goal.available()

            else:
                navigation_failures += 1
                if not gm.has_blacklisted_goals():
                    break
            self.log('Goal statuses: {}'.format({ g.identifier : g.is_available() for g in gm.goals}))

        self.log('No more goals available')

        yield None




class EscapeToAnywhere(statemachine.State):

    def on_enter(self):
        exit_reason = TRAJECTORY_OPPONENT_DETECTED


    def on_timer_tick(self):
        # We cannot use a simple while here. We have to give control back to the eventloop at each try
        # otherwise we will block all communications / statemachines
        x = random.randrange(300, 1700) / 1000.0
        y = random.randrange(600, 2700) / 1000.0
        move = yield Navigate(x, y)
        exit_reason = move.exit_reason
        if exit_reason == TRAJECTORY_BLOCKED:
            move = yield MoveLineRelative(0.1, DIRECTION_BACKWARDS)
            exit_reason = move.exit_reason
        if exit_reason == TRAJECTORY_DESTINATION_REACHED:
            yield None


