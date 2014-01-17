# encoding: utf-8


import datetime
import functools
import math

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


    def on_enter(self):
        self.end_time = datetime.datetime.now() + datetime.timedelta(0, 0, 0, self.miliseconds)


    def on_timer_tick(self):
        if datetime.datetime.now() > self.end_time:
            self.return_value = Timer.TIMEOUT
            return self.on_timeout()


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

    def __init__(self, packet_to_send, packet_class_to_wait):
        super().__init__()
        self.packet_to_send = packet_to_send
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
            self.packet.vmax_limit = 88.0


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
        if self.move_direction == DIRECTION_FORWARDS:
            direction = DIRECTION_BACKWARDS
            distance = -0.150
        else:
            direction = DIRECTION_FORWARDS
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
    def __init__(self, backout, retries: int or None=None, wait_delay: float or None=None):
        self.backout = backout
        self.retries_count = retries
        self.wait_delay = wait_delay


NO_OPPONENT_HANDLING = OpponentHandlingConfig(False, 0, 0)
OPPONENT_HANDLING = OpponentHandlingConfig(True, None, None)




class AbstractMove(statemachine.State):

    def __init__(self, chained, opponent_leave_config: OpponentHandlingConfig):
        self.current_opponent = None
        self.chained = chained
        self.opponent_leave_config = opponent_leave_config


    def on_enter(self):
        if self.chained is not None and self.chained.exit_reason != REASON_DESTINATION_REACHED:
            self.exit_reason = self.chained.exit_reason
            yield None
        elif self.event_loop.opponent_detector.main_opponent.detected or self.event_loop.opponent_detector.secondary_opponent.detected:
            config = self.opponent_leave_config
            leave_state = yield WaitForOpponentLeave(self.current_opponent, config.wait_delay,
                                                     self.packet.direction, config.retries_count)
            reason = leave_state.exit_reason
            self.current_opponent = None
            if reason in (WaitForOpponentLeave.TIMEOUT, TRAJECTORY_BLOCKED) :
                self.exit_reason = TRAJECTORY_OPPONENT_DETECTED
                yield None
            else:
                self.send_packet(self.packet)
        else:
            self.send_packet(self.packet)


    def on_opponent_detected(self, packet):
        if not isinstance(self, Rotate) and self.packet.direction == packet.direction and self.current_opponent is None:
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
            config = self.opponent_leave_config
            if config.backout:
                leave_state = yield WaitForOpponentLeave(self.current_opponent, config.wait_delay,
                                                     self.packet.direction, config.retries_count)
                reason = leave_state.exit_reason
                self.current_opponent = None
                if reason in (WaitForOpponentLeave.TIMEOUT, TRAJECTORY_BLOCKED) :
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

    def __init__(self, angle, direction = DIRECTION_FORWARDS, chained = None):
        AbstractMove.__init__(self, chained, NO_OPPONENT_HANDLING)
        pose = position.Pose(0.0, 0.0, angle, True)
        self.packet = packets.Rotate(direction = direction, angle = pose.angle)




class LookAt(AbstractMove):

    def __init__(self, x, y, direction = DIRECTION_FORWARDS, chained = None):
        AbstractMove.__init__(self, chained, NO_OPPONENT_HANDLING)
        self.pose = position.Pose(x, y, None, True)
        self.direction = direction


    def on_enter(self):
        current_pose = self.robot.pose
        dx = self.pose.x - current_pose.x
        dy = self.pose.y - current_pose.y
        angle = math.atan2(dy, dx)
        self.packet = packets.Rotate(direction = self.direction, angle = angle)
        return AbstractMove.on_enter(self)




class LookAtOpposite(AbstractMove):

    def __init__(self, x, y, direction = DIRECTION_FORWARDS, chained = None):
        AbstractMove.__init__(self, chained, NO_OPPONENT_HANDLING)
        self.pose = position.Pose(x, y, None, True)
        self.direction = direction


    def on_enter(self):
        current_pose = self.robot.pose
        dx = self.pose.x - current_pose.x
        dy = self.pose.y - current_pose.y
        angle = math.atan2(dy, dx) + math.pi
        self.packet = packets.Rotate(direction = self.direction, angle = angle)
        return AbstractMove.on_enter(self)




class MoveCurve(AbstractMove):

    def __init__(self, angle, points, direction = DIRECTION_FORWARDS, chained = None, opponent_handling =  OPPONENT_HANDLING):
        AbstractMove.__init__(self, chained = chained, opponent_leave_config = opponent_handling)
        apose = position.Pose(0.0, 0.0, angle, True)
        poses = []
        for pt in points:
            if isinstance(pt, tuple):
                poses.append(position.Pose(pt[0], pt[1], None, True))
            else:
                poses.append(pt)
        self.packet = packets.MoveCurve(direction = direction, angle = apose.angle, points = poses)




class MoveCurveTo(MoveCurve):

    def __init__(self, angle, pose, direction = DIRECTION_FORWARDS, chained = None):
        MoveCurve.__init__(self, angle, [pose], direction, chained)




class MoveLine(AbstractMove):

    def __init__(self, points, direction = DIRECTION_FORWARDS, chained = None, opponent_handling = OPPONENT_HANDLING):
        AbstractMove.__init__(self, chained, opponent_handling)
        poses = []
        for pt in points:
            if type(pt) == tuple:
                poses.append(position.Pose(pt[0], pt[1], None, True))
            else:
                poses.append(pt)
        self.packet = packets.MoveLine(direction = direction, points = poses)




class MoveLineTo(MoveLine):

    def __init__(self, x, y, direction = DIRECTION_FORWARDS, chained = None, opponent_handling = OPPONENT_HANDLING):
        MoveLine.__init__(self, [position.Pose(x, y, None, True)], direction, chained, opponent_handling)




class MoveRelative(statemachine.State):

    def __init__(self, distance, direction = DIRECTION_FORWARDS, chained = None):
        self.distance = distance
        self.direction = direction
        self.chained = chained


    def on_enter(self):
        current_pose = self.robot.pose
        x = current_pose.virt.x + math.cos(current_pose.virt.angle) * self.distance
        y = current_pose.virt.y + math.sin(current_pose.virt.angle) * self.distance
        yield MoveLineTo(x, y, self.direction, self.chained)
        yield None




class RotateRelative(AbstractMove):

    def __init__(self, relative_angle, chained = None):

        AbstractMove.__init__(self, chained, NO_OPPONENT_HANDLING)

        self.relative_angle = relative_angle
        self.chained = chained

    def on_enter(self):
        current_pose = self.robot.pose
        yield Rotate(current_pose.angle+self.relative_angle, chained=self.chained)
        yield None




class MoveArc(AbstractMove):

    def __init__(self, center_x, center_y, radius, points, direction = DIRECTION_FORWARDS, chained = None):
        AbstractMove.__init__(self, chained, OPPONENT_HANDLING)
        cpose = position.Pose(center_x, center_y, None, True)
        angles = []
        for a in points:
            apose = position.Pose(0.0, 0.0, a, True)
            angles.append(apose.angle)
        self.packet = packets.MoveArc(direction = direction, center = cpose, radius = radius, points = angles)




class FollowPath(statemachine.State):

    def __init__(self, path, direction = DIRECTION_FORWARDS):
        super().__init__()
        self.path = path
        self.direction = direction
        self.exit_reason = None


    def on_enter(self):
        move = None
        for pose in self.path:
            if self.direction == DIRECTION_FORWARDS:
                if not self.robot.is_looking_at(pose):
                    move = yield LookAt(pose.virt.x, pose.virt.y, DIRECTION_FORWARDS, move)
            else:
                if not self.robot.is_looking_at_opposite(pose):
                    move = yield LookAtOpposite(pose.virt.x, pose.virt.y, DIRECTION_FORWARDS, move)
            move = yield MoveLine([pose], self.direction, move)
        self.exit_reason = move.exit_reason
        yield None




class StopAll(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.StopAll())


    def on_stop_all(self, packet):
        yield None




class Navigate(statemachine.State):

    def __init__(self, x, y, direction = DIRECTION_FORWARDS):
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




class CalibratePosition(statemachine.State):

    def __init__(self, x = YELLOW_START_X, test = False):
        self.x = x
        self.test = test


    def on_enter(self):
        if IS_HOST_DEVICE_PC or self.test:
            yield DefinePosition(self.x, ROBOT_CENTER_X, math.pi / 2.0)
        else:
            estimated_start_y = 1.0
            yield DefinePosition(ROBOT_CENTER_X, estimated_start_y, 0.0)
            yield MoveLineTo(self.x, estimated_start_y)
            yield Rotate(math.pi / 2.0)
            yield SpeedControl(0.2)
            yield MoveLineTo(self.x, 0.0, DIRECTION_BACKWARDS)
            yield SpeedControl()
            yield DefinePosition(None, ROBOT_CENTER_X, math.pi / 2.0)
        yield None




class ServoControl(statemachine.State):

    def __init__(self, *args):
        """
            args can be 3 arguments: type, id, angle for a single servo control or
            a list of tuples (type, id, angle) for multiple servos control
        """
        if len(args) > 0:
            if type(args[0]) == tuple:
                self.servo_commands = list(args)
            elif len(args) == 3:
                self.servo_commands = [ args ]


    def on_enter(self):
        for type, id, angle in self.servo_commands:
            self.send_packet(packets.ServoControl(type, id, angle))


    def on_servo_control(self, packet):
        i = 0
        for type, id, angle in self.servo_commands:
            if packet.type == type and packet.id == id:
                break
            i += 1
        if i < len(self.servo_commands):
            del self.servo_commands[i]
        if len(self.servo_commands) == 0:
            yield None




class ElectromagnetControl(statemachine.State):

    def __init__(self, *args):
        """
            args can be 2 arguments: id, action for a single electromagnet control or
            a list of tuples (id, action) for multiple electromagnet control
        """
        if len(args) > 0:
            if type(args[0]) == tuple:
                self.magnet_commands = list(args)
            elif len(args) == 3:
                self.magnet_commands = [ args ]


    def on_enter(self):
        for id, action in self.magnet_commands:
            self.send_packet(packets.ElectromagnetControl(id, action))


    def on_electromagnet_control(self, packet):
        i = 0
        for id, action in self.magnet_commands:
            if packet.id == id:
                break
            i += 1
        if i < len(self.magnet_commands):
            del self.magnet_commands[i]
        if len(self.magnet_commands) == 0:
            yield None




class SuctionPump(statemachine.State):

    def __init__(self, action):
        self.action = action


    def on_enter(self):
        self.send_packet(packets.SuctionPump(action))


    def on_suction_pump(self, packet):
        yield None


##################################################
# GOAL MANAGEMENT
##################################################


class FindNextGoal(statemachine.State):
    def on_enter(self):
        gm = self.robot.goal_manager

        navigation_failure = False

        while True:

            if not navigation_failure :
                goal = gm.get_best_goal(gm.harvesting_goals)
            else :
                goal = gm.get_least_recent_tried_goal()

            if goal :
                logger.log('Next goal is {}'.format(goal.identifier))

                gm.goal_doing(goal)

                if goal.navigate :
                    logger.log('Navigating to goal')
                    move = yield Navigate(goal.x, goal.y, goal.direction)
                    logger.log('End of navigation : {}'.format(TRAJECTORY.lookup_by_value[move.exit_reason]))

                    if  move.exit_reason != TRAJECTORY_DESTINATION_REACHED :
                        logger.log('Cannot navigate to goal -> picking another')
                        goal.increment_trials()
                        gm.goal_available(goal)
                        navigation_failure = True
                        continue
                    # else :
                    #     navigation_failure = False

                    logger.log('Navigation was successful')

                state = goal.get_state()

                yield state

                logger.log('State exit reason : {}'.format(GOAL_STATUS.lookup_by_value[state.exit_reason]))

                if state.exit_reason == GOAL_DONE :
                    gm.goal_done(goal)
                else :
                    goal.increment_trials()
                    gm.goal_available(goal)
            else :
                break

        self.log('No more goals available')
        self.log(str({ g.identifier : g.is_available() for g in gm.all_goals}))

        yield None
