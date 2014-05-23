# encoding: utf-8

import collections
import math

import commonstates
import packets

from definitions import *




class Main(commonstates.Timer):

    MAIN_IN_FRONT_IDS = [  2,  3,  4,  5,  6,  7 ]
    MAIN_IN_BACK_IDS  = [ 11, 12, 13, 14, 15, 16 ]
    MAIN_ANGLES = [ -90, -70, -50, -30, -10, 10, 30, 50, 70, 90, 110, 130, 150, 170, -170, -150, -130, -110 ]

    SECONDARY_IN_FRONT_IDS = [  2,  3,  4,  5,  6,  7 ]
    SECONDARY_IN_BACK_IDS  = [ 11, 12, 13, 14, 15, 16 ]
    SECONDARY_ANGLES = [ -90, -70, -50, -30, -10, 10, 30, 50, 70, 90, 110, 130, 150, 170, -170, -150, -130, -110 ]

    PACKET_BUFFER_SIZE = 6

    def __init__(self):
        super().__init__(OPPONENT_DETECTION_DISAPPEARING_MS)
        self.detections = collections.deque()
        self.in_front_ids = self.MAIN_IN_FRONT_IDS if IS_MAIN_ROBOT else self.SECONDARY_IN_FRONT_IDS
        self.in_back_ids  = self.MAIN_IN_BACK_IDS  if IS_MAIN_ROBOT else self.SECONDARY_IN_BACK_IDS
        self.angles       = self.MAIN_ANGLES       if IS_MAIN_ROBOT else self.SECONDARY_ANGLES


    def on_enter(self):
        # Do not call super().on_enter() as we don't want to start the timer immediately
        self.opponent_direction = None
        self.x = None
        self.y = None
        if not hasattr(self.fsm, "enabled"):
            self.fsm.enabled = True


    def on_turret_detect(self, packet):
        if not self.fsm.enabled:
            return
        if packet.robot != self.fsm.opponent_type:
            return

        self.detections.append(packet.distance)
        if len(self.detections) < self.PACKET_BUFFER_SIZE:
            return
        elif len(self.detections) > self.PACKET_BUFFER_SIZE:
            self.detections.popleft()

        # if a near detection is present in the 6 last detections, the opponent is near us
        if OPPONENT_DISTANCE_NEAR in self.detections:
            opponent_distance = OPPONENT_DISTANCE_NEAR
        else:
            opponent_distance = OPPONENT_DISTANCE_FAR

        if opponent_distance == OPPONENT_DISTANCE_NEAR:
            distance = TURRET_SHORT_DISTANCE_DETECTION_RANGE
        else:
            distance = TURRET_LONG_DISTANCE_DETECTION_RANGE

        robot_pose = self.robot.pose
        real_angle = math.radians(self.angles[packet.angle]) + robot_pose.angle
        self.x = robot_pose.x + distance * math.cos(real_angle)
        self.y = robot_pose.y + distance * math.sin(real_angle)

        previous_direction = self.opponent_direction
        if opponent_distance == OPPONENT_DISTANCE_NEAR:
            if packet.angle in self.in_front_ids:
                self.opponent_direction = DIRECTION_FORWARD
            elif packet.angle in self.in_back_ids:
                self.opponent_direction = DIRECTION_BACKWARDS
            else:
                self.opponent_direction = None
        else:
            self.opponent_direction = None

        self.send_packet(packets.OpponentPosition(robot = self.fsm.opponent_type, distance = opponent_distance, x = self.x, y = self.y))
        self.restart()

        if self.opponent_direction is not None:
            if self.opponent_direction != previous_direction:
                self.log("{} opponent detected at ({:.2f}, {:.2f})".format(self.opponent_name(), self.x, self.y))
                self.set_detected(self.opponent_direction)
                self.send_packet(packets.OpponentDetected(robot = self.fsm.opponent_type, direction = self.opponent_direction, x = self.x, y = self.y))
        elif self.opponent_direction is None and previous_direction is not None:
            self.opponent_disappeared()


    def on_timeout(self):
        self.opponent_disappeared()


    def opponent_disappeared(self):
        self.log("{} opponent disappeared".format(self.opponent_name()))
        self.set_detected(None)
        self.stop()
        self.detections = collections.deque()
        self.send_packet(packets.OpponentPosition(robot = self.fsm.opponent_type, x = None, y = None))
        if self.opponent_direction is not None:
            self.send_packet(packets.OpponentDisappeared(robot = self.fsm.opponent_type, direction = self.opponent_direction))
        self.opponent_direction = None


    def opponent_name(self):
        if self.fsm.opponent_type == OPPONENT_ROBOT_MAIN:
            return "Main"
        else:
            return "Secondary"


    def set_detected(self, direction):
        if self.fsm.opponent_type == OPPONENT_ROBOT_MAIN:
            self.robot.main_opponent_direction = direction
        else:
            self.robot.secondary_opponent_direction = direction

