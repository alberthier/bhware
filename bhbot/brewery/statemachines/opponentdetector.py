# encoding: utf-8

import math

import commonstates
import packets

from definitions import *




class Main(commonstates.Timer):

    IN_FRONT_ANGLES = [16, 17, 0, 1, 2]
    IN_BACK_ANGLES = [7, 8, 9, 10, 11]

    def __init__(self):
        super().__init__(OPPONENT_DETECTION_DISAPPEARING_MS)
        self.detections = []


    def on_enter(self):
        # Do not call super().on_enter() as we don't want to start the timer immediately
        self.opponent_direction = None
        self.x = None
        self.y = None
        self.enabled = True
        if not hasattr(self.fsm, "offset_deg"):
            self.fsm.offset_deg = 0.0


    def on_turret_detect(self, packet):
        if not self.enabled:
            return
        if packet.robot != self.fsm.opponent_type:
            return

        self.detections.append(packet.distance)
        if len(self.detections) < 2:
            return

        # if a near detection is present in the 6 last detections, the opponent is near us
        if OPPONENT_DISTANCE_NEAR in self.detections:
            distance = TURRET_SHORT_DISTANCE_DETECTION_RANGE
        else:
            distance = TURRET_LONG_DISTANCE_DETECTION_RANGE
        self.detections = []

        angle = (18 - packet.angle) % 18

        robot_pose = self.robot.pose
        real_angle = (angle * 20.0 / 180.0) * math.pi
        real_angle += robot_pose.angle + math.radians(self.fsm.offset_deg)
        self.x = robot_pose.x + distance * math.cos(real_angle)
        self.y = robot_pose.y + distance * math.sin(real_angle)

        previous_direction = self.opponent_direction
        if packet.distance == OPPONENT_DISTANCE_NEAR:
            if angle in self.IN_FRONT_ANGLES:
                self.opponent_direction = DIRECTION_FORWARD
            elif angle in self.IN_BACK_ANGLES:
                self.opponent_direction = DIRECTION_BACKWARDS
            else:
                self.opponent_direction = None
        else:
            self.opponent_direction = None

        self.send_packet(packets.OpponentPosition(robot = self.fsm.opponent_type, distance = packet.distance, x = self.x, y = self.y))
        self.restart()

        if self.opponent_direction is not None:
            if self.opponent_direction != previous_direction:
                self.log("{} opponent detected at ({:.2f}, {:.2f})".format(self.opponent_name(), self.x, self.y))
                self.set_detected(True)
                self.send_packet(packets.OpponentDetected(robot = self.fsm.opponent_type, direction = self.opponent_direction, x = self.x, y = self.y))
        elif self.opponent_direction is None and previous_direction is not None:
            self.opponent_disappeared()


    def on_timeout(self):
        self.opponent_disappeared()


    def opponent_disappeared(self):
        self.log("{} opponent disappeared".format(self.opponent_name()))
        self.set_detected(False)
        self.stop()
        self.detections = []
        self.send_packet(packets.OpponentPosition(robot = self.fsm.opponent_type, x = None, y = None))
        if self.opponent_direction is not None:
            self.send_packet(packets.OpponentDisappeared(robot = self.fsm.opponent_type, direction = self.opponent_direction))
        self.opponent_direction = None


    def opponent_name(self):
        if self.fsm.opponent_type == OPPONENT_ROBOT_MAIN:
            return "Main"
        else:
            return "Secondary"


    def set_detected(self, detected):
        if self.fsm.opponent_type == OPPONENT_ROBOT_MAIN:
            self.robot.main_opponent_detected = detected
        else:
            self.robot.secondary_opponent_detected = detected

