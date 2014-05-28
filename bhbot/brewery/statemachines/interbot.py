# encoding: utf-8

import leds
import packets
import tools

from commonstates import *
from definitions import *




class Main(Timer):

    def __init__(self):
        super().__init__(TEAMMATE_INFO_DELAY_MS)
        self.connected = False
        self.started = False
        self.teammate_collision_detection = True


    def on_interbot_connected(self, packet):
        self.connected = True
        self.log('Other robot connected')
        leds.driver.set_mode(leds.driver.MODE_HEARTBEAT_ALTERNATE)


    def on_interbot_disconnected(self, packet):
        self.connected = False
        self.log('Other robot disconnected')
        leds.driver.set_mode(leds.driver.MODE_HEARTBEAT_GREEN)


    def on_start(self, packet):
        self.started = True


    def on_timeout(self):
        if self.connected and self.started:
            packet = packets.InterbotPosition(pose = self.robot.pose)
            packet.is_moving = self.robot.destination is not None
            if packet.is_moving:
                packet.destination = self.robot.destination
            self.event_loop.send_packet(packet)


    def on_interbot_position(self, packet):
        if not self.teammate_collision_detection:
            return

        d = tools.distance(packet.pose.x, packet.pose.y, self.robot.pose.x, self.robot.pose.y)
        if d < MAIN_ROBOT_GYRATION_RADIUS + SECONDARY_ROBOT_GYRATION_RADIUS + 0.6:
            a = tools.angle_between(0.0, 0.0, packet.pose.x - self.robot.pose.x, packet.pose.y - self.robot.pose.y)
            a1 = a
            a -= self.robot.pose.angle
            cos_a = math.cos(a)
            self.log("Other robot detected m=({:0.4},{:0.4},{:0.4}) o=({:0.4},{:0.4}) d={} a1={} a={} cosa={}".format(self.robot.pose.x, self.robot.pose.y, self.robot.pose.angle, packet.pose.x, packet.pose.y, d, a1, a, cos_a))
            packet = packets.OpponentDetected(x = packet.pose.x, y = packet.pose.y, robot = OPPONENT_ROBOT_TEAMMATE)
            if cos_a > 0.8:
                packet.direction = DIRECTION_FORWARD
                self.send_packet(packet)
            elif cos_a < -0.8:
                packet.direction = DIRECTION_BACKWARDS
                self.send_packet(packet)

    def set_teammate_collision_detection(self, status):
        self.teammate_collision_detection = status
