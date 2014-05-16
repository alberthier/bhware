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
            packet.is_moving = self.robot.destination != None
            if packet.is_moving:
                packet.destination = self.robot.destination
            self.event_loop.send_packet(packet)


#    def on_interbot_position(self, packet):
#        d = tools.distance(packet.pose.x, packet.pose.y, self.robot.pose.x, self.robot.pose.y)
#        if d < MAIN_ROBOT_GYRATION_RADIUS + SECONDARY_ROBOT_GYRATION_RADIUS:
#            a = tools.angle_between(packet.pose.x, packet.pose.y, self.robot.pose.x, self.robot.pose.y)
#            packet = packets.OpponentDetected(x = packet.pose.x, y = packet.pose.y)
#            if tools.is_between(-math.pi / 8.0, 0.0, a) or tools.is_between(0.0, math.pi / 0.8, a):
#                packet.direction = DIRECTION_FORWARD
#            if tools.is_between(-math.pi, - 7.0 * math.pi / 8.0, a) or tools.is_between(math.pi, 7.0 * math.pi / 8.0, a):
#                packet.direction = DIRECTION_BACKWARDS
#            self.send_packet(packet)

