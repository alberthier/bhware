# encoding: utf-8

import datetime

import eventloop
import leds
import logger
import packets
from definitions import *




class InterBotManager :

    def __init__(self, event_loop):
        self.event_loop = event_loop
        leds.driver.set_mode(leds.driver.MODE_HEARTBEAT_GREEN)
        eventloop.Timer(self.event_loop, TEAMMATE_INFO_DELAY_MS, self.send_periodic_info)


    def on_connect(self):
        logger.log('Other robot connected')
        leds.driver.set_mode(leds.driver.MODE_HEARTBEAT_ALTERNATE)


    def on_disconnect(self):
        logger.log('Other robot disconnected')
        leds.driver.set_mode(leds.driver.MODE_HEARTBEAT_GREEN)


    def send_periodic_info(self):
        packet = InterbotPosition(pose = self.event_loop.robot.pose)
        self.event_loop.send_packet(packet)
