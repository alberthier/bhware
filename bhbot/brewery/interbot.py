import datetime
import logger
import leds

from definitions import *
from packets import *


class InterBotManager :
    def __init__(self, eventloop):
        self.eventloop = eventloop
        self.last_pos_sent_time = None
        leds.driver.set_mode(leds.driver.MODE_HEARTBEAT_GREEN)

    def on_connect(self):
        # logger.log('Other robot connected')
        leds.driver.set_mode(leds.driver.MODE_HEARTBEAT_ALTERNATE)

    def on_keep_alive(self, packet):
        if not self.last_pos_sent_time or (datetime.datetime.now() - self.last_pos_sent_time).seconds > \
                TEAMMATE_INFO_DELAY :
            packet_to_send = InterbotPosition()
            packet_to_send.current_pose = packet.current_pose
            packet_to_send.main_robot = IS_MAIN_ROBOT
            self.eventloop.send_packet(packet_to_send)
            self.last_pos_sent_time = datetime.datetime.now()

    def on_interbot_position(self, packet):
        if packet.main_robot :
            logger.log("Got interbot position from main")
        else :
            logger.log("Got interbot position from secondary")

    def on_interbot_hello(self, packet):
        logger.log('Received "Hello" from other robot')

    def on_disconnect(self):
        # logger.log('Other robot disconnected')
        leds.driver.set_mode(leds.driver.MODE_HEARTBEAT_GREEN)

