import datetime
import logger
import leds

from definitions import *
from packets import *


class InterBotManager :
    def __init__(self, eventloop):
        self.eventloop = eventloop
        self.last_pos_sent_time = None
        self.setup = False
        leds.driver.set_mode(leds.driver.MODE_HEARTBEAT_GREEN)

    def goal_state_change(self, goal):
        logger.log('A goal status changed, notifying my buddy : {} -> {}'.format(goal.identifier, goal.status))
        packet = InterbotGoalStatus()
        packet.main_robot = IS_MAIN_ROBOT
        packet.goal_identifier = goal.identifier
        packet.goal_status = goal.status
        self.eventloop.send_packet(packet)


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

        if not self.setup :
            self.init()

    def on_interbot_position(self, packet):
        if packet.main_robot :
            pass
            # logger.log("Got interbot position from main")
        else :
            pass
            # logger.log("Got interbot position from secondary")

    def on_interbot_hello(self, packet):
        logger.log('Received "Hello" from other robot')

    def on_disconnect(self):
        # logger.log('Other robot disconnected')
        leds.driver.set_mode(leds.driver.MODE_HEARTBEAT_GREEN)

    def init(self):
        if self.eventloop.robot.goal_manager :
            self.eventloop.robot.goal_manager.on_goal_state_change.connect(self.goal_state_change)
            self.setup = True

    def on_interbot_goal_status(self, packet):
        goal_id = packet.goal_identifier
        goal_status = packet.goal_status
        logger.log('Got goal status : {} = {}'.format(goal_id, goal_status))
        if self.eventloop.robot.goal_manager :
            self.eventloop.robot.goal_manager.internal_goal_update(goal_id, goal_status)

