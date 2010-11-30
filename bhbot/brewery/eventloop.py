#!/usr/bin/env python
# encoding: utf-8

import asyncore
import os
import socket
import imp
import inspect
import time

import logger
import packets
import config
import statemachine
import leds
import robot




class TurretChannel(asyncore.file_dispatcher):

    def __init__(self, serial_port_path, eventloop):
        asyncore.file_dispatcher.__init__(file(serial_port_path))
        self.eventloop = eventloop


    def writable(self):
        return False


    def handle_read(self):
        self.eventloop.handle_read(self)




class RobotControlDeviceChannel(asyncore.dispatcher_with_send):

    def __init__(self, eventloop):
        asyncore.dispatcher_with_send.__init__(self)
        self.eventloop = eventloop
        self.close_requested = False


    def setup(self):
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        logger.log("Connecting to {0}:{1} ...".format(config.remote_ip, config.remote_port))
        while not self.connected and not self.close_requested:
            try:
                self.connect((config.remote_ip, config.remote_port))
            except Exception as e:
                logger.log("Unable to connect to {0}:{1} ({2}), retrying".format(config.remote_ip, config.remote_port, e))
                time.sleep(1)
                leds.orange.toggle()


    def handle_connect(self):
        leds.orange.off()
        self.eventloop.create_fsm()


    def handle_read(self):
        self.eventloop.handle_read(self)


    def close(self):
        asyncore.dispatcher_with_send.close(self)
        self.close_requested = True




class ChannelData(object):

    def __init__(self):
        self.buffer = ""
        self.packet = None




class EventLoop(object):

    def __init__(self, state_machine_name):
        self.channels = {}
        self.robot_control_channel = None
        self.fsm = None
        self.robot = robot.Robot(self)
        self.state_machine_name = state_machine_name


    def handle_read(self, channel):
        channel_data = self.channels[channel]
        if channel_data.packet == None:
            channel_data.buffer += channel.recv(1)
            channel_data.packet = packets.create_packet(channel_data.buffer)
        else:
            channel_data.buffer += channel.recv(channel_data.packet.MAX_SIZE - len(channel_data.buffer))
            if len(channel_data.buffer) == channel_data.packet.MAX_SIZE:
                # A complete packet has been received, notify the state machine
                channel_data.packet.deserialize(channel_data.buffer)
                channel_data.buffer = ""

                logger.log_packet("PIC", channel_data.packet)

                if isinstance(channel_data.packet, packets.DeviceBusy):
                    self.fsm.state.on_device_busy()
                elif isinstance(channel_data.packet, packets.DeviceReady):
                    self.robot.team = channel_data.packet.team
                    self.fsm.state.on_device_ready(channel_data.packet.team)
                elif isinstance(channel_data.packet, packets.Start):
                    self.fsm.state.on_start(channel_data.packet.team)
                    self.robot.team = channel_data.packet.team
                elif isinstance(channel_data.packet, packets.GotoStarted):
                    self.fsm.state.on_goto_started()
                elif isinstance(channel_data.packet, packets.GotoFinished):
                    self.robot.pose = channel_data.packet.current_pose
                    self.fsm.state.on_goto_finished(channel_data.packet.reason, channel_data.packet.current_pose)
                elif isinstance(channel_data.packet, packets.Blocked):
                    self.fsm.state.on_blocked(channel_data.packet.side)
                elif isinstance(channel_data.packet, packets.KeepAlive):
                    self.send_packet(channel_data.packet)
                    self.robot.pose = channel_data.packet.current_pose
                    leds.green.heartbeat_tick()
                    self.fsm.state.on_keep_alive(channel_data.packet.current_pose, channel_data.packet.match_started, channel_data.packet.match_time)
                elif isinstance(channel_data.packet, packets.TurretDetect):
                    self.fsm.state.on_turret_detect(channel_data.packet.mean_angle, channel_data.packet.angular_size)

                channel_data.packet = None


    def send_packet(self, packet):
        if self.fsm != None:
            logger.log_packet("ARM", packet)
            buffer = packet.serialize()
            self.robot_control_channel.send(buffer)


    def create_fsm(self):
        state_machines_dir = os.path.join(os.path.dirname(__file__), "statemachines")
        state_machine_file = os.path.join(state_machines_dir, self.state_machine_name + ".py")
        state_machine_module = imp.load_source(self.state_machine_name, state_machine_file)
        for (item_name, item_type) in inspect.getmembers(state_machine_module):
            if inspect.isclass(item_type) and issubclass(item_type, statemachine.StateMachine):
                self.fsm = item_type()
                self.fsm.event_loop = self
                self.fsm.start()
                self.send_packet(packets.ControllerReady())
                return
        logger.log("No state machine found in '{0}'".format(state_machine_file))
        self.stop()


    def start(self):
        if (config.serial_port_path != None):
            self.channels[TurretChannel(config.serial_port_path, self)] = ChannelData()
        self.robot_control_channel = RobotControlDeviceChannel(self)
        self.channels[self.robot_control_channel] = ChannelData()
        self.robot_control_channel.setup()
        if self.robot_control_channel.connected:
            logger.log("Starting brewery with state machine '{0}'".format(self.state_machine_name))
            asyncore.loop()


    def stop(self):
        logger.log("Stopping...")
        for channel in self.channels.keys():
            channel.close()
        logger.close()
