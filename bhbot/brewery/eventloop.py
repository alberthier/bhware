#!/usr/bin/env python
# encoding: utf-8

import asyncore
import socket
import logger

import packets
import config




class TurretChannel(asyncore.file_dispatcher):

    def __init__(self, serial_port_filepath, eventloop):
        asyncore.file_dispatcher.__init__(file(serial_port_filepath))
        self.eventloop = eventloop


    def writable(self):
        return False


    def handle_read(self):
        self.eventloop.handle_read(self)




class RobotControlDeviceChannel(asyncore.dispatcher_with_send):

    def __init__(self, eventloop):
        asyncore.dispatcher_with_send.__init__(self)
        self.eventloop = eventloop
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        connected = False
        while not connected:
            try:
                self.connect((config.remote_ip, config.remote_port))
                connected = True
            except:
                logger.log("Unable to connect to {0}:{1}, retrying".format(config.remote_ip, config.remote_port))


    def handle_connect(self):
        self.eventloop.create_fsm()


    def handle_read(self):
        eventloop.handle_read(self)




class ChannelData(object):

    def __init__(self):
        self.buffer = ""
        self.packet = None




class EventLoop(object):

    def __init__(self):
        self.channels = {}
        if (config.serial_port_filepath != None):
            self.channels[TurretChannel(config.serial_port_filepath, self)] = ChannelData()
        robot_control_channel = RobotControlDeviceChannel(self)
        self.channels[robot_control_channel] = ChannelData()
        self.fsm = None


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
                    self.fsm.state.on_device_ready(channel_data.packet.team)
                elif isinstance(channel_data.packet, packets.Start):
                    self.fsm.state.on_start(channel_data.packet.team)
                elif isinstance(channel_data.packet, packets.GotoStarted):
                    self.fsm.state.on_goto_started()
                elif isinstance(channel_data.packet, packets.GotoFinished):
                    self.fsm.state.on_goto_finished(channel_data.packet.reason)
                elif isinstance(channel_data.packet, packets.Blocked):
                    self.fsm.state.on_blocked(channel_data.packet.side)
                elif isinstance(channel_data.packet, packets.KeepAlive):
                    self.fsm.state.on_keep_alive(channel_data.packet.current_pose, channel_data.packet.match_started, channel_data.packet.match_time)
                elif isinstance(channel_data.packet, packets.TurretDetect):
                    self.fsm.state.on_turret_detect(channel_data.packet.mean_angle, channel_data.packet.angular_size)

                channel_data.packet = None


    def send_packet(self, packet):
        buffer = packet.serialize()
        self.robot_control_channel.send(buffer)
        logger.log_packet("ARM", packet)


    def create_fsm(self):
        statemachines_dir = os.path.join(os.path.dirname(__file__)), "statemachines")
        self.fsm = None # todo


    def start(self):
        asyncore.loop()

