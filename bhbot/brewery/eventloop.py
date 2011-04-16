#!/usr/bin/env python
# encoding: utf-8

import asyncore
import os
import socket
import imp
import inspect
import time
import traceback
import errno
if config.with_serial : import serial

import logger
import packets
import config
import statemachine
import leds
import robot




class TurretChannel(asyncore.file_dispatcher):

    def __init__(self, serial_port_path, serial_port_speed, eventloop):
        asyncore.file_dispatcher.__init__(self, serial.Serial(serial_port_path, serial_port_speed))
        self.eventloop = eventloop
        self.buffer = ""
        self.packet = None


    def writable(self):
        return False


    def handle_read(self):
        self.eventloop.handle_read(self)




class RobotControlDeviceChannel(asyncore.dispatcher_with_send):

    def __init__(self, eventloop):
        asyncore.dispatcher_with_send.__init__(self)
        self.eventloop = eventloop
        self.close_requested = False
        self.buffer = ""
        self.packet = None


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




class EventLoop(object):

    def __init__(self, state_machine_name):
        self.robot_control_channel = None
        self.turret_channel = None
        self.fsm = None
        self.robot = robot.Robot(self)
        self.state_machine_name = state_machine_name


    def handle_read(self, channel):
        while True :
            if channel.packet == None:
                try:
                    received_data = channel.recv(1)
                    if len(received_data) == 0:
                        return
                    channel.buffer += received_data
                    channel.packet = packets.create_packet(channel.buffer)
                except socket.error as err:
                    if err.errno in [errno.EAGAIN, errno.EINTR]:
                        return
                    for line in traceback.format_exc().strip().split('\n'):
                        logger.log(line)
                    return
            else:
                try:
                    try:
                        received_data = channel.recv(channel.packet.MAX_SIZE - len(channel.buffer))
                        if len(received_data) == 0:
                            return
                        channel.buffer += received_data
                    except socket.error as err:
                        if err.errno in [errno.EAGAIN, errno.EINTR]:
                            return
                        for line in traceback.format_exc().strip().split('\n'):
                            logger.log(line)
                        return
                    if len(channel.buffer) == channel.packet.MAX_SIZE:
                        # A complete packet has been received, notify the state machine
                        channel.packet.deserialize(channel.buffer)
                        channel.buffer = ""

                        logger.log_packet("PIC", channel.packet)

                        if isinstance(channel.packet, packets.DeviceBusy):
                            self.fsm.state.on_device_busy()
                        elif isinstance(channel.packet, packets.DeviceReady):
                            self.robot.team = channel.packet.team
                            self.fsm.state.on_device_ready(channel.packet.team)
                        elif isinstance(channel.packet, packets.Start):
                            self.fsm.state.on_start(channel.packet.team)
                            self.robot.team = channel.packet.team
                        elif isinstance(channel.packet, packets.GotoStarted):
                            self.fsm.state.on_goto_started()
                        elif isinstance(channel.packet, packets.GotoFinished):
                            self.robot.pose = channel.packet.current_pose
                            self.fsm.state.on_goto_finished(channel.packet.reason, channel.packet.current_pose)
                        elif isinstance(channel.packet, packets.Blocked):
                            self.fsm.state.on_blocked(channel.packet.side)
                        elif isinstance(channel.packet, packets.KeepAlive):
                            self.send_packet(channel.packet)
                            self.robot.pose = channel.packet.current_pose
                            leds.green.heartbeat_tick()
                            self.fsm.state.on_keep_alive(channel.packet.current_pose, channel.packet.match_started, channel.packet.match_time)
                        elif isinstance(channel.packet, packets.PieceDetected):
                            self.fsm.state.on_piece_detected(channel.packet.left_sensor, channel.packet.right_sensor, channel.packet.center_sensor_angle)
                        elif isinstance(channel.packet, packets.PieceStored):
                            self.robot.stored_piece_count = channel.packet.piece_count
                            self.fsm.state.on_piece_stored(channel.packet.piece_count)
                        elif isinstance(channel.packet, packets.TurretDetect):
                            self.fsm.state.on_turret_detect(channel.packet.mean_angle, channel.packet.angular_size)
                        channel.packet = None
                except:
                    for line in traceback.format_exc().strip().split('\n'):
                        logger.log(line)


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
            try:
                self.turret_channel = TurretChannel(config.serial_port_path, config.serial_port_speed, self)
            except serial.SerialException:
                logger.log("Unable to open serial port {0}".format(config.serial_port_path))
                self.turret_channel = None
        self.robot_control_channel = RobotControlDeviceChannel(self)
        self.robot_control_channel.setup()
        if self.robot_control_channel.connected:
            logger.log("Starting brewery with state machine '{0}'".format(self.state_machine_name))
            asyncore.loop()


    def stop(self):
        logger.log("Stopping...")
        if self.turret_channel != None:
            self.turret_channel.close()
        if self.robot_control_channel != None:
            self.robot_control_channel.close()
        logger.close()
