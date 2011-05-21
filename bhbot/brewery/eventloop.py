#!/usr/bin/env python
# encoding: utf-8

import config
import asyncore
import os
import socket
import imp
import inspect
import time
import errno

import logger
import packets
import config
import statemachine
import leds
import robot
import figuredetector
import opponentdetector
from definitions import *



class TurretChannel(asyncore.file_dispatcher):

    def __init__(self, serial_port_path, serial_port_speed, eventloop):
        import serial
        asyncore.file_dispatcher.__init__(self, serial.Serial(serial_port_path, serial_port_speed))
        self.eventloop = eventloop
        self.buffer = ""
        self.packet = None


    def writable(self):
        return False


    def handle_read(self):
        self.eventloop.handle_read(self)


    def handle_close(self):
        # Ignore close events
        pass




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
        self.root_state = statemachine.State()
        self.root_state.event_loop = self
        self.robot = robot.Robot(self)
        self.state_machine_name = state_machine_name
        self.figure_detector = figuredetector.FigureDetector(self.robot)
        self.opponent_detector = opponentdetector.OpponentDetector(self)


    def handle_read(self, channel):
        try :
            return self.do_read(channel)
        except Exception, e :
            logger.exception(e)


    def do_read(self, channel):
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
                    logger.exception(err)
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
                        logger.exception(err)
                        return
                    if len(channel.buffer) == channel.packet.MAX_SIZE:
                        # A complete packet has been received, notify the state machine
                        channel.packet.deserialize(channel.buffer)
                        channel.buffer = ""

                        logger.log_packet("PIC", channel.packet)

                        if isinstance(channel.packet, packets.DeviceBusy):
                            self.get_current_state().on_device_busy()
                        elif isinstance(channel.packet, packets.DeviceReady):
                            self.robot.team = channel.packet.team
                            self.get_current_state().on_device_ready(channel.packet.team)
                        elif isinstance(channel.packet, packets.Start):
                            self.robot.team = channel.packet.team
                            self.get_current_state().on_start(channel.packet.team)
                        elif isinstance(channel.packet, packets.GotoStarted):
                            self.get_current_state().on_goto_started()
                        elif isinstance(channel.packet, packets.GotoFinished):
                            self.robot.pose = channel.packet.current_pose
                            self.get_current_state().on_goto_finished(channel.packet.reason, channel.packet.current_pose)
                        elif isinstance(channel.packet, packets.Blocked):
                            self.get_current_state().on_blocked(channel.packet.side)
                        elif isinstance(channel.packet, packets.EnableAntiBlocking):
                            self.get_current_state().on_anti_blocking_enabled()
                        elif isinstance(channel.packet, packets.DisableAntiBlocking):
                            self.get_current_state().on_anti_blocking_disabled()
                        elif isinstance(channel.packet, packets.KeepAlive):
                            self.send_packet(channel.packet)
                            self.robot.pose = channel.packet.current_pose
                            leds.green.heartbeat_tick()
                            self.opponent_detector.on_keep_alive(channel.packet.current_pose, channel.packet.match_started, channel.packet.match_time)
                            self.get_current_state().on_keep_alive(channel.packet.current_pose, channel.packet.match_started, channel.packet.match_time)
                        elif isinstance(channel.packet, packets.PositionControlConfig):
                            self.get_current_state().on_position_control_configured()
                        elif isinstance(channel.packet, packets.Resettle):
                            self.get_current_state().on_resettled(channel.packet.axis, channel.packet.position, channel.packet.angle)
                        elif isinstance(channel.packet, packets.Deployment):
                            self.get_current_state().on_deployed()
                        elif isinstance(channel.packet, packets.PieceDetected):
                            self.figure_detector.on_piece_detected(channel.packet.start_pose, channel.packet.start_distance, channel.packet.end_pose, channel.packet.end_distance, channel.packet.sensor, channel.packet.angle)
                            self.get_current_state().on_piece_detected(channel.packet.start_pose, channel.packet.start_distance, channel.packet.end_pose, channel.packet.end_distance, channel.packet.sensor, channel.packet.angle)
                        elif isinstance(channel.packet, packets.StorePiece1):
                            self.robot.stored_piece_count = channel.packet.piece_count
                            self.get_current_state().on_piece_stored1(channel.packet.piece_count)
                        elif isinstance(channel.packet, packets.StorePiece2):
                            self.robot.stored_piece_count = channel.packet.piece_count
                            self.get_current_state().on_piece_stored2(channel.packet.piece_count)
                        elif isinstance(channel.packet, packets.StorePiece3):
                            self.robot.stored_piece_count = channel.packet.piece_count
                            self.get_current_state().on_piece_stored3(channel.packet.piece_count)
                        elif isinstance(channel.packet, packets.ReleasePiece):
                            self.robot.stored_piece_count = 0
                            self.get_current_state().on_piece_released()
                        elif isinstance(channel.packet, packets.OpenNippers):
                            self.get_current_state().on_nippers_opened()
                        elif isinstance(channel.packet, packets.CloseNippers):
                            self.get_current_state().on_nippers_closed()
                        elif isinstance(channel.packet, packets.EnableLateralSensors):
                            self.get_current_state().on_lateral_sensors_enabled()
                        elif isinstance(channel.packet, packets.DisableLateralSensors):
                            self.get_current_state().on_lateral_sensors_disabled()
                        elif isinstance(channel.packet, packets.Reinitialize):
                            self.get_current_state().on_reinitialized()
                        elif isinstance(channel.packet, packets.TurretDetect):
                            self.opponent_detector.on_turret_detect(channel.packet.angle)
                        channel.packet = None
                except Exception, e:
                    logger.exception(e)


    def send_packet(self, packet):
        if self.root_state.sub_state != None:
            logger.log_packet("ARM", packet)
            buffer = packet.serialize()
            self.robot_control_channel.send(buffer)


    def inject_goto_finished(self):
        self.get_current_state().on_goto_finished(REASON_DESTINATION_REACHED, self.robot.pose)


    def get_current_state(self):
        state = self.root_state
        while state.sub_state != None:
            state = state.sub_state
        return state


    def create_fsm(self):
        state = statemachine.instantiate_state_machine(self.state_machine_name, self)
        if state == None :
            logger.log("No 'Main' state machine found in '{0}'".format(state_machine_file))
            self.stop()
        else:
            self.root_state.switch_to_substate(state)
            self.send_packet(packets.ControllerReady())


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
            asyncore.loop(30.0, True)


    def stop(self):
        logger.log("Stopping...")
        if self.turret_channel != None:
            self.turret_channel.close()
        if self.robot_control_channel != None:
            self.robot_control_channel.close()
        logger.close()
