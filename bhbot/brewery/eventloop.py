#!/usr/bin/env python
# encoding: utf-8

import config
import asyncore
import os
import socket
import imp
import inspect
import errno
import traceback
import datetime
import bisect
import fcntl
import termios
import ctypes

import logger
import packets
import config
import statemachine
import asyncwsgiserver
import web.webinterface
import leds
import robot
import opponentdetector
from definitions import *




class TurretChannel(asyncore.file_dispatcher):

    def __init__(self, serial_port_path, serial_port_speed, eventloop):
        import serial
        self.port = serial.PosixPollSerial(serial_port_path, serial_port_speed, timeout = 0)
        self.port.nonblocking()
        asyncore.file_dispatcher.__init__(self, self.port)
        self.eventloop = eventloop
        self.buffer = ""
        self.packet = None


    def bytes_available(self):
        return self.port.inWaiting()


    def writable(self):
        return False


    def handle_read(self):
        self.eventloop.handle_read(self)


    def handle_close(self):
        logger.log("handle_close")
        for l in traceback.format_stack() :
            logger.log(l)




class RobotControlDeviceChannel(asyncore.dispatcher_with_send):

    def __init__(self, eventloop, socket):
        asyncore.dispatcher_with_send.__init__(self, socket)
        self.eventloop = eventloop
        self.buffer = ""
        self.packet = None


    def bytes_available(self):
        available = ctypes.c_int()
        fcntl.ioctl(self.socket.fileno(), termios.FIONREAD, available)
        return available.value


    def handle_read(self):
        self.eventloop.handle_read(self)




class RobotLogDeviceChannel(asyncore.dispatcher):

    def __init__(self, socket):
        asyncore.dispatcher.__init__(self, socket)
        self.buffer = ""


    def bytes_available(self):
        available = ctypes.c_int()
        fcntl.ioctl(self.socket.fileno(), termios.FIONREAD, available)
        return available.value


    def handle_read(self):
        try:
            self.buffer += self.recv(self.bytes_available())
            while True:
                i = self.buffer.find("\n")
                if i != -1:
                    logger.log(self.buffer[:i], "PIC")
                    self.buffer = self.buffer[i + 1:]
                else:
                    break;
        except:
            pass




class Timer(object):

    def __init__(self, eventloop, timeout_ms, callback, single_shot = True):
        self.eventloop = eventloop
        self.timeout_ms = timeout_ms
        self.callback = callback
        self.single_shot = single_shot
        self.timeout_date = None


    def check_timeout(self):
        if datetime.datetime.now() >= self.timeout_date:
            self.stop()
            if not self.single_shot:
                self.start()
            self.callback()
            return True
        return False


    def start(self):
        self.timeout_date = datetime.datetime.now() + datetime.timedelta(milliseconds = self.timeout_ms)
        bisect.insort_left(self.eventloop.timers, self)


    def stop(self):
        try:
            self.eventloop.timers.remove(self)
        except:
            pass
        self.timeout_date = None


    def __cmp__(self, other):
        return cmp(self.timeout_date, other.timeout_date)




class RobotControlDeviceStarter(object):

    def __init__(self, eventloop):
        logger.log("Connecting to {}:{} ...".format(config.remote_ip, config.remote_port))
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.log_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.eventloop = eventloop
        self.timer = Timer(eventloop, 1000, self.try_connect, False)
        if not self.try_connect():
            self.timer.start()


    def try_connect(self):
        connected = False
        try:
            self.control_socket.connect((config.remote_ip, config.remote_port))
            self.eventloop.robot_control_channel = RobotControlDeviceChannel(self.eventloop, self.control_socket)
            leds.orange.off()
            connected = True
        except Exception as e:
            logger.log("Unable to connect to {}:{} ({}), retrying".format(config.remote_ip, config.remote_port, e))
            leds.orange.toggle()
        if connected:
            try:
                self.log_socket.connect((config.remote_ip, config.remote_log_port))
                self.eventloop.robot_log_channel = RobotLogDeviceChannel(self.log_socket)
                logger.log("Connected to the log stocket {}:{}".format(config.remote_ip, config.remote_log_port))
            except Exception as e:
                # Log socket is not mandatory. If the connection fails, continue without it.
                logger.log("Unable to connect to the log stocket {}:{} ({}), continuing without PIC logs".format(config.remote_ip, config.remote_log_port, e))
            self.eventloop.create_fsm()
        return connected




class EventLoop(object):

    def __init__(self, state_machine_name, webserver_port):
        self.robot_control_channel = None
        self.robot_log_channel = None
        self.turret_channel = None
        self.web_server = None
        self.root_state = statemachine.State()
        self.root_state.event_loop = self
        self.robot = robot.Robot(self)
        self.state_machine_name = state_machine_name
        self.webserver_port = webserver_port
        self.opponent_detector = opponentdetector.OpponentDetector(self)
        self.stopping = False
        self.timers = []


    def handle_read(self, channel):
        try :
            return self.do_read(channel)
        except Exception, e :
            logger.log_exception(e)


    def do_read(self, channel):
        while True :
            if channel.packet == None:
                try:
                    b = channel.bytes_available()
                    if b == 0:
                        return
                    received_data = channel.recv(1)
                    if len(received_data) == 0:
                        return
                    channel.buffer += received_data
                    channel.packet = packets.create_packet(channel.buffer)
                except socket.error as err:
                    if err.errno in [errno.EAGAIN, errno.EINTR]:
                        return
                    logger.log_exception(err)
                    return
            else:
                try:
                    try:
                        if channel.bytes_available() < channel.packet.MAX_SIZE - 1:
                            return
                        received_data = channel.recv(channel.packet.MAX_SIZE - len(channel.buffer))
                        if len(received_data) == 0:
                            return
                        channel.buffer += received_data
                    except socket.error as err:
                        if err.errno in [errno.EAGAIN, errno.EINTR]:
                            return
                        logger.log_exception(err)
                        return
                    if len(channel.buffer) == channel.packet.MAX_SIZE:
                        # A complete packet has been received, notify the state machine
                        channel.packet.deserialize(channel.buffer)
                        channel.buffer = ""

                        logger.log_packet(channel.packet, "PIC")

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
                            self.get_current_state().on_goto_finished(channel.packet.reason, channel.packet.current_pose, channel.packet.current_point_index)
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
                            self.get_current_state().on_keep_alive(channel.packet.current_pose, channel.packet.match_started, channel.packet.match_time)
                        elif isinstance(channel.packet, packets.PositionControlConfig):
                            self.get_current_state().on_position_control_configured()
                        elif isinstance(channel.packet, packets.Resettle):
                            self.get_current_state().on_resettled(channel.packet.axis, channel.packet.position, channel.packet.angle)
                        elif isinstance(channel.packet, packets.Deployment):
                            self.get_current_state().on_deployed()
                        elif isinstance(channel.packet, packets.PieceDetected):
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
                        elif isinstance(channel.packet, packets.OpenMandibles):
                            self.get_current_state().on_mandibles_opened()
                        elif isinstance(channel.packet, packets.CloseMandibles):
                            self.get_current_state().on_mandibles_closed()
                        elif isinstance(channel.packet, packets.Reinitialize):
                            self.get_current_state().on_reinitialized()
                        elif isinstance(channel.packet, packets.TurretDetect):
                            self.opponent_detector.on_turret_detect(channel.packet.angle)
                        channel.packet = None
                except Exception, e:
                    logger.log_exception(e)


    def send_packet(self, packet):
        if self.root_state.sub_state != None:
            logger.log_packet(packet, "ARM")
            buffer = packet.serialize()
            self.robot_control_channel.send(buffer)


    def inject_goto_finished(self):
        self.get_current_state().on_goto_finished(REASON_DESTINATION_REACHED, self.robot.pose, 0)


    def get_current_state(self):
        state = self.root_state
        while state.sub_state != None:
            state = state.sub_state
        return state


    def create_fsm(self):
        state = statemachine.instantiate_state_machine(self.state_machine_name, self)
        if state == None :
            logger.log("No 'Main' state machine found in '{}'".format(state_machine_file))
            self.stop()
        else:
            self.root_state.switch_to_substate(state)
            self.send_packet(packets.ControllerReady())


    def start(self):
        logger.log("Starting internal web server on port {}".format(self.webserver_port))
        self.web_server = asyncwsgiserver.WsgiServer("", self.webserver_port, web.webinterface.create_app(self))
        if (config.serial_port_path != None):
            try:
                self.turret_channel = TurretChannel(config.serial_port_path, config.serial_port_speed, self)
            except serial.SerialException:
                logger.log("Unable to open serial port {}".format(config.serial_port_path))
                self.turret_channel = None
        RobotControlDeviceStarter(self)
        logger.log("Starting brewery with state machine '{}'".format(self.state_machine_name))
        while not self.stopping:
            asyncore.loop(EVENT_LOOP_TICK_RESOLUTION, True, None, 1)
            self.get_current_state().on_timer_tick()
            self.opponent_detector.on_timer_tick()
            while len(self.timers) != 0:
                if not self.timers[0].check_timeout():
                    break


    def stop(self):
        logger.log("Stopping...")
        self.stopping = True
        if self.turret_channel != None:
            self.turret_channel.close()
        if self.robot_control_channel != None:
            self.robot_control_channel.close()
        logger.close()
