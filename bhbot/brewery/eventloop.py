# encoding: utf-8

import asyncore
import socket
import errno
import datetime
import bisect
import fcntl
import termios
import ctypes
import struct

import logger
import packets
import statemachine
import asyncwsgiserver
import nanow
import leds
import robot
import opponentdetector
import trajectory
import graphpathfinding
import commonstates
import geometry
import goalmanager
import tools
import webinterface
import colordetector
from definitions import *

if IS_HOST_DEVICE_ARM :
    import serial




class TurretChannel(asyncore.file_dispatcher):

    def __init__(self, serial_port_path, serial_port_speed, eventloop):
        self.port = serial.PosixPollSerial(serial_port_path, serial_port_speed, timeout = 0)
        self.port.nonblocking()
        asyncore.file_dispatcher.__init__(self, self.port)
        self.eventloop = eventloop
        self.buffer = bytes()
        self.packet = None
        self.synchronized = False
        self.out_buffer = bytes()
        self.turret_packets = [ packets.TurretDetect.TYPE, packets.TurretInit.TYPE, packets.TurretDistances.TYPE, packets.TurretBoot.TYPE ]


    def bytes_available(self):
        return self.port.inWaiting()


    def initiate_send(self):
        num_sent = 0
        num_sent = asyncore.file_dispatcher.send(self, self.out_buffer[:512])
        self.out_buffer = self.out_buffer[num_sent:]


    def handle_write(self):
        self.initiate_send()


    def send(self, data):
        self.out_buffer = self.out_buffer + data
        self.initiate_send()


    def writable(self):
        return self.connected and len(self.out_buffer) != 0


    def handle_read(self):
        if not self.synchronized:
            while self.bytes_available():
                data = self.recv(1)
                (packet_type,) = struct.unpack("<B", data)
                if packet_type in self.turret_packets:
                    self.synchronized = True
                    self.buffer += data
                    self.packet = packets.create_packet(self.buffer)
                    break
        self.eventloop.handle_read(self)


    def close(self):
        self.port.close()
        asyncore.file_dispatcher.close(self)


    def handle_close(self):
        logger.log("handle_close")




class RobotControlDeviceChannel(asyncore.dispatcher_with_send):

    def __init__(self, eventloop, socket):
        asyncore.dispatcher_with_send.__init__(self, socket)
        self.eventloop = eventloop
        self.buffer = bytes()
        self.packet = None


    def bytes_available(self):
        available = ctypes.c_int()
        fcntl.ioctl(self.socket.fileno(), termios.FIONREAD, available)
        return available.value


    def handle_read(self):
        self.eventloop.handle_read(self)


    def handle_close(self):
        connected = False
        while not connected:
            try:
                logger.log("*** WARNING *** Reconnecting to {}:{}".format(REMOTE_IP, REMOTE_PORT))
                control_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                control_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                control_socket.connect((REMOTE_IP, REMOTE_PORT))
                self.eventloop.robot_control_channel = RobotControlDeviceChannel(self.eventloop, control_socket)
                leds.orange.off()
                connected = True
            except Exception as e:
                logger.log("Unable to connect to {}:{} ({}), retrying".format(REMOTE_IP, REMOTE_PORT, e))
                leds.orange.toggle()
                time.sleep(0.5)




class RobotLogDeviceChannel(asyncore.dispatcher):

    def __init__(self, socket):
        asyncore.dispatcher.__init__(self, socket)
        self.buffer = bytes()


    def bytes_available(self):
        available = ctypes.c_int()
        fcntl.ioctl(self.socket.fileno(), termios.FIONREAD, available)
        return available.value


    def writable(self):
        return False


    def handle_read(self):
        try:
            self.buffer += self.recv(self.bytes_available())
            while True:
                i = self.buffer.find("\n")
                if i != -1:
                    logger.log(self.buffer[:i].rstrip().encode("string_escape"), "PIC")
                    self.buffer = self.buffer[i + 1:]
                else:
                    break
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


    def restart(self):
        self.stop()
        self.start()


    def start(self):
        self.timeout_date = datetime.datetime.now() + datetime.timedelta(milliseconds = self.timeout_ms)
        bisect.insort_left(self.eventloop.timers, self)


    def stop(self):
        try:
            self.eventloop.timers.remove(self)
        except:
            pass
        self.timeout_date = None


    def __lt__(self, other):
        return self.timeout_date < other.timeout_date


    def __le__(self, other):
        return self.timeout_date <= other.timeout_date




class RobotControlDeviceStarter(object):

    def __init__(self, eventloop):
        logger.log("Connecting to {}:{} ...".format(REMOTE_IP, REMOTE_PORT))
        self.control_socket = None
        self.log_socket = None
        self.eventloop = eventloop
        self.timer = Timer(eventloop, 1000, self.try_connect, False)
        if not self.try_connect():
            self.timer.start()


    def try_connect(self):
        connected = False
        try:
            self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.control_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.control_socket.connect((REMOTE_IP, REMOTE_PORT))
            self.eventloop.robot_control_channel = RobotControlDeviceChannel(self.eventloop, self.control_socket)
            leds.orange.off()
            connected = True
        except Exception as e:
            logger.log("Unable to connect to {}:{} ({}), retrying".format(REMOTE_IP, REMOTE_PORT, e))
            leds.orange.toggle()
        if connected:
            try:
                self.log_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.log_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.log_socket.connect((REMOTE_IP, REMOTE_LOG_PORT))
                self.eventloop.robot_log_channel = RobotLogDeviceChannel(self.log_socket)
                logger.log("Connected to the log socket {}:{}".format(REMOTE_IP, REMOTE_LOG_PORT))
            except Exception as e:
                # Log socket is not mandatory. If the connection fails, continue without it.
                logger.log("Unable to connect to the log socket {}:{} ({}), continuing without PIC logs".format(REMOTE_IP, REMOTE_LOG_PORT, e))
            self.timer.stop()
            self.eventloop.on_turret_boot(None)
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
        self.eval_map = trajectory.Map(self)
        self.map = graphpathfinding.Map(self)
        self.timers = []
        self.state_history = []
        self.last_ka_date = datetime.datetime.now()
        if IS_HOST_DEVICE_ARM:
            self.colordetector = colordetector.ColorDetector()
            self.colordetector.reinit()
        else:
            self.colordetector = None


    def handle_read(self, channel):
        try :
            return self.do_read(channel)
        except Exception as e :
            logger.log_exception(e)


    def do_read(self, channel):
        while True :
            if channel.packet is None:
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
                        channel.buffer = bytes()

                        logger.log_packet(channel.packet, "PIC")

                        channel.packet.dispatch(self)
                        channel.packet.dispatch(self.robot)
                        channel.packet.dispatch(self.opponent_detector)
                        channel.packet.dispatch(self.map)
                        channel.packet.dispatch(self.eval_map)
                        channel.packet.dispatch(self.get_current_state())

                        channel.packet = None
                except Exception as e:
                    channel.packet = None
                    logger.log_exception(e)


    def on_device_ready(self, packet):
        logger.set_team(packet.team)


    def on_keep_alive(self, packet):
        now = datetime.datetime.now()
        if (now - self.last_ka_date).total_seconds() > KEEP_ALIVE_MINIMUM_AGE_S:
            self.last_ka_date = now
            self.send_packet(packet)
            leds.green.heartbeat_tick()


    def on_turret_boot(self, packet):
        if self.turret_channel is not None:
            packet = packets.TurretInit()
            packet.mode = TURRET_INIT_MODE_WRITE
            packet.short_distance = TURRET_SHORT_DISTANCE_DETECTION_ID
            packet.long_distance = TURRET_LONG_DISTANCE_DETECTION_ID
            buffer = packet.serialize()
            logger.log_packet(packet, "ARM")
            self.turret_channel.send(buffer)


    def send_packet(self, packet):
        if self.root_state.sub_state is not None:
            logger.log_packet(packet, "ARM")
            buffer = packet.serialize()
            if packet.TYPE < 50:
                self.turret_channel.send(buffer)
            elif packet.TYPE < 200:
                self.robot_control_channel.send(buffer)
            else:
                self.interbot_channel.send(buffer)


    def inject_goto_finished(self):
        packet = packets.GotoFinished()
        packet.reason = REASON_DESTINATION_REACHED
        packet.current_pose = self.robot.pose
        packet.current_point_index = 0
        packet.dispatch(self.get_current_state())


    def get_current_state(self):
        state = self.root_state
        while state.sub_state is not None:
            state = state.sub_state
        return state


    def create_fsm(self):
        state = statemachine.instantiate_state_machine(self.state_machine_name, self)
        if state is None:
            self.stop()
        else:
            self.root_state.switch_to_substate(state)
            self.send_packet(packets.ControllerReady())


    def start(self):
        logger.log("Starting internal web server on port {}".format(self.webserver_port))
        self.web_server = asyncwsgiserver.WsgiServer("", self.webserver_port, nanow.Application(webinterface.BHWeb(self)))
        if SERIAL_PORT_PATH is not None:
            try:
                self.turret_channel = TurretChannel(SERIAL_PORT_PATH, SERIAL_PORT_SPEED, self)
            except serial.SerialException:
                logger.log("Unable to open serial port {}".format(SERIAL_PORT_PATH))
                self.turret_channel = None
        RobotControlDeviceStarter(self)
        logger.log("Starting brewery with state machine '{}'".format(self.state_machine_name))
        while not self.stopping:
            asyncore.loop(EVENT_LOOP_TICK_RESOLUTION_S, True, None, 1)
            self.get_current_state().on_timer_tick()
            while len(self.timers) != 0:
                if not self.timers[0].check_timeout():
                    break


    def stop(self):
        logger.log("Stopping...")
        self.stopping = True
        if self.turret_channel is not None:
            self.turret_channel.close()
        if self.robot_control_channel is not None:
            self.robot_control_channel.close()
        if self.robot_log_channel is not None:
            self.robot_log_channel.close()
        if self.colordetector is not None:
            self.colordetector.quit()
