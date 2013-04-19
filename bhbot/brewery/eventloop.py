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
import time
import collections

import logger
import packets
import statemachine
import asyncwsgiserver
import nanow
import leds
import robot
import opponentdetector
import trajectory
import graphmap
import commonstates
import geometry
import goalmanager
import tools
import webinterface
import colordetector
import interbot

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
        self.origin = "TUR"


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
                if packet_type < packets.TURRET_RANGE_END:
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
        self.origin = "PIC"


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




class InterbotChannel(asyncore.dispatcher_with_send):

    def __init__(self, event_loop, sock = None):
        asyncore.dispatcher_with_send.__init__(self, sock)
        self.event_loop = event_loop
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buffer = bytes()
        self.packet = None
        self.event_loop.interbot_channel = self
        self.origin = "TMM"
        logger.log("Interbot channel connected")


    def bytes_available(self):
        available = ctypes.c_int()
        fcntl.ioctl(self.socket.fileno(), termios.FIONREAD, available)
        return available.value


    def handle_read(self):
        self.event_loop.handle_read(self)




class InterbotServer(asyncore.dispatcher):

    def __init__(self, event_loop, host, port):
        self.event_loop = event_loop
        try:
            asyncore.dispatcher.__init__ (self)
            self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
            self.set_reuse_addr()
            self.bind((host, port))
            self.listen(5)
            self.event_loop.interbot_channel = self
            logger.log("Starting interbot server on port {}".format(MAIN_INTERBOT_PORT))
        except Exception as e:
            logger.log("Unable to start interbot server on port {}".format(MAIN_INTERBOT_PORT))


    def handle_accepted(self, sock, addr):
        InterbotChannel(self.event_loop, sock)




class InterbotClientStarter(object):

    def __init__(self, event_loop):
        logger.log("Interbot - Connecting to {}:{} ...".format(MAIN_INTERBOT_IP, MAIN_INTERBOT_PORT))
        self.socket = None
        self.event_loop = event_loop
        self.timer = Timer(event_loop, 1000, self.try_connect, False)
        if not self.try_connect():
            self.timer.start()


    def try_connect(self):
        try:
            if self.event_loop.is_match_started:
                self.timer.stop()
                return
            # Connect to the main robot
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((MAIN_INTERBOT_IP, MAIN_INTERBOT_PORT))
            self.event_loop.robot_control_channel = InterbotChannel(self.event_loop, self.socket)
            self.timer.stop()
            return True
        except Exception as e:
            logger.log("Interbot - Unable to connect to {}:{} ({}), retrying".format(MAIN_INTERBOT_IP, MAIN_INTERBOT_PORT, e))
        return False




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
        self.interbot_channel = None
        self.interbot_server = None
        self.web_server = None
        self.robot = robot.Robot(self)
        self.main_state = None
        self.end_of_match_state = None
        self.fsm = None
        self.state_machine_name = state_machine_name
        self.webserver_port = webserver_port
        self.opponent_detector = opponentdetector.OpponentDetector(self)
        self.interbot_manager = interbot.InterBotManager(self)
        self.stopping = False
        self.is_match_started = False
        self.map = graphmap.Map(self)
        self.timers = []
        self.last_ka_date = datetime.datetime.now()
        self.start_date = None
        self.packet_queue = collections.deque()

        if IS_HOST_DEVICE_ARM and IS_MAIN_ROBOT:
            self.colordetector = colordetector.ColorDetector(self)
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

                        logger.log_packet(channel.packet, channel.origin)

                        self.enqueue_packet(channel.packet)

                        channel.packet = None

                        self.process_packets_and_dispatch()

                except Exception as e:
                    channel.packet = None
                    logger.log_exception(e)


    def on_device_ready(self, packet):
        logger.set_team(packet.team)
        if self.colordetector is not None:
            self.colordetector.set_team(packet.team)


    def on_keep_alive(self, packet):
        now = datetime.datetime.now()
        if (now - self.last_ka_date).total_seconds() > KEEP_ALIVE_MINIMUM_AGE_S:
            self.last_ka_date = now
            self.send_packet(packet)
            leds.green.heartbeat_tick()


    def on_start(self, packet):
        self.is_match_started = True
        self.start_date = datetime.datetime.now()
        Timer(self, MATCH_DURATION_MS, self.on_end_of_match).start()


    def on_turret_boot(self, packet):
        if self.turret_channel is not None:
            packet = packets.TurretInit()
            packet.mode = TURRET_INIT_MODE_WRITE
            packet.short_distance = TURRET_SHORT_DISTANCE_DETECTION_ID
            packet.long_distance = TURRET_LONG_DISTANCE_DETECTION_ID
            buffer = packet.serialize()
            logger.log_packet(packet, "ARM")
            self.turret_channel.send(buffer)


    def on_end_of_match(self):
        logger.log("End of match. Starting the funny action")
        self.fsm = statemachine.StateMachine(self, self.end_of_match_state)


    def get_elapsed_match_time(self):
        if self.start_date is None:
            return 0.0
        delta = datetime.datetime.now() - self.start_date
        return delta.total_seconds()



    def send_packet(self, packet):
        logger.log_packet(packet, "ARM")
        buffer = packet.serialize()
        if packet.TYPE < packets.TURRET_RANGE_END:
            self.turret_channel.send(buffer)
        elif packet.TYPE < packets.PIC32_RANGE_END:
            self.robot_control_channel.send(buffer)
        elif packet.TYPE < packets.SIMULATOR_RANGE_END:
            self.robot_control_channel.send(buffer)
        elif packet.TYPE >= packets.INTERBOT_RANGE_START:
            if self.interbot_channel :
                self.interbot_channel.send(buffer)


    def inject_goto_finished(self):
        packet = packets.GotoFinished()
        packet.reason = REASON_DESTINATION_REACHED
        packet.current_pose = self.robot.pose
        packet.current_point_index = 0
        if self.fsm is not None:
            self.fsm.dispatch(packet)


    def get_current_state(self):
        if self.fsm is not None:
            return self.fsm.current_state


    def create_fsm(self):
        (self.main_state, self.end_of_match_state) = statemachine.instantiate_state_machine(self.state_machine_name)
        if self.main_state is None or self.end_of_match_state is None:
            logger.log("One of 'Main' or 'EndOfMatch' state is missing")
            self.stop()
        else:
            self.fsm = statemachine.StateMachine(self, self.main_state)
            self.send_packet(packets.ControllerReady())


    def start(self):
        if IS_MAIN_ROBOT:
            InterbotServer(self, "", MAIN_INTERBOT_PORT)
        else:
            InterbotClientStarter(self)
        logger.log("Starting internal web server on port {}".format(self.webserver_port))
        self.web_server = asyncwsgiserver.WsgiServer("", self.webserver_port, nanow.Application(webinterface.BHWeb(self)))
        if SERIAL_PORT_PATH is not None:
            try:
                self.turret_channel = TurretChannel(SERIAL_PORT_PATH, SERIAL_PORT_SPEED, self)
            except serial.SerialException:
                logger.log("Unable to open serial port {}".format(SERIAL_PORT_PATH))
                self.turret_channel = None
        logger.log("Starting brewery with state machine '{}'".format(self.state_machine_name))
        RobotControlDeviceStarter(self)
        while not self.stopping:
            asyncore.loop(EVENT_LOOP_TICK_RESOLUTION_S, True, None, 1)
            if self.fsm is not None:
                self.fsm.on_timer_tick()
            while len(self.timers) != 0:
                if not self.timers[0].check_timeout():
                    break

    def enqueue_packet(self, packet):
        self.packet_queue.appendleft(packet)

    def process_packets_and_dispatch(self):
        # logger.log('process packets')
        while self.packet_queue :
            packet = self.packet_queue.pop()
            self.dispatch(packet)

    def dispatch(self, packet):
        # logger.log('dispatch packet {}'.format(packet))
        packet.dispatch(self)
        packet.dispatch(self.robot)
        packet.dispatch(self.opponent_detector)
        packet.dispatch(self.map)
        packet.dispatch(self.interbot_manager)

        if self.fsm is not None:
            packet.dispatch(self.fsm)

    def stop(self):
        logger.log("Stopping...")
        self.stopping = True
        if self.turret_channel is not None:
            self.turret_channel.close()
        if self.robot_control_channel is not None:
            self.robot_control_channel.close()
        if self.robot_log_channel is not None:
            self.robot_log_channel.close()
        if self.interbot_channel is not None:
            self.interbot_channel.close()
        if self.interbot_server is not None:
            self.interbot_server.close()
        if self.colordetector is not None:
            self.colordetector.quit()
