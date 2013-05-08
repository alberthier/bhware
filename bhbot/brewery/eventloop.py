# encoding: utf-8

import sys
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




class ClientSocketChannel(asyncore.dispatcher_with_send):

    def __init__(self, event_loop, origin, remote):
        self.event_loop = event_loop
        self.origin = origin
        self.existing_socket = None
        self.address = None
        self.show_reconnect_error_log = True
        if isinstance(remote, socket.socket):
            self.existing_socket = remote
        else:
            self.address = remote
        super().__init__(self.existing_socket)
        if self.existing_socket is None:
            self.try_connect()


    def bytes_available(self):
        available = ctypes.c_int()
        fcntl.ioctl(self.socket.fileno(), termios.FIONREAD, available)
        return available.value


    def handle_connect(self):
        self.show_reconnect_error_log = True
        logger.log("{}: Connected".format(self.origin))


    def handle_close(self):
        self.close()
        if self.existing_socket is None:
            if self.show_reconnect_error_log:
                logger.log("{}: *** WARNING *** Connection closed, reconnecting".format(self.origin))
            self.try_connect()


    def handle_error(self):
        if self.existing_socket is None:
            self.close()
            next_try = 1000
            exc_info = sys.exc_info()
            etype, evalue = None, None
            if exc_info is not None:
                etype, evalue = sys.exc_info()[0:2]
                if etype in [SyntaxError,]:
                    err = evalue
                err = str(evalue)
            else:
                err = "No Exception"
            if self.show_reconnect_error_log:
                logger.log("{}: Unable to connect to {}:{} ({}: {}), retrying every {}ms".format(
                    self.origin,
                    self.address[0],
                    self.address[1],
                    etype.__name__,
                    err,
                    next_try)
                )
                self.show_reconnect_error_log = False
            Timer(self.event_loop, next_try, self.try_connect).start()
        else:
            super().handle_error()


    def try_connect(self):
        if not self.connected and not self.connecting:
            self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.connect(self.address)




class PacketClientSocketChannel(ClientSocketChannel):

    def __init__(self, event_loop, origin, remote):
        super().__init__(event_loop, origin, remote)
        self.buffer = bytes()
        self.packet = None


    def handle_read(self):
        if self.bytes_available() == 0:
            # Socket is closing
            self.recv(0)
        else:
            self.event_loop.handle_read(self)




class TurretChannel(asyncore.file_dispatcher):

    def __init__(self, serial_port_path, serial_port_speed, eventloop):
        self.port = serial.PosixPollSerial(serial_port_path, serial_port_speed, timeout = 0)
        self.port.nonblocking()
        super().__init__(self.port)
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
        num_sent = super().send(self.out_buffer[:512])
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
                if packet_type in [ packets.TurretDetect.TYPE,
                                    packets.TurretInit.TYPE,
                                    packets.TurretDistances.TYPE,
                                    packets.TurretBoot.TYPE ]:
                    self.synchronized = True
                    self.buffer = data
                    self.packet = packets.create_packet(self.buffer)
                    break
        try:
            self.eventloop.handle_read(self)
        except KeyError as e:
            logger.log("Turret channel is desynchronized. Resynchronizing. Unknown packet type: {}".format(e.args[0]))
            self.synchronized = False


    def close(self):
        self.port.close()
        asyncore.file_dispatcher.close(self)


    def handle_close(self):
        logger.log("Closing Turret channel")




class PicControlChannel(PacketClientSocketChannel):

    def __init__(self, event_loop):
        self.first_connection = True
        super().__init__(event_loop, "PIC", (REMOTE_IP, REMOTE_PORT))


    def handle_connect(self):
        super().handle_connect()
        if self.first_connection:
            self.first_connection = False
            self.event_loop.on_turret_boot(None)
            statemachine.StateMachine(self.event_loop, self.event_loop.state_machine_name)
            self.event_loop.send_packet(packets.ControllerReady())


class PicLogChannel(ClientSocketChannel):

    def __init__(self, event_loop):
        super().__init__(event_loop, "PLG", (REMOTE_IP, REMOTE_LOG_PORT))
        self.buffer = bytes()


    def handle_read(self):
        try:
            self.buffer += self.recv(self.bytes_available())
            while True:
                i = self.buffer.find(b"\n")
                if i != -1:
                    logger.log(self.buffer[:i].rstrip().encode("string_escape"), "PIC")
                    self.buffer = self.buffer[i + 1:]
                else:
                    break
        except:
            pass




class InterbotServer(asyncore.dispatcher):

    def __init__(self, event_loop):
        self.event_loop = event_loop
        try:
            asyncore.dispatcher.__init__ (self)
            self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
            self.set_reuse_addr()
            self.bind(("", MAIN_INTERBOT_PORT))
            self.listen(5)
            self.event_loop.interbot_channel = self
            logger.log("Starting interbot server on port {}".format(MAIN_INTERBOT_PORT))
        except Exception as e:
            logger.log("Unable to start interbot server on port {}".format(MAIN_INTERBOT_PORT))


    def handle_accepted(self, sock, addr):
        self.event_loop.interbot_manager.on_connect()
        self.event_loop.interbot_channel = InterbotControlChannel(self.event_loop, "TMM", sock)



class InterbotControlChannel(PacketClientSocketChannel):

    def handle_connect(self):
        super().handle_connect()
        self.event_loop.interbot_manager.on_connect()


    def handle_close(self):
        super().handle_close()
        self.event_loop.interbot_manager.on_disconnect()



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




class EventLoop(object):

    def __init__(self, state_machine_name, webserver_port, interbot_enabled = True):
        self.pic_control_channel = None
        self.pic_log_channel = None
        self.turret_channel = None
        self.interbot_channel = None
        self.interbot_server = None
        self.web_server = None
        self.robot = robot.Robot(self)
        self.fsms = []
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
        self.interbot_enabled = interbot_enabled

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
            leds.driver.heartbeat_tick()


    def on_start(self, packet):
        self.is_match_started = True
        self.start_date = datetime.datetime.now()
        Timer(self, MATCH_DURATION_MS, self.on_end_of_match).start()
        Timer(self, BREWERY_LIFETIME_MS, self.stop).start()


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
        for fsm in self.fsms:
            fsm.switch_to_end_of_match()


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
            self.pic_control_channel.send(buffer)
        elif packet.TYPE < packets.SIMULATOR_RANGE_END:
            self.pic_control_channel.send(buffer)
        elif packet.TYPE < packets.INTERBOT_RANGE_END:
            if self.interbot_channel :
                self.interbot_channel.send(buffer)
        elif packet.TYPE < packets.INTERNAL_RANGE_END:
            self.enqueue_packet(packet)
            self.process_packets_and_dispatch()


    def start(self):
        leds.orange.off()
        leds.green.on()

        if self.interbot_enabled :
            if IS_MAIN_ROBOT:
                InterbotServer(self)
            else:
                self.interbot_channel = InterbotControlChannel(self, "TMM", (MAIN_INTERBOT_IP, MAIN_INTERBOT_PORT))
        self.web_server = asyncwsgiserver.WsgiServer("", self.webserver_port, nanow.Application(webinterface.BHWeb(self)))
        if SERIAL_PORT_PATH is not None:
            try:
                self.turret_channel = TurretChannel(SERIAL_PORT_PATH, SERIAL_PORT_SPEED, self)
            except serial.SerialException:
                logger.log("Unable to open serial port {}".format(SERIAL_PORT_PATH))
                self.turret_channel = None
        logger.log("Starting brewery with state machine '{}'".format(self.state_machine_name))
        self.pic_log_channel = PicLogChannel(self)
        self.pic_control_channel = PicControlChannel(self)

        if self.colordetector :
            self.colordetector.on_startup()

        while not self.stopping:
            asyncore.loop(EVENT_LOOP_TICK_RESOLUTION_S, True, None, 1)
            for fsm in self.fsms:
                fsm.on_timer_tick()
            while len(self.timers) != 0:
                if not self.timers[0].check_timeout():
                    break


    def enqueue_packet(self, packet):
        self.packet_queue.appendleft(packet)


    def process_packets_and_dispatch(self):
        while self.packet_queue :
            packet = self.packet_queue.pop()
            self.dispatch(packet)


    def dispatch(self, packet):
        packet.dispatch(self)
        packet.dispatch(self.robot)
        packet.dispatch(self.opponent_detector)
        packet.dispatch(self.map)
        packet.dispatch(self.interbot_manager)

        for fsm in self.fsms:
            packet.dispatch(fsm)


    def stop(self):
        logger.log("Stopping...")
        self.stopping = True
        if self.turret_channel is not None:
            self.turret_channel.close()
        if self.pic_control_channel is not None:
            self.pic_control_channel.close()
        if self.pic_log_channel is not None:
            self.pic_log_channel.close()
        if self.interbot_channel is not None:
            self.interbot_channel.close()
        if self.interbot_server is not None:
            self.interbot_server.close()
        if self.colordetector is not None:
            self.colordetector.quit()

