# encoding: utf-8

import asyncore
import bisect
import collections
import ctypes
import datetime
import errno
import fcntl
import socket
import struct
import subprocess
import sys
import termios

import asyncwsgiserver
import builder
import graphmap
import leds
import logger
import packets
import robot
import statemachine
import webinterface

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
            try :
                self.connect(self.address)
            except :
                self.handle_error()




class BinaryPacketHandler:

    def __init__(self):
        self.buffer = bytes()
        self.packet = None


    def write_packet(self, packet):
        self.send(packet.serialize())


    def read_packet(self):
        while True :
            if self.packet is None:
                try:
                    b = self.bytes_available()
                    if b == 0:
                        return
                    received_data = self.recv(1)
                    if len(received_data) == 0:
                        return
                    self.buffer += received_data
                    self.packet = packets.create_packet(self.buffer)
                except socket.error as err:
                    if err.errno in [errno.EAGAIN, errno.EINTR]:
                        return
                    logger.log_exception(err)
                    return
            else:
                try:
                    try:
                        if self.bytes_available() < self.packet.MAX_SIZE - 1:
                            return
                        received_data = self.recv(self.packet.MAX_SIZE - len(self.buffer))
                        if len(received_data) == 0:
                            return
                        self.buffer += received_data
                    except socket.error as err:
                        if err.errno in [errno.EAGAIN, errno.EINTR]:
                            return
                        logger.log_exception(err)
                        return
                    if len(self.buffer) == self.packet.MAX_SIZE:
                        # A complete packet has been received, notify the state machine
                        self.packet.deserialize(self.buffer)
                        self.event_loop.process(self, self.packet)
                        self.buffer = bytes()
                        self.packet = None
                except Exception as e:
                    self.packet = None
                    self.buffer = bytes()
                    logger.log_exception(e)




class PacketClientSocketChannel(ClientSocketChannel, BinaryPacketHandler):

    def __init__(self, event_loop, origin, remote):
        ClientSocketChannel.__init__(self, event_loop, origin, remote)
        BinaryPacketHandler.__init__(self)


    def handle_read(self):
        try :
            if self.bytes_available() == 0:
                # Socket is closing
                self.recv(0)
            else:
                self.read_packet()
        except Exception as e :
            logger.log_exception(e)




class FileDispatcherWithSend(asyncore.file_dispatcher):

    def __init__(self, fd, map = None):
        super().__init__(fd, map)
        self.out_buffer = bytes()


    def bytes_available(self):
        available = ctypes.c_int()
        fcntl.ioctl(self.socket.fileno(), termios.FIONREAD, available)
        return available.value


    def writable(self):
        return self.connected and len(self.out_buffer) != 0


    def initiate_send(self):
        num_sent = super().send(self.out_buffer[:4096])
        self.out_buffer = self.out_buffer[num_sent:]


    def handle_write(self):
        self.initiate_send()


    def send(self, data):
        self.out_buffer += data
        self.initiate_send()




class TurretChannel(FileDispatcherWithSend, BinaryPacketHandler):

    def __init__(self, event_loop, serial_port_path, serial_port_speed):
        self.port = serial.PosixPollSerial(serial_port_path, serial_port_speed, timeout = 0)
        self.port.nonblocking()
        FileDispatcherWithSend.__init__(self, self.port)
        BinaryPacketHandler.__init__(self)
        self.event_loop = event_loop
        self.synchronized = False
        self.origin = "TUR"


    def bytes_available(self):
        return self.port.inWaiting()


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
            self.read_packet()
        except Exception as e:
            logger.log("Turret channel is desynchronized. Resynchronizing. Unknown packet type: {}".format(e.args[0]))
            self.synchronized = False


    def close(self):
        self.port.close()
        super().close()


    def handle_close(self):
        logger.log("Closing Turret channel")




class ProcessChannel:

    def __init__(self, event_loop, origin, cmd):
        self.process = subprocess.Popen(cmd, bufsize = 0, stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
        self.event_loop = event_loop
        self.origin = origin
        self.stdin = FileDispatcherWithSend(self.process.stdin)
        self.stdout = asyncore.file_dispatcher(self.process.stdout)
        self.stdout.handle_read = self.handle_read_stdout
        self.stdout.writable = lambda: False
        self.stderr = asyncore.file_dispatcher(self.process.stderr)
        self.stderr.handle_read = self.handle_read_stderr
        self.stderr.writable = lambda: False
        self.stdout_buffer = bytes()
        self.stderr_buffer = bytes()
        self.connected = True


    def write_packet(self, packet):
        self.send(packet.serialize_as_text())


    def send(self, data):
        self.stdin.send(bytes(data + "\n", "utf-8"))


    def handle_read_stdout(self):
        result = []
        self.stdout_buffer = self.handle_read_channel(self.stdout_buffer, self.stdout, result)
        for code in result:
            try:
                packet = eval(code)
                if type(packet) == tuple :
                    packet = packet[1]
                self.event_loop.process(self, packet)
            except Exception as e:
                logger.log("Unable to evaluate the process answer: '{}'".format(code))
                logger.log_exception(e)


    def handle_read_stderr(self):
        result = []
        self.stderr_buffer = self.handle_read_channel(self.stderr_buffer, self.stderr, result)
        for log in result:
            logger.log(log)


    def handle_read_channel(self, buffer, channel, result):
        chunk = channel.recv(4096)
        buffer += chunk
        while True:
            i = buffer.find(b'\n')
            if i == -1:
                break
            else:
                result.append(str(buffer[:i], "utf-8"))
                buffer = buffer[i + 1:]
        return buffer


    def close(self):
        self.process.kill()




class PicControlChannel(PacketClientSocketChannel):

    def __init__(self, event_loop):
        self.first_connection = True
        super().__init__(event_loop, "PIC", (REMOTE_IP, REMOTE_PORT))


    def handle_connect(self):
        super().handle_connect()
        if self.first_connection:
            self.first_connection = False
            self.event_loop.on_turret_boot(None)
            Timer(self.event_loop, 0, self.ready).start()


    def ready(self):
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
                    msg = self.buffer[:i].rstrip().decode("unicode_escape")
                    msg = msg.replace("\0", "")
                    logger.log(msg, "PIC")
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
        self.event_loop.interbot_channel = InterbotControlChannel(self.event_loop, "IBT", sock)
        self.event_loop.send_packet(packets.InterbotConnected())




class InterbotControlChannel(PacketClientSocketChannel):

    def handle_connect(self):
        super().handle_connect()
        self.event_loop.send_packet(packets.InterbotConnected())


    def handle_close(self):
        super().handle_close()
        self.event_loop.send_packet(packets.InterbotDisconnected())




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
        self.stopping = False
        self.is_match_started = False
        self.map = graphmap.Map(self)
        self.timers = []
        self.last_ka_date = datetime.datetime.now()
        self.start_date = None
        self.packet_queue = collections.deque()
        self.interbot_enabled = interbot_enabled

        if IS_HOST_DEVICE_ARM and IS_MAIN_ROBOT:
            folder = os.path.join(os.path.dirname(__file__), "colordetector")
            cmds = ["g++", "-Wall", "-O2", "-o", "colordetector", "colordetector.cpp", "-l", "opencv_core", "-l", "opencv_highgui", "-l", "opencv_imgproc"]
            builder.Builder("colordetector.cpp", "colordetector", cmds, folder).build()
            self.colordetector = ProcessChannel(self, "CAM", [os.path.join(folder, "colordetector"), "-q", "1"])
        else:
            self.colordetector = None


    def on_keep_alive(self, packet):
        now = datetime.datetime.now()
        if (now - self.last_ka_date).total_seconds() > KEEP_ALIVE_MINIMUM_AGE_S:
            self.last_ka_date = now
            self.send_packet(packet)
            leds.driver.heartbeat_tick()


    def on_device_ready(self, packet):
        logger.set_team(packet.team)


    def on_start(self, packet):
        self.is_match_started = True
        self.start_date = datetime.datetime.now()


    def on_turret_boot(self, packet):
        if self.turret_channel is not None:
            packet = packets.TurretInit()
            packet.mode = TURRET_INIT_MODE_WRITE
            packet.short_distance = TURRET_SHORT_DISTANCE_DETECTION_ID
            packet.long_distance = TURRET_LONG_DISTANCE_DETECTION_ID
            self.send_packet(packet)


    def process(self, channel, packet):
        logger.log_packet(packet, channel.origin)
        packet.dispatch(self)
        self.packet_queue.appendleft((packet, None))
        self.process_packets_and_dispatch()


    def process_packets_and_dispatch(self):
        while self.packet_queue:
            packet, sender = self.packet_queue.pop()
            packet.dispatch(self.robot)
            packet.dispatch(self.map)
            for fsm in self.fsms:
                # avoid sending internal packets to the emitter, only other FSMs will receive the packet
                if fsm is not sender:
                    packet.dispatch(fsm)


    def get_elapsed_match_time(self):
        if self.start_date is None:
            return 0.0
        delta = datetime.datetime.now() - self.start_date
        return delta.total_seconds()

    def get_remaining_match_time(self):
        return MATCH_DURATION_MS / 1000.0 - self.get_elapsed_match_time()


    def send_packet(self, packet, sender = None):
        """
            packet: packet to send
            sender: the fsm which sent the packet
        """
        if self.do_send_packet(packet, packets.TURRET_RANGE_START, packets.TURRET_RANGE_END, self.turret_channel):
            return
        elif self.do_send_packet(packet, packets.PIC32_RANGE_START, packets.PIC32_RANGE_END, self.pic_control_channel):
            return
        elif IS_HOST_DEVICE_PC and self.do_send_packet(packet, packets.SIMULATOR_RANGE_START, packets.SIMULATOR_RANGE_END, self.pic_control_channel):
            return
        elif self.do_send_packet(packet, packets.COLORDET_RANGE_START, packets.COLORDET_RANGE_END, self.colordetector):
            return
        elif self.do_send_packet(packet, packets.INTERBOT_RANGE_START, packets.INTERBOT_RANGE_END, self.interbot_channel):
            return
        elif packet.TYPE < packets.INTERNAL_RANGE_END:
            # add the packet to send to the list of packets to dispatch
            logger.log_packet(packet, "ARM" + (":" + sender.name if sender is not None else ""))
            self.packet_queue.appendleft((packet, sender))
            Timer(self, 0, self.process_packets_and_dispatch).start()


    def do_send_packet(self, packet, packet_range_start, packet_range_end, channel):
        if packet.TYPE >= packet_range_start and packet.TYPE < packet_range_end:
            if channel is not None and channel.connected:
                logger.log_packet(packet, "ARM")
                channel.write_packet(packet)
            else:
                logger.log("Channel for packet {} doesn't exist or isn't connected".format(packet.name))
            return True
        return False


    def start(self):
        leds.orange.off()
        leds.green.on()

        if self.interbot_enabled :
            if IS_MAIN_ROBOT:
                InterbotServer(self)
            else:
                self.interbot_channel = InterbotControlChannel(self, "IBT", (MAIN_INTERBOT_IP, MAIN_INTERBOT_PORT))
        self.web_server = asyncwsgiserver.WsgiServer("", self.webserver_port, webinterface.WebInterface(self))
        if SERIAL_PORT_PATH is not None:
            try:
                self.turret_channel = TurretChannel(self, SERIAL_PORT_PATH, SERIAL_PORT_SPEED)
            except serial.SerialException:
                logger.log("Unable to open serial port {}".format(SERIAL_PORT_PATH))
                self.turret_channel = None
        self.pic_log_channel = PicLogChannel(self)
        self.pic_control_channel = PicControlChannel(self)

        logger.log("Starting brewery with state machine '{}'".format(self.state_machine_name))
        statemachine.StateMachine(self, self.state_machine_name)

        while not self.stopping:
            asyncore.loop(EVENT_LOOP_TICK_RESOLUTION_S, True, None, 1)
            for fsm in self.fsms:
                fsm.on_timer_tick()
            while len(self.timers) != 0:
                if not self.timers[0].check_timeout():
                    break


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
            self.colordetector.close()

