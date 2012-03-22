#!/usr/bin/env python
# encoding: utf-8

import os
import random

from PyQt4.QtCore import *
from PyQt4.QtNetwork import *

import packets
from definitions import *




class RobotController(object):

    def __init__(self, team, game_controller, view, robot_layer, robot_trajectory_layer, robot_routing_layer):
        self.team = team
        self.game_controller = game_controller
        self.view = view
        self.robot_layer = robot_layer
        self.robot_trajectory_layer = robot_trajectory_layer
        self.robot_routing_layer = robot_routing_layer
        robot_layer.robot_controller = self
        self.process = None
        self.socket = None
        self.incoming_packet_buffer = ""
        self.incoming_packet = None
        self.ready = False

        self.resettle_count = 0


    def is_process_started(self):
        return self.process != None


    def is_connected(self):
        return self.socket != None


    def is_ready(self):
        return self.ready


    def setup(self):
        if self.process == None:
            self.incoming_packet_buffer = ""
            self.incoming_packet = None
            self.resettle_count = 0

            self.view.clear()
            self.robot_layer.setup()
            self.robot_trajectory_layer.setup()

            brewery = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))), "brewery", "brewery.py")
            self.process = QProcess()
            self.process.setReadChannelMode(QProcess.MergedChannels)
            self.process.readyRead.connect(self.read_output)
            if self.team == TEAM_RED:
                port = "8000"
            else:
                port = "8001"
            self.process.start(brewery, ["--webserver-port", port])


    def reset(self):
        self.robot_layer.reset()


    def shutdown(self):
        if self.process != None:
            if self.socket != None:
                self.socket.disconnected.disconnect(self.shutdown)
            self.process.terminate()
            self.process.waitForFinished()
            self.process = None
            self.socket = None
            self.ready = False
            self.robot_layer.terminate()


    def connected(self, socket):
        self.socket = socket
        self.socket.setSocketOption(QAbstractSocket.LowDelayOption, 1)
        self.socket.disconnected.connect(self.shutdown)
        self.socket.readyRead.connect(self.read_packet)


    def read_output(self):
        while self.process.canReadLine():
            log = str(self.process.readLine()).rstrip()
            self.view.add_log(log)


    def read_packet(self):
        if self.socket != None:
            while self.socket.bytesAvailable() > 0 :
                if self.incoming_packet == None:
                    self.incoming_packet = packets.create_packet(self.socket.peek(1))
                if self.socket.bytesAvailable() >= self.incoming_packet.MAX_SIZE:
                    buf = self.socket.read(self.incoming_packet.MAX_SIZE)
                    packet = self.incoming_packet
                    self.incoming_packet = None
                    packet.deserialize(buf)
                    packet.dispatch(self)
                    packet.dispatch(self.robot_layer)
                    packet.dispatch(self.robot_trajectory_layer)
                    packet.dispatch(self.robot_routing_layer)


    def on_goto(self, packet):
        self.send_packet(packets.GotoStarted())


    def on_controller_ready(self, packet):
        self.try_device_ready()


    def on_position_control_config(self, packet):
        self.send_packet(packet)


    def on_stop(self, packet):
        self.stop()


    def on_sweeper_control(self, packet):
        self.send_packet(packet)


    def on_empty_tank_control(self, packet):
        self.send_packet(packet)


    def on_resettle(self, packet):
        self.send_packet(packet)
        self.resettle_count += 1
        if self.resettle_count == 2:
            self.ready = True
            self.game_controller.try_start()


    def on_simulator_data(self, packet):
        self.view.handle_led(packet.leds)


    def send_packet(self, packet):
        if self.is_connected():
            self.socket.write(packet.serialize())


    def try_device_ready(self):
        if random.choice([True, False]):
            # OK, ready
            self.send_ready()
        else:
            # Still busy
            packet = packets.DeviceBusy()
            packet.remote_device = REMOTE_DEVICE_SIMULATOR
            self.send_packet(packet)
            QTimer.singleShot(500, self.send_ready)


    def send_ready(self):
        packet = packets.DeviceReady()
        packet.team = self.team
        packet.remote_device = REMOTE_DEVICE_SIMULATOR
        self.send_packet(packet)


    def send_keep_alive(self):
        packet = packets.KeepAlive()
        packet.current_pose = self.robot_layer.get_pose()
        packet.match_started = self.game_controller.started
        packet.match_time = self.game_controller.time
        self.send_packet(packet)


    def send_start_signal(self):
        packet = packets.Start()
        packet.team = self.team
        self.send_packet(packet)


    def send_goto_finished(self, reason, current_point_index):
        self.goto_packet = None
        packet = packets.GotoFinished()
        packet.reason = reason
        packet.current_pose = self.robot_layer.get_pose()
        packet.current_point_index = current_point_index
        self.send_packet(packet)


    def send_opponent_detected(self):
        packet = packets.TurretDetect()
        packet.angle = math.pi
        self.send_packet(packet)


    def pause(self):
        if self.robot_layer.robot.move_animation.state() == QAbstractAnimation.Running:
            self.robot_layer.robot.move_animation.pause()


    def stop(self):
        if self.robot_layer.robot.move_animation.state() == QAbstractAnimation.Running:
            self.robot_layer.robot.move_animation.stop()


    def resume(self):
        if self.robot_layer.robot.move_animation.state() == QAbstractAnimation.Paused:
            self.robot_layer.robot.move_animation.resume()
