#!/usr/bin/env python
# encoding: utf-8

import os
import random

from PyQt4.QtCore import *

import packets
import trajectory
from robotview import *



class RobotController(object):

    def __init__(self, team, game_controller, view, scene):
        self.team = team
        self.game_controller = game_controller
        self.view = view
        self.scene = scene

        self.field_item = None
        self.process = None
        self.socket = None
        self.incoming_packet_buffer = ""
        self.incoming_packet = None
        self.robot_pose = None
        self.ready = False


    def is_process_started(self):
        return self.process != None


    def is_connected(self):
        return self.socket != None


    def is_ready(self):
        return self.ready


    def setup(self):
        if self.process == None:
            self.field_item = None
            self.incoming_packet_buffer = ""
            self.incoming_packet = None
            self.robot_pose = trajectory.Pose(0.0, 0.0, 0.0)

            self.view.clear()
            brewery = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))), "brewery", "brewery.py")
            self.process = QProcess()
            self.process.setReadChannelMode(QProcess.MergedChannels)
            self.process.readyRead.connect(self.read_output)
            self.process.start(brewery, [])


    def stop(self):
        if self.process != None:
            self.socket.disconnected.disconnect(self.stop)
            self.process.terminate()
            self.process.waitForFinished()
            self.process = None
            self.socket = None
            self.ready = False


    def remove_field_item(self):
        self.scene.removeItem(self.field_item)
        self.field_item = None


    def connected(self, socket):
        self.socket = socket
        self.socket.disconnected.connect(self.stop)
        self.socket.readyRead.connect(self.read_packet)

        self.field_item = GraphicsRobotItem()
        if self.team == TEAM_RED:
            self.field_item.setPos(0.0, 50.0)
            self.field_item.setRotation(90.0)
            angle = -90.0
        elif self.team == TEAM_BLUE:
            self.field_item.setRotation(90.0)
            self.field_item.setPos(3000.0, 50.0 + self.field_item.boundingRect().height())
            angle = 90.0
        self.scene.addItem(self.field_item)
        self.field_item.robot_rotation(angle)


    def read_output(self):
        while self.process.canReadLine():
            log = str(self.process.readLine()).rstrip()
            self.view.add_log(log)


    def read_packet(self):
        if self.socket != None:
            self.incoming_packet_buffer += self.socket.readAll()
            if self.incoming_packet == None:
                self.incoming_packet = packets.create_packet(self.incoming_packet_buffer)
            if len(self.incoming_packet_buffer) >= self.incoming_packet.MAX_SIZE:
                buf = self.incoming_packet_buffer[:self.incoming_packet.MAX_SIZE]
                self.incoming_packet_buffer = self.incoming_packet_buffer[self.incoming_packet.MAX_SIZE:]
                packet = self.incoming_packet
                self.incoming_packet = None
                packet.deserialize(buf)
                self.on_packet(packet)


    def on_packet(self, packet):
        if isinstance(packet, packets.ControllerReady):
            self.try_device_ready()
        elif isinstance(packet, packets.Goto):
            pass
        elif isinstance(packet, packets.KeepAlive):
            pass
        elif isinstance(packet, packets.PositionControlConfig):
            pass
        elif isinstance(packet, packets.Stop):
            pass
        elif isinstance(packet, packets.Resettle):
            pass
        elif isinstance(packet, packets.SimulatorData):
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
        self.ready = True
        self.game_controller.try_start()


    def send_keep_alive(self):
        packet = packets.KeepAlive()
        packet.current_pose = self.robot_pose
        packet.match_started = self.game_controller.started
        packet.match_time = self.game_controller.time
        self.send_packet(packet)


    def send_start_signal(self):
        packet = packets.Start()
        packet.team = self.team
        self.send_packet(packet)

