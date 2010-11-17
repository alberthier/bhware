#!/usr/bin/env python
# encoding: utf-8

import os

from PyQt4.QtCore import *

import packets
from robotview import *



class RobotController(object):

    def __init__(self, team, view, scene):
        self.team = team
        self.view = view
        self.scene = scene
        self.field_item = None
        self.process = None
        self.socket = None
        self.incoming_packet_buffer = ""
        self.incoming_packet = None


    def is_started(self):
        return self.process != None


    def is_connected(self):
        return self.socket != None


    def setup(self):
        if self.process == None:
            self.view.clear()
            brewery = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))), "brewery", "brewery.py")
            self.process = QProcess()
            self.process.setReadChannelMode(QProcess.MergedChannels)
            self.process.readyRead.connect(self.read_output)
            self.process.start(brewery, [])


    def stop(self):
        if self.process != None:
            self.process.terminate()
            self.process.waitForFinished()
            self.process = None
            self.socket = None


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
        self.incomming_packet_buffer += self.socket.readAll()
        if self.incomming_packet == None:
            self.incomming_packet = packets.create_packet(self.incomming_packet_buffer)
        if len(self.incomming_packet_buffer) >= self.incomming_packet.MAX_SIZE:
            buf = self.incomming_packet_buffer[:self.incomming_packet.MAX_SIZE]
            self.incomming_packet_buffer = self.incomming_packet_buffer[self.incomming_packet.MAX_SIZE:]
            packet = self.incomming_packet
            self.incomming_packet = None
            packet.deserialize(buf)
            self.on_packet(packet)


    def on_packet(self, packet):
        if isinstance(packet, packets.ControllerReady):
            packet = packets.DeviceReady()
            packet.team = self.team
            self.send_packet(packet)
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
            pass


    def send_packet(self, packet):
        if self.is_connected():
            self.socket.write(packet.serialize())

