#!/usr/bin/env python
# encoding: utf-8

import os

from PyQt4.QtCore import *

import packets




class RobotController(object):

    def __init__(self, team, view):
        self.team = team
        self.view = view
        self.process = None
        self.socket = None
        self.incomming_packet_buffer = ""
        self.incomming_packet = None


    def is_started(self):
        return self.process != None


    def is_connected(self):
        return self.socket != None


    def setup(self):
        if self.process == None:
            self.view.clear()
            brewery = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "brewery", "brewery.py")
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
            self.incomming_packet = None
            packet = self.incomming_packet
            packet.deserialize(buf)
            self.on_packet(packet)


    def on_packet(self, packet):
        print packet
        # packet = packets.DeviceReady()
        # packet.team = self.team
        # self.send_packet(packet)



    def send_packet(self, packet):
        if self.is_connected():
            self.socket.write(packet.serialize())
