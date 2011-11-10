#!/usr/bin/env python
# encoding: utf-8

import os
import random

from PyQt4.QtCore import *

import packets
from definitions import *




class RobotController(object):

    def __init__(self, team, game_controller, view, robot_layer, robot_trajectory_layer):
        self.team = team
        self.game_controller = game_controller
        self.view = view
        self.robot_layer = robot_layer
        self.robot_trajectory_layer = robot_trajectory_layer

        self.process = None
        self.socket = None
        self.incoming_packet_buffer = ""
        self.incoming_packet = None
        self.ready = False
        self.goto_packet = None
        self.goto_packet_point_index = 0

        self.resettle_count = 0

        self.robot_layer.robot.movement_finished.connect(self.movement_finished)


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
            self.socket.disconnected.disconnect(self.shutdown)
            self.process.terminate()
            self.process.waitForFinished()
            self.process = None
            self.socket = None
            self.ready = False


    def connected(self, socket):
        self.socket = socket
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
                    self.on_packet(packet)
                    self.read_packet()


    def on_packet(self, packet):
        if isinstance(packet, packets.ControllerReady):
            self.try_device_ready()
        elif isinstance(packet, packets.Goto):
            self.goto_packet = packet
            self.goto_packet_point_index = 0
            self.send_packet(packets.GotoStarted())
            self.process_goto()
        elif isinstance(packet, packets.KeepAlive):
            pass
        elif isinstance(packet, packets.PositionControlConfig):
            self.send_packet(packet)
        elif isinstance(packet, packets.Stop):
            self.stop()
        elif isinstance(packet, packets.Resettle):
            if packet.axis == AXIS_X:
                self.robot_layer.set_x(packet.position)
            elif packet.axis == AXIS_Y:
                self.robot_layer.set_y(packet.position)
            self.robot_layer.set_rotation(packet.angle)
            self.send_packet(packet)
            self.resettle_count += 1
            if self.resettle_count == 2:
                self.ready = True
                self.game_controller.try_start()
        elif isinstance(packet, packets.StorePiece1):
            self.send_packet(packet)
        elif isinstance(packet, packets.StorePiece2):
            self.send_packet(packet)
        elif isinstance(packet, packets.StorePiece3):
            self.send_packet(packet)
        elif isinstance(packet, packets.ReleasePiece):
            self.send_packet(packet)
        elif isinstance(packet, packets.OpenNippers):
            self.send_packet(packet)
        elif isinstance(packet, packets.CloseNippers):
            self.send_packet(packet)
        elif isinstance(packet, packets.SimulatorData):
            self.view.handle_led(packet.leds)
        elif isinstance(packet, packets.Deployment):
            self.send_packet(packet)
        elif isinstance(packet, packets.EnableLateralSensors):
            self.send_packet(packet)
        elif isinstance(packet, packets.DisableLateralSensors):
            self.send_packet(packet)
        elif isinstance(packet, packets.OpenMandibles):
            self.send_packet(packet)
        elif isinstance(packet, packets.CloseMandibles):
            self.send_packet(packet)
        else :
            print "Unhandled packet type : ",packet.__class__.__name__


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


    def movement_finished(self):
        self.goto_packet_point_index += 1
        self.process_goto()


    def process_goto(self):
        if self.goto_packet != None:
            # direction is ignored for the moment
            if self.goto_packet_point_index != len(self.goto_packet.points):
                point = self.goto_packet.points[self.goto_packet_point_index]
                if self.goto_packet.direction == DIRECTION_BACKWARD:
                    point.angle += math.pi
                if self.goto_packet.movement == MOVEMENT_ROTATE:
                    self.robot_layer.robot_rotation(point.angle)
                elif self.goto_packet.movement == MOVEMENT_LINE:
                    self.robot_layer.robot_line(point.x, point.y)
                elif self.goto_packet.movement == MOVEMENT_MOVE:
                    if point.angle != None:
                        self.robot_layer.robot_move(point.x, point.y, point.angle)
                    else:
                        self.robot_layer.robot_line(point.x, point.y)
            else:
                self.send_goto_finished(REASON_DESTINATION_REACHED)


    def pause(self):
        if self.robot_layer.move_animation.state() == QAbstractAnimation.Running:
            self.robot_layer.move_animation.pause()

    def stop(self):
        if self.robot_layer.move_animation.state() == QAbstractAnimation.Running:
            self.robot_layer.move_animation.stop()


    def resume(self):
        if self.robot_layer.move_animation.state() == QAbstractAnimation.Paused:
            self.robot_layer.move_animation.resume()


    def send_goto_finished(self, reason):
        self.goto_packet = None
        packet = packets.GotoFinished()
        packet.reason = reason
        packet.current_pose = self.robot_layer.get_pose()
        packet.current_point_index = self.goto_packet_point_index
        self.send_packet(packet)


    def send_opponent_detected(self):
        packet = packets.TurretDetect()
        packet.angle = math.pi
        self.send_packet(packet)
