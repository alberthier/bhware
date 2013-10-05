# encoding: utf-8

import os
import random

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtNetwork import *

import packets
import binarizer
from definitions import *

import simulatorfieldview




class RobotController(object):

    def __init__(self, game_controller, main_window, output_view, offset):
        self.game_controller = game_controller
        self.output_view = output_view
        self.offset = offset

        self.team = None
        self.team_name = "?"
        self.team_color = "#555753"
        self.is_main = None
        self.fsm_name = None

        self.layers = []
        self.robot_layer = simulatorfieldview.RobotLayer(main_window.field_view_controller, self)
        self.layers.append(self.robot_layer)
        self.trajectory_layer = simulatorfieldview.RobotTrajectoryLayer(main_window.field_view_controller, self)
        self.layers.append(self.trajectory_layer)
        self.grid_routing_layer = simulatorfieldview.GridRoutingLayer(main_window.field_view_controller, self)
        self.layers.append(self.grid_routing_layer)
        self.graph_routing_layer = simulatorfieldview.GraphRoutingLayer(main_window.field_view_controller, self)
        self.layers.append(self.graph_routing_layer)

        self.process = None
        self.socket = None
        self.incoming_packet_buffer = ""
        self.incoming_packet = None
        self.ready = False
        self.resettle_count = 0
        self.stop_requested = False


    def is_process_started(self):
        return self.process != None


    def is_connected(self):
        return self.socket != None


    def is_ready(self):
        return self.ready


    def hide_all(self):
        for layer in self.layers:
            layer.hide()


    def setup(self, team, is_main, debug_host = None, debug_port=0, fsm_name=None):
        if self.process == None:
            self.incoming_packet_buffer = ""
            self.incoming_packet = None
            self.resettle_count = 0
            self.team = team
            self.is_main = is_main
            self.fsm_name = fsm_name

            if team == TEAM_YELLOW:
                self.team_name = "yellow"
                self.team_color = TEAM_COLOR_YELLOW
            else:
                self.team_name = "red"
                self.team_color = TEAM_COLOR_RED
            if is_main:
                self.team_name = "Main " + self.team_name
            else:
                self.team_name = "Secondary " + self.team_name
                self.team_color = QColor(self.team_color).lighter().name()

            self.output_view.setup(team, is_main)
            for layer in self.layers:
                layer.show()
                layer.setup()

            brewery = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))), "brewery", "brewery.py")
            self.process = QProcess()
            self.process.setReadChannelMode(QProcess.MergedChannels)
            self.process.readyRead.connect(self.read_output)
            args = []



            if self.is_main:
                args.append("--hostname")
                args.append("sheldon")

            else:
                args.append("--hostname")
                args.append("leonard")


            if debug_host :
                args.append("--pydev-debug")
                args.append(debug_host)
                args.append(str(debug_port))

            if self.fsm_name :
                args.append(self.fsm_name)
            else :
                if is_main :
                    args.append('sheldon')
                else :
                    args.append('leonard')



            self.process.start(brewery, args)


    def shutdown(self):
        if self.process != None:
            if self.socket != None:
                self.socket.disconnected.disconnect(self.shutdown)
            self.process.terminate()
            self.process.waitForFinished()
            self.process = None
            self.socket = None
            self.ready = False


    def connected(self, socket):
        self.socket = socket
        self.socket.setSocketOption(QAbstractSocket.LowDelayOption, 1)
        self.socket.disconnected.connect(self.shutdown)
        self.socket.readyRead.connect(self.read_packet)


    def read_output(self):
        while self.process.canReadLine():
            log = str(self.process.readLine(), "utf-8").rstrip()
            self.output_view.add_log(log)


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
                    packet.dispatch(self.trajectory_layer)
                    packet.dispatch(self.grid_routing_layer)
                    packet.dispatch(self.graph_routing_layer)


    def on_enable_anti_blocking(self, packet):
        self.send_packet(packet)


    def on_disable_anti_blocking(self, packet):
        self.send_packet(packet)


    def on_controller_ready(self, packet):
        self.try_device_ready()


    def on_position_control_config(self, packet):
        self.send_packet(packet)


    def on_stop(self, packet):
        self.stop()


    def on_resettle(self, packet):
        self.send_packet(packet)
        self.resettle_count += 1
        if self.resettle_count == 2:
            self.ready = True
            self.game_controller.try_start()


    def on_bottom_holder(self, packet):
        self.send_packet(packet)


    def on_lifter(self, packet):
        self.send_packet(packet)


    def on_gripper(self, packet):
        self.send_packet(packet)


    def on_top_holder(self, packet):
        self.send_packet(packet)


    def on_pump(self, packet):
        self.send_packet(packet)


    def on_stop_all(self, packet):
        self.send_packet(packet)


    def on_simulator_data(self, packet):
        self.output_view.handle_led(packet.leds)


    def on_simulator_fetch_colors(self, packet):
        detect = []
        if self.team == TEAM_YELLOW:
            upper = self.game_controller.game_elements_layer.upper_candles
            lower = self.game_controller.game_elements_layer.lower_candles
        else:
            upper = reversed(self.game_controller.game_elements_layer.upper_candles)
            lower = reversed(self.game_controller.game_elements_layer.lower_candles)
        for i, candle in enumerate(upper):
            detect.append(binarizer.StructInstance(index = i + 101, detect = candle.color == self.team_color))
        for i, candle in enumerate(lower):
            detect.append(binarizer.StructInstance(index = i + 1, detect = candle.color == self.team_color))
        self.send_packet(packets.SimulatorFetchColors(colors = detect))


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
        if self.robot_layer.robot.item is not None:
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


    def pause(self):
        if self.robot_layer.robot.move_animation.state() == QAbstractAnimation.Running:
            self.robot_layer.robot.pause_animation()


    def stop(self):
        if self.robot_layer.robot.move_animation.state() == QAbstractAnimation.Running:
            self.robot_layer.robot.stop_animation()


    def resume(self):
        if self.robot_layer.robot.move_animation.state() == QAbstractAnimation.Paused:
            self.robot_layer.robot.resume_animation()
