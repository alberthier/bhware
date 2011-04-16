#!/usr/bin/env python
# encoding: utf-8

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtNetwork import *

import config
from definitions import *

from robotcontroller import *




class GameController(object):

    def __init__(self, main_window):
        self.server = QTcpServer()
        self.server.newConnection.connect(self.brewery_connected)
        self.server.listen(QHostAddress.Any, config.remote_port)
        self.red_robot = RobotController(TEAM_RED, self, main_window.red_robot_dock.widget(), main_window.field_scene)
        self.blue_robot = RobotController(TEAM_BLUE, self, main_window.blue_robot_dock.widget(), main_window.field_scene)        
        self.trajectory_drawer = None
        self.main_bar = main_window.main_bar_dock.widget()
        self.main_bar.reload.clicked.connect(self.setup)
        self.main_bar.start_pause.clicked.connect(self.start_pause)
        self.main_bar.stop.clicked.connect(self.user_stop)
        QCoreApplication.instance().lastWindowClosed.connect(self.stop)
        self.start_requested = False
        self.started = False
        self.setting_up = False
        self.time = 900
        self.keep_alive_timer = QTimer()
        self.keep_alive_timer.setInterval(100)
        self.keep_alive_timer.timeout.connect(self.timer_tick)


    def setup(self):
        # self.trajectory_drawer = TrajectoryDrawer(main_window.field_scene, self.)
        if not self.start_requested and not self.setting_up:
            self.user_stop()
        if not self.red_robot.is_process_started():
            self.time = 900
            self.main_bar.chronograph.setText(str(round(self.time/10.0, 1)))
            self.setting_up = True
            self.red_robot.setup()
        elif not self.blue_robot.is_process_started():
            self.blue_robot.setup()
        else:
            self.setting_up = False
            self.keep_alive_timer.start()
            if self.start_requested:
                self.try_start()


    def start_pause(self):
        if self.started:
            if self.keep_alive_timer.isActive():
                self.main_bar.set_icon(self.main_bar.start_pause, "start")
                self.keep_alive_timer.stop()
            else:
                self.main_bar.set_icon(self.main_bar.start_pause, "pause")
                self.keep_alive_timer.start()
        elif not self.start_requested:
            self.start_requested = True
            self.setup()
        elif not self.setting_up:
            self.start_requested = True
            self.try_start()


    def user_stop(self):
        self.stop()
        self.red_robot.remove_field_item()
        self.blue_robot.remove_field_item()


    def stop(self):
        self.started = False
        self.start_requested = False
        self.main_bar.set_icon(self.main_bar.start_pause, "start")
        self.keep_alive_timer.stop()
        self.red_robot.stop()
        self.blue_robot.stop()


    def send_start_signal(self):
        self.started = True
        self.main_bar.set_icon(self.main_bar.start_pause, "pause")
        self.red_robot.send_start_signal()
        self.blue_robot.send_start_signal()


    def try_start(self):
        if self.start_requested and self.red_robot.is_ready() and self.blue_robot.is_ready():
            self.start_requested = False
            self.send_start_signal()


    def brewery_connected(self):
        if not self.red_robot.is_connected():
            self.red_robot.connected(self.server.nextPendingConnection())
            self.setup()
        elif not self.blue_robot.is_connected():
            self.blue_robot.connected(self.server.nextPendingConnection())
            self.setup()
        else:
            # Only two robots can play
            self.server.nextPendingConnection().close()
            return


    def timer_tick(self):
        self.red_robot.send_keep_alive()
        self.blue_robot.send_keep_alive()
        if self.started:
            self.time -= 1
            self.main_bar.chronograph.setText(str(round(self.time/10.0, 1)))
        if self.time == 0:
            self.stop()
