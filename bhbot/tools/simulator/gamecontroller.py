# encoding: utf-8

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtNetwork import *

from definitions import *

from robotcontroller import *




class GameController(object):

    def __init__(self, main_window):
        self.server = QTcpServer()
        self.server.newConnection.connect(self.brewery_connected)
        self.server.listen(QHostAddress.Any, REMOTE_PORT)
        self.blue_robot = RobotController(TEAM_BLUE,
                                            self,
                                            main_window.blue_robot_view,
                                            main_window.field_controller.blue_robot_layer,
                                            main_window.field_controller.blue_robot_trajectrory_layer,
                                            main_window.field_controller.blue_robot_routing_layer,
                                            main_window.field_controller.blue_robot_routing_graph_layer)
        self.red_robot = RobotController(TEAM_RED,
                                         self,
                                         main_window.red_robot_view,
                                         main_window.field_controller.red_robot_layer,
                                         main_window.field_controller.red_robot_trajectrory_layer,
                                         main_window.field_controller.red_robot_routing_layer,
                                         main_window.field_controller.red_robot_routing_graph_layer)
        self.game_elements_layer = main_window.field_controller.game_elements_layer
        self.main_bar = main_window.main_bar
        self.main_bar.reload.clicked.connect(self.setup)
        self.main_bar.start_pause.clicked.connect(self.start_pause)
        self.main_bar.stop.clicked.connect(self.user_stop)
        QCoreApplication.instance().lastWindowClosed.connect(self.stop)
        self.start_requested = False
        self.started = False
        self.setting_up = False
        self.time = MATCH_DURATION_MS
        self.expected_teams = []
        self.keep_alive_timer = QTimer()
        self.keep_alive_timer.setInterval(KEEP_ALIVE_DELAY_MS)
        self.keep_alive_timer.timeout.connect(self.timer_tick)


    def setup(self):
        if not self.start_requested and not self.setting_up:
            self.user_stop()
        if not self.red_robot.is_process_started() and not self.blue_robot.is_process_started():
            self.game_elements_layer.setup()
            self.time = MATCH_DURATION_MS
            self.main_bar.chronograph.setText(str(round(self.time/1000.0, 1)))
            self.setting_up = True
            self.expected_teams = []
        if not self.red_robot.is_process_started() and self.main_bar.red_robot.isChecked():
            self.expected_teams.append(TEAM_RED)
            self.red_robot.robot_layer.use_advanced_dynamics(self.main_bar.advanced_dynamics.isChecked())
            self.red_robot.setup()
        elif not self.blue_robot.is_process_started() and self.main_bar.blue_robot.isChecked():
            self.expected_teams.append(TEAM_BLUE)
            self.blue_robot.robot_layer.use_advanced_dynamics(self.main_bar.advanced_dynamics.isChecked())
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
                self.red_robot.pause()
                self.blue_robot.pause()
            else:
                self.main_bar.set_icon(self.main_bar.start_pause, "pause")
                self.keep_alive_timer.start()
                self.red_robot.resume()
                self.blue_robot.resume()
        elif not self.start_requested:
            self.start_requested = True
            self.setup()
        elif not self.setting_up:
            self.start_requested = True
            self.try_start()


    def user_stop(self):
        self.stop()
        self.red_robot.reset()
        self.blue_robot.reset()


    def stop(self):
        self.started = False
        self.start_requested = False
        self.main_bar.set_icon(self.main_bar.start_pause, "start")
        self.keep_alive_timer.stop()
        self.red_robot.shutdown()
        self.blue_robot.shutdown()


    def send_start_signal(self):
        self.started = True
        self.main_bar.set_icon(self.main_bar.start_pause, "pause")
        self.red_robot.send_start_signal()
        self.blue_robot.send_start_signal()


    def try_start(self):
        red = TEAM_RED in self.expected_teams and self.red_robot.is_ready() or not TEAM_RED in self.expected_teams
        blue = TEAM_BLUE in self.expected_teams and self.blue_robot.is_ready() or not TEAM_BLUE in self.expected_teams
        if self.start_requested and red and blue:
            self.start_requested = False
            self.send_start_signal()


    def brewery_connected(self):
        if TEAM_RED in self.expected_teams and not self.red_robot.is_connected():
            self.red_robot.connected(self.server.nextPendingConnection())
            self.setup()
        elif TEAM_BLUE in self.expected_teams and not self.blue_robot.is_connected():
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
            self.time -= KEEP_ALIVE_DELAY_MS
            self.main_bar.chronograph.setText(str(round(self.time/1000.0, 1)))
        if self.time == 0:
            self.stop()

