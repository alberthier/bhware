# encoding: utf-8

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtNetwork import *

from definitions import *

from robotcontroller import *




class GameController(object):

    def __init__(self, main_window, args):
        self.args = args
        self.server = QTcpServer()
        self.server.newConnection.connect(self.brewery_connected)
        self.server.listen(QHostAddress.Any, REMOTE_PORT)
        self.robot_a = RobotController(self, main_window, main_window.robot_a_output_view, 0)
        self.robot_b = RobotController(self, main_window, main_window.robot_b_output_view, 1)
        self.game_elements_layer = main_window.field_view_controller.game_elements_layer
        self.main_bar = main_window.main_bar
        self.main_bar.reload.clicked.connect(self.setup)
        self.main_bar.start_pause.clicked.connect(self.start_pause)
        self.main_bar.stop.clicked.connect(self.user_stop)
        QCoreApplication.instance().lastWindowClosed.connect(self.stop)
        self.start_requested = False
        self.started = False
        self.setting_up = False
        self.time = 0
        self.expected_robots = []
        self.keep_alive_timer = QTimer()
        self.keep_alive_timer.setInterval(KEEP_ALIVE_DELAY_MS)
        self.keep_alive_timer.timeout.connect(self.timer_tick)


    def setup(self):
        if not self.start_requested and not self.setting_up:
            self.user_stop()
        if not self.robot_a.is_process_started() and not self.robot_b.is_process_started():
            self.game_elements_layer.setup()
            self.time = 0
            self.main_bar.chronograph.setText(str(round(self.time / 1000.0, 1)))
            self.setting_up = True
            self.robot_a.hide_all()
            self.robot_b.hide_all()
            self.expected_robots = self.main_bar.get_expected_robots()
        if not self.robot_a.is_process_started() and len(self.expected_robots) >= 1:
            self.robot_a.setup(*self.expected_robots[0])
        elif not self.robot_b.is_process_started() and len(self.expected_robots) >= 2:
            del self.expected_robots[0]
            self.robot_b.setup(*self.expected_robots[0])
        else:
            self.expected_robots = []
            self.setting_up = False
            self.keep_alive_timer.start()
            if self.start_requested:
                self.try_start()


    def start_pause(self):
        if self.started:
            if self.keep_alive_timer.isActive():
                self.main_bar.set_icon(self.main_bar.start_pause, "start")
                self.keep_alive_timer.stop()
                self.robot_a.pause()
                self.robot_b.pause()
            else:
                self.main_bar.set_icon(self.main_bar.start_pause, "pause")
                self.keep_alive_timer.start()
                self.robot_a.resume()
                self.robot_b.resume()
        elif not self.start_requested:
            self.start_requested = True
            self.setup()
        elif not self.setting_up:
            self.start_requested = True
            self.try_start()


    def user_stop(self):
        self.stop()
        self.robot_a.stop()
        self.robot_b.stop()


    def stop(self):
        self.started = False
        self.start_requested = False
        self.main_bar.set_icon(self.main_bar.start_pause, "start")
        self.keep_alive_timer.stop()
        self.robot_a.shutdown()
        self.robot_b.shutdown()


    def send_start_signal(self):
        self.started = True
        self.main_bar.set_icon(self.main_bar.start_pause, "pause")
        self.robot_a.send_start_signal()
        self.robot_b.send_start_signal()


    def try_start(self):
        if self.start_requested and len(self.expected_robots) == 0:
            self.start_requested = False
            self.send_start_signal()


    def brewery_connected(self):
        if not self.robot_a.is_connected():
            self.robot_a.connected(self.server.nextPendingConnection())
            self.setup()
        elif not self.robot_b.is_connected():
            self.robot_b.connected(self.server.nextPendingConnection())
            self.setup()
        else:
            # Only two robots can play
            self.server.nextPendingConnection().close()
            return


    def timer_tick(self):
        self.robot_b.send_keep_alive()
        self.robot_a.send_keep_alive()
        if self.started:
            self.time += KEEP_ALIVE_DELAY_MS
            self.main_bar.chronograph.setText(str(round(self.time / 1000.0, 1)))
