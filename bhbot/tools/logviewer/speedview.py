# encoding: utf-8

import math

from PyQt4.QtCore import *
from PyQt4.QtGui import *

import logger
import packets
from definitions import *

import logtools
import matplotlibext




class LinearSpeedViewController(QObject):

    def __init__(self, ui, parent = None):
        QObject.__init__(self)
        self.ui = ui

        self.points = []
        self.speeds = [0.0]

        self.max_speed = 0.0
        self.average_speed = 0.0
        self.average_speed_count = 0

        self.plot_widget = matplotlibext.PlotWidget()
        self.ui.linear_speed_tab_layout.addWidget(self.plot_widget)


    def process_log_line(self, log_line, lineno, last_lineno):
        packet_type = log_line[logger.LOG_LINE_PACKET]
        sender = log_line[logger.LOG_LINE_SENDER]
        if packet_type is packets.KeepAlive and sender == "PIC":
            pose = logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "current_pose")
            x = logtools.get_value(pose, "x")
            y = logtools.get_value(pose, "y")
            if len(self.points) > 0:
                oldx, oldy = self.points[-1]
                speed = math.sqrt((x - oldx) ** 2 + (y - oldy) ** 2) / (KEEP_ALIVE_DELAY_MS / 1000.0)
                self.speeds.append(speed)
                if speed > self.max_speed:
                    self.max_speed = speed
                if speed > 0.001:
                    self.average_speed += speed
                    self.average_speed_count += 1
            self.points.append((x, y))


    def log_loaded(self):
        if self.average_speed_count > 0.0:
            self.average_speed = self.average_speed / float(self.average_speed_count)
            self.speeds.append(0.0)
            if self.plot_widget.has_matplotlib:
                x = list(map(lambda x: x * KEEP_ALIVE_DELAY_MS / 1000.0, range(len(self.speeds))))
                speed_plot = self.plot_widget.add_plot()
                speed_plot.set_xlabel("Time (s)")
                speed_plot.set_ylabel("Speed (m/s)")
                speed_plot.fill(x, self.speeds, color='#729fcf', edgecolor='#3465a4', alpha=0.75)
                speed_plot.axhline(self.average_speed, color='#73d216')
                speed_plot.axhline(self.max_speed, color='#cc0000')




class AngularSpeedViewController(QObject):

    def __init__(self, ui, parent = None):
        QObject.__init__(self)
        self.ui = ui

        self.angles = []
        self.speeds = [0.0]

        self.max_speed = 0.0
        self.average_speed = 0.0
        self.average_speed_count = 0

        self.plot_widget = matplotlibext.PlotWidget()
        self.ui.angular_speed_tab_layout.addWidget(self.plot_widget)


    def process_log_line(self, log_line, lineno, last_lineno):
        packet_type = log_line[logger.LOG_LINE_PACKET]
        sender = log_line[logger.LOG_LINE_SENDER]
        if packet_type is packets.KeepAlive and sender == "PIC":
            pose = logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "current_pose")
            angle = logtools.get_value(pose, "angle")
            if len(self.angles) > 0:
                speed = abs(self.angles[-1] - angle)
                self.speeds.append(speed)
                if speed > self.max_speed:
                    self.max_speed = speed
                if speed > 0.01:
                    self.average_speed += speed
                    self.average_speed_count += 1
            self.angles.append(angle)


    def log_loaded(self):
        if self.average_speed_count > 0.0:
            self.average_speed = self.average_speed / float(self.average_speed_count)
            self.speeds.append(0.0)
            if self.plot_widget.has_matplotlib:
                x = list(map(lambda x: x * KEEP_ALIVE_DELAY_MS / 1000.0, range(len(self.speeds))))
                speed_plot = self.plot_widget.add_plot()
                speed_plot.set_xlabel("Time (s)")
                speed_plot.set_ylabel("Speed (rd/s)")
                speed_plot.fill(x, self.speeds, color='#729fcf', edgecolor='#3465a4', alpha=0.75)
                speed_plot.axhline(self.average_speed, color='#73d216')
                speed_plot.axhline(self.max_speed, color='#cc0000')
