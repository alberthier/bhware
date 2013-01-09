# encoding: utf-8

import math

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *

import logger
import packets
from definitions import *

import logtools

matplotlib = None
try:
    import matplotlib
    v = [i for i in map(int, matplotlib.__version__.split('.'))]
    if not (v[0] >= 1 and v[1] >= 2):
        del matplotlib
        matplotlib = None
    else:
        from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
        from matplotlib.figure import Figure
except:
    print("You need matplotlib v1.2.0 or newer")




class SpeedViewController(QObject):

    def __init__(self, ui, parent = None):
        QObject.__init__(self)
        self.ui = ui

        self.points = []
        self.speeds = []

        self.max_speed = 0.0
        self.average_speed = 0.0
        self.average_speed_count = 0

        if matplotlib is None:
            label = QLabel(self.ui.speed_tab)
            label.setText("You need matplotlib v1.2.0 or newer")
            label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
            self.ui.speed_tab_layout.addWidget(label)
        else:
            self.figure = Figure((5.0, 4.0), dpi=100)
            self.speed_plot = self.figure.add_subplot(111)
            self.speed_plot.grid(True)
            self.figure_canvas = FigureCanvas(self.figure)
            self.figure_canvas.setParent(self.ui.speed_tab)
            self.ui.speed_tab_layout.addWidget(self.figure_canvas)


    def process_log_line(self, log_line, lineno, last_lineno):
        if not matplotlib is None:
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
                    if speed > 0.01:
                        self.average_speed += speed
                        self.average_speed_count += 1
                self.points.append((x, y))
            if lineno == last_lineno:
                self.average_speed / float(len(self.speeds))


    def log_loaded(self):
        if not matplotlib is None:
            x = [i for i in map(lambda x: x * KEEP_ALIVE_DELAY_MS / 1000.0, range(len(self.speeds)))]
            self.speed_plot.fill(x, self.speeds, color='#93ABFF', edgecolor='#1E41B9')
            self.average_speed = self.average_speed / float(self.average_speed_count)
            self.speed_plot.axhline(self.average_speed, color='g')
            self.speed_plot.axhline(self.max_speed, color='r')
