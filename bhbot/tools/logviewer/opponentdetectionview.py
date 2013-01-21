# encoding: utf-8

from PyQt4.QtCore import *
from PyQt4.QtGui import *

import logger
import packets
from definitions import *

import logtools
import matplotlibext




class OpponentDectectionViewController(QObject):

    def __init__(self, ui, parent = None):
        QObject.__init__(self)
        self.ui = ui

        num_detectors = 20
        self.main_opponent_short_distance      = [0 for x in range(num_detectors)]
        self.main_opponent_long_distance       = [0 for x in range(num_detectors)]
        self.secondary_opponent_short_distance = [0 for x in range(num_detectors)]
        self.secondary_opponent_long_distance  = [0 for x in range(num_detectors)]

        self.main_plot_widget = matplotlibext.PlotWidget()
        self.ui.opponent_detection_layout.addWidget(self.main_plot_widget)
        self.secondary_plot_widget = matplotlibext.PlotWidget()
        self.ui.opponent_detection_layout.addWidget(self.secondary_plot_widget)

        self.radar = matplotlibext.radar_factory(num_detectors, frame = 'polygon')


    def process_log_line(self, log_line, lineno, last_lineno):
        packet_type = log_line[logger.LOG_LINE_PACKET]
        if packet_type is packets.TurretDetect:
            angle = logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "angle")
            if logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "robot") == OPPONENT_ROBOT_MAIN:
                if logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "distance") == 0:
                    self.main_opponent_short_distance[angle] += 1
                else:
                    self.main_opponent_long_distance[angle] += 1
            else:
                if logtools.get_value(log_line[logger.LOG_LINE_CONTENT], "distance") == 0:
                    self.secondary_opponent_short_distance[angle] += 1
                else:
                    self.secondary_opponent_long_distance[angle] += 1


    def log_loaded(self):
        if self.main_plot_widget.has_matplotlib:
            for widget, name, (long_distance, short_distance) in zip([self.main_plot_widget, self.secondary_plot_widget],
                                                                     ["Main", "Secondary"],
                                                                     [[self.main_opponent_long_distance, self.main_opponent_short_distance],
                                                                      [self.secondary_opponent_long_distance, self.secondary_opponent_short_distance]]):
                plot = widget.add_plot(projection = 'radar')
                plot.set_title(name + " opponent")
                plot.fill(self.radar, long_distance, color='#73d216', edgecolor='#4e9a06', alpha=0.75)
                plot.fill(self.radar, short_distance, color='#729fcf', edgecolor='#3465a4', alpha=0.75)
                legend = plot.legend(["Far", "Near"], loc=(0, 0.97), fontsize='small')
