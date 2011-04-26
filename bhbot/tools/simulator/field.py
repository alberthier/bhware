#!/usr/bin/env python
# encoding: utf-8

import os
import random

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *
from PyQt4.Qt import Qt




class FieldView(QGraphicsView):

    def __init__(self, scene, parent = None):
        QGraphicsView.__init__(self, scene, parent)
        policy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        policy.setHorizontalStretch(5)
        policy.setVerticalStretch(255)
        self.setSizePolicy(policy)

        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self.setSceneRect(-200.0, -200.0, 3404, 2504)


    def resizeEvent(self, event):
        QGraphicsView.resizeEvent(self, event)
        self.fitInView(-200.0, -200.0, 3404, 2504, Qt.KeepAspectRatio)




class Piece(QGraphicsItemGroup):

    def __init__(self, x, y, piece_type = None):
        QGraphicsItemGroup.__init__(self)
        brush = QBrush(QColor("#e2df03"))
        pen = QPen()
        pen.setWidth(10)

        circle = QGraphicsEllipseItem(-100.0, -100.0, 200.0, 200.0)
        circle.setBrush(brush)
        circle.setPen(pen)
        self.addToGroup(circle)

        if piece_type != None:
            font = QFont()
            font.setPointSize(70)
            text = QGraphicsTextItem()
            text.setDefaultTextColor(Qt.black)
            text.setPos(-37.5, -60.0)
            text.setFont(font)
            text.setPlainText(piece_type)
            self.addToGroup(text)

        self.setPos(y, x)




class FieldScene(QGraphicsScene):

    def __init__(self, piece_config):
        QGraphicsScene.__init__(self)

        gradiant = QLinearGradient(0.5, 0.1, 0.5, 0.9)
        gradiant.setCoordinateMode(QGradient.ObjectBoundingMode)
        gradiant.setColorAt(0.0, QColor("#729fcf"))
        gradiant.setColorAt(1.0, QColor("#eeeeec"))
        self.setBackgroundBrush(gradiant)

        self.field = QGraphicsSvgItem(os.path.join(os.path.dirname(__file__), "field.svg"))
        self.addItem(self.field)
        self.field.setPos(-102.0, -102.0)

        pen = QPen(QColor("#2e3436"), 10)
        font = QFont()
        font.setPointSize(70)
        self.addLine(-170.0, -170.0, -170.0, 100.0, pen)
        self.addText("x", font).setPos(-200.0, 80.0)
        self.addLine(-170.0, -170.0, 100.0, -170.0, pen)
        self.addText("y", font).setPos(120.0, -230.0)

        config = self.parse_piece_config(piece_config)
        self.display_config(config)

        self.setup_green_zone(config[0])
        self.setup_line(config[1], 0)
        self.setup_line(config[2], 350)
        self.addItem(Piece(1050.0, 1500.0))


    def parse_piece_config(self, piece_config):
        if len(piece_config) == 6:
            config = ((int(piece_config[0]), int(piece_config[1])),
                      (int(piece_config[2]), int(piece_config[3])),
                      (int(piece_config[4]), int(piece_config[5])))
            ok = config[0][0] != config[0][1] and config[1][0] != config[1][1] and config[2][0] != config[2][1]
            ok &= config[0][0] >= 0 and config[0][0] <= 4
            ok &= config[0][1] >= 0 and config[0][1] <= 4
            ok &= config[1][0] >= 0 and config[1][0] <= 4
            ok &= config[1][1] >= 0 and config[1][1] <= 4
            ok &= config[2][0] >= 0 and config[2][0] <= 4
            ok &= config[2][1] >= 0 and config[2][1] <= 4
            if ok:
                return config
        if len(piece_config) > 0:
            print("Invalid piece config")

        config = (self.random_tuple(), self.random_tuple(), self.random_tuple())
        return config


    def display_config(self, config):
        s = ""
        for c in config:
            for r in c:
                s += str(r)
        font = QFont()
        font.setPointSize(70)
        self.addText("Config:\n{0}".format(s), font).setPos(-500.0, 200.0)


    def random_tuple(self):
        r1 = random.randint(0, 4)
        r2 = random.randint(0, 4)
        while r1 == r2:
            r2 = random.randint(0, 4)
        return (r1, r2)


    def setup_green_zone(self, config):
        brush = QBrush(QColor("#e2df03"))
        pen = QPen()
        pen.setWidth(10)
        font = QFont()
        font.setPointSize(70)
        for i in xrange(5):
            x = 690.0 + i * 280.0
            for y in [200.0, 2800.0]:
                if i == config[0]:
                    piece = Piece(x, y, "R")
                elif i == config[1]:
                    piece = Piece(x, y, "Q")
                else:
                    piece = Piece(x, y)
                self.addItem(piece)


    def setup_line(self, config, offset):
        for i in config:
            x = 350.0 * (i + 1)
            self.addItem(Piece(x, 800.0 + offset))
            self.addItem(Piece(x, 2200.0 - offset))
