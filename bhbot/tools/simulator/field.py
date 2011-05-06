#!/usr/bin/env python
# encoding: utf-8

import os
import random
import math

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

        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)

        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        self.setSceneRect(-200.0, -200.0, 3404, 2504)


    def resizeEvent(self, event):
        QGraphicsView.resizeEvent(self, event)
        self.fitInView(-200.0, -200.0, 3404, 2504, Qt.KeepAspectRatio)


    def enterEvent(self, event):
        QGraphicsView.enterEvent(self, event)
        self.scene().mouse_item.setVisible(True)


    def leaveEvent(self, event):
        QGraphicsView.leaveEvent(self, event)
        self.scene().mouse_item.setVisible(False)




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
            self.piece_type = piece_type
            font = QFont()
            font.setPointSize(70)
            text = QGraphicsTextItem()
            text.setDefaultTextColor(Qt.black)
            text.setPos(-37.5, -60.0)
            text.setFont(font)
            text.setPlainText(piece_type)
            self.addToGroup(text)
        else:
            self.piece_type = "P"

        self.setPos(y, x)




class FieldScene(QGraphicsScene):

    def __init__(self, piece_config, main_bar):
        QGraphicsScene.__init__(self)

        self.main_bar = main_bar

        self.pieces = []

        self.setBackgroundBrush(QBrush(QColor("#82a2cf")))

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

        # Mouse ghost robot item
        ghost_pen = QPen(QColor("#edd400"))
        self.mouse_item = QGraphicsItemGroup()
        gyration = QGraphicsEllipseItem(-288.0, -288.0, 576.0, 576.0)
        gyration.setPen(ghost_pen)
        self.mouse_item.addToGroup(gyration)
        piece_gyration = QGraphicsEllipseItem(-325.0, -325.0, 650.0, 650.0)
        piece_gyration.setPen(ghost_pen)
        self.mouse_item.addToGroup(piece_gyration)
        robot_ghost = QGraphicsRectItem(-31.0, -170.0, 256.0, 340.0)
        robot_ghost.setPen(ghost_pen)
        self.mouse_item.addToGroup(robot_ghost)
        line = QGraphicsLineItem(-50.0, 0.0, 50.0, 0.0)
        line.setPen(ghost_pen)
        self.mouse_item.addToGroup(line)
        line = QGraphicsLineItem(0.0, -50.0, 0.0, 50.0)
        line.setPen(ghost_pen)
        self.mouse_item.addToGroup(line)
        self.mouse_item.setVisible(False)
        self.addItem(self.mouse_item)

        self.config = self.parse_piece_config(piece_config)
        self.display_config()

        self.setup()


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


    def display_config(self):
        s = ""
        for c in self.config:
            for r in c:
                s += str(r)
        self.main_bar.config.setText(s)


    def random_tuple(self):
        r1 = random.randint(0, 4)
        r2 = random.randint(0, 4)
        while r1 == r2:
            r2 = random.randint(0, 4)
        return (r1, r2)


    def add_piece(self, x, y, piece_type = None):
        piece = Piece(x, y, piece_type)
        self.addItem(piece)
        self.pieces.append(piece)


    def setup(self):
        for piece in self.pieces:
            self.removeItem(piece)
        self.pieces = []
        self.setup_green_zone(self.config[0])
        self.setup_line(self.config[1], 0)
        self.setup_line(self.config[2], 350)
        self.add_piece(1050.0, 1500.0)



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
                    self.add_piece(x, y, "R")
                elif i == config[1]:
                    self.add_piece(x, y, "Q")
                else:
                    self.add_piece(x, y)


    def setup_line(self, config, offset):
        for i in config:
            x = 350.0 * (i + 1)
            self.add_piece(x, 800.0 + offset)
            self.add_piece(x, 2200.0 - offset)


    def mouseMoveEvent(self, mouseEvent):
        self.mouse_item.setVisible(True)
        pos = mouseEvent.scenePos()
        self.mouse_item.setPos(pos)
        self.main_bar.x.setText("{0:=0.03f}".format(pos.y() / 1000.0))
        self.main_bar.y.setText("{0:=0.03f}".format(pos.x() / 1000.0))
        self.main_bar.angle.setText("{0:=0.03f}".format(self.convert_angle()))


    def wheelEvent(self, wheelEvent):
        self.mouse_item.setVisible(True)
        angle = (self.mouse_item.rotation() + wheelEvent.delta() / 8) % 360.0
        self.mouse_item.setRotation(angle)
        self.main_bar.angle.setText("{0:=0.03f}".format(self.convert_angle()))


    def convert_angle(self):
        angle = (self.mouse_item.rotation() % 360.0) / 180.0 * math.pi
        return math.atan2(math.cos(angle), math.sin(angle))
