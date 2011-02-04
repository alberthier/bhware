#!/usr/bin/env python
# encoding: utf-8

import os

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




class FieldScene(QGraphicsScene):

    def __init__(self):
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
