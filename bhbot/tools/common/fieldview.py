#!/usr/bin/env python
# encoding: utf-8


from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtSvg import *
import os




class FieldScene(QGraphicsScene):

    def __init__(self, parent = None):
        QGraphicsScene.__init__(self, parent)
        self.setBackgroundBrush(QBrush(QColor("#82a2cf")))
        self.root = QGraphicsItemGroup()
        self.addItem(self.root)
        field = QGraphicsSvgItem(os.path.join(os.path.dirname(os.path.dirname(__file__)), "simulator", "field.svg"))
        self.root.addToGroup(field)
        field.setPos(-102.0, -102.0)

        pen = QPen(QColor("#2e3436"), 10)
        font = QFont()
        font.setPointSize(70)
        self.addLine(-170.0, -170.0, -170.0, 100.0, pen)
        self.addText("x", font).setPos(-200.0, 80.0)
        self.addLine(-170.0, -170.0, 100.0, -170.0, pen)
        self.addText("y", font).setPos(120.0, -230.0)

        self.layers = []


    def add_layer(self, layer):
        self.layers.append(layer)
        layer.setParentItem(self.root)




class Layer(QGraphicsItemGroup):

    def __init__(self, parent = None):
        QGraphicsItemGroup.__init__(self)
        self.name = "Unnamed"




class FieldView(QGraphicsView):

    def __init__(self, parent = None):
        QGraphicsView.__init__(self, parent)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setSceneRect(-200.0, -200.0, 3404, 2504)


    def resizeEvent(self, event):
        QGraphicsView.resizeEvent(self, event)
        self.fitInView(-200.0, -200.0, 3404, 2504, Qt.KeepAspectRatio)
