# encoding: utf-8

from PyQt4.Qt import Qt
from PyQt4.QtGui import *

import os
import subprocess
import collections

from definitions import *



def get_last_remote_logfile():
    drunkstar_ip = "192.168.1.42"
    drunkstar_bhware = "/root/logs"
    ls = subprocess.Popen(["ssh", "root@{}".format(drunkstar_ip), "ls {}/brewerylog*".format(drunkstar_bhware)], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    (out, err) = ls.communicate()
    logs = out.split("\n")
    if len(logs) != 0:
        logs.sort()
        log = logs[-1]
        if not os.path.exists("/tmp/bh"):
            os.makedirs("/tmp/bh")
        local_file = "/tmp/bh/remote_{}".format(os.path.basename(log))
        subprocess.call(["scp", "root@{}:{}".format(drunkstar_ip, log), local_file])
        return local_file

    return None




def get_last_logfile():
    brewery_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
    log_dir = os.path.join(brewery_root_path, "logs")
    index = 0
    if os.path.exists(log_dir):
        while True:
            filepath = os.path.join(log_dir, "brewerylog_{:=#04}.py".format(index))
            if os.path.exists(filepath):
                index += 1
            else:
                index -= 1
                return os.path.join(log_dir, "brewerylog_{:=#04}.py".format(index))
    return None




def create_main_robot_base_item(pen, brush, gyration_pen):
    robot = QGraphicsItemGroup()

    path = QPainterPath()
    path.moveTo(-147.5, 150.0)
    path.lineTo(127.5, 150.0)
    path.arcTo(107.5, 110.0, 40.0, 40.0, -90.0, 90.0)
    path.arcTo(130.0, 20.88, 130.0, 130.0, -150, -70)
    path.lineTo(147.5, -130.0)
    path.arcTo(107.5, -150.0, 40.0, 40.0, 0.0, 90.0)
    path.lineTo(-147.5, -150.0)
    path.closeSubpath()
    base = QGraphicsPathItem(path)
    base.setPen(pen)
    base.setBrush(brush)
    robot.addToGroup(base)
    path = QPainterPath()
    path.moveTo(37.5, 150.0)
    path.arcTo(37.5, 70.0, 80.0, 80.0, -90.0, 90.0)
    path.lineTo(117.5, -150.0)
    item = QGraphicsPathItem(path)
    item.setPen(pen)
    robot.addToGroup(item)
    item = QGraphicsLineItem(-147.5, -110.0, 117.5, -110.0)
    item.setPen(pen)
    robot.addToGroup(item)
    item = QGraphicsRectItem(-147.5, 100.0, 152.0, 30.0)
    item.setPen(pen)
    robot.addToGroup(item)

    gyration_radius = MAIN_ROBOT_GYRATION_RADIUS * 1000.0
    gyration = QGraphicsEllipseItem(-gyration_radius, -gyration_radius, 2.0 * gyration_radius, 2.0 * gyration_radius)
    gyration.setPen(gyration_pen)

    all_group = QGraphicsItemGroup()
    all_group.addToGroup(robot)
    all_group.addToGroup(gyration)

    return (all_group, robot, gyration)




def create_secondary_robot_base_item(pen, brush, gyration_pen):
    robot = QGraphicsItemGroup()

    path = QPainterPath()
    path.moveTo(-77.5, -64.04)
    path.lineTo(-77.5, 64.04)
    path.lineTo(-64.04, 77.5)
    path.lineTo(64.04, 77.5)
    path.lineTo(77.5, 64.04)
    path.lineTo(77.5, -64.04)
    path.lineTo(64.04, -77.5)
    path.lineTo(-64.04, -77.5)
    path.closeSubpath()
    base = QGraphicsPathItem(path)
    base.setPen(pen)
    base.setBrush(brush)
    robot.addToGroup(base)

    robot.addToGroup(create_arrow(pen, -20.0, -60.0, 40.0, 135.0))

    gyration_radius = SECONDARY_ROBOT_GYRATION_RADIUS * 1000.0
    gyration = QGraphicsEllipseItem(-gyration_radius, -gyration_radius, 2.0 * gyration_radius, 2.0 * gyration_radius)
    gyration.setPen(gyration_pen)

    all_group = QGraphicsItemGroup()
    all_group.addToGroup(robot)
    all_group.addToGroup(gyration)

    return (all_group, robot, gyration)




def create_arrow(pen, x, y, width, length):
    path = QPainterPath()
    path.moveTo(y, x)
    path.lineTo(y + length - width * 2.0, x)
    path.lineTo(y + length, x + width / 2.0)
    path.lineTo(y + length - width * 2.0, x + width)
    path.lineTo(y, x + width)
    path.closeSubpath()

    item = QGraphicsPathItem(path)
    item.setPen(pen)
    return item
