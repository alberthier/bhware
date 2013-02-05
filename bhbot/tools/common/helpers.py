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
    robot = QGraphicsRectItem(-126.0, -126.0, 252.0, 254.0)
    robot.setPen(pen)
    robot.setBrush(brush)

    gyration_radius = MAIN_ROBOT_GYRATION_RADIUS * 1000.0
    gyration = QGraphicsEllipseItem(-gyration_radius, -gyration_radius, 2.0 * gyration_radius, 2.0 * gyration_radius)
    gyration.setPen(gyration_pen)

    all_group = QGraphicsItemGroup()
    all_group.addToGroup(robot)
    all_group.addToGroup(gyration)

    return (all_group, robot, gyration)




def create_secondary_robot_base_item(pen, brush, gyration_pen):
    robot = QGraphicsRectItem(-63.5, -53.0, 127.0, 172.0)
    robot.setPen(pen)
    robot.setBrush(brush)

    gyration_radius = SECONDARY_ROBOT_GYRATION_RADIUS * 1000.0
    gyration = QGraphicsEllipseItem(-gyration_radius, -gyration_radius, 2.0 * gyration_radius, 2.0 * gyration_radius)
    gyration.setPen(gyration_pen)

    all_group = QGraphicsItemGroup()
    all_group.addToGroup(robot)
    all_group.addToGroup(gyration)

    return (all_group, robot, gyration)

