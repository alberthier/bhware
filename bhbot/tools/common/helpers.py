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




def packet_dump_to_text(dump):
    if isinstance(dump, list) or isinstance(dump, tuple):
        text = "["
        first = True
        for value in dump:
            if first:
                first = False
            else:
                text += ", "
            text += packet_dump_to_text(value)
        text += "]"
    elif isinstance(dump, collections.OrderedDict) or isinstance(dump, dict):
        text = "{"
        first = True
        for key, value in dump.items():
            if first:
                first = False
            else:
                text += ", "
            text += packet_dump_to_text(key) + ": " + packet_dump_to_text(value)
        text += "}"
    else:
        text = str(dump)
    return text




def create_main_robot_base_item(pen, brush, gyration_pen):

    base = QPainterPath()
    base.moveTo(177.0, -161.0)
    base.lineTo(177.0, 161.0)
    base.lineTo(-71.0, 161.0)
    base.arcTo(-99.0, 105.0, 56.0, 56.0, -90.0, -90.0)
    base.lineTo(-99.0, -133.0)
    base.arcTo(-99.0, -161.0, 56.0, 56.0, 180.0, -90.0)
    base.closeSubpath()

    top = QPainterPath()
    top.moveTo(-71.0, -161.0)
    top.lineTo(-71.0, -83.0)
    top.arcTo(-99.0, -92.0, 28.0, 28.0, 0.0, -90.0)
    top.arcTo(-99.0, -64.0, 28.0, 28.0, 90.0, 90.0)
    top.lineTo(-99.0, 50.0)
    top.arcTo(-99.0, 36.0, 28.0, 28.0, 180.0, 90.0)
    top.arcTo(-99.0, 64.0, 28.0, 28.0, 90.0, -90.0)
    top.lineTo(-71.0, 161.0)

    gyration_radius = ROBOT_GYRATION_RADIUS * 1000.0
    gyration = QGraphicsEllipseItem(-gyration_radius, -gyration_radius, 2.0 * gyration_radius, 2.0 * gyration_radius)
    gyration.setPen(gyration_pen)

    expanded_gripper_gyration_radius = ROBOT_EXPANDED_GRIPPER_GYRATION_RADIUS * 1000.0
    expanded_gripper_gyration = QGraphicsEllipseItem(-expanded_gripper_gyration_radius, -expanded_gripper_gyration_radius, 2.0 * expanded_gripper_gyration_radius, 2.0 * expanded_gripper_gyration_radius)
    expanded_gripper_gyration.setPen(gyration_pen)

    expanded_sweeper_gyration_radius = ROBOT_EXPANDED_SWEEPER_GYRATION_RADIUS * 1000.0
    expanded_sweeper_gyration = QGraphicsEllipseItem(-expanded_sweeper_gyration_radius, -expanded_sweeper_gyration_radius, 2.0 * expanded_sweeper_gyration_radius, 2.0 * expanded_sweeper_gyration_radius)
    expanded_sweeper_gyration.setPen(gyration_pen)

    robot_group = QGraphicsItemGroup()

    base_item = QGraphicsPathItem(base)
    base_item.setPen(pen)
    base_item.setBrush(brush)
    robot_group.addToGroup(base_item)

    top_item = QGraphicsPathItem(top)
    top_item.setPen(pen)
    top_item.setBrush(brush)
    robot_group.addToGroup(top_item)

    gyration_group = QGraphicsItemGroup()

    gyration_group.addToGroup(gyration)
    gyration_group.addToGroup(expanded_gripper_gyration)
    gyration_group.addToGroup(expanded_sweeper_gyration)

    all_group = QGraphicsItemGroup()
    all_group.addToGroup(robot_group)
    all_group.addToGroup(gyration_group)

    return (all_group, robot_group, gyration_group)




def create_secondary_robot_base_item(pen, brush, gyration_pen):
    all_group = QGraphicsItemGroup()
    robot_group = QGraphicsItemGroup()
    gyration_group = QGraphicsItemGroup()

    base_item = QGraphicsRectItem(-59, -202 / 2, 188, 202)
    base_item.setPen(pen)
    base_item.setBrush(brush)
    robot_group.addToGroup(base_item)

    gyration = QGraphicsEllipseItem(-164, -164, 328, 328)
    gyration.setPen(pen)
    gripper_gyration = QGraphicsEllipseItem(-261, -261, 522, 522)
    gripper_gyration.setPen(pen)

    gyration_group.addToGroup(gyration)
    gyration_group.addToGroup(gripper_gyration)

    all_group.addToGroup(robot_group)
    all_group.addToGroup(gyration_group)
    all_group.addToGroup(gripper_gyration)

    return (all_group, robot_group, gyration_group)
