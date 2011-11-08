#!/usr/bin/env python
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
    ls = subprocess.Popen(["ssh", "root@{0}".format(drunkstar_ip), "ls {0}/brewerylog*".format(drunkstar_bhware)], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    (out, err) = ls.communicate()
    logs = out.split("\n")
    if len(logs) != 0:
        logs.sort()
        log = logs[-1]
        print log
        if not os.path.exists("/tmp/bh"):
            os.makedirs("/tmp/bh")
        local_file = "/tmp/bh/remote_{0}".format(os.path.basename(log))
        subprocess.call(["scp", "root@{0}:{1}".format(drunkstar_ip, log), local_file])
        return local_file

    return None




def get_last_logfile():
    brewery_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
    log_dir = os.path.join(brewery_root_path, "logs")
    index = 0
    if os.path.exists(log_dir):
        while True:
            filepath = os.path.join(log_dir, "brewerylog_{0:=#04}.py".format(index))
            if os.path.exists(filepath):
                index += 1
            else:
                index -= 1
                return os.path.join(log_dir, "brewerylog_{0:=#04}.py".format(index))
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
        for key, value in dump.iteritems():
            if first:
                first = False
            else:
                text += ", "
            text += packet_dump_to_text(key) + ": " + packet_dump_to_text(value)
        text += "}"
    else:
        text = str(dump)
    return text




def create_robot_base_item(pen, brush, gyration_pen):

    base = QPainterPath()
    base.moveTo(-161.0, 177.0)
    base.lineTo(161.0, 177.0)
    base.lineTo(161.0, -71.0)
    base.arcTo(105.0, -99.0, 56.0, 56.0, 0.0, 90.0)
    base.lineTo(-133.0, -99.0)
    base.arcTo(-161.0, -99.0, 56.0, 56.0, 90.0, 90.0)
    base.closeSubpath()

    top = QPainterPath()
    top.moveTo(-161.0, -71.0)
    top.lineTo(-83.0, -71.0)
    top.arcTo(-92.0, -99.0, 28.0, 28.0, 270.0, 90.0)
    top.arcTo(-64.0, -99.0, 28.0, 28.0, 180.0, -90.0)
    top.lineTo(50.0, -99.0)
    top.arcTo(36.0, -99.0, 28.0, 28.0, 90.0, -90.0)
    top.arcTo(64.0, -99.0, 28.0, 28.0, 180.0, 90.0)
    top.lineTo(161.0, -71.0)

    gyration_radius = ROBOT_GYRATION_RADIUS * 1000.0
    gyration = QGraphicsEllipseItem(-gyration_radius, -gyration_radius, 2.0 * gyration_radius, 2.0 * gyration_radius)
    gyration.setPen(gyration_pen)

    expanded_gyration_radius = ROBOT_EXPANDED_GYRATION_RADIUS * 1000.0
    expanded_gyration = QGraphicsEllipseItem(-expanded_gyration_radius, -expanded_gyration_radius, 2.0 * expanded_gyration_radius, 2.0 * expanded_gyration_radius)
    expanded_gyration.setPen(gyration_pen)

    group = QGraphicsItemGroup()

    base_item = QGraphicsPathItem(base)
    base_item.setPen(pen)
    base_item.setBrush(brush)
    group.addToGroup(base_item)

    top_item = QGraphicsPathItem(top)
    top_item.setPen(pen)
    top_item.setBrush(brush)
    group.addToGroup(top_item)

    group.addToGroup(gyration)
    group.addToGroup(expanded_gyration)

    return group
