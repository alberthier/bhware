#!/usr/bin/env python
# encoding: utf-8

import os

from definitions import *

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




def translate_packet_data(packet_type, packet_data):
    if packet_type == "DeviceReady" or packet_type == "Start":
        if packet_data["team"] == TEAM_BLUE:
            packet_data["team"] = "TEAM_BLUE"
        elif packet_data["team"] == TEAM_RED:
            packet_data["team"] = "TEAM_RED"

    elif packet_type == "Goto":
        if packet_data["movement"] == MOVEMENT_ROTATE:
            packet_data["movement"] = "MOVEMENT_ROTATE"
        elif packet_data["movement"] == MOVEMENT_MOVE:
            packet_data["movement"] = "MOVEMENT_MOVE"

        if packet_data["direction"] == DIRECTION_FORWARD:
            packet_data["direction"] = "DIRECTION_FORWARD"
        elif packet_data["direction"] == DIRECTION_BACKWARD:
            packet_data["direction"] = "DIRECTION_BACKWARD"

    elif packet_type == "GotoFinished":
        if packet_data["reason"] == REASON_DESTINATION_REACHED:
            packet_data["reason"] = "REASON_DESTINATION_REACHED"
        elif packet_data["reason"] == REASON_POWN_FOUND:
            packet_data["reason"] = "REASON_POWN_FOUND"
        elif packet_data["reason"] == REASON_QWEEN_FOUND:
            packet_data["reason"] = "REASON_QWEEN_FOUND"
        elif packet_data["reason"] == REASON_KING_FOUND:
            packet_data["reason"] = "REASON_KING_FOUND"

    elif packet_type == "Blocked":
        if packet_data["side"] == BLOCKED_FRONT:
            packet_data["side"] = "BLOCKED_FRONT"
        elif packet_data["side"] == BLOCKED_BACK:
            packet_data["side"] = "BLOCKED_BACK"

    elif packet_type == "Resettle":
        if packet_data["axis"] == AXIS_ABSCISSA:
            packet_data["axis"] = "AXIS_ABSCISSA"
        elif packet_data["axis"] == AXIS_ORDINATE:
            packet_data["axis"] = "AXIS_ORDINATE"

    return packet_data
