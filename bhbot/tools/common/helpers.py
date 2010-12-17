#!/usr/bin/env python
# encoding: utf-8

import os
import subprocess

from definitions import *



def get_last_remote_logfile():
    drunkstar_ip = "192.168.1.200"
    drunkstar_bhware = "/root/bhware/bhbot/logs"
    ls = subprocess.Popen(["ssh", "root@{0}".format(drunkstar_ip), "ls {0}".format(drunkstar_bhware)], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    (out, err) = ls.communicate()
    logs = out.split("\n")
    if len(logs) != 0:
        logs.sort()
        log = logs[-1]
        if not os.path.exists("/tmp/bh"):
            os.makedirs("/tmp/bh")
        local_file = "/tmp/bh/remote_{0}".format(log)
        subprocess.call(["scp", "root@{0}:{1}/{2}".format(drunkstar_ip, drunkstar_bhware, log), local_file])
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




def translate_packet_data(packet_type, packet_data):

    if packet_type == 'DeviceReady' or packet_type == 'DeviceBusy':
        packet_data['remote_device'] = lookup_defs('REMOTE_DEVICE', packet_data['remote_device'])

    if packet_type == 'DeviceReady' or packet_type == 'Start':
        packet_data['team'] = lookup_defs('TEAM', packet_data['team'])

    elif packet_type == 'Goto':
        packet_data['movement'] = lookup_defs('MOVEMENT', packet_data['movement'])
        packet_data['direction'] = lookup_defs('DIRECTION', packet_data['direction'])

    elif packet_type == 'GotoFinished':
        packet_data['reason'] = lookup_defs('REASON', packet_data['reason'])

    elif packet_type == 'Blocked':
        packet_data['side'] = lookup_defs('BLOCKING', packet_data['side'])

    elif packet_type == 'Resettle':
        packet_data['axis'] = lookup_defs('AXIS', packet_data['axis'])

    elif packet_type == 'PieceDetected':
        packet_data['left_sensor'] = lookup_defs('PIECE_SENSOR', packet_data['left_sensor'])
        packet_data['right_sensor'] = lookup_defs('PIECE_SENSOR', packet_data['right_sensor'])

    return packet_data
