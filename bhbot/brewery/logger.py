#!/usr/bin/env python
# encoding: utf-8

import os
import sys
import stat
import datetime
import subprocess

import packets
import config
from definitions import *


filepath = None
log_file = None
start_time = None


def initialize():
    global filepath
    global log_file
    global start_time
    if log_file == None:
        start_time = datetime.datetime.now()
        filepath = get_next_log_filepath()
        log_file = file(filepath, "w")
        log_file.write("#!/usr/bin/env python\n")
        log_file.write("# encoding: utf-8\n\n")
        log_file.write("log = []\n\n")
        log("Logging to '{0}'".format(os.path.split(filepath)[1]))


def close():
    global log_file
    global filepath
    if log_file == None:
        return

    log_file.write("\nif __name__ == '__main__':\n")
    log_file.write("    for line in log:\n")
    log_file.write("        print(line)\n")
    log_file.close()
    os.chmod(filepath, stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)
    filepath = None
    log_file = None
    subprocess.Popen(["/bin/sync"]).wait()


def log(text):
    global log_file
    global start_time
    if log_file == None:
        initialize()
    delta = datetime.datetime.now() - start_time
    time = "'{0:=0.02f}'".format(float(delta.seconds) + (float(delta.microseconds)/1000000.0))
    log_file.write("log.append([" + time + ",\"# " + text + "\"])\n")
    if config.host_device == HOST_DEVICE_PC:
        sys.stdout.write(text + "\n")
        sys.stdout.flush()


def log_packet(sender, packet):
    delta = datetime.datetime.now() - start_time
    time = "'{0:=0.02f}'".format(float(delta.seconds) + (float(delta.microseconds)/1000000.0))
    text = "'" + type(packet).__name__ + "', '" + sender + "', " + str(packet.to_dict()) + "]"
    log_file.write("log.append([" + time + ", " + text + ")\n")
    if config.host_device == HOST_DEVICE_PC and not isinstance(packet, packets.KeepAlive) and not isinstance(packet, packets.SimulatorData):
        sys.stdout.write("[" + text + "\n")
        sys.stdout.flush()


def get_next_log_filepath():
    index = 0
    if not os.path.exists(config.log_dir):
        os.mkdir(config.log_dir)
    while True:
        filepath = os.path.join(config.log_dir, "brewerylog_{0:=#04}.py".format(index))
        if os.path.exists(filepath):
            index += 1
        else:
            return filepath
