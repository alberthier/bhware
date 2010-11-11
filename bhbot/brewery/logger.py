#!/usr/bin/env python
# encoding: utf-8

import os
import stat

import packets
import config
from definitions import *


filepath = None
log_file = None


def initialize():
    global filepath
    global log_file
    if log_file == None:
        filepath = get_next_log_filepath()
        log_file = file(filepath, "w")
        log_file.write("#!/usr/bin/env python\n")
        log_file.write("# encoding: utf-8\n\n")
        log_file.write("log = []\n\n")


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


def log(text):
    global log_file
    if log_file == None:
        initialize()
    log_file.write("log.append(\"" + text + "\")\n")
    if config.host_device == HOST_DEVICE_PC:
        print(text)


def log_packet(sender, packet):
    text = "[\"" + type(packet).__name__ + "\", \"" + sender + "\", " + str(packet.to_dict()) + "]"
    log(text)


def get_next_log_filepath():
    brewery_root_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    log_dir = os.path.join(brewery_root_path, "logs")
    index = 0
    if not os.path.exists(log_dir):
        os.mkdir(log_dir)
    while True:
        filepath = os.path.join(log_dir, "brewerylog_{0:=#04}.py".format(index))
        if os.path.exists(filepath):
            index += 1
        else:
            return filepath
