# encoding: utf-8

import os
import sys
import stat
import datetime
import subprocess
import traceback

import packets
from definitions import *


LOG_LINE_TIME   = 0
LOG_LINE_SENDER = 1
LOG_LINE_DATA   = 2
LOG_LINE_COUNT  = 3

filepath = None
log_file = None
start_time = None
color_start = ""
color_stop = ""


def initialize():
    global filepath
    global log_file
    global start_time
    if log_file is None:
        start_time = datetime.datetime.now()
        filepath = get_next_log_filepath()
        #noinspection PyBroadException
        try:
            log_file = open(filepath, "w")
            os.chmod(filepath, stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)
        except:
            log_file = open(os.devnull, "w")
        log_file.write("#!/usr/bin/env python3\n")
        log_file.write("# encoding: utf-8\n\n")
        log_file.write("from packets import *\n")
        log_file.write("from trajectory import *\n\n")
        log_file.write("log = []\n\n")
        log_file.write("if __name__ == '__main__':\n")
        log_file.write("    def l(line):\n")
        log_file.write("        print(line)\n")
        log_file.write("else:\n")
        log_file.write("    def l(line):\n")
        log_file.write("        global log\n")
        log_file.write("        log.append(line)\n\n")
        log("Logging to '{}'".format(os.path.split(filepath)[1]))


def set_team(team):
    global color_start
    global color_stop
    if os.isatty(sys.stdout.fileno()):
        if team == TEAM_PURPLE:
            color_start = "\033[34m"
        else:
            color_start = "\033[31m"
        color_stop = "\033[00m"


def close():
    global log_file
    global filepath
    if log_file is None:
        return
    log_file.close()
    filepath = None
    log_file = None
    subprocess.Popen(["/bin/sync"]).wait()


def log(text, sender = "ARM"):
    global log_file
    global start_time
    global color_start
    global color_stop
    initialize()
    delta = datetime.datetime.now() - start_time
    time = "'{:=0.02f}'".format(delta.total_seconds())
    if type(text) != str:
        text = str(text)
    #noinspection PyBroadException
    try:
        log_file.write("l([" + time + ",'" + sender + "','LOG','# " + text.replace("'", "\\'") + "'])\n")
    except:
        print("Failed to write to file")
    sys.stdout.write(color_start + text + color_stop + "\n")
    sys.stdout.flush()


def log_exception(exc):
    msg = ""
    for l in traceback.format_exception(type(exc), exc, None):
        msg += l
    log(msg)


def log_packet(packet, sender = "ARM"):
    initialize()
    delta = datetime.datetime.now() - start_time
    time = "'{:=0.02f}'".format(delta.total_seconds())
    text = "'" + sender + "'," + type(packet).__name__ + "," + packet.to_dump() + "]"
    #noinspection PyBroadException
    try:
        log_file.write("l([" + time + "," + text + ")\n")
    except:
        print("Failed to write to file")
    if not isinstance(packet, packets.KeepAlive) and not type(packet).__name__.startswith("Simulator"):
        sys.stdout.write("[" + text + "\n")
        sys.stdout.flush()


def get_next_log_filepath():
    index = 0
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)
    while True:
        filepath = os.path.join(LOG_DIR, "brewerylog_{:=#04}.py".format(index))
        if os.path.exists(filepath):
            index += 1
        else:
            return filepath
