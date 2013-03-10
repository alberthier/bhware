# encoding: utf-8

import os
import sys
import stat
import datetime
import subprocess
import traceback

from definitions import *


LOG_LINE_TIME    = 0
LOG_LINE_SENDER  = 1
LOG_LINE_PACKET  = 2
LOG_LINE_CONTENT = 3

filepath = None
log_lines = []
start_time = None
color_start = ""
color_stop = ""


def initialize():
    global filepath
    global start_time
    if filepath is None:
        start_time = datetime.datetime.now()
        filepath = get_next_log_filepath()
        log("Logging to '{}'".format(os.path.split(filepath)[1]))


def set_team(team):
    global color_start
    global color_stop
    if os.isatty(sys.stdout.fileno()):
        if team == TEAM_BLUE:
            color_start = "\033[34m"
        else:
            color_start = "\033[31m"
        color_stop = "\033[00m"


def close():
    global filepath
    global log_lines
    if filepath is None:
        return
    try:
        log_file = open(filepath, "w")
        os.chmod(filepath, stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)
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
        for log_line in log_lines:
            log_file.write(log_line)
        log_file.close()
    except:
        pass
    filepath = None
    log_lines = []
    subprocess.Popen(["/bin/sync"]).wait()


def log(text, sender = "ARM"):
    global start_time
    global color_start
    global color_stop
    global log_lines
    initialize()
    delta = datetime.datetime.now() - start_time
    time = "'{:=0.02f}'".format(delta.total_seconds())
    if type(text) != str:
        text = str(text)
    log_lines.append("l([" + time + ",'" + sender + "','LOG','# " + text.replace("'", "\\'") + "'])\n")
    sys.stdout.write(color_start + text + color_stop + "\n")
    sys.stdout.flush()


def log_exception(exc):
    msg = ""
    for l in traceback.format_exception(type(exc), exc, None):
        msg += l
    log(msg)


def log_packet(packet, sender = "ARM"):
    global log_lines
    initialize()
    delta = datetime.datetime.now() - start_time
    time = "'{:=0.02f}'".format(delta.total_seconds())
    text = "'" + sender + "'," + type(packet).__name__ + ",\"" + packet.to_dump() + "\"]"
    if not packet.is_keep_alive() or packet.match_time > 0 and packet.match_time < 90000:
        log_lines.append("l([" + time + "," + text + ")\n")
    if not packet.is_keep_alive() and not type(packet).__name__.startswith("Simulator"):
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
