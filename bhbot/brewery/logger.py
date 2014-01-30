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

TERM_RESET   = "\033[0m"
TERM_BOLD    = "\033[1m"

TERM_BLACK   = "\033[30m"
TERM_RED     = "\033[31m"
TERM_GREEN   = "\033[32m"
TERM_YELLOW  = "\033[33m"
TERM_BLUE    = "\033[34m"
TERM_MAGENTA = "\033[35m"
TERM_CYAN    = "\033[36m"
TERM_WHITE   = "\033[37m"

filepath = None
log_file = None
start_time = None
term_enable_colors = False
term_color = ""


def initialize(args = None):
    global filepath
    global log_file
    global start_time
    global term_enable_colors
    if filepath is None:
        start_time = datetime.datetime.now()
        filepath = get_next_log_filepath()
        if args is None:
            term_enable_colors = os.isatty(sys.stdout.fileno())
        else:
            if args.color == "always":
                term_enable_colors = True
            elif args.color == "never":
                term_enable_colors = False
            else:
                term_enable_colors = os.isatty(sys.stdout.fileno())
        if filepath != None:
            try:
                log("Logging to '{}'".format(os.path.split(filepath)[1]))
                log_file = open(filepath, "w")
                os.chmod(filepath, stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)
                log_file.write("#!/usr/bin/env python3\n")
                log_file.write("# encoding: utf-8\n\n")
                log_file.write("from packets import *\n\n")
                log_file.write("log = []\n\n")
                log_file.write("if __name__ == '__main__':\n")
                log_file.write("    def l(line):\n")
                log_file.write("        print(line)\n")
                log_file.write("else:\n")
                log_file.write("    def l(line):\n")
                log_file.write("        global log\n")
                log_file.write("        log.append(line)\n\n")
            except Exception as e:
                log_file = None
                log("Error opening log file for writing")
                log_exception(e)


def set_team(team):
    global term_color
    if team == TEAM_YELLOW:
        term_color = TERM_YELLOW
    else:
        term_color = TERM_RED


def close():
    global filepath
    global log_file
    if log_file is not None:
        try:
            log_file.close()
            subprocess.Popen(["/bin/sync"]).wait()
        except:
            pass
        filepath = None
        log_file = None


def log(obj, sender = "ARM", bold = False):
    global start_time
    global term_enable_colors
    global term_color
    global log_file
    delta = datetime.datetime.now() - start_time
    time = "'{:=0.02f}'".format(delta.total_seconds())
    if type(obj) != str:
        obj = str(obj)
    if log_file != None:
        try:
            log_file.write("l([" + time + ",'" + sender + "','LOG','# " + obj.replace("'", "\\'") + "'])\n")
            log_file.flush()
        except:
            pass
    try:
        if term_enable_colors:
            if bold:
                obj = TERM_BOLD + obj
            obj = term_color + obj
            if bold or len(term_color) != 0:
                obj += TERM_RESET
        sys.stdout.write(obj + "\n")
        sys.stdout.flush()
    except:
        pass


def dbg(obj, sender = "ARM"):
    pass
    #log(obj, sender)


def log_exception(exc):
    log("")
    for l in traceback.format_exception(type(exc), exc, None):
        for ll in l.splitlines():
            log(ll.rstrip(), "ARM", True)
    log("")


def log_packet(packet, sender = "ARM"):
    global log_file
    global start_time
    delta = datetime.datetime.now() - start_time
    time = "'{:=0.02f}'".format(delta.total_seconds())
    text = "'" + sender + "'," + type(packet).__name__ + ",\"" + packet.to_dump() + "\"]"
    ignore = type(packet).__name__ == "KeepAlive" and (packet.match_time <= 0 or packet.match_time >= 100000)
    if not ignore and log_file is not None:
        try:
            log_file.write("l([" + time + "," + text + ")\n")
            log_file.flush()
        except:
            pass
    ignore |= type(packet).__name__ in [ "InterbotPosition", "KeepAlive" ]
    if not ignore and not type(packet).__name__.startswith("Simulator"):
        try:
            sys.stdout.write("[" + text + "\n")
            sys.stdout.flush()
        except:
            pass


def get_next_log_filepath():
    if not os.path.exists(LOG_DIR):
        try:
            os.makedirs(LOG_DIR)
        except:
            log("Unable to create the log directory")
            return None
    max_index = 0
    for f in os.listdir(LOG_DIR):
        if f.startswith("brewerylog_") and f.endswith(".py"):
            try:
                i = int(f[11:15]) + 1
                max_index = max(max_index, i)
            except:
                pass
    return os.path.join(LOG_DIR, "brewerylog_{:=#04}.py".format(max_index))
