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
log_file = None
start_time = None
term_color = ""
term_reset = "\033[00m"
term_bold = "\033[01m"


def initialize():
    global filepath
    global log_file
    global start_time
    if filepath is None:
        start_time = datetime.datetime.now()
        filepath = get_next_log_filepath()
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
    if os.isatty(sys.stdout.fileno()):
        if team == TEAM_YELLOW:
            term_color = "\033[33m"
        else:
            term_color = "\033[31m"


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
    global term_color
    global term_reset
    global term_bold
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
        bld = term_bold if bold else ""
        sys.stdout.write(term_color + bld + obj + term_reset + "\n")
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
