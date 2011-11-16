#!/usr/bin/env python
# encoding: utf-8

import os
import sys
import stat
import datetime
import subprocess
import traceback

import packets
import config
from definitions import *


LOG_LINE_TIME   = 0
LOG_LINE_SENDER = 1
LOG_LINE_DATA   = 2
LOG_LINE_COUNT  = 3

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
        try:
            log_file = file(filepath, "w")
            os.chmod(filepath, stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)
        except:
            log_file = file(os.devnull, "w")
        log_file.write("#!/usr/bin/env python\n")
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


def close():
    global log_file
    global filepath
    if log_file == None:
        return
    log_file.close()
    filepath = None
    log_file = None
    subprocess.Popen(["/bin/sync"]).wait()


def log(text, sender = "ARM"):
    global log_file
    global start_time
    initialize()
    delta = datetime.datetime.now() - start_time
    time = "'{:=0.02f}'".format(float(delta.seconds) + (float(delta.microseconds)/1000000.0))
    try:
        log_file.write("l([" + time + ",'" + sender + "','# " + text.replace("'", "\\'") + "'])\n")
    except:
        print("Failed to write to file")
    sys.stdout.write(text + "\n")
    sys.stdout.flush()


def log_exception(exc, msg = None):
    if msg == None:
        msg = "Exception "
    log(msg + str(exc))
    for l in traceback.format_exc(exc).split("\n") :
        log("   "+l)


def log_packet(packet, sender = "ARM"):
    initialize()
    delta = datetime.datetime.now() - start_time
    time = "'{:=0.02f}'".format(float(delta.seconds) + (float(delta.microseconds)/1000000.0))
    text = "'" + sender + "'," + packet.to_code() + "]"
    try:
        log_file.write("l([" + time + "," + text + ")\n")
    except:
        print("Failed to write to file")
    if not isinstance(packet, packets.KeepAlive) and not isinstance(packet, packets.SimulatorData):
        sys.stdout.write("[" + text + "\n")
        sys.stdout.flush()


def get_next_log_filepath():
    index = 0
    if not os.path.exists(config.log_dir):
        os.mkdir(config.log_dir)
    while True:
        filepath = os.path.join(config.log_dir, "brewerylog_{:=#04}.py".format(index))
        if os.path.exists(filepath):
            index += 1
        else:
            return filepath
