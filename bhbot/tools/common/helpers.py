#!/usr/bin/env python
# encoding: utf-8

import os
import subprocess

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
