#!/usr/bin/env python
# encoding: utf-8

import os
import subprocess


def get_last_logfile():
    brewery_root_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
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


if __name__ == "__main__":
    path = get_last_logfile()
    print("# file: '{0}'".format(os.path.split(path)[1]))
    print("")
    subprocess.call([path])
