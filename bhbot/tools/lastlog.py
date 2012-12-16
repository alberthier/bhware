#!/usr/bin/env python3
# encoding: utf-8

import sys
import os
import subprocess

# Update PYTHONPATH
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "brewery"))
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "common"))

import helpers


if __name__ == "__main__":
    path = helpers.get_last_logfile()
    if path != None:
        subprocess.call([path])
    else:
        print("No log file found")
