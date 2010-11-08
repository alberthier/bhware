#!/usr/bin/env python
# encoding: utf-8

import os
import subprocess


if __name__ == "__main__":
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
                filepath = os.path.join(log_dir, "brewerylog_{0:=#04}.py".format(index))
                subprocess.call([filepath])
                break
