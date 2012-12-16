#!/usr/bin/env python3
# encoding: utf-8

import os
import shutil




if __name__ == "__main__":
    brewery_root_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    for (dirpath, dirnames, filenames) in os.walk(brewery_root_path):
        for filename in filenames:
            if filename.endswith(".pyc"):
                os.remove(os.path.join(dirpath, filename))

    log_dir = os.path.join(brewery_root_path, "logs")
    if os.path.exists(log_dir):
        shutil.rmtree(log_dir)
