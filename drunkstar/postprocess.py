#!/usr/bin/env python
# encoding: utf-8

import os
import sys
from subprocess import *


def copy_skeleton():
    skeleton = os.path.join(os.path.dirname(os.getcwd()), "skeleton")
    dest_root = sys.argv[1]

    for item in os.listdir(skeleton):
        call(["cp", "-rf", os.path.join(skeleton, item), dest_root]) 


if __name__ == "__main__":
    copy_skeleton()
    sys.exit(0)
