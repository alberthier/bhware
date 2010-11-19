#!/usr/bin/env python
# encoding: utf-8

import os
import sys
from subprocess import *


def copy_skeleton():
    skeleton = os.path.join(os.path.dirname(os.getcwd()), "skeleton")
    for item in os.listdir(skeleton):
        call(["cp", "-rf", os.path.join(skeleton, item), sys.argv[1]])
 

if __name__ == "__main__":
    copy_skeleton()
    sys.exit(0)
