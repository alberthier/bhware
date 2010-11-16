#!/usr/bin/env python
# encoding: utf-8

import os
import sys
from subprocess import *

if __name__ == "__main__":
    skeleton = os.path.join(os.path.dirname(os.getcwd()), "skeleton", "*")
    call(["cp", "-rf", skeleton, sys.argv[1]])
    sys.exit(0)
