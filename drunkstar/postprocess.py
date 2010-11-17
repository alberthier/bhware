#!/usr/bin/env python
# encoding: utf-8

import os
import sys
import shutil
from subprocess import *

def copy_skeleton():
    skeleton = os.path.join(os.path.dirname(os.getcwd()), "skeleton")
    for item in os.listdir(skeleton):
        call(["cp", "-rf", os.path.join(skeleton, item), sys.argv[1]])
 
def copy_kernel():
    boot = os.path.join(sys.argv[1], "boot")
    if not os.path.exists(boot):
        os.mkdir(boot)
    kernel = os.path.join(os.path.dirname(sys.argv[1]), "images", "uImage")
    shutil.copy2(kernel, boot)

if __name__ == "__main__":
    copy_skeleton()
    copy_kernel()
    sys.exit(0)
