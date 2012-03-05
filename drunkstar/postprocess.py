#!/usr/bin/env python
# encoding: utf-8

import os
import sys
from subprocess import *


def copy_skeleton():
    skeleton = os.path.join(os.path.dirname(os.getcwd()), "skeleton")
    dest_root = sys.argv[1]

    sshd_start_script = os.path.join(dest_root, "etc", "init.d", "S50sshd")
    if os.path.exists(sshd_start_script):
        os.remove(sshd_start_script)
    for item in os.listdir(skeleton):
        call(["cp", "-rf", os.path.join(skeleton, item), dest_root]) 
    call(["ln", "-s", "../lib/sftp-server", os.path.join(dest_root, "usr", "libexec")])


if __name__ == "__main__":
    copy_skeleton()
    sys.exit(0)
