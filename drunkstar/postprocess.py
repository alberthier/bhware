#!/usr/bin/env python
# encoding: utf-8

import os
import sys
import shutil
from subprocess import *


def copy_skeleton():
    skeleton = os.path.join(os.path.dirname(__file__), "skeleton")
    dest_root = sys.argv[1]

    for item in os.listdir(skeleton):
        call(["cp", "-rf", os.path.join(skeleton, item), dest_root])

    dotvim = os.path.join(os.path.expanduser("~"), ".vim")
    call(["cp", "-rf", dotvim, os.path.join(dest_root, "root")])
    call(["ln", "-s", ".vim/vimrc", os.path.join(dest_root, "root", ".vimrc")])


def cleanup():
    vimdoc = os.path.join(sys.argv[1], "usr", "share", "vim", "vim73", "doc")
    if os.path.exists(vimdoc):
        shutil.rmtree(vimdoc)


if __name__ == "__main__":
    copy_skeleton()
    cleanup()
    sys.exit(0)
