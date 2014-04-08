#!/usr/bin/env python3

import os
import shutil
from subprocess import *

ROOTFS = "rootfs"
SKELETON = "skeleton"
LOCAL_DRUNKSTAR_CORE = "dl/drunkstar-core.tar.bz2"

def cleanup():
    if os.path.exists(ROOTFS):
        print("Removing previous rootfs...")
        shutil.rmtree(ROOTFS)


def install_core():
    os.mkdir(ROOTFS)
    print("Extracting Drunkstar Core...")
    call(["tar", "xjf", LOCAL_DRUNKSTAR_CORE, "-C", ROOTFS])
    # remove VIM documentation
    shutil.rmtree(ROOTFS + "/usr/share/vim/vim73/doc")


def install_skeleton():
    print("Installing customized skeleton...")
    for f in os.listdir(SKELETON):
        call(["cp", "-rf", os.path.join(SKELETON, f), ROOTFS])


if __name__ == "__main__":
    # cd to the script directory
    if os.getuid() != 0:
        print("You must run this program as root")
    else:
        os.chdir(os.path.dirname(__file__))
        cleanup()
        install_core()
        install_skeleton()
