#!/usr/bin/env python3

import os
import shutil
from subprocess import *

ROOTFS = "rootfs"
SKELETON = "skeleton"
DRUNKSTAR_CORE_URL = "https://bitbucket.org/bhteam/bhware-open/downloads/drunkstar_core.tar.bz2"
LOCAL_DRUNKSTAR_CORE = "dl/drunkstar_core.tar.bz2"
PYTHON2_SOFTS_URL = "https://bitbucket.org/bhteam/bhware-open/downloads/drunkstar_python-2.7.6_mercurial-2.9.2.tar.bz2"
LOCAL_PYTHON2_SOFTS = "dl/drunkstar_python-2.7.6_mercurial-2.9.2.tar.bz2"
PYTHON3_SOFTS_URL = "https://bitbucket.org/bhteam/bhware-open/downloads/drunkstar_python-3.4.0_pyserial-2.7.tar.bz2"
LOCAL_PYTHON3_SOFTS = "dl/drunkstar_python-3.4.0_pyserial-2.7.tar.bz2"
OPENCV_URL = "https://bitbucket.org/bhteam/bhware-open/downloads/drunkstar_opencv-2.4.8.tar.bz2"
LOCAL_OPENCV = "dl/drunkstar_opencv-2.4.8.tar.bz2"

PACKAGES_URLS     = [DRUNKSTAR_CORE_URL,   PYTHON2_SOFTS_URL,   PYTHON3_SOFTS_URL,   OPENCV_URL]
PACKAGES_ARCHIVES = [LOCAL_DRUNKSTAR_CORE, LOCAL_PYTHON2_SOFTS, LOCAL_PYTHON3_SOFTS, LOCAL_OPENCV]

def download():
    if not os.path.exists("dl"):
        os.mkdir("dl")
    for url, archive in zip(PACKAGES_URLS, PACKAGES_ARCHIVES):
        if not os.path.exists(archive):
            call(["wget", "-O", archive, url])

def cleanup():
    if os.path.exists(ROOTFS):
        print("Removing previous rootfs...")
        shutil.rmtree(ROOTFS)


def install_core():
    os.mkdir(ROOTFS)
    print("Extracting Drunkstar Core...")
    for archive in PACKAGES_ARCHIVES:
        call(["tar", "xjf", archive, "-C", ROOTFS])
    # remove VIM documentation
    shutil.rmtree(ROOTFS + "/usr/share/vim/vim73/doc")


def install_skeleton():
    print("Installing customized skeleton...")
    for f in os.listdir(SKELETON):
        call(["cp", "-rf", os.path.join(SKELETON, f), ROOTFS])
    # VIM customization
    call(["git", "clone", "https://github.com/alberthier/dotvim.git", "root/.vim"], cwd = ROOTFS)
    call(["git", "submodule", "init"], cwd = ROOTFS + "/root/.vim")
    call(["git", "submodule", "update"], cwd = ROOTFS + "/root/.vim")
    call(["ln", "-s", ".vim/vimrc", ".vimrc"], cwd = ROOTFS + "/root")
    shutil.rmtree(ROOTFS + "/root/.vim/.git")


if __name__ == "__main__":
    # cd to the script directory
    if os.getuid() != 0:
        print("You must run this program as root")
    else:
        os.chdir(os.path.dirname(__file__))
        download()
        cleanup()
        install_core()
        install_skeleton()
