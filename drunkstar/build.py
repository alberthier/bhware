#!/usr/bin/env python
# encoding: utf-8

import os
import sys
import shutil
from subprocess import *


BUILDROOT_VERSION="2010.08"
BUILDROOT="buildroot-{0}".format(BUILDROOT_VERSION)
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), BUILDROOT, "output")


def download():
    if not os.path.exists(BUILDROOT):
        archive = "{0}.tar.bz2".format(BUILDROOT)
        if not os.path.exists(archive):
            call(["wget", "http://buildroot.uclibc.org/downloads/{0}".format(archive)])
        call(["tar", "xjf", archive])


def build():
    os.chdir(BUILDROOT)

    # Make without creating the rootfs
    config_src_filepath = "../{0}.config".format(BUILDROOT)
    config_src = file(config_src_filepath, "r")
    config_dst = file(".config", "w")

    for line in config_src:
        if not line.startswith("BR2_TARGET_ROOTFS_"):
            config_dst.write(line)

    config_src.close()
    config_dst.close()
    
    call(["make"])

    # Copy the kernel in the rootfs
    boot = os.path.join(OUTPUT_DIR, "target", "boot")
    if not os.path.exists(boot):
        os.mkdir(boot)
    shutil.copy(os.path.join(OUTPUT_DIR, "images", "uImage"), boot)

    # Make with the creation of the rootfs
    shutil.copy(config_src_filepath, ".config")
    call(["make"])


if __name__ == "__main__":
    # cd to the script directory
    os.chdir(os.path.dirname(__file__))
    download()
    build()
