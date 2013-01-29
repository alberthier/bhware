#!/usr/bin/env python
# encoding: utf-8

import os
import sys
import shutil
from subprocess import *


BUILDROOT_VERSION = "2012.08"
BUILDROOT         = "buildroot-{}".format(BUILDROOT_VERSION)
BUILDROOT_ARCHIVE = "{}.tar.bz2".format(BUILDROOT)
BUILDROOT_URL     = "http://www.buildroot.org/downloads/{}".format(BUILDROOT_ARCHIVE)

PACKAGES = [
             (BUILDROOT_ARCHIVE, BUILDROOT_URL)
           ]

OUTPUT_DIR        = os.path.join(os.path.dirname(os.path.realpath(__file__)), BUILDROOT, "output")


def download():
    for (archive_name, url) in PACKAGES:
        archive_file = "dl/{}".format(archive_name)
        if not os.path.exists("dl"):
            os.mkdir("dl")
        if not os.path.exists(archive_file):
            call(["wget", url, "-O", archive_file])

    if not os.path.exists("{}/Makefile".format(BUILDROOT)):
        archive_file = "dl/{}".format(BUILDROOT_ARCHIVE)
        call(["tar", "xjf", archive_file])
        for f in sorted(os.listdir(".")):
            if f.startswith(BUILDROOT) and f.endswith(".patch"):
                print("Applying {}".format(f))
                call(["patch", "-d", BUILDROOT, "-p1", "-i", "../{}".format(f)])


def build():
    # Build
    os.chdir(BUILDROOT)
    if not os.path.exists(".config"):
        shutil.copyfile("../buildroot.config", ".config")
    call(["make"])


if __name__ == "__main__":
    # cd to the script directory
    os.chdir(os.path.dirname(__file__))
    download()
    build()

