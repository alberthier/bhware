#!/usr/bin/env python
# encoding: utf-8

import os
import sys
import shutil
from subprocess import *


PYTHON_ARCHIVE="Python-2.7.tar.bz2"
PYTHON_URL="http://python.org/ftp/python/2.7/{0}".format(PYTHON_ARCHIVE)
PYSERIAL_ARCHIVE="pyserial-2.5.tar.gz"
PYSERIAL_URL="http://pypi.python.org/packages/source/p/pyserial/{0}".format(PYSERIAL_ARCHIVE)
MERCURIAL_ARCHIVE = "mercurial-1.7.1.tar.gz"
MERCURIAL_URL = "http://mercurial.selenic.com/release/{0}".format(MERCURIAL_ARCHIVE)
BUILDROOT_VERSION = "2010.08"
BUILDROOT = "buildroot-{0}".format(BUILDROOT_VERSION)
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), BUILDROOT, "output")


def download():
    if not os.path.exists(BUILDROOT):
        archive = "{0}.tar.bz2".format(BUILDROOT)
        if not os.path.exists(archive):
            call(["wget", "http://buildroot.uclibc.org/downloads/{0}".format(archive)])
        call(["tar", "xjf", archive])
        call(["patch", "-d", BUILDROOT, "-p1", "-i", "../buildroot-2010.08-target-gcc.patch"])
        call(["patch", "-d", BUILDROOT, "-p1", "-i", "../buildroot-2010.08-dropbear-no-host-lookup.patch"])


def install_package(archive_url, archive):
    package = os.path.join(OUTPUT_DIR, "target", "root", archive)
    if not os.path.exists(package):
        call(["wget", archive_url, "-O", package])


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

    install_package(PYTHON_URL, PYTHON_ARCHIVE)
    install_package(PYSERIAL_URL, PYSERIAL_ARCHIVE)
    install_package(MERCURIAL_URL, MERCURIAL_ARCHIVE)

    # Make with the creation of the rootfs
    shutil.copy(config_src_filepath, ".config")
    call(["make"])


if __name__ == "__main__":
    # cd to the script directory
    os.chdir(os.path.dirname(__file__))
    download()
    build()
