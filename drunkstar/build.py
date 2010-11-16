#!/usr/bin/env python
# encoding: utf-8

import os
import sys
from subprocess import *


BUILDROOT_VERSION="2010.08"
BUILDROOT="buildroot-{0}".format(BUILDROOT_VERSION)


def download():
    if not os.path.exists(BUILDROOT):
        archive = "{0}.tar.bz2".format(BUILDROOT)
        if not os.path.exists(archive):
            call(["wget", "http://buildroot.uclibc.org/downloads/{0}".format(archive)])
        call(["tar", "xjf", archive])
        call(["ln", "-s", "../{0}.config".format(BUILDROOT), "{0}/.config".format(BUILDROOT)])


def build():
    call(["make"])


if __name__ == "__main__":
    # cd to the script directory
    os.chdir(os.path.dirname(__file__))
    download()
    build()
