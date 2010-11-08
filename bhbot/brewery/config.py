#!/usr/bin/env python
# encoding: utf-8

import socket

from definitions import *




if socket.gethostname() == "drunkstar":
    host_device = HOST_DEVICE_ARM
    remote_ip = "192.168.0.201"
else:
    host_device = HOST_DEVICE_PC
    remote_ip = "127.0.0.1"

remote_port = 2000

