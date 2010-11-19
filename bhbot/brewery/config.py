#!/usr/bin/env python
# encoding: utf-8

import socket

from definitions import *




if socket.gethostname() == "drunkstar":
    host_device = HOST_DEVICE_ARM
    remote_ip = "192.168.0.201"
    serial_port_path = "/dev/ttyUSB0"
    orange_led_device_path = "/sys/class/leds/dockstar:orange:health/brightness"
    green_led_device_path = "/sys/class/leds/dockstar:green:health/brightness"
else:
    host_device = HOST_DEVICE_PC
    remote_ip = "127.0.0.1"
    serial_port_filepath = None
    orange_led_device_path = None
    green_led_device_path = None

remote_port = 2000
state_machine = "default"

