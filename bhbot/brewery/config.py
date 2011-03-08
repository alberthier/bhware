#!/usr/bin/env python
# encoding: utf-8

import os
import socket

from definitions import *

if socket.gethostname() == "drunkstar":
    host_device = HOST_DEVICE_ARM
    remote_ip = "192.168.1.200"
    serial_port_path = "/dev/ttyUSB0"
    orange_led_device_path = "/sys/class/leds/dockstar:orange:health/brightness"
    green_led_device_path = "/sys/class/leds/dockstar:green:health/brightness"
    log_dir = "/root/logs"
else:
    host_device = HOST_DEVICE_PC
    remote_ip = "127.0.0.1"
    serial_port_path = None
    orange_led_device_path = None
    green_led_device_path = None
    log_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "logs")

serial_port_speed = 115200
remote_port = 7001
state_machine = "default"
